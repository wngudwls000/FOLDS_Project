#include "delivery_car/localization.h"

// Define Function
EKF::EKF()
{
    gnss_sub = nh.subscribe("/ublox_gps/fix", 100, &EKF::GNSS_Callback, this);
    gnss_heading_sub = nh.subscribe("/ublox_gps/navpvt", 100, &EKF::GNSS_Heading_Callback, this);
    imu_sub = nh.subscribe("/odom", 100, &EKF::IMU_Callback, this);
    car_pose_pub = nh.advertise<nav_msgs::Odometry>("/mission_odometry", 100);

    // GNSS
    gnss_init = false;
    call_gnss = false;
    lon_raw = 0.0;
    lat_raw = 0.0;
    lon_rad = 0.0; 
    lat_rad = 0.0;
    local_x = 0.0;
    local_y = 0.0;
    prev_gnss_time = ros::Time::now();
    gnss_time = ros::Time::now();
    gnss_dt = 0.0;
    gnss_yaw_raw = 0.0;
    gnss_yaw_rad = 0.0;

    // IMU
    imu_init = false;
    angular_velocity_z = 0.0;
    linear_velocity_x = 0.0;
    linear_velocity_y = 0.0;
    imu_quaternion_x;
    imu_quaternion_y;
    imu_quaternion_z;
    imu_quaternion_w;
    imu_roll_rad;
    imu_pitch_rad;
    imu_yaw_rad;
    imu_roll_deg;
    imu_pitch_deg;
    imu_yaw_deg;
    prev_imu_time = ros::Time::now();
    imu_time = ros::Time::now();
    imu_dt = 0.0;

    //EKF
    isInit = false;
    linear_vel_x = 0.0;
    linear_vel_y = 0.0;
    velocity = 0.0;
    Q <<    0.0001d, 0.0d, 0.0d,           // [x_std_m^2 0 0;
            0.0d, 0.0001d, 0.0d,           //  0 y_std_m^2 0;
            0.0d, 0.0d, 0.01d;           //  0 0 yaw_std_deg^2]
    R <<    0.16d, 0.0d, 0.0d,                //  [x_measure_std_m^2 0;
            0.0d, 0.16d, 0.0d,
            0.0d, 0.0d, 0.0d;                //   0 y_measure_std_m^2]
    eye <<  1.0d, 0.0d, 0.0d,
            0.0d, 1.0d, 0.0d,
            0.0d, 0.0d, 1.0d;
    world_x = 0.0;
    world_y = 0.0;
    
    // ENU Conversion Data -- WGS84
    a = 6378137.0;
    ff = 1/298.257223563;
    e_sq = 2 * ff - ff * ff;

}

void EKF::GNSS_Callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gnss_init = true;
    call_gnss = true;
    lon_raw = msg->longitude;
    lat_raw = msg->latitude;
    hei_raw = msg->altitude;

    geometry_msgs::PoseStamped local_pose = geodetic_to_enu(lat_raw, lon_raw, hei_raw, 0.0, 0.0, 0.0);

    local_x = local_pose.pose.position.x;
    local_y = local_pose.pose.position.y;

    prev_gnss_time = gnss_time;
    gnss_time = msg->header.stamp;
    gnss_duration = gnss_time - prev_gnss_time;
    gnss_dt = gnss_duration.toSec();
}

void EKF::GNSS_Heading_Callback(const ublox_msgs::NavPVT &msg)
{
    gnss_yaw_raw = (msg.heading) / 100000.0;
    gnss_yaw_rad = gnss_yaw_raw * D_DEG_2_RAD;
}

void EKF::IMU_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    imu_init = true;
    angular_velocity_z = msg->twist.twist.angular.z;
    linear_velocity_x = msg->twist.twist.linear.x;
    linear_velocity_y = msg->twist.twist.linear.y;
    imu_quaternion_x = msg->pose.pose.orientation.x;
    imu_quaternion_y = msg->pose.pose.orientation.y;
    imu_quaternion_z = msg->pose.pose.orientation.z;
    imu_quaternion_w = msg->pose.pose.orientation.w;
    tf::Quaternion imu_q(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
    tf::Matrix3x3 imu_rpy(imu_q);
    
    imu_rpy.getRPY(imu_roll_rad, imu_pitch_rad, imu_yaw_rad);
    
    if (imu_yaw_rad > M_PI)
    {
        imu_yaw_rad -= 2 * M_PI;
    }
    if (imu_yaw_rad < -M_PI)
    {
        imu_yaw_rad += 2 * M_PI;
    }
    imu_roll_deg = imu_roll_rad * D_RAD_2_DEG;
    imu_pitch_deg = imu_pitch_rad * D_RAD_2_DEG;
    imu_yaw_deg = imu_yaw_rad * D_RAD_2_DEG;

    prev_imu_time = imu_time;
    imu_time = msg->header.stamp;
    imu_duration = imu_time - prev_imu_time;
    imu_dt = imu_duration.toSec();
}

Eigen::Matrix<double, 3, 1> EKF::Dead_Reckoning(Eigen::Matrix<double, 3, 1> x_pri, double vel, double angular_vel_z, double dt)
{
    dr <<   x_pri(0, 0) + vel * dt * cos(x_pri(2, 0)),
            x_pri(1, 0) + vel * dt * sin(x_pri(2, 0)),
            x_pri(2, 0) + angular_vel_z * dt;
    if (dr(2, 0) > M_PI)
    {
        dr(2, 0)  -= 2 * M_PI;
    }

    if (dr(2, 0)  < -M_PI)
    {
        dr(2, 0)  += 2 * M_PI;
    }

    return dr;
}

void EKF::Pose_Estimation_EKF()
{   
    std::cout << "ekf running"<< std::endl;

    if (Only_GNSS == true)
    {
        ros::Rate loop_rate_only_gnss(10.0);
        while(ros::ok())
        {
            estimated_pose(0,0) = local_x;
            estimated_pose(1,0) = local_y;
            estimated_pose(2,0) = gnss_yaw_rad;
            std::cout<<"gnss_yaw"<<gnss_yaw_rad<<std::endl;
            SetPose(estimated_pose(0, 0), estimated_pose(1, 0), estimated_pose(2, 0));
            Pub();
            loop_rate_only_gnss.sleep();
            ros::spinOnce();
        }
    }
    else
    {
        ros::Rate loop_rate(100.0);
        while(ros::ok())
        {
            if(gnss_init == true && isInit == false)
            {
                x_prior <<  local_x,
                            local_y,
                            gnss_yaw_rad;
                P_prior <<  1000.0d, 0.0d, 0.0d,
                            0.0d, 1000.0d, 0.0d,
                            0.0d, 0.0d, 1000.0d;
                estimated_pose = x_prior;

                isInit = true;
            }
            else
            {
                // System function
                linear_vel_x = linear_velocity_x;
                linear_vel_y = linear_velocity_y;
                // velocity = linear_vel_x * cos(imu_pitch_rad) * cos(imu_yaw_rad) + linear_vel_y * cos(imu_pitch_rad) * sin(imu_yaw_rad);
                velocity = sqrt(pow(linear_vel_x, 2) + pow(linear_vel_y, 2));
                f = Dead_Reckoning(x_prior, velocity, angular_velocity_z, imu_dt);

                // System jacobian
                F <<    1.0d, 0.0d, ((-1.0) * (velocity * imu_dt * sin(x_prior(2, 0)))),
                        0.0d, 1.0d, (velocity * imu_dt * cos(x_prior(2, 0))),
                        0.0d, 0.0d, 1.0d;

                x_prior = f;
                P_prior = F * P_prior * F.transpose() + Q;

                // Gnss Measurement
                z <<    local_x,
                        local_y,
                        gnss_yaw_rad;

                if (call_gnss == true && Only_Prediction == false)  // Measurement Update 
                {
                    // Measurement Update
                    h <<    x_prior(0, 0),
                            x_prior(1, 0),
                            x_prior(2, 0);

                    // Residual
                    y = z - h;
                    // Measurement jacobian
                    H <<    1.0d, 0.0d, 0.0d,
                            0.0d, 1.0d, 0.0d,
                            0.0d, 0.0d, 1.0d;

                    // Innovation covariance
                    S = H * P_prior * H.transpose() + R;

                    // Kalman gain
                    K = P_prior * H.transpose() * S.inverse();

                    //Update
                    x_posterior = x_prior + K * y;
                    P_posterior = (eye - K * H) * P_prior;
                    call_gnss = false;
                }
                else
                {
                    x_posterior = x_prior;
                    P_posterior = P_prior;
                }
                
                // Estimate Pose
                estimated_pose = x_posterior;
                x_prior = x_posterior;
                P_prior = P_posterior;
            }
            SetPose(estimated_pose(0, 0), estimated_pose(1, 0), estimated_pose(2, 0));
            Pub();
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
}

void EKF::SetPose(double pose_x, double pose_y, double pose_yaw)
{
    car_pose_msg.pose.pose.position.x = pose_x; // local x
    car_pose_msg.pose.pose.position.y = pose_y; // local y
    car_pose_msg.pose.pose.position.z = pose_yaw; // local yaw (차 기준 각도)

    // Imu yaw rate (Quaternion)
    car_pose_msg.pose.pose.orientation.x = imu_quaternion_x;
    car_pose_msg.pose.pose.orientation.y = imu_quaternion_y;
    car_pose_msg.pose.pose.orientation.z = imu_quaternion_z;
    car_pose_msg.pose.pose.orientation.w = imu_quaternion_w;

    car_pose_msg.twist.twist.linear.x = lat_raw; // 위도 (latitude)
    car_pose_msg.twist.twist.linear.y = lon_raw; // 경도 (longitude)
    car_pose_msg.twist.twist.linear.z = gnss_yaw_raw; // GPS 절대각 (GPS Heading)

    car_pose_msg.twist.twist.angular.z = imu_yaw_deg;
}

void EKF::Pub()
{
    car_pose_pub.publish(car_pose_msg);
}

geometry_msgs::PoseStamped EKF::geodetic_to_enu(double lat, double lon, double h, double lat_ref, double lon_ref, double h_ref)
{
    geodetic_to_enu_progress = geodetic_to_ecef(lat, lon, h);
    geodetic_to_enu_result = ecef_to_enu(geodetic_to_enu_progress(0, 0), geodetic_to_enu_progress(1, 0), geodetic_to_enu_progress(2, 0), lat_ref, lon_ref, h_ref);
    geometry_msgs::PoseStamped psstp_pose;

    psstp_pose.header.stamp = ros::Time::now();
    psstp_pose.header.frame_id = "map";

    psstp_pose.pose.position.x = geodetic_to_enu_result(0,0);
    psstp_pose.pose.position.y = geodetic_to_enu_result(1,0);
    psstp_pose.pose.position.z = geodetic_to_enu_result(2,0);

    return(psstp_pose);
}

Eigen::Matrix<double, 3, 1> EKF::geodetic_to_ecef(double lat, double lon, double h)
{
    double lambda = lon * D_DEG_2_RAD;
    double phi = lat * D_DEG_2_RAD;
    double N = a / sqrt(1 - e_sq * sin(phi) * sin(phi));
    geodetic_to_ecef_result <<      (h + N) * cos(lambda) * cos(phi),
                                    (h + N) * sin(lambda) * cos(phi),
                                    (h + (1 - e_sq) * N) * sin(phi);
    return geodetic_to_ecef_result;
}
Eigen::Matrix<double, 3, 1> EKF::ecef_to_enu(double x, double y, double z, double lat0, double lon0, double h0)
{
    double lambda= lon0 * D_DEG_2_RAD;
    double phi = lat0 * D_DEG_2_RAD;
    double N = a / sqrt(1 - e_sq * sin(phi) * sin(phi));
    double x0 = (h0 + N) * cos(phi) * cos(lambda);
    double y0 = (h0 + N) * cos(phi) * sin(lambda);
    double z0 = (h0 + (1 - e_sq) * N) * sin(phi);
    double xd = x - x0;
    double yd = y - y0;
    double zd = z - z0;
    ecef_to_enu_result <<   (-1) * sin(lambda) * xd + cos(lambda) * yd,
                            (-1) * cos(lambda) * sin(phi) * xd - sin(phi) * sin(lambda) * yd + cos(phi) * zd,
                            cos(lambda) * cos(phi) * xd + cos(phi) * sin(lambda) * yd + sin(phi) * zd;
    return ecef_to_enu_result;
}