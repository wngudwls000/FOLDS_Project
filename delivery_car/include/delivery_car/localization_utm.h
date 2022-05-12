#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include "ublox_msgs/NavPVT.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

#define Only_Prediction false
#define Only_GNSS false

class EKF{
public:
    EKF();
    Eigen::Matrix<double, 3, 1> Dead_Reckoning(Eigen::Matrix<double, 3, 1> x_pri, double vel, double angular_vel_z, double dt);
    void Pose_Estimation_EKF();
    void SetPose(double pose_x, double pose_y, double pose_yaw);
    void Pub();

    const double D_RAD_2_DEG = 180 / M_PI;
    const double D_DEG_2_RAD = M_PI / 180;

private:
    void UTM_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void GNSS_Heading_Callback(const ublox_msgs::NavPVT &msg);
    void IMU_Callback(const nav_msgs::Odometry::ConstPtr &msg);

    // ROS Data
    ros::NodeHandle nh;
    ros::Publisher car_pose_pub;
    ros::Subscriber utm_sub;
    ros::Subscriber gnss_heading_sub;
    ros::Subscriber imu_sub;
    nav_msgs::Odometry car_pose_msg;

    // GNSS Data
    bool gnss_init;
    bool call_gnss;
    double local_x;
    double local_y;
    double gnss_yaw_raw;  // [deg]
    double gnss_yaw_rad;

    // IMU Data
    bool imu_init;
    double angular_velocity_z;    // [rad/s]
    double linear_velocity_x;
    double linear_velocity_y;
    double imu_quaternion_x;
    double imu_quaternion_y;
    double imu_quaternion_z;
    double imu_quaternion_w;
    double imu_roll_rad;
    double imu_pitch_rad;
    double imu_yaw_rad;
    double imu_roll_deg;
    double imu_pitch_deg;
    double imu_yaw_deg;
    ros::Time prev_imu_time;
    ros::Time imu_time;
    ros::Duration imu_duration;
    double imu_dt;

    // Dead Reckoning Data
    Eigen::Matrix<double, 3, 1> dr;

    // EKF Data
    bool isInit;
    double linear_vel_x;
    double linear_vel_y;
    double velocity;
    Eigen::Matrix<double, 3, 1> f;
    Eigen::Matrix3d F;
    Eigen::Matrix<double, 3, 1> x_prior;
    Eigen::Matrix3d P_prior;
    Eigen::Matrix3d Q;
    Eigen::Matrix3d R;
    Eigen::Matrix<double, 3, 1> z;
    Eigen::Matrix<double, 3, 1> h;
    Eigen::Matrix<double, 3, 1> y;
    Eigen::Matrix3d H;
    Eigen::Matrix3d S;
    Eigen::Matrix3d K;
    Eigen::Matrix<double, 3, 1> x_posterior;
    Eigen::Matrix3d P_posterior;
    Eigen::Matrix<double, 3, 1> estimated_pose;
    Eigen::Matrix3d eye;
    double world_x;
    double world_y;
};