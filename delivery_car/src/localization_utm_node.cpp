#include "delivery_car/localization_utm.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_utm");
    ros::NodeHandle nh;

    EKF ekf;
    ekf.Pose_Estimation_EKF();
    
    return 0;
}