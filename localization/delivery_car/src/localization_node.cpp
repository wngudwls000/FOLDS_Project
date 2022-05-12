#include "delivery_car/localization.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization");
    ros::NodeHandle nh;

    EKF ekf;
    ekf.Pose_Estimation_EKF();
    
    return 0;
}