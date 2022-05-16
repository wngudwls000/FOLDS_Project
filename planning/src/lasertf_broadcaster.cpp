#include "geometry_msgs/Quaternion.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Vector3.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class LaserTFBroadCaster {
private:
  ros::NodeHandle nh;
  ros::Subscriber posesub;
  geometry_msgs::Pose current_pose;

public:
  LaserTFBroadCaster() {
    posesub =
        nh.subscribe("/odom", 1000, &LaserTFBroadCaster::odomCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_pose = msg->pose.pose;
    static tf::TransformBroadcaster br;
    tf::Transform tf;
    tf.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion quat;
    geometry_msgs::Quaternion geo_quat;
    geo_quat.x = 0;
    geo_quat.y = 0;
    geo_quat.z = 0;
    geo_quat.w = 1;
    tf::quaternionMsgToTF(geo_quat, quat);
    tf.setRotation(quat);
    br.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "map", "odom"));
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lasertf_node");
  LaserTFBroadCaster lasertf;

  ros::spin();
}