#include "PID.h"
#include "ros/forwards.h"
#include "ros/init.h"
#include "ros/subscriber.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TwistTester {
private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Subscriber goal_sub;
  geometry_msgs::Twist twist;
  geometry_msgs::Pose goal;

  nav_msgs::Odometry odom;
  double max_speed;
  double angular_velocity_input;
  bool received_odom;

public:
  TwistTester()
      : max_speed(0.5), angular_velocity_input(0), received_odom(false) {
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    sub = nh.subscribe("odom", 1, &TwistTester::OdomCallback, this);
    goal_sub =
        nh.subscribe("mission_goal", 1, &TwistTester::GoalCallback, this);
  }
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr msg) {
    goal = msg->pose;
  }

  void publish() {
    tf2::Quaternion current_quat;
    tf2::convert(odom.pose.pose.orientation, current_quat);
    double CurrentAngle = EulerToQuaternion(current_quat);
    double DesiredAngle = AngleBetweenCurrentAndNext();

    PID AngularVelocityPID(0.1, max_speed, -max_speed, 0.1, 0.01, 0.5);

    double angular_velocity_increment;

    angular_velocity_increment = AngularVelocityPID.calculate(
        DesiredAngle - CurrentAngle, angular_velocity_input);

    std::cout << "current angle in radians: " << CurrentAngle << std::endl;
    std::cout << "angular_velocity_input: " << angular_velocity_input << "\n"
              << std::endl;
    angular_velocity_input += angular_velocity_increment;

    twist.linear.x = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = angular_velocity_input;
    // twist.angular.z = 0;
    pub.publish(twist);
  }

  double AngleBetweenCurrentAndNext() {
    double nextx = goal.position.x;
    double nexty = goal.position.y;
    double currentx = odom.pose.pose.position.x;
    double currenty = odom.pose.pose.position.y;
    double lengthx = nextx - currentx;
    double lengthy = nexty - currenty;
    double angle;

    if (currenty == nexty) {
      if (nextx < currentx)
        angle = M_PI;
      else
        angle = 0;
    } else if (currentx == nextx) {
      if (nexty > currenty)
        angle = M_PI / 2;
      else
        angle = -M_PI / 2;
    } else {
      angle = atan(lengthy / lengthx);
      if (lengthx < 0 && lengthy > 0)
        angle += M_PI;
      else if (lengthx < 0 && lengthy < 0)
        angle -= M_PI;
    }

    std::cout << "angle: " << angle << std::endl;
    return angle;
  }

  double EulerToQuaternion(tf2::Quaternion Quat) {
    tf2::Matrix3x3 m(Quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr msg) {
    odom = *msg;
    received_odom = true;
  }
  bool ReceivedOdom() { return received_odom; }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test1");

  TwistTester test;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    if (test.ReceivedOdom())
      test.publish();
    ros::spinOnce();
    loop_rate.sleep();
  }
}