#include "PID.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

enum Direction { // 열거형 정의
  UP = 0,        // 초깃값 할당
  DOWN,
  RIGHT,
  LEFT
};

class PathGetter {
private:
  ros::NodeHandle nh_;
  ros::Subscriber path_sub;
  nav_msgs::Path current_path;
  bool received_path;

public:
  PathGetter() : received_path(false) {
    path_sub =
        nh_.subscribe("mission_path", 1, &PathGetter::PathCallback, this);
  }
  void PathCallback(const nav_msgs::Path::ConstPtr msg) {
    received_path = true;
  }
  bool ReceivedPath() { return received_path; }
};

class PathFollower {
private:
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Subscriber path_sub;
  ros::Publisher twist_pub;

  geometry_msgs::Pose current_pose;
  geometry_msgs::Twist mission_twist;
  geometry_msgs::Twist current_twist;
  geometry_msgs::Pose next_pose;
  nav_msgs::Path shrinked_path;
  nav_msgs::Path current_path;
  vector<Direction> direction_list;

  bool received_path;

  int index_counter;
  int path_size;

  double current_angle;
  double desired_angle;
  double linear_velocity_input;
  double angular_velocity_input;

  const double max_speed;
  const double distance_error_limit;
  const double angle_error_limit;

  PID LinearVelocityPID;
  PID AngularVelocityPID;

public:
  PathFollower()
      : index_counter(0), max_speed(0.5), angular_velocity_input(0),
        linear_velocity_input(0), distance_error_limit(0.005),
        angle_error_limit(0.005),
        LinearVelocityPID(PID(0.1, max_speed, -max_speed, 0.1, 0.01, 0.5)),
        AngularVelocityPID(PID(0.1, max_speed, -max_speed, 0.1, 0.01, 0.5)) {
    odom_sub = nh.subscribe("odom", 1, &PathFollower::OdomCallback, this);
    twist_pub = nh.advertise<geometry_msgs::Twist>("mission_twist", 100);
    path_sub =
        nh.subscribe("mission_path", 1, &PathFollower::PathCallback, this);

    std::cout << "does at least begin" << std::endl;
  }
  void PathCallback(const nav_msgs::Path::ConstPtr msg) {
    current_path = *msg;
    path_size = current_path.poses.size();
    received_path = true;
  }
  bool ReceivedPath() { return received_path; }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr msg) {
    std::cout << "OdomCallback()" << std::endl;
    current_pose = msg->pose.pose;
    current_twist = msg->twist.twist;
  }

  void FollowPath() {
    next_pose = shrinked_path.poses[0].pose;
    if(DistanceBetweenCurrentAndNext())
  }

  void CreatingDirectionList() {
    Direction direction = DeterminingDirection(current_path.poses[0].pose,current_path.poses[1].pose);
    direction_list.push_back(direction);
    shrinked_path.poses.push_back(current_path.poses[0]);

    for (int i = 1; i < path_size; i++) {
        Direction new_direction = DeterminingDirection(current_path.poses[i].pose,current_path.poses[i+1].pose);
        if(direction == new_direction)
            continue;
        else{
            direction = new_direction;
            direction_list.push_back(direction);
            shrinked_path.poses.push_back(current_path.poses[i]);
        }
    }
  }

  Direction DeterminingDirection(geometry_msgs::Pose current_pose,
                                 geometry_msgs::Pose next_pose) {
    Direction direction;
    double distance_difference_vertical =
        current_pose.position.y - next_pose.position.y;
    double distance_difference_lateral =
        current_pose.position.x - next_pose.position.x;
    if (fabs(distance_difference_lateral) >
        fabs(distance_difference_vertical)) {
      if (distance_difference_lateral > 0)
        direction = LEFT;
      else
        direction = RIGHT;
    } else {
      if (distance_difference_vertical > 0)
        direction = DOWN;
      else
        direction = UP;
    }
    return direction;
  }

  void RotateUp(){SettingAngularVelocity(M_PI/2);}
  void RotateDown(){SettingAngularVelocity(-M_PI/2);}
  void RotateRight(){SettingAngularVelocity(0);}
  void RotateLeft(){SettingAngularVelocity(M_PI);}

  void SettingAngularVelocity(double setpoint) {
    double angular_velocity_increment =
        AngularVelocityPID.calculate(setpoint, angular_velocity_input);

    angular_velocity_input += angular_velocity_increment;

    mission_twist.angular.x = 0;
    mission_twist.angular.y = 0;
    mission_twist.angular.z = angular_velocity_input;
  }



  bool CheckArrivingToNextPoint() {
    if (DistanceBetweenCurrentAndNext() < distance_error_limit)
      return true;
    else
      return false;
  }

  bool CheckRotatingToNextPoint() {
    if (AngleBetweenCurrentAndNext() < angle_error_limit)
      return true;
    else
      return false;
  }

  void MovingToNextPoint() {
    SettingLinearVelocity(DistanceBetweenCurrentAndNext());
  }

  void RotatingToNextPoint() {
    SettingAngularVelocity(desired_angle - current_angle);
  }

  void SettingLinearVelocity(double setpoint) {
    double linear_velocity_increment =
        LinearVelocityPID.calculate(setpoint, linear_velocity_input);
    linear_velocity_input += linear_velocity_increment;

    mission_twist.linear.x = linear_velocity_input;
    mission_twist.linear.y = 0;
    mission_twist.linear.z = 0;
  }

  double DistanceBetweenCurrentAndNext() {
    double lengthx = next_pose.position.x - current_pose.position.x;
    double lengthy = next_pose.position.y - current_pose.position.y;

    double distance = sqrt(lengthx * lengthx + lengthy * lengthy);

    return distance;
  }

  
  void GettingDesiredAngle() {
    double angle = AngleBetweenCurrentAndNext();

    current_angle = QuaternionToEuler(current_pose.orientation);
    desired_angle = angle + current_angle;
    if (desired_angle > M_PI)
      desired_angle -= M_PI;
    else if (desired_angle < -M_PI)
      desired_angle += M_PI;
  }

  double AngleBetweenCurrentAndNext() {
    double lengthx = next_pose.position.x - current_pose.position.x;
    double lengthy = next_pose.position.y - current_pose.position.y;

    double angle = atan2(lengthy, lengthx);
    return angle;
  }

  double QuaternionToEuler(geometry_msgs::Quaternion quat_geometry) {
    tf2::Quaternion quat_tf2;
    tf2::convert(quat_geometry, quat_tf2);

    tf2::Matrix3x3 m(quat_tf2);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
  }

  void publish() { twist_pub.publish(mission_twist); }
};

int main(int argc, char **argv) {

  // 이 구조를 바꾸자
  // 이 구조에 문제가 있을수도
  ros::init(argc, argv, "path_follow");

  PathFollower path_follower;
  ros::Rate loop_rate(20);

  std::cout << "before going into while" << std::endl;
  while (1) {
    ros::spinOnce();
    if (path_follower.ReceivedPath()) {
      path_follower.FollowPath();
    } else {
      ROS_INFO("waiting for path...\n");
    }
    loop_rate.sleep();
  }
}
