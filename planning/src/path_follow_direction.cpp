#include "PID.h"
#include "geometry_msgs/Quaternion.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
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

class PathFollower {
private:
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Subscriber path_sub;
  ros::Publisher twist_pub;
  ros::Subscriber goal_sub;
  //
  ros::Publisher next_pose_pub;
  int path_size;

  geometry_msgs::Pose current_pose;
  geometry_msgs::Twist mission_twist;
  geometry_msgs::Pose next_pose;
  geometry_msgs::PoseStamped next_pose_stamped;
  geometry_msgs::Pose goal_pose;
  nav_msgs::Path current_path;

  Direction current_direction;

  bool received_path;
  bool moving_state;
  double angular_velocity_input;
  double linear_velocity_input;
  double desired_angle;
  double current_angle;
  double angle_setpoint;
  double angle_error_limit;
  const double distance_error_limit;
  const double linear_velocity_increment_factor;

  PID LinearVelocityPID;
  PID AngularVelocityPID;

public:
  PathFollower()
      : received_path(false), moving_state(false), angular_velocity_input(0),
        linear_velocity_input(0), desired_angle(0), current_angle(0),
        angle_setpoint(0), angle_error_limit(0.001), distance_error_limit(0.3),
        linear_velocity_increment_factor(3),
        LinearVelocityPID(PID(0.1, 100, -100, 0.1, 0.01, 0.5)),
        AngularVelocityPID(PID(0.1, 100, -100, 0.1, 0.01, 0.5)) {
    odom_sub = nh.subscribe("/odom", 1, &PathFollower::OdomCallback, this);
    twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    path_sub =
        nh.subscribe("/mission_path", 1, &PathFollower::PathCallback, this);
    goal_sub =
        nh.subscribe("/mission_goal", 1, &PathFollower::GoalCallback, this);
    //
    next_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/next_pose", 100);
  }
  void PathCallback(const nav_msgs::Path::ConstPtr msg) {
    current_path = *msg;
    path_size = current_path.poses.size();
    // if (received_path == false)
    //   next_pose = SettingNextPose();
    received_path = true;
  }
  bool ReceivedPath() { return received_path; }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr msg) {
    current_pose = msg->pose.pose;
  }
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr msg) {
    goal_pose = msg->pose;
  }

  void FollowPath() {
    if (CheckArrivingAtGoal()) {
      StopMoving();
      StopRotating();
      std::cout << "Arrived At Destination!!" << endl;
    } else {
      GoToNextPose();
    }
    publish();
  }

  bool CheckArrivingAtGoal() {
    if (DistanceBetweenTwoPoints(current_pose, goal_pose) <
        distance_error_limit)
      return true;
    else
      return false;
  }

  void GoToNextPose() {
    SettingDirectionAndNextPose();
    SettingCurrentAngle();
    SettingDesiredAngle();
    SettingAngleSetpoint();
    SettingAngleErrorLimit();

    if (CheckRotatingToNextPoint()) {
      std::cout << "Moving" << std::endl;
      MovingToNextPoint();
      StopRotating();
    } else {
      std::cout << "Stopping" << std::endl;
      StopMoving();
      RotatingToNextPoint();
    }
  }

  void SettingDirectionAndNextPose() {
    Direction direction = DeterminingDirection(current_path.poses[0].pose,
                                               current_path.poses[1].pose);
    int index_counter;
    for (int i = 1; i < path_size; i++) {
      Direction new_direction = DeterminingDirection(
          current_path.poses[i].pose, current_path.poses[i + 1].pose);
      if (direction != new_direction) {
        index_counter = i;
        break;
      }
    }
    next_pose = current_path.poses[index_counter].pose;
    current_direction = direction;
    std::cout << "direction: " << direction << std::endl;
  }

  void SettingCurrentAngle() {
    current_angle = QuaternionToEuler(current_pose.orientation);
    std::cout << "current angle in radians: " << current_angle << std::endl;
  }

  void SettingDesiredAngle() {
    switch (current_direction) {
    case UP:
      desired_angle = M_PI / 2;
      break;
    case DOWN:
      desired_angle = -M_PI / 2;
      break;
    case RIGHT:
      desired_angle = 0;
      break;
    case LEFT:
      desired_angle = M_PI;
      break;
    default:
      break;
    }
    std::cout << "desired angle in radians: " << desired_angle << std::endl;
  }

  void SettingAngleSetpoint() {
    angle_setpoint = desired_angle - current_angle;
    if (fabs(angle_setpoint) > M_PI) {
      if (current_angle > 0)
        angle_setpoint = 2 * M_PI - fabs(desired_angle) - fabs(current_angle);
      if (desired_angle > 0)
        angle_setpoint = -2 * M_PI + fabs(desired_angle) + fabs(current_angle);
    }
    std::cout << "angle setpoint in radians: " << angle_setpoint << std::endl;
  }

  void SettingAngleErrorLimit() {
    if (moving_state)
      angle_error_limit = 0.1;
    else
      angle_error_limit = 0.001;
  }

  bool CheckRotatingToNextPoint() {
    if (fabs(angle_setpoint) < angle_error_limit)
      return true;
    else
      return false;
  }

  void MovingToNextPoint() {
    SettingLinearVelocity(DistanceBetweenTwoPoints(current_pose, next_pose) *
                          linear_velocity_increment_factor);
    moving_state = true;
  }

  void RotatingToNextPoint() { SettingAngularVelocity(angle_setpoint); }

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

  void StopMoving() {
    mission_twist.linear.x = 0;
    moving_state = false;
  }
  void StopRotating() { mission_twist.angular.z = 0; }

  void SettingAngularVelocity(double setpoint) {
    double angular_velocity_increment =
        AngularVelocityPID.calculate(setpoint, angular_velocity_input);

    angular_velocity_input += angular_velocity_increment;

    mission_twist.angular.x = 0;
    mission_twist.angular.y = 0;
    mission_twist.angular.z = angular_velocity_input;
  }

  void SettingLinearVelocity(double setpoint) {
    double linear_velocity_increment =
        LinearVelocityPID.calculate(setpoint, linear_velocity_input);
    linear_velocity_input += linear_velocity_increment;

    mission_twist.linear.x =
        linear_velocity_input * linear_velocity_increment_factor;
    mission_twist.linear.y = 0;
    mission_twist.linear.z = 0;
  }

  double DistanceBetweenTwoPoints(geometry_msgs::Pose current_pose,
                                  geometry_msgs::Pose next_pose) {
    double lengthx = next_pose.position.x - current_pose.position.x;
    double lengthy = next_pose.position.y - current_pose.position.y;
    double distance = sqrt(lengthx * lengthx + lengthy * lengthy);
    return distance;
  }

  double QuaternionToEuler(geometry_msgs::Quaternion quat_geometry) {
    tf2::Quaternion quat_tf2;
    tf2::convert(quat_geometry, quat_tf2);
    tf2::Matrix3x3 m(quat_tf2);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  void publish() {
    std::cout << mission_twist << std::endl;
    twist_pub.publish(mission_twist);
    //
    next_pose_stamped.pose = next_pose;
    next_pose_stamped.header.frame_id = "map";
    next_pose_pub.publish(next_pose_stamped);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_follow_direction");

  PathFollower path_follower;
  ros::Rate loop_rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    if (path_follower.ReceivedPath()) {
      path_follower.FollowPath();
    } else {
      ROS_INFO("waiting for path...\n");
    }
    loop_rate.sleep();
  }
}
