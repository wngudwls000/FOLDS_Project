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

class PathFollower {
private:
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Subscriber path_sub;
  ros::Publisher twist_pub;
  ros::Subscriber goal_sub;
  //
  ros::Publisher next_pose_pub;

  geometry_msgs::Pose current_pose;
  geometry_msgs::Twist mission_twist;
  geometry_msgs::Pose next_pose;
  geometry_msgs::Pose goal_pose;
  nav_msgs::Path current_path;

  bool received_path;

  int path_size;
  int look_ahead;

  double linear_velocity_input;
  double angular_velocity_input;
  double current_angle;
  double desired_angle;
  double angle_setpoint;
  double linear_velocity_increment_factor;

  const double linear_max_speed;
  const double angular_max_speed;
  const double distance_error_limit;
  double angle_error_limit;
  bool moving_state;

  PID LinearVelocityPID;
  PID AngularVelocityPID;

public:
  PathFollower()
      : look_ahead(10), linear_max_speed(0.5), angular_max_speed(1.0),
        angular_velocity_input(0), linear_velocity_input(0),
        moving_state(false), distance_error_limit(0.3), angle_error_limit(0.01),
        received_path(false), current_angle(0), desired_angle(0),
        linear_velocity_increment_factor(1),
        LinearVelocityPID(
            PID(0.1, linear_max_speed, -linear_max_speed, 0.1, 0.01, 0.5)),
        AngularVelocityPID(
            PID(0.1, angular_max_speed, -angular_max_speed, 0.1, 0.01, 0.5)) {
    odom_sub = nh.subscribe("/odom", 1, &PathFollower::OdomCallback, this);
    twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    path_sub =
        nh.subscribe("/mission_path", 1, &PathFollower::PathCallback, this);
    goal_sub =
        nh.subscribe("/mission_goal", 1, &PathFollower::GoalCallback, this);
    //
    next_pose_pub = nh.advertise<geometry_msgs::Pose>("/next_pose", 100);
  }
  void PathCallback(const nav_msgs::Path::ConstPtr msg) {
    current_path = *msg;
    path_size = current_path.poses.size();
    if (received_path == false)
      next_pose = SettingNextPose();
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
    next_pose = SettingNextPose();
    current_angle = QuaternionToEuler(current_pose.orientation);
    desired_angle = AngleBetweenCurrentAndPoint(current_pose, next_pose);
    angle_setpoint = SettingAngleSetpoint();
    angle_error_limit = SettingAngleErrorLimit();

    if (CheckArrivingAtGoal()) {
      StopMoving();
      StopRotating();
      std::cout << "Arrived At Destination!!" << endl;
    } else {
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
    publish();
  }

  enum Direction { // 열거형 정의
    UP = 0,        // 초깃값 할당
    DOWN,
    RIGHT,
    LEFT
  };

  geometry_msgs::Pose SettingNextPose() {

    geometry_msgs::Pose current = current_path.poses[0].pose;
    geometry_msgs::Pose next = current_path.poses[1].pose;
    int counter = 1;

    Direction current_direction = DeterminingDirection(current, next);

    while (true) {
      geometry_msgs::Pose next_next = current_path.poses[counter + 1].pose;
      if (DeterminingDirection(next, next_next) == current_direction) {
        next = next_next;
        counter++;
      } else {
        break;
      }
    }
    return next;
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

  double SettingAngleSetpoint() {
    double angle_setpoint = desired_angle - current_angle;
    if (fabs(angle_setpoint) > M_PI) {
      if (current_angle > 0)
        angle_setpoint = 2 * M_PI - fabs(desired_angle) - fabs(current_angle);
      if (desired_angle > 0)
        angle_setpoint = -2 * M_PI + fabs(desired_angle) + fabs(current_angle);
    }
    return angle_setpoint;
  }

  double SettingAngleErrorLimit() {
    if (moving_state)
      angle_error_limit = 0.1;
    else
      angle_error_limit = 0.01;
    return angle_error_limit;
  }

  bool CheckArrivingAtGoal() {
    if (DistanceBetweenCurrentAndNext(current_pose, goal_pose) <
        distance_error_limit)
      return true;
    else
      return false;
  }
  bool CheckRotatingToNextPoint() {
    if (fabs(angle_setpoint) < angle_error_limit)
      return true;
    else
      return false;
    // }
  }

  void StopMoving() {
    mission_twist.linear.x = 0;
    moving_state = false;
  }
  void StopRotating() { mission_twist.angular.z = 0; }
  void MovingToNextPoint() {
    moving_state = true;
    SettingLinearVelocity(
        DistanceBetweenCurrentAndNext(current_pose, next_pose) *
        linear_velocity_increment_factor);
  }
  void RotatingToNextPoint() {
    std::cout << "current angle in radians: " << current_angle << std::endl;
    SettingAngularVelocity(angle_setpoint);
  }

  void SettingLinearVelocity(double setpoint) {
    double linear_velocity_increment =
        LinearVelocityPID.calculate(setpoint, linear_velocity_input);
    linear_velocity_input += linear_velocity_increment;

    mission_twist.linear.x = linear_velocity_input * 20;
    mission_twist.linear.y = 0;
    mission_twist.linear.z = 0;
  }
  void SettingAngularVelocity(double setpoint) {
    double angular_velocity_increment =
        AngularVelocityPID.calculate(setpoint, angular_velocity_input);

    angular_velocity_input += angular_velocity_increment;

    angular_velocity_input = CalmDownIfVelocityPopsOverMaxSpeed(
        angular_velocity_input, angular_max_speed);

    mission_twist.angular.x = 0;
    mission_twist.angular.y = 0;
    mission_twist.angular.z = angular_velocity_input;
  }

  double CalmDownIfVelocityPopsOverMaxSpeed(double velocity_input,
                                            double max_speed) {
    if (velocity_input > max_speed) {
      velocity_input = max_speed;
    } else if (velocity_input < -max_speed) {
      velocity_input = -max_speed;
    }
    return angular_velocity_input;
  }

  double DistanceBetweenCurrentAndNext(geometry_msgs::Pose current_pose,
                                       geometry_msgs::Pose next_pose) {
    double lengthx = next_pose.position.x - current_pose.position.x;
    double lengthy = next_pose.position.y - current_pose.position.y;
    double distance = sqrt(lengthx * lengthx + lengthy * lengthy);
    return distance;
  }

  double AngleBetweenCurrentAndPoint(geometry_msgs::Pose current_pose,
                                     geometry_msgs::Pose point_pose) {
    double pointx = point_pose.position.x;
    double pointy = point_pose.position.y;
    double currentx = current_pose.position.x;
    double currenty = current_pose.position.y;
    double lengthx = pointx - currentx;
    double lengthy = pointy - currenty;
    double angle;
    if (currenty == pointy) {
      if (pointx < currentx)
        angle = M_PI;
      else
        angle = 0;
    } else if (currentx == pointx) {
      if (pointy > currenty)
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
    next_pose_pub.publish(next_pose);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_follow");

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
