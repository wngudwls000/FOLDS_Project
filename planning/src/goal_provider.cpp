#include "ros/node_handle.h"
#include "ros/publisher.h"
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>

class GoalProvider {
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;

  geometry_msgs::PoseStamped goal;

  bool button;

public:
  GoalProvider() {
    sub = nh.subscribe("move_base_simple/goal", 1, &GoalProvider::GoalCallback,
                       this);
    pub = nh.advertise<geometry_msgs::PoseStamped>("/mission_goal", 100);
    button = false;
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    goal = *msg;
    button = true;
  }

  void publishGoal() {
    if (button) {
      std::cout << "publishing goal..." << std::endl;
      pub.publish(goal);
    } else
      std::cout << "waiting for goal..." << std::endl;
  }

  
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "goal_provider");
  GoalProvider mission_goal;
  ros::Rate loop_rate(20);

  while (ros::ok()) {
    ros::spinOnce();
    mission_goal.publishGoal();
    loop_rate.sleep();
  }

  return 0;
}