#include "Cell.h"
#include "Map.h"
#include "Node.h"
#include "Point.h"
#include "ros/init.h"
#include "ros/subscriber.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
using namespace std;

// 해결
int rounding(double fnum) {
  int temp = (int)fnum;
  if (fnum > 0) {
    if (fnum - temp >= 0.5)
      return temp + 1;
    else
      return temp;
  } else {
    if (temp - fnum >= 0.5)
      return temp - 1;
    else
      return temp;
  }
}

class DStarLite {
private:
  ros::NodeHandle nh;

  ros::Subscriber mission_current_sub;
  ros::Subscriber grid_sub;
  ros::Subscriber odom_sub;
  // goal 위치 받는거
  ros::Subscriber goal_sub;
  //   ros::Subscriber repath_sub;
  ros::Publisher mission_path_pub;

  nav_msgs::OccupancyGrid current_grid;
  geometry_msgs::Pose current_position;
  geometry_msgs::Pose origin;

  int km;
  int width;
  int height;
  int path_size;
  int lidar_length;
  vector<Point> predecessor_rule;
  vector<Point> successor_rule;

  Point start;
  Point goal;
  Map *map_env;
  Node **map;
  priority_queue<Point, vector<Point>, less<Point> > Q;
  // Path로 바꾸셈
  vector<Point> path;
  bool grid_button;
  // 임시로 goal setting
  bool goal_button;
  bool odom_button;
  bool initial_path_planned;

  bool restart_button;

public:
  DStarLite(int width, int height)
      : width(width), height(height), km(0), path_size(0), lidar_length(0) {
    grid_sub = nh.subscribe("/mission_grid", 1, &DStarLite::GridCallback, this);
    odom_sub = nh.subscribe("/odom", 1, &DStarLite::OdomCallback, this);
    mission_path_pub = nh.advertise<nav_msgs::Path>("/mission_path", 1000);
    // 임시로 goal setting
    goal_sub = nh.subscribe("mission_goal", 1, &DStarLite::GoalCallback, this);
    // repath_sub = nh.subscribe("mission_repath_button", 1,
    // &DStarLite::RepathCallback, this);

    // goal은 parameter space에서 받아와서 값 넣어주기
    map_env = new Map(width, height);
    map = map_env->getMap();

    // predecessor이랑 successor를 어떻게 할지는 나중에 결정
    // Point p_rule[] = {Point(0, 1),   Point(-1, 1), Point(1, 1), Point(0, -1),
    //                   Point(-1, -1), Point(1, -1), Point(1, 0), Point(-1,
    //                   0)};
    Point p_rule[] = {Point(0, 1), Point(0, -1), Point(1, 0), Point(-1, 0)};
    // Point s_rule[] = {Point(0, 1),   Point(-1, 1), Point(1, 1), Point(0, -1),
    //                   Point(-1, -1), Point(1, -1), Point(1, 0), Point(-1,
    //                   0)};
    Point s_rule[] = {Point(0, 1), Point(0, -1), Point(1, 0), Point(-1, 0)};

    for (int i = 0; i < 8; i++)
      predecessor_rule.push_back(p_rule[i]);
    for (int i = 0; i < 8; i++)
      successor_rule.push_back(s_rule[i]);

    initial_path_planned = false;
    grid_button = false;
    goal_button = false;
    odom_button = false;

    restart_button = false;
  }
  ~DStarLite() { delete map_env; }

  //   void RepathCallback(const std_msgs::Bool::ConstPtr msg){

  //   }

  //   임시로 goal setting
  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    if (grid_button) {
      if (goal_button == false) {
        goal.setX(rounding((msg->pose.position.x - origin.position.x) /
                           current_grid.info.resolution));
        goal.setY(rounding((msg->pose.position.y - origin.position.y) /
                           current_grid.info.resolution));
        map[goal.getX()][goal.getY()].setRHS(0);
        ComputeKey(goal);
        Q.push(goal);

        goal_button = true;
      } else {
        int temp_goal_x = rounding((msg->pose.position.x - origin.position.x) /
                                   current_grid.info.resolution);
        int temp_goal_y = rounding((msg->pose.position.y - origin.position.y) /
                                   current_grid.info.resolution);
        if (goal.getX() != temp_goal_x || goal.getY() != temp_goal_y) {
          restart_button = true;
        }
      }
    }
  }

  bool restart() {
    if (restart_button)
      cout << "got new goal position, restarting..." << endl;
    return restart_button;
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_position = msg->pose.pose;

    start.setX(rounding((current_position.position.x - origin.position.x) /
                        current_grid.info.resolution));
    start.setY(rounding((current_position.position.y - origin.position.y) /
                        current_grid.info.resolution));
    odom_button = true;
  }

  void GridCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    current_grid = *msg;
    origin = current_grid.info.origin;

    // parameter space에서 lidar 길이 받아오기. 여기서 임의로 8설정.
    float lidar = 8.0;
    int tempx;
    int tempy;
    lidar_length = (int)(lidar / current_grid.info.resolution);

    // 현재 위치에서 lidar 거리 안에 occupancy grid에서 새로 생긴 장애물들에
    // map을 update를 시킴.
    for (int i = 1 - lidar_length; i < lidar_length; i++) {
      for (int j = 1 - lidar_length; j < lidar_length; j++) {
        tempx = start.getX() + i;
        tempy = start.getY() + j;
        if (tempx < 0 || tempx > width - 1 || tempy < 0 || tempy > height - 1)
          continue;
        if (map[tempx][tempy].getMemory() == false) {
          if (is_obstacle(Point(tempx, tempy))) {
            map[tempx][tempy].setG(inf);
            map[tempx][tempy].setRHS(inf);
            updatePredecessors(tempx, tempy);
            map[tempx][tempy].setMemory(true);
          } else {
            continue;
          }
        } else {
          if (is_obstacle(Point(tempx, tempy))) {
            continue;
          } else {
            map[tempx][tempy].setG(inf);
            updateRHS(tempx, tempy);
            updatePredecessors(tempx, tempy);
          }
        }
      }
    }
    grid_button = true;
  }

  float heuristics(Point &s, Point &start) {
    float h;
    int side1 = abs(s.getX() - start.getX());
    int side2 = abs(s.getY() - start.getY());
    h = sqrt(side1 * side1 + side2 * side2);

    return h;
  }

  float minimum(float a, float b) {
    if (b < a)
      return b;
    else
      return a;
  }

  float cost(Point a, Point b) {
    int side1 = abs(a.getX() - b.getX());
    int side2 = abs(a.getY() - b.getY());
    return sqrt(side1 * side1 + side2 * side2);
  }

  void ComputeKey(Point &s) {
    s.setKey1(minimum(map[s.getX()][s.getY()].getG(),
                      map[s.getX()][s.getY()].getRHS()) +
              heuristics(s, start) + km);
    s.setKey2(minimum(map[s.getX()][s.getY()].getG(),
                      map[s.getX()][s.getY()].getRHS()));
  }

  bool in_Q(Point s) {
    bool Qbutton = false;
    if (Q.empty())
      return Qbutton;
    vector<Point> temp;
    while (!Q.empty()) {
      Point q = Q.top();
      temp.push_back(q);
      Q.pop();
      if (q.getX() == s.getX() && q.getY() == s.getY()) {
        Qbutton = true;
      }
    }
    vector<Point>::iterator iter;
    for (iter = temp.begin(); iter != temp.end(); iter++)
      Q.push(*iter);
    return Qbutton;
  }

  bool is_obstacle(Point s) {
    int index = s.getX() + width * s.getY();
    if (current_grid.data[index] == 0)
      return false;
    else
      return true;
  }

  void recomputeQ() {

    if (Q.empty())
      return;
    vector<Point> temp;
    while (!Q.empty()) {
      Point q = Q.top();
      temp.push_back(q);
      Q.pop();
    }
    vector<Point>::iterator iter;
    for (iter = temp.begin(); iter != temp.end(); iter++) {
      ComputeKey(*iter);
      Q.push(*iter);
    }
    return;
  }

  void updateRHS(int s_x, int s_y) {
    if (s_x == goal.getX() && s_y == goal.getY()) {
      return;
    }
    float rhs = inf;
    float temp;
    vector<Point>::iterator iter;
    for (iter = successor_rule.begin(); iter != successor_rule.end(); iter++) {
      // map 범위를 벗어나면 skip
      int nextx = s_x + iter->getX();
      int nexty = s_y + iter->getY();
      if (nextx < 0 || nextx > width - 1 || nexty < 0 || nexty > height - 1 ||
          is_obstacle(Point(nextx, nexty))) {
        continue;
      }
      temp =
          map[nextx][nexty].getG() + cost(Point(s_x, s_y), Point(nextx, nexty));
      if (temp < rhs)
        rhs = temp;
    }
    map[s_x][s_y].setRHS(rhs);
    if (map[s_x][s_y].getG() != map[s_x][s_y].getRHS() &&
        !in_Q(Point(s_x, s_y))) {
      Point s(s_x, s_y);
      ComputeKey(s);
      Q.push(s);
    }
  }

  void updatePredecessors(int s_x, int s_y) {
    vector<Point>::iterator iter;
    for (iter = predecessor_rule.begin(); iter != predecessor_rule.end();
         iter++) {
      int nextx = s_x + iter->getX();
      int nexty = s_y + iter->getY();
      if (nextx < 0 || nextx > width - 1 || nexty < 0 || nexty > height - 1 ||
          is_obstacle(Point(nextx, nexty))) {
        continue;
      }
      updateRHS(nextx, nexty);
    }
  }

  void local_overconsistent(int s_x, int s_y) {
    map[s_x][s_y].setG(map[s_x][s_y].getRHS());
    updatePredecessors(s_x, s_y);
  }
  void local_underconsistent(int s_x, int s_y) {
    map[s_x][s_y].setG(inf);
    updatePredecessors(s_x, s_y);
    updateRHS(s_x, s_y);
  }

  void get_consistency(int s_x, int s_y) {
    if (map[s_x][s_y].getG() > map[s_x][s_y].getRHS())
      local_overconsistent(s_x, s_y);
    else if (map[s_x][s_y].getG() < map[s_x][s_y].getRHS())
      local_underconsistent(s_x, s_y);
  }

  void makePath() {
    // backtracking
    vector<Point>::iterator iter;
    int x = start.getX();
    int y = start.getY();
    int tempx = x;
    int tempy = y;
    float lowest_g;
    path.push_back(start);
    path_size = 1;
    Point last_direction_memory = Point(0, 1);

    while (!(x == goal.getX() && y == goal.getY())) {
      tempx = x + last_direction_memory.getX();
      tempy = y + last_direction_memory.getY();
      lowest_g = map[tempx][tempy].getG();
      for (iter = successor_rule.begin(); iter != successor_rule.end();
           iter++) {
        if (x + iter->getX() < 0 || x + iter->getX() > width - 1 ||
            y + iter->getY() < 0 || y + iter->getY() > height - 1)
          continue;
        if (map[x + iter->getX()][y + iter->getY()].getG() < lowest_g) {
          tempx = x + iter->getX();
          tempy = y + iter->getY();
          lowest_g = map[tempx][tempy].getG();
          last_direction_memory = *iter;
        }
      }
      x = tempx;
      y = tempy;
      path.push_back(Point(x, y));
      path_size++;
    }
  }

  void ComputePath() {
    while (ros::ok()) {
      Point s_p = Q.top();
      Q.pop();
      get_consistency(s_p.getX(), s_p.getY());
      if (s_p.getX() == start.getX() && s_p.getY() == start.getY()) {
        initial_path_planned = true;
        break;
      }
    }
  }

  void clear_path(int size) {
    for (int i = 0; i < size; i++)
      path.pop_back();
  }

  bool in_path(int x, int y) {
    vector<Point>::iterator iter;
    for (iter = path.begin(); iter != path.end(); iter++) {
      if (iter->getX() == x && iter->getY() == y)
        return true;
    }
    return false;
  }

  void publishPath() {
    nav_msgs::Path mission_path;
    mission_path.header = current_grid.header;
    mission_path.header.frame_id = "odom";
    vector<Point>::iterator iter;
    for (iter = path.begin(); iter != path.end(); iter++) {
      geometry_msgs::PoseStamped temp_pose;
      temp_pose.pose.position.x =
          (iter->getX() * current_grid.info.resolution) + origin.position.x;
      temp_pose.pose.position.y =
          (iter->getY() * current_grid.info.resolution) + origin.position.y;
      mission_path.poses.push_back(temp_pose);
    }
    cout << "publishing path..." << endl;
    mission_path_pub.publish(mission_path);
  }

  //   bool repath() {
  //     bool repath = map[start.getX()][start.getY()].getG() !=
  //                   map[start.getX()][start.getY()].getRHS();

  //     return repath;
  //   }

  void run() {
    if (!grid_button || !goal_button || !odom_button)
      cout << "waiting for inital map..." << endl;
    else {
      if (!initial_path_planned)
        ComputePath();
      if (map[start.getX()][start.getY()].getG() !=
          map[start.getX()][start.getY()].getRHS()) {
        cout << "found new obstacle, recomputing path..." << endl;
        km += 1;
        recomputeQ();
        ComputePath();
      }
      makePath();
      publishPath();
      clear_path(path_size);
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_mission_node");

  // width와 height parameter space에서 받아와
  int width = 576;
  int height = 672;

  while (true) {
    DStarLite *mission = new DStarLite(width, height);
    ros::Rate loop_rate(20);
    while (ros::ok()) {
      ros::spinOnce();
      if (mission->restart() == true) {
        delete mission;
        break;
      }
      mission->run();
      loop_rate.sleep();
    }
  }
}