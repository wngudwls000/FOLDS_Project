#include "ros/service_client.h"
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>

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

class MapSetter {
private:
  ros::NodeHandle nh;
  ros::ServiceClient og_grid_client;
  ros::Subscriber laser_scan_sub;
  ros::Publisher map_pub;
  tf::TransformListener listener;

  nav_msgs::OccupancyGrid occupancy_grid;
  nav_msgs::OccupancyGrid new_occupancy_grid;
  geometry_msgs::Pose origin;
  int safety_index;
  int width;
  int height;

public:
  MapSetter(int safety_index) {
    og_grid_client = nh.serviceClient<nav_msgs::GetMap>("/static_map");
    laser_scan_sub =
        nh.subscribe("kobuki/laser/scan", 1, &MapSetter::scanCallback, this);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/mission_grid", 1000);

    this->safety_index = safety_index;

    nav_msgs::GetMap srv;
    ROS_INFO("Waiting for map..");
    if (og_grid_client.call(srv)) {
      ROS_INFO("Grid is here!\n");
      occupancy_grid = srv.response.map;
      width = occupancy_grid.info.width;
      height = occupancy_grid.info.height;
      origin = occupancy_grid.info.origin;
    } else {
      ROS_ERROR("Failed to call service /GetMap");
    }
  }

  int grid2array(int x, int y) { return x + (width * y); }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
    if (!listener.waitForTransform(
            scan->header.frame_id, "/odom",
            scan->header.stamp + ros::Duration().fromSec(scan->ranges.size() *
                                                         scan->time_increment),
            ros::Duration(1.0))) {
      return;
    }

    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector;
    // projector.projectLaser(*scan, cloud);
    projector.transformLaserScanToPointCloud("/odom", *scan, cloud, listener);

    // 항상 리프레쉬 됨.
    new_occupancy_grid = occupancy_grid;

    for (int k = 0; k < cloud.points.size(); k++) {
      geometry_msgs::Point32 point = cloud.points[k];
      int tempx = rounding((point.x - origin.position.x) /
                           occupancy_grid.info.resolution);
      int tempy = rounding((point.y - origin.position.y) /
                           occupancy_grid.info.resolution);

      for (int i = tempx - safety_index; i <= tempx + safety_index; i++) {
        for (int j = tempy - safety_index; j <= tempy + safety_index; j++) {
          if (i < 0 || i > width - 1 || j < 0 || j > height - 1) {
            continue;
          }
          new_occupancy_grid.data[grid2array(i, j)] = 100;
        }
      }
    }
  }

  void publish_grid() { map_pub.publish(new_occupancy_grid); }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "mission_grid_setter");

  MapSetter current_grid(5);
  ros::Rate loop_rate(20);

  ROS_INFO("publishing current grid..");
  while (ros::ok()) {
    ros::spinOnce();
    current_grid.publish_grid();
    loop_rate.sleep();
  }
  return 0;
}