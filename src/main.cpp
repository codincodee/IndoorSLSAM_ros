#include <IndoorSLSAM/slsam.h>
#include <ros/ros.h>
#include <IndoorSLSAM_ros/sensor_data.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>

using namespace std;
using namespace slsam_ros;

Scan2D gScan;

bool AddScan(slsam::Slsam& slsam, const Scan2D& scan) {
  static unsigned last_seq = 0;
  if (gScan.scan) {
    if (gScan.scan->header.seq != last_seq) {
      slsam.AddScan(scan.ToSlsamScan());
      last_seq = gScan.scan->header.seq;
      return true;
    }
  }
  return false;
}

void LaserScanCB(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (!gScan.scan) {
    gScan.scan.reset(new sensor_msgs::LaserScan);
  }
  *gScan.scan = std::move(*msg);
}

bool GetMap(slsam::Slsam& slsam, nav_msgs::OccupancyGrid& map) {
  auto slsam_map = slsam.GenerateMap();
  if (!slsam_map) {
    return false;
  }
  map.header.stamp = ros::Time::now();
  map.header.frame_id = "map";

  auto width = slsam_map->iw();
  auto height = slsam_map->ih();
  cout << width << " " << height << endl;
  
  map.info.resolution = 0.5;
  map.info.width = width;
  map.info.height = height;
  map.data.reserve(width * height);
  for (int h = 0; h < height; ++h) {
    for (int w = 0; w < width; ++w) {
      map.data.push_back(slsam_map->map(w, h) * 100);
    }
  }
  return true;
}

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "IndoorSLSAM");
  ros::NodeHandle nh;
  ros::Subscriber scan_sub = 
      nh.subscribe<sensor_msgs::LaserScan>("laserscan", 1, LaserScanCB);
  ros::Publisher map_pub = 
      nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  slsam::Slsam slsam;
  while (ros::ok()) {
    ros::spinOnce();
    if (!AddScan(slsam, gScan)) {
      continue;
    }
    nav_msgs::OccupancyGrid map;
    GetMap(slsam, map);
    map_pub.publish(map);
  }
}
