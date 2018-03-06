#include <IndoorSLSAM/slsam.h>
#include <ros/ros.h>
#include <IndoorSLSAM_ros/sensor_data.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

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
  
  map.info.resolution = slsam_map->resolution;

  // Handle the difference of map conventions between ros and IndoorSLSAM
  map.info.width = height;
  map.info.height = width;
  for (int w = 0; w < width; ++w) {
    for (int h = 0; h < height; ++h) {
      map.data.push_back(slsam_map->map(w, h) * 100);
    }
  }
  map.info.origin.position.x = slsam_map->origin.y;
  map.info.origin.position.y = slsam_map->origin.x;
  map.info.origin.position.z = 0.0;
  return true;
}

void PublishPose(slsam::Slsam& slsam, tf::TransformBroadcaster& broadcaster) {
  slsam::Translation2 translation;
  slsam::Rotation2 rotation;
  if (!slsam.GetPose(translation, rotation)) {
    return;
  }
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(translation.x(), translation.y(), 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, rotation.angle());
  transform.setRotation(q);
  broadcaster.sendTransform(
    tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "IndoorSLSAM");
  ros::NodeHandle nh;
  ros::Subscriber scan_sub = 
      nh.subscribe<sensor_msgs::LaserScan>("laserscan", 1, LaserScanCB);
  ros::Publisher map_pub = 
      nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  tf::TransformBroadcaster tf_br;
  slsam::Slsam slsam;
  slsam.Init();
  while (ros::ok()) {
    ros::spinOnce();
    if (!AddScan(slsam, gScan)) {
      continue;
    }
    nav_msgs::OccupancyGrid map;
    GetMap(slsam, map);
    map_pub.publish(map);
    PublishPose(slsam, tf_br);
  }
}
