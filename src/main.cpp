#include <IndoorSLSAM/slsam.h>
#include <ros/ros.h>
#include <IndoorSLSAM_ros/sensor_data.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace slsam_ros;

Scan2D gScan;
Odom gOdom;

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

bool AddOdom(slsam::Slsam& slsam, const Odom& odom) {
  static unsigned last_seq = 0;
  if (gOdom.odom) {
    if (gOdom.odom->header.seq != last_seq) {
      slsam.AddOdom(odom.ToSlsamOdom());
      last_seq = gOdom.odom->header.seq;
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

void OdomCB(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!gOdom.odom) {
    gOdom.odom.reset(new nav_msgs::Odometry);
  }
  *gOdom.odom = std::move(*msg);
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
  map.info.width = width;
  map.info.height = height;
  for (int h = 0; h < height; ++h) {
    for (int w = 0; w < width; ++w) {
      map.data.push_back(slsam_map->map(w, h) * 100);
    }
  }
  map.info.origin.position.x = slsam_map->origin.x;
  map.info.origin.position.y = slsam_map->origin.y;
  map.info.origin.position.z = 0.0;
  return true;
}

bool PublishTransform(
    slsam::Slsam& slsam, 
    tf::TransformListener& listener,
    tf::TransformBroadcaster& broadcaster) {
  slsam::Translation2 translation;
  slsam::Rotation2 rotation;
  if (!slsam.GetPose(translation, rotation)) {
    return false;
  }
  tf::StampedTransform odom_to_base_link;
  try {
    listener.lookupTransform(
        "odom", "base_link", ros::Time(0), odom_to_base_link);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    return false;
  }
  tf::Transform trans;
  trans.setOrigin(
      tf::Vector3(
          translation.x() - odom_to_base_link.getOrigin().x(),
          translation.y() - odom_to_base_link.getOrigin().y(),
          0.0f));
  
  double r, p, y;
  tf::Matrix3x3 m(odom_to_base_link.getRotation());
  m.getRPY(r, p, y);
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, rotation.angle() - y);
  trans.setRotation(q);
  
  tf::Transform tff;
  tff.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion qq;
  qq.setRPY(0.0, 0.0, 0.0);
  tff.setRotation(qq);
  broadcaster.sendTransform(
    tf::StampedTransform(trans, ros::Time::now(), "map", "odom"));
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "IndoorSLSAM");
  ros::NodeHandle nh;
  ros::Subscriber scan_sub = 
      nh.subscribe<sensor_msgs::LaserScan>("laserscan", 1, LaserScanCB);
  ros::Subscriber odom_sub =
      nh.subscribe<nav_msgs::Odometry>("odom", 1, OdomCB);
  ros::Publisher map_pub = 
      nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  tf::TransformBroadcaster tf_br;
  tf::TransformListener tf_listener;
  slsam::Slsam slsam;
  slsam.Init();
  while (ros::ok()) {
    ros::spinOnce();
    AddOdom(slsam, gOdom);
    if (!AddScan(slsam, gScan)) {
      continue;
    }
    nav_msgs::OccupancyGrid map;
    GetMap(slsam, map);
    map_pub.publish(map);
    PublishTransform(slsam, tf_listener, tf_br);
  }
}
