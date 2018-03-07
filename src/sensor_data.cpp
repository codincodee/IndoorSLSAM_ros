#include <IndoorSLSAM_ros/sensor_data.h>
#include <cmath>
#include <tf/transform_datatypes.h>

using namespace std;

namespace slsam_ros {
// Scan2D
Scan2D::Scan2D() {}

Scan2D::Scan2D( shared_ptr < sensor_msgs :: LaserScan > msg)
  : scan(msg) {}

std::shared_ptr<slsam::Scan2D> Scan2D::ToSlsamScan() const {
  if (!scan) {
    return shared_ptr<slsam::Scan2D>();
  }
  std::shared_ptr<slsam::Scan2D> slsam_scan(new slsam::Scan2D);
  slsam_scan->stamp = 
      slsam::time_now() - 
      slsam::NSec((ros::Time::now() - scan->header.stamp).toNSec());
  slsam_scan->angle_min = scan->angle_min;
  slsam_scan->angle_max = scan->angle_max;
  slsam_scan->angle_increment = scan->angle_increment;
  slsam_scan->time_increment = scan->time_increment;
  slsam_scan->scan_time = scan->scan_time;
  slsam_scan->range_min = scan->range_min;
  slsam_scan->range_max = scan->range_max;
  slsam_scan->ranges = scan->ranges;
  slsam_scan->intensities = scan->intensities;
  return slsam_scan;
}

// Odom
Odom::Odom() {}

Odom::Odom( shared_ptr < nav_msgs :: Odometry > msg) : odom(msg) {}

std::shared_ptr<slsam::Odom2D> Odom::ToSlsamOdom() const {
  if (!odom) {
    return shared_ptr<slsam::Odom2D>();
  }
  shared_ptr<slsam::Odom2D> slsam_odom(new slsam::Odom2D);
  slsam_odom->stamp = 
      slsam::time_now() - 
      slsam::NSec((ros::Time::now() - odom->header.stamp).toNSec());
  slsam_odom->position = 
      slsam::Translation2{
          (float)odom->pose.pose.position.x, 
          (float)odom->pose.pose.position.y};
  slsam_odom->heading = 
        slsam::Rotation2(tf::getYaw(odom->pose.pose.orientation));
  return slsam_odom;
}
} // namespace slsam_ros
