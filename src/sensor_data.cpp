#include <IndoorSLSAM_ros/sensor_data.h>
#include <cmath>

using namespace std;

namespace slsam_ros {
Scan2D::Scan2D() {}
Scan2D::Scan2D( shared_ptr < sensor_msgs :: LaserScan > msg)
  : scan(msg) {}
std::shared_ptr<slsam::Scan2D> Scan2D::ToSlsamScan() const {
  if (!scan) {
    return shared_ptr<slsam::Scan2D>();
  }
  std::shared_ptr<slsam::Scan2D> slsam_scan(new slsam::Scan2D);
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
} // namespace slsam_ros
