#include <IndoorSLSAM/slsam.h>
#include <ros/ros.h>
#include <IndoorSLSAM_ros/sensor_data.h>

using namespace slsam_ros;

Scan2D gScan;

void AddScan(slsam::Slsam& slsam, const Scan2D& scan) {
  static unsigned last_seq = 0;
  if (gScan.scan) {
    if (gScan.scan->header.seq != last_seq) {
      slsam.AddScan(scan.ToSlsamScan());
      last_seq = gScan.scan->header.seq;
    }
  }
}

void LaserScanCB(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (!gScan.scan) {
    gScan.scan.reset(new sensor_msgs::LaserScan);
  }
  *gScan.scan = std::move(*msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "IndoorSLSAM");
  ros::NodeHandle nh;
  ros::Subscriber scan_sub = 
      nh.subscribe<sensor_msgs::LaserScan>(
          "laserscan", 
          1, 
          LaserScanCB);
  slsam::Slsam slsam;
  while (ros::ok()) {
    ros::spinOnce();
    AddScan(slsam, gScan);
  }
}
