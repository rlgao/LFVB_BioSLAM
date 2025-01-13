/************ main_lidar_to_scan ************/

#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <algorithm>

typedef pcl::PointXYZ PointType;

ros::Publisher mid_cloud_pub;
ros::Publisher mid_scan_pub;


void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& pc2_in)
{
  // ROS_INFO_STREAM("------");

  // ros msg PointCloud2 --> pcl pointcloud
  pcl::PointCloud<PointType>::Ptr pcl_cloud;
  pcl_cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*pc2_in, *pcl_cloud);
  // ROS_INFO_STREAM("scan amount: " << pcl_cloud->points.size());

  // get single scan in the middle
  pcl::PointCloud<PointType>::Ptr mid_cloud;
  mid_cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  const float wanted_z_low = -0.05, wanted_z_up = 0.05;
  for (const auto& point : pcl_cloud->points)
  {
    float dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (dist < 3.0)  // remove points human following the car
    {
      continue;
    }
    // wanted point
    if (point.z >= wanted_z_low && point.z <= wanted_z_up)
    {
      mid_cloud->points.push_back(point);
    }
  }
  // int mid_points_size = mid_cloud->points.size();
  // ROS_INFO_STREAM("mid points: " << mid_cloud->points.size());


  // pcl pointcloud --> ros msg PointCloud2
  sensor_msgs::PointCloud2 pc2_out;
  pcl::toROSMsg(*mid_cloud, pc2_out);
  pc2_out.header.stamp = pc2_in->header.stamp;
  pc2_out.header.frame_id = pc2_in->header.frame_id;
  mid_cloud_pub.publish(pc2_out);


  // pcl pointcloud --> ros msg LaserScan
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header = pc2_in->header;

  const float num_points = 1024;
  const float angle_min = -3.14;
  const float angle_max = 3.14;
  const float angle_increment = (angle_max - angle_min) / (num_points - 1);
  scan_msg.angle_min = angle_min;
  scan_msg.angle_max = angle_max;
  scan_msg.angle_increment = angle_increment;
  // scan_msg.time_increment = time_increment;
  // scan_msg.scan_time = scan_time;
  scan_msg.range_min = 0.0;
  scan_msg.range_max = std::numeric_limits<float>::max();

  scan_msg.ranges.resize(num_points);
  int num_valid_ranges = 0;
  for (int i = 0; i < num_points; i++)
  {
    scan_msg.ranges[i] = std::numeric_limits<double>::max();

    float angle = angle_min + i * angle_increment;

    for (const auto& point : mid_cloud->points)
    {
      float point_angle = std::atan2(point.y, point.x);
      float diff_angle = std::fabs(angle - point_angle);
      if (diff_angle <= angle_increment / 2.0)  // if (diff_angle < 0.01)
      {
        // std::cout << "diff_angle " << diff_angle << std::endl;
        float dist_xy = std::sqrt(point.x * point.x + point.y * point.y);
        // scan_msg.ranges.emplace_back(dist_xy);
        scan_msg.ranges[i] = dist_xy;
        num_valid_ranges++;
        break;
      }
    }
  }
  // ROS_INFO_STREAM("num_valid_ranges: " << num_valid_ranges);

  mid_scan_pub.publish(scan_msg);
}


int main(int argc, char * argv[])
{
  std::string topic_lidar = "/ouster/points";

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMLidarToScan");
  }
  ros::NodeHandle node;

  ros::Subscriber lidar_sub = node.subscribe(topic_lidar, 1, lidar_callback);

  mid_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("/mid_cloud", 1, true);
  
  mid_scan_pub = node.advertise<sensor_msgs::LaserScan>("/mid_scan", 1, true);
  // mid_scan_pub = node.advertise<sensor_msgs::LaserScan>("/base_scan", 1, true);  // gmapping

  ros::spin();

  return 0;
}
