/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
*
* Modifications: Jeremie Deray
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2D.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// #include <visualization_msgs/Marker.h>


namespace rf2o {

class CLaserOdometry2DNode : CLaserOdometry2D
{
public:
  CLaserOdometry2DNode();
  ~CLaserOdometry2DNode() = default;

  // void process(const ros::TimerEvent &);
  void process();

  void publish();

  bool setLaserPoseFromTf();

public:
  bool publish_tf, new_scan_available;

  // double freq;

  std::string  laser_scan_topic;
  std::string  odom_topic;
  std::string  path_topic;
  std::string  pose_topic;

  std::string  base_frame_id;
  std::string  odom_frame_id;
  std::string  init_pose_from_topic;

  ros::NodeHandle           n;
  sensor_msgs::LaserScan    last_scan;
  bool                      GT_pose_initialized;
  tf::TransformListener     tf_listener;          //Do not put inside the callback
  tf::TransformBroadcaster  odom_broadcaster;
  nav_msgs::Odometry        initial_robot_pose;

  ros::Subscriber laser_sub;
  ros::Subscriber initPose_sub;

  ros::Publisher odom_pub;
  // ros::Publisher  marker_pub;  //
  ros::Publisher path_pub;
  ros::Publisher pose_pub;


  bool scan_available();

  //Callbacks
  void LaserCallback(const sensor_msgs::LaserScan::ConstPtr & new_scan);
  void initPoseCallback(const nav_msgs::Odometry::ConstPtr & new_initPose);

  // visualization_msgs::Marker marker;  //
  nav_msgs::Path path;

};


CLaserOdometry2DNode::CLaserOdometry2DNode() : CLaserOdometry2D()
{
  ROS_INFO("Initializing RF2O node...");

  //Read Parameters
  //----------------
  ros::NodeHandle pn("~");
  pn.param<std::string>("laser_scan_topic",laser_scan_topic,"/laser_scan");
  pn.param<std::string>("odom_topic", odom_topic, "/odom_rf2o");
  pn.param<std::string>("path_topic", path_topic, "/path_rf2o");
  pn.param<std::string>("pose_topic", pose_topic, "/pose_rf2o");

  pn.param<std::string>("base_frame_id", base_frame_id, "base_link");
  pn.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  pn.param<bool>("publish_tf", publish_tf, true);
  pn.param<std::string>("init_pose_from_topic", init_pose_from_topic, "/base_pose_ground_truth");
  // pn.param<double>("freq", freq, 10.0);
  pn.param<bool>("verbose", verbose, true);


  //Publishers and Subscribers
  //--------------------------
  odom_pub = pn.advertise<nav_msgs::Odometry>(odom_topic, 1);
  // marker_pub = pn.advertise<visualization_msgs::Marker>("visualization_marker", 5);  //
  path_pub = pn.advertise<nav_msgs::Path>(path_topic, 1);
  pose_pub = pn.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);
  
  laser_sub  = n.subscribe<sensor_msgs::LaserScan>(laser_scan_topic, 1, &CLaserOdometry2DNode::LaserCallback, this);


  //init pose?
  if (init_pose_from_topic != "")
  {
    ROS_INFO("-- init_pose_from_topic != null --");
    initPose_sub = n.subscribe<nav_msgs::Odometry>(init_pose_from_topic, 1, &CLaserOdometry2DNode::initPoseCallback, this);
    GT_pose_initialized  = false;
  }
  else  // init_pose_from_topic == ""
  {
    ROS_INFO("-- init_pose_from_topic == null --");
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;

    // initial_robot_pose.pose.pose.orientation.w = 0;
    initial_robot_pose.pose.pose.orientation.w = 1;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;

    GT_pose_initialized = true;
  }

  setLaserPoseFromTf();  // identity

  //Init variables
  module_initialized = false;
  first_laser_scan   = true;
  new_scan_available = false; //*************

  ROS_INFO_STREAM("Listening laser scan from topic: " << laser_sub.getTopic());


  // marker.header.frame_id = odom_frame_id;  // "map";
  // // marker.header.stamp = ros::Time();
  // // marker.ns = "my_namespace";
  // // marker.id = 0;
  // marker.type = visualization_msgs::Marker::POINTS;
  // marker.scale.x = 0.01;
  // marker.scale.y = 0.01;
  // marker.color.g = 1.0;
  // marker.color.a = 1.0;

}


bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  bool retrieved = false;

  // Set laser pose on the robot (through tF)
  // This allow estimation of the odometry with respect to the robot base reference system.

  tf::StampedTransform transform;
  transform.setIdentity();

  try
  {
    // lookupTransform: /base_link, /laser
    tf_listener.lookupTransform(base_frame_id, last_scan.header.frame_id, ros::Time(0), transform);
    retrieved = true;
  }
  catch (tf::TransformException &ex)
  {
    // ROS_INFO("-- tf::TransformException --");
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    retrieved = false;
  }

  //TF:transform -> Eigen::Isometry3d(Pose3d)
  const tf::Matrix3x3 &basis = transform.getBasis();
  
  Eigen::Matrix3d R;
  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  setLaserPose(laser_tf);

  return retrieved;
}


bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}


void CLaserOdometry2DNode::process()
{
  // std::cout << "is_initialized() : " << is_initialized() << std::endl;  // 1
  // std::cout << "scan_available() : " << scan_available() << std::endl;  // 255

  if (is_initialized() && scan_available())
  {
    odometryCalculation(last_scan);  // process odometry estimation
    publish();
    new_scan_available = false;  // avoid the possibility to run twice on the same laser scan

    // ROS_INFO("---------------------------");
  }
  else
  {
    // ROS_WARN("Waiting for laser_scans....");
  }
}


//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2DNode::LaserCallback(const sensor_msgs::LaserScan::ConstPtr & new_scan)
{
  // store new scan
  if (GT_pose_initialized)
  {
    //Keep in memory the last received laser_scan
    last_scan = *new_scan;
    current_scan_time = last_scan.header.stamp;

    if (!first_laser_scan)
    {
      //copy laser scan to internal variable
      for (unsigned int i = 0; i < width; i++)
      {
        range_wf(i) = new_scan->ranges[i];
      }
      new_scan_available = true;
    }
    else  //Initialize module on first scan
    {
      init(last_scan, initial_robot_pose.pose.pose);
      first_laser_scan = false;
    }
  }

  // process new scan
  process();
}


void CLaserOdometry2DNode::initPoseCallback(const nav_msgs::Odometry::ConstPtr & new_initPose)
{
  //Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}


void CLaserOdometry2DNode::publish()
{
  //---------------------------------------
  //first, we'll publish the odometry over tf
  if (publish_tf)
  {
    //ROS_INFO("[rf2o] Publishing TF: [base_link] to [odom]");
    geometry_msgs::TransformStamped odom_trans;

    // odom_trans.header.stamp = ros::Time::now();  // Message removed because it is too old (frame=[os_sensor], stamp=[1678432141.149685248])
    odom_trans.header.stamp = current_scan_time;

    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;

    odom_trans.transform.translation.x = robot_pose_.translation()(0);
    odom_trans.transform.translation.y = robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(rf2o::getYaw(robot_pose_.rotation()));
    
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
  }


  //-------------------------------------------------
  //next, we'll publish the [odometry] message over ROS
  ROS_INFO_COND(verbose, "[rf2o] Publishing Odom Topic");
  nav_msgs::Odometry odom;
  // odom.header.stamp = ros::Time::now();
  odom.header.stamp = current_scan_time;
  odom.header.frame_id = odom_frame_id;
  odom.child_frame_id = base_frame_id;

  //set the position
  odom.pose.pose.position.x = robot_pose_.translation()(0);
  odom.pose.pose.position.y = robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(rf2o::getYaw(robot_pose_.rotation()));

  //set the velocity
  odom.twist.twist.linear.x = lin_speed;    //linear speed
  odom.twist.twist.angular.z = ang_speed;   //angular speed
  ROS_INFO_STREAM_COND(verbose, "lin_speed: " << lin_speed);
  ROS_INFO_STREAM_COND(verbose, "ang_speed: " << ang_speed);

  // std::cout << "[RF2O] [x y yaw]: " << robot_pose_.translation()(0)
  //                              << " " << robot_pose_.translation()(1)
  //                              << " " << rf2o::getYaw(robot_pose_.rotation())
  //                              << std::endl;

  ROS_INFO_STREAM(
    "[RF2O] [x y yaw]: " 
    << robot_pose_.translation()(0)
    << " " << robot_pose_.translation()(1)
    << " " << rf2o::getYaw(robot_pose_.rotation())
  );


  //publish the message
  odom_pub.publish(odom);

  //------------------------------------------------- 
  //next, we'll publish the [pose] message over ROS
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = odom.header;
  pose_stamped.pose = odom.pose.pose;
  pose_pub.publish(pose_stamped);

  //next, we'll publish the [path] message over ROS
  path.header = odom.header;
  path.poses.push_back(pose_stamped);
  path_pub.publish(path);



  // ----------------------------------------------
  // scan visualization marker
  // ----------------------------------------------
  // transform last_scan to current robot local coordinate
  // float x_robot   = robot_pose_.translation()(0);
  // float y_robot   = robot_pose_.translation()(1);
  // float yaw_robot = rf2o::getYaw(robot_pose_.rotation());

  // // sensor_msgs::LaserScan transformed_scan = last_scan;
  // float angle_min       = last_scan.angle_min;
  // float angle_increment = last_scan.angle_increment;

  // for (int i = 0; i < last_scan.ranges.size(); i++)
  // {
  //   if (!std::isfinite(last_scan.ranges[i]))  // infinite
  //   {
  //     continue;
  //   }
  //   float angle, range, x_local, y_local;
  //   angle = angle_min + i * angle_increment;
  //   range = last_scan.ranges[i];
  //   x_local = range * cos(angle);
  //   y_local = range * sin(angle);

  //   float x_global, y_global;
  //   x_global = x_robot + x_local * cos(yaw_robot) - y_local * sin(yaw_robot);
  //   y_global = y_robot + x_local * sin(yaw_robot) + y_local * cos(yaw_robot);

  //   // ---------------------------------
  //   geometry_msgs::Point point;
  //   point.x = x_global;
  //   point.y = y_global;
  //   // point.x = -y_global;  // x_global;
  //   // point.y = x_global;  // y_global;
  //   point.z = 0.0;
  //   // ---------------------------------
  //   marker.points.emplace_back(point);
  // }

  // marker_pub.publish(marker);
  
}

} /* namespace rf2o */


//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  ros::init(argc, argv, "RF2O_LaserOdom");

  rf2o::CLaserOdometry2DNode myLaserOdomNode;

  ros::spin();

  return EXIT_SUCCESS;
}
