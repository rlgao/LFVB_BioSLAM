/*
 * openRatSLAM
 *
 * main_lv - ROS interface bindings for the local view cells
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "utils/utils.h"

#include <boost/property_tree/ini_parser.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <ratslam_ros/ViewTemplate.h>
#include <ros/console.h>

#include <image_transport/image_transport.h>

#include "ratslam/local_view_match.h"

#if HAVE_IRRLICHT
#include "graphics/local_view_scene.h"
ratslam::LocalViewScene *lvs = NULL;
bool use_graphics;
#endif


using namespace ratslam;
ratslam::LocalViewMatch *lv = NULL;


void image_callback(sensor_msgs::ImageConstPtr image, ros::Publisher * pub_vt)
{
  ROS_DEBUG_STREAM("LV:image_callback{" << ros::Time::now() << "} seq=" << image->header.seq);

  static ratslam_ros::ViewTemplate vt_output;


  bool greyscale;
  if (image->encoding == "bgr8" || image->encoding == "rgb8")
  {
    greyscale = false;
  }
  else
  {
    greyscale = true;
  }
  // ROS_INFO_STREAM("greyscale: " << greyscale);  // 0

  // lv->on_image(&image->data[0], (image->encoding == "bgr8" ? false : true), // bool greyscale
  //             image->width, image->height);
  lv->on_image(&image->data[0], greyscale,
              image->width, image->height);

  // ROS_INFO_STREAM("image->width: " << image->width);    // 1920
  // ROS_INFO_STREAM("image->height: " << image->height);  // 1200


//***************************************************************
//***************************************************************
  // ROS_INFO_STREAM("---------------");
  // ROS_INFO_STREAM("image->header.seq: " << image->header.seq);
  // ROS_INFO_STREAM("image->width: " << image->width);     // 416
  // ROS_INFO_STREAM("image->height: " << image->height);   // 240
  // ROS_INFO_STREAM("image->encoding: " << image->encoding);  // rgb8
  // ROS_INFO_STREAM("image->data.size(): " << image->data.size());  // 299520=416*240*3
//***************************************************************
//***************************************************************


  vt_output.header.stamp = ros::Time::now();
  vt_output.header.seq++;
  vt_output.current_id = lv->get_current_vt();      // lv->current_vt (int)
  vt_output.relative_rad = lv->get_relative_rad();  // lv->vt_relative_rad (double)

  pub_vt->publish(vt_output);


// #ifdef HAVE_IRRLICHT
//   // ROS_INFO("HAVE_IRRLICHT!!!");
//   if (use_graphics)
//   {
//     lvs->draw_all();
//   }
// #endif
}


int main(int argc, char * argv[])
{
  // ROS_INFO_STREAM(argv[0] << " - openRatSLAM Copyright (C) 2012 David Ball and Scott Heath");
  // ROS_INFO_STREAM("RatSLAM algorithm by Michael Milford and Gordon Wyeth");
  // ROS_INFO_STREAM("Distributed under the GNU GPL v3, see the included license file.");

  if (argc < 2)
  {
    ROS_FATAL_STREAM("USAGE: " << argv[0] << " <config_file>");
    exit(-1);
  }
  
  std::string topic_root = "";

  boost::property_tree::ptree settings, ratslam_settings, general_settings;
  read_ini(argv[1], settings);

  get_setting_child(general_settings, settings, "general", true);
  get_setting_from_ptree(topic_root, general_settings, "topic_root", (std::string)"");
  get_setting_child(ratslam_settings, settings, "ratslam", true);
  
  lv = new ratslam::LocalViewMatch(ratslam_settings);

  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "RatSLAMViewTemplate");
  }
  ros::NodeHandle node;

  ros::Publisher pub_vt = node.advertise<ratslam_ros::ViewTemplate>(topic_root + "/LocalView/Template", 0);


  image_transport::ImageTransport it(node);
  image_transport::Subscriber sub;
  /********************************************************/
  ROS_INFO_STREAM("topic_root: " << topic_root);

  if (topic_root == "/shihushan")
  {
    ROS_INFO_STREAM("topic_root == /shihushan");
    sub = it.subscribe(topic_root + "/rotated_image", 0, boost::bind(image_callback, _1, &pub_vt));
  }
  else
  {
    ROS_INFO_STREAM("topic_root != /shihushan");
    sub = it.subscribe(topic_root + "/camera/image", 0, boost::bind(image_callback, _1, &pub_vt));
  }
  /********************************************************/


// #ifdef HAVE_IRRLICHT
//     boost::property_tree::ptree draw_settings;
//     get_setting_child(draw_settings, settings, "draw", true);
//     get_setting_from_ptree(use_graphics, draw_settings, "enable", true);
//     if (use_graphics)
//       lvs = new ratslam::LocalViewScene(draw_settings, lv);
// #endif

  ros::spin();

  return 0;
}
