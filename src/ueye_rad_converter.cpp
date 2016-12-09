/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Ye Cheng
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <dynamic_reconfigure/server.h>
#include <ueye_rad_converter/UeyeRadConfig.h>


namespace enc = sensor_msgs::image_encodings;
using namespace std;

image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_color;
image_transport::Publisher image_pub_mono;

int min_value_= 13200;//0;
int max_value_ = 13800;//65355;
bool auto_range_ = true;
int color_map_ = 2;
int num_bins = 1000;
bool hist_equal = false;

float minBound;
float maxBound;

string raw_topic;
string mono_topic;
string color_topic;

cv::MatND imHist;

//color mappings
enum
{
  COLORMAP_AUTUMN = 0,
   COLORMAP_BONE = 1,
   COLORMAP_JET = 2,
   COLORMAP_WINTER = 3,
   COLORMAP_RAINBOW = 4,
   COLORMAP_OCEAN = 5,
   COLORMAP_SUMMER = 6,
   COLORMAP_SPRING = 7,
   COLORMAP_COOL = 8,
   COLORMAP_HSV = 9,
   COLORMAP_PINK = 10,
   COLORMAP_HOT = 11
};

void configCb(ueye_rad_converter::UeyeRadConfig &newconfig, uint32_t level)
{
  if (newconfig.auto_range)
    {
      //update reconfig box with current values of max and min
      newconfig.min_value = min_value_;
      newconfig.max_value = max_value_;
    }
  else
    {
      min_value_ = newconfig.min_value;
      max_value_ = newconfig.max_value;
    }
  if (newconfig.auto_range != auto_range_)
    auto_range_ = newconfig.auto_range;
  if (newconfig.color_map != color_map_)
    color_map_ = newconfig.color_map;
  if (newconfig.hist_equal != color_map_)
    hist_equal = newconfig.hist_equal;
  if (newconfig.num_bins != num_bins)
    num_bins = newconfig.num_bins;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat gray_img_(cv::Size(msg->width, msg->height), CV_8U);
  cv_bridge::CvImagePtr cv_ptr_;
  try {
    cv_ptr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  gray_img_ = cv_ptr_->image;
  
  //mono image
  cv_ptr_->image = gray_img_;
  cv_ptr_->encoding = "mono8";

  sensor_msgs::ImagePtr output_img =cv_ptr_->toImageMsg();
  output_img->header=msg->header;
  output_img->encoding = "mono8";
  image_pub_mono.publish(output_img);

  //color image
  cv_ptr_->encoding = "rgb8";
  cv::applyColorMap(cv_ptr_->image, cv_ptr_->image, color_map_);

  // convert openCV's BGR to RGB for proper visualization in RVIZ
  cv::cvtColor(cv_ptr_->image, cv_ptr_->image, cv::COLOR_BGR2RGB);
  sensor_msgs::ImagePtr output_img_color =cv_ptr_->toImageMsg();
  output_img_color->header=msg->header;
  output_img_color->encoding = "rgb8";
  image_pub_color.publish(output_img_color);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ueye_rad_converter");
  ros::NodeHandle nh("~");

  dynamic_reconfigure::Server<ueye_rad_converter::UeyeRadConfig> server;
  dynamic_reconfigure::Server<ueye_rad_converter::UeyeRadConfig>::CallbackType f;

  f = boost::bind(&configCb, _1, _2);
  server.setCallback(f);

  ros::param::get("~max_val", max_value_);
  ros::param::get("~min_val", min_value_);
  ros::param::get("~auto_range", auto_range_);
  ros::param::get("~color_map", color_map_);

  ros::param::get("~raw_topic", raw_topic);
  ros::param::get("~mono_topic", mono_topic);
  ros::param::get("~color_topic", color_topic);
  ros::param::get("~num_bins", num_bins);
  ros::param::get("~hist_equal", hist_equal);

  image_transport::ImageTransport it_(nh);

  image_pub_color = it_.advertise(color_topic, 1);
  image_pub_mono = it_.advertise(mono_topic, 1);

  image_sub_ = it_.subscribe(raw_topic, 1, &imageCb);


  ros::spin();
  return 0;
}
