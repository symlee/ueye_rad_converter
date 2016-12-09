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


image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_color;
image_transport::Publisher image_pub_mono;

int min_value_= 13200;//0;
int max_value_ = 13800;//65355;
bool auto_range_ = true;
int color_map_ = 12;
int num_bins = 1000;
bool hist_equal = false;

float minBound;
float maxBound;

std::string raw_topic;
std::string mono_topic;
std::string color_topic;

cv::MatND imHist;


//color mappings
//enum
//{
//   COLORMAP_AUTUMN = 0,
//    COLORMAP_BONE = 1,
//    COLORMAP_JET = 2,
//    COLORMAP_WINTER = 3,
//    COLORMAP_RAINBOW = 4,
//    COLORMAP_OCEAN = 5,
//    COLORMAP_SUMMER = 6,
//    COLORMAP_SPRING = 7,
//    COLORMAP_COOL = 8,
//    COLORMAP_HSV = 9,
//    COLORMAP_PINK = 10,
//    COLORMAP_HOT = 11
//}

void applyIronColorMap(cv::Mat& im_gray, cv::Mat& im_color)
{
  unsigned char b[] = {0,16,33,42,49,56,63,70,77,83,87,91,95,99,103,106,110,115,116,118,120,122,124,127,129,131,133,135,137,138,140,141,143,144,146,147,148,149,149,150,150,151,151,152,152,153,154,155,155,155,155,156,156,156,157,157,157,157,157,157,157,157,156,156,155,155,155,155,155,155,155,154,154,153,153,152,152,151,151,150,149,149,149,148,147,147,146,146,145,144,143,142,141,139,138,136,134,133,131,129,126,123,121,118,116,113,111,108,104,101,98,95,91,87,81,76,70,65,59,54,48,42,37,31,28,25,23,21,19,17,15,13,11,10,9,8,8,7,7,6,5,4,4,3,3,3,3,2,2,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,2,3,4,6,8,10,11,12,14,16,20,24,28,32,36,39,44,50,56,62,67,73,79,85,92,98,105,111,119,127,135,142,149,156,164,171,178,184,190,195,201,206,212,218,224,229,235,240,244,249,252,255};
  unsigned char g[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,3,4,4,5,5,5,6,7,9,10,11,12,13,15,17,18,20,21,23,24,26,27,28,30,32,34,36,38,40,42,44,46,47,49,51,53,54,56,57,59,61,63,65,67,69,71,72,74,76,77,78,80,82,84,86,87,89,91,92,94,95,97,99,100,102,103,104,106,107,109,111,113,114,115,117,119,121,124,126,128,129,131,133,134,136,137,139,140,142,143,144,146,148,150,153,155,157,159,161,163,166,168,170,172,174,176,177,178,181,183,185,186,188,190,191,193,195,197,199,200,202,203,205,206,207,209,211,213,215,216,218,219,220,221,222,224,225,227,228,229,230,231,233,234,236,237,238,238,239,240,241,241,242,243,244,244,245,245,246,247,247,248,248,249,250,251,252,253,253,254,254,255,255,255};
  unsigned char r[] = {0,0,0,0,0,0,0,0,0,0,1,2,3,4,5,7,9,11,12,13,16,19,22,25,28,31,34,38,42,45,48,52,55,58,61,63,65,68,71,74,76,79,82,85,88,92,94,97,101,104,107,110,112,114,117,121,124,126,129,132,135,137,140,143,146,149,152,154,157,159,161,164,166,168,170,172,174,175,177,178,180,182,183,185,186,188,189,190,191,192,193,194,195,197,198,200,201,202,203,204,206,207,208,208,209,210,211,212,213,214,216,217,218,219,220,221,222,223,223,224,224,225,226,227,228,228,229,230,231,231,232,233,234,234,235,235,236,236,236,237,237,238,238,239,239,240,240,241,241,241,241,242,242,243,243,243,244,244,244,244,245,245,246,246,247,247,248,248,248,248,249,249,249,249,249,250,250,251,251,252,252,253,253,253,253,253,253,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255};
  cv::Mat channels[] = {cv::Mat(256,1, CV_8U, r), cv::Mat(256,1, CV_8U, g), cv::Mat(256,1, CV_8U, b)};
  cv::Mat lut; // Create a lookup table
  merge(channels, 3, lut);

  cv::cvtColor(im_gray.clone(), im_gray, cv::COLOR_GRAY2BGR);

  cv::LUT(im_gray, lut, im_color);

}

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
        cv_ptr_ = cv_bridge::toCvCopy(msg, enc::MONO16);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (auto_range_) //find min and max values in image
    {
        cv::Mat img = cv_ptr_->image.clone();

        if (img.type() != CV_32F) {
            img.convertTo(img,
                          CV_32F);//, (255.0f/65535.0f));//convert to float between 0 and 255 to not break all uint8 based float aspects of program
        }

        // int histSize = 1000;    // bin size
        float range[] = {0, 65536};
        const float *ranges[] = {range};
        bool uniform = true;
        bool accumulate = false;
        cv::calcHist(&img, 1, 0, cv::Mat(), imHist, 1, &num_bins, ranges, uniform, accumulate);
        float maxHist = 0;
        int binNum = -1;
        for (int i = 0; i < num_bins; i++) {
            float histVal = imHist.at<float>(i);
            if (histVal > maxHist) {
                maxHist = histVal;
                binNum = i;
            }
        }
        //printf("bin: %d\n", binNum);
        //printf("maxHist: %f\n", maxHist);
        float binSize = (range[1] - range[0]) / (float) num_bins;
        //printf("binSize: %f\n", binSize);
        float meanBin = binSize * binNum;
        minBound = meanBin - 5 * binSize;
        maxBound = meanBin + 5 * binSize;
        //printf("maxBound: %f\n", maxBound);
        //printf("minBound: %f\n", minBound);
	min_value_=minBound;
	max_value_=maxBound;
    }
    else {
        minBound = min_value_;
        maxBound = max_value_;
    }
        //scale image between 0 and 255 and convert to mono 8-bit
        cv_ptr_->image.convertTo(gray_img_, CV_8UC1, 255.0 / (maxBound - minBound),
                                 -minBound * 255.0 / (maxBound - minBound));
        if(hist_equal)
			cv::equalizeHist(gray_img_,gray_img_);

		cv::medianBlur(gray_img_,gray_img_,3);

        //mono image
        cv_ptr_->image = gray_img_;
        cv_ptr_->encoding = "mono8";

        sensor_msgs::ImagePtr output_img =cv_ptr_->toImageMsg();
        output_img->header=msg->header;
        output_img->encoding = "mono8";
        image_pub_mono.publish(output_img);

        //color image
        cv_ptr_->encoding = "rgb8";
        if (color_map_ < 12)
            cv::applyColorMap(cv_ptr_->image, cv_ptr_->image, color_map_);
        else
            applyIronColorMap(cv_ptr_->image, cv_ptr_->image);

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
