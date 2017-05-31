/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
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
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
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
 */

/** 
  * @file depth_segmentation.cpp
  * @author Jordi Pages
  * @date July 2014
  * @brief Depth (expressed as sensor_msgs/Image) based segmentation node.
  */

// PAL headers
#include <pal_vision_segmentation/DepthSegmentConfig.h>

// ROS headers
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost headers
#include <boost/scoped_ptr.hpp>


// parameters
double threshold;
int dilate_iterations;
int dilate_size;
int erode_iterations;
int erode_size;

image_transport::Publisher mask_pub, img_masked_pub;
cv::Mat imgDepth;
cv::Mat mask;

boost::scoped_ptr< message_filters::Subscriber<sensor_msgs::Image> > imSub;
boost::scoped_ptr< message_filters::Subscriber<sensor_msgs::Image> > imDepthSub;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::Image> ApproxSync;

boost::scoped_ptr< message_filters::Synchronizer<ApproxSync> > synchronizer;

bool enabled;

void getImage(const sensor_msgs::ImageConstPtr& msg,
              const std::string& encoding,
              cv::Mat& img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
      cv_ptr = cv_bridge::toCvCopy(msg, encoding);
      img = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void publishResult(const sensor_msgs::ImageConstPtr& imgMsg)
{
  if ( mask_pub.getNumSubscribers() > 0 )
  {
    cv_bridge::CvImage mask_msg;
    mask_msg.header = imgMsg->header;
    mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
    mask_msg.image = mask;
    mask_pub.publish(mask_msg.toImageMsg());
  }

  if ( img_masked_pub.getNumSubscribers() > 0 )
  {
    cv::Mat img;

    getImage(imgMsg,
             sensor_msgs::image_encodings::BGR8,
             img);

    cv::Mat img_masked;
    img.copyTo(img_masked, mask);

    cv_bridge::CvImage img_masked_msg;
    img_masked_msg.header = imgMsg->header;
    img_masked_msg.encoding = sensor_msgs::image_encodings::BGR8;
    img_masked_msg.image = img_masked;
    img_masked_pub.publish(img_masked_msg.toImageMsg());
  }
}

void callback(const sensor_msgs::ImageConstPtr& imMsg,
              const sensor_msgs::ImageConstPtr& imDepthMsg)
{
  getImage(imDepthMsg,
           sensor_msgs::image_encodings::TYPE_32FC1,
           imgDepth);

  //threshold image
  mask = cv::Mat::zeros(imgDepth.size(), CV_8UC1);
  for (int r = 0; r < imgDepth.size().height; ++r)
    for (int c = 0; c < imgDepth.size().width; ++c)
    {
      float depth = imgDepth.at<float>(r, c);
      if ( !std::isnan(depth) && depth < threshold )
        mask.at<uchar>(r, c) = 255;
    }

  //erode and dilate operations
  cv::Mat tmp;
  cv::erode(mask, tmp,
            cv::Mat::ones(erode_size, erode_size, CV_8UC1),
            cv::Point(-1, -1), erode_iterations);

  cv::dilate(tmp, mask,
             cv::Mat::ones(dilate_size, dilate_size, CV_8UC1),
             cv::Point(-1, -1), dilate_iterations);

  publishResult(imMsg);
}

void reconf_callback(pal_vision_segmentation::DepthSegmentConfig &config, uint32_t level)
{
  threshold         = config.threshold;
  dilate_iterations = config.dilate_iterations;
  dilate_size       = config.dilate_size;
  erode_iterations  = config.erode_iterations;
  erode_size        = config.erode_size;
}

void getParameters(ros::NodeHandle& nh)
{
  nh.param<double>("threshold", threshold, 1.6); //in m
  nh.param<int>("dilate_iterations", dilate_iterations, 9);
  nh.param<int>("dilate_size", dilate_size, 7);
  nh.param<int>("erode_iterations", erode_iterations, 1);
  nh.param<int>("erode_size", erode_size, 3);
}

/**
 * @brief start start subscribers and topic synchronizer
 */
void start(ros::NodeHandle& nh)
{
  ROS_WARN("Enabling node because there are subscribers");
  // Define a subscriber to the rgb image
  imSub.reset( new
               message_filters::Subscriber<sensor_msgs::Image>(nh,
                                                               "image",
                                                               5) );

  // Define a subscriber to the depth image
  imDepthSub.reset( new
                    message_filters::Subscriber<sensor_msgs::Image>(nh,
                                                                    "depth_image",
                                                                    5) );

  //synchronizer for fullbody and leg detections topics
  synchronizer.reset( new message_filters::Synchronizer<ApproxSync>(10,
                                                                    *imSub,
                                                                    *imDepthSub) );

  synchronizer->registerCallback(boost::bind(&callback, _1, _2));

  enabled = true;
}

/**
 * @brief stop stop subscribers and topic synchronizer
 */
void stop()
{
  ROS_WARN("Disabling node because there are no subscribers");
  synchronizer.reset();
  imSub.reset();
  imDepthSub.reset();
  enabled = false;
}


int main(int argc, char *argv[] )
{
    ros::init(argc, argv, "depth_segmentation");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::CallbackQueue cbQueue;
    nh.setCallbackQueue(&cbQueue);
    pnh.setCallbackQueue(&cbQueue);

    enabled = false;

    getParameters(nh);

    dynamic_reconfigure::Server<pal_vision_segmentation::DepthSegmentConfig> server(pnh);
    dynamic_reconfigure::Server<pal_vision_segmentation::DepthSegmentConfig>::CallbackType f;
    f = boost::bind(&reconf_callback, _1, _2);
    server.setCallback(f);

    //set up publishers
    image_transport::ImageTransport it(pnh);
    mask_pub       = it.advertise("mask", 1);
    img_masked_pub = it.advertise("image_masked", 1);

    int rate = 30;
    ros::Rate loopRate(rate);
    double halfPeriod = 0.5*1.0/rate;

    while ( ros::ok() )
    {
      bool anySubscriber = img_masked_pub.getNumSubscribers() > 0 ||
                           mask_pub.getNumSubscribers() > 0;

      if ( !enabled && anySubscriber )
        start(nh);
      else if ( enabled && !anySubscriber )
        stop();

      //check for subscriber's callbacks
      cbQueue.callAvailable(ros::WallDuration(halfPeriod));

      loopRate.sleep();
    }

}

