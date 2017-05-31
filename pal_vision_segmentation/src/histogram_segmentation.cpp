/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2012, PAL Robotics, S.L.
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
  * @file histogram_segmentation.cpp
  * @author Bence Magyar
  * @date May 2012
  * @brief Histogram based segmentation node. \
  * Using histogram backprojection, normalization, thresholding and dilate operator. \
  * Erode operator can be added by uncommenting lines \
  * in histogram_segmentation.cpp and the corresponding cfg file.
  */

// PAL headers
#include <pal_vision_segmentation/HistogramSegmentConfig.h>
#include <pal_vision_segmentation/image_processing.h>
#include <pal_vision_segmentation/histogram.h>

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

// Boost headers
#include <boost/filesystem.hpp>


/***Variables used in callbacks***/
image_transport::Publisher mask_pub;
int threshold;
int dilate_iterations;
int dilate_size;
int erode_iterations;
int erode_size;
int dark_pixels_threshold;
cv::Mat image_rect;
image_transport::Publisher image_pub;
image_transport::Publisher debug_pub;
cv::MatND target_hist;
/***end of callback section***/


/**
 * @brief removeDarkPixels given a BGR image and a binary image computed from it, removes the
 *        white pixel in the latter corresponding to low luminosity pixels in the former one.
 * @param[in] originalImg
 * @param[out] binaryImg
 * @param[in] threshold
 */
void removeDarkPixels(const cv::Mat& originalImg,
                      cv::Mat& binaryImg,
                      int threshold = 10)
{
  pal_vision_util::Image imgOriginalImg(originalImg), imgL, img_a, img_b;

  pal_vision_util::getLab(imgOriginalImg, imgL, img_a, img_b);

  //update header of cv::Mat L from imgL:
  cv::Mat L;
  L = cv::cvarrToMat(imgL.getIplImg());

  cv::Mat L_thresholded;
  L_thresholded = L.clone();
  cv::threshold(L, L_thresholded, threshold, 255, cv::THRESH_BINARY);
  cv::bitwise_and(binaryImg, L_thresholded, binaryImg);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    if(image_pub.getNumSubscribers() > 0 ||
       mask_pub.getNumSubscribers() > 0 ||
       debug_pub.getNumSubscribers() > 0)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        image_rect = cv_ptr->image;

        cv::Mat backProject;
        pal_vision_util::backProject(image_rect, target_hist, threshold, backProject);

        if ( dark_pixels_threshold > 0 )
          removeDarkPixels(image_rect, backProject, dark_pixels_threshold);

        cv::Mat mask, tmp1;

        if(dilate_iterations == 0 && erode_iterations == 0)
            mask = backProject;

        if(dilate_iterations > 0)
        {
            cv::dilate(backProject, erode_iterations == 0 ? mask: tmp1,
                       cv::Mat::ones(dilate_size, dilate_size, CV_8UC1),
                       cv::Point(-1, -1), dilate_iterations);
        }

        if(erode_iterations > 0)
        {
            cv::erode(dilate_iterations == 0 ? backProject : tmp1, mask,
                      cv::Mat::ones(erode_size, erode_size, CV_8UC1),
                      cv::Point(-1, -1), erode_iterations);
        }

        ros::Time now = msg->header.stamp; //ros::Time::now();

        if(mask_pub.getNumSubscribers() > 0)
        {
            cv_bridge::CvImage mask_msg;
            mask_msg.header = msg->header;
            mask_msg.header.stamp = now; //ros::Time::now();
            mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
            mask_msg.image = mask;
            mask_pub.publish(mask_msg.toImageMsg());
        }

        if(image_pub.getNumSubscribers() > 0)
        {
            cv::Mat masked;
            image_rect.copyTo(masked, mask);
            cv_bridge::CvImage masked_msg;
            masked_msg.header = msg->header;
            masked_msg.encoding = sensor_msgs::image_encodings::BGR8;
            masked_msg.image = masked;
            masked_msg.header.stamp  = now; //ros::Time::now();
            image_pub.publish(*masked_msg.toImageMsg());
        }

        //DEBUG
        if(debug_pub.getNumSubscribers() > 0)
        {
            cv_bridge::CvImage debug_msg;
            debug_msg.header = msg->header;
            debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
            debug_msg.image = backProject;
            debug_pub.publish(*debug_msg.toImageMsg());
        }
    }
}

void reconf_callback(pal_vision_segmentation::HistogramSegmentConfig &config, uint32_t level)
{
    threshold = config.threshold;
    dilate_iterations = config.dilate_iterations;
    dilate_size = config.dilate_size;
    erode_iterations = config.erode_iterations;
    erode_size = config.erode_size;
    dark_pixels_threshold = config.dark_pixels_threshold;
}

void computeHistogramFromFile(const std::string& template_path, cv::MatND& hist)
{
  if ( !boost::filesystem::is_directory(template_path) ) //if the given path is not a folder
  {
    if ( !boost::filesystem::is_regular_file(template_path) )
      throw std::runtime_error("The given path is not a file: " + template_path);

    cv::Mat raw_template = cv::imread(template_path);
    pal_vision_util::calcHSVHist(raw_template, hist);
    cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX, -1, cv::Mat());
    cv::threshold(hist, hist, 128, 255, cv::THRESH_BINARY);            //all those bin values > threshold are set to 255, otherwise 0
  }
  else //otherwise, it should be a folder
  {
    boost::filesystem::directory_iterator end_iter;
    std::vector<cv::Mat> images;
    for( boost::filesystem::directory_iterator dir_iter(template_path) ; dir_iter != end_iter ; ++dir_iter)
    {
      if ( boost::filesystem::is_regular_file(dir_iter->status()) )
      {
        std::string fullPathFileName = dir_iter->path().string();
        cv::Mat raw_template = cv::imread(fullPathFileName);
        if ( !raw_template.empty() )
          images.push_back(raw_template);
      }
    }
    if ( !images.empty() )
    {
      //pal_vision_util::calcHSVHist(images, hist);
      std::vector<cv::MatND> hists;
      //compute a separate histogram for each image
      pal_vision_util::calcHSVHist(images, hists);

      //create a new histogram where every bin takes the maximum value if any of the histogram's bin value is
      //larger than the threshold
      for (int h = 0; h < hists[0].rows; ++h)
        for (int s = 0; s < hists[0].cols; ++s)
        {
          unsigned int i = 0;
          bool found = false;
          while ( i < hists.size() && !found )
          {
           if ( hists[i].at<float>(h, s) > threshold )
           {
              hists[0].at<float>(h, s) = 255;
              found = true;
           }
           ++i;
          }
          if ( !found )
            hists[0].at<float>(h, s) = 0;
        }
      hist = hists[0].clone();
    }
    else
      throw std::runtime_error("No images found in the given path: " + template_path);
  }
}

int main(int argc, char *argv[] )
{
    ros::init(argc, argv, "histogram_segmentation"); //initialize with a default name
    ros::NodeHandle nh("~"); //use node name as sufix of the namespace
    nh.param<int>("threshold", threshold, 254);
    nh.param<int>("dilate_iterations", dilate_iterations, 9);
    nh.param<int>("dilate_size", dilate_size, 7);
    nh.param<int>("erode_iterations", erode_iterations, 0);
    nh.param<int>("erode_size", erode_size, 3);
    nh.param<int>("dark_pixels_threshold", dark_pixels_threshold, 15);

    if(argc < 2)
    {
        ROS_ERROR("Histogram segmentation needs a template to search for. Please provide it as a command line argument.");
        return -1;
    }

    std::string template_path(argv[1]);
    ROS_INFO_STREAM("Computing histogram from: " << template_path);

    computeHistogramFromFile( template_path, target_hist );

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rect_sub = it.subscribe("/image", 1, &imageCb);
    image_pub = it.advertise("image_masked", 1);
    mask_pub = it.advertise("mask", 1);
    debug_pub = it.advertise("debug",1);

    dynamic_reconfigure::Server<pal_vision_segmentation::HistogramSegmentConfig> server;
    dynamic_reconfigure::Server<pal_vision_segmentation::HistogramSegmentConfig>::CallbackType f;
    f = boost::bind(&reconf_callback, _1, _2);
    server.setCallback(f);

    ros::spin();
}
