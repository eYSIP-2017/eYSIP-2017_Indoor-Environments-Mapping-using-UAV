
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
  * @file template_segmentation.cpp
  * @author Bence Magyar
  * @date May 2012
  * @brief Template matchined based segmentation node
  */

// PAL headers
#include <pal_vision_segmentation/TemplateSegmentConfig.h>
#include <pal_vision_segmentation/image_processing.h>

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

/***Variables used in callbacks***/
image_transport::Publisher mask_pub;
int window_width;
int window_height;
int coeffs_to_cancel;
cv::Mat image_rect;
image_transport::Publisher image_pub;
image_transport::Publisher debug_pub;
cv::Mat target_template;
cv::Mat raw_template;
int offset_x, offset_y;
/***end of callback section***/

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

        double ticksBefore = cv::getTickCount();

        cv::Mat matched;
        cv::Mat dct_d;
        pal_vision_util::dctNormalization(image_rect, dct_d, coeffs_to_cancel);
        cv::matchTemplate(dct_d, target_template, matched, CV_TM_CCORR_NORMED);

        double min_val, max_val;
        cv::Point max_loc, min_loc;
        cv::minMaxLoc(matched, &min_val, &max_val, &min_loc, &max_loc, cv::Mat());

        cv::Mat mask;
        mask = cv::Mat::zeros(image_rect.rows, image_rect.cols, CV_8UC1);

        const int start_col = (max_loc.x + offset_x - window_width/2  < 0) ?
                              0 : (max_loc.x + offset_x - window_width/2);
        const int start_row = (max_loc.y + offset_y - window_height/2 < 0) ?
                              0 : (max_loc.y + offset_y - window_height/2);
        const int end_col = (window_width/2 + max_loc.x + offset_x > mask.rows) ?
                            mask.rows : (window_width/2 + offset_x + max_loc.x);
        const int end_row = (window_height/2 + max_loc.y + offset_y > mask.cols) ?
                            mask.cols : (window_height/2 + max_loc.y + offset_y);
        for(int i=start_row; i<end_row; ++i)
            for(int j=start_col; j<end_col; ++j)
                mask.at<uchar>(i,j) = 255;

        if(mask_pub.getNumSubscribers() > 0)
        {
            cv_bridge::CvImage mask_msg;
            mask_msg.header = msg->header;
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
            image_pub.publish(*masked_msg.toImageMsg());
        }

        //DEBUG
        if(debug_pub.getNumSubscribers() > 0)
        {
            cv::rectangle(dct_d, cv::Rect(start_col, start_row, window_width, window_height), cv::Scalar(255,0,0), 2);
            cv::circle(dct_d, cv::Point(max_loc.x + offset_x, max_loc.y + offset_y), 5, cv::Scalar(255,0,0), 1);
            cv_bridge::CvImage debug_msg;
            debug_msg.header = msg->header;
            debug_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
            debug_msg.image = dct_d;
            debug_pub.publish(*debug_msg.toImageMsg());
        }

        ROS_INFO("imageCb runtime: %f ms",
                 1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());
    }
}

void reconf_callback(pal_vision_segmentation::TemplateSegmentConfig &config, uint32_t level)
{
    window_width = config.window_width;
    window_height = config.window_height;
    coeffs_to_cancel = config.coeffs_to_cancel;

    pal_vision_util::dctNormalization(raw_template, target_template, coeffs_to_cancel);
}

int main(int argc, char *argv[] )
{
    ros::init(argc, argv, "template_segmentation");
    ros::NodeHandle nh("template_segmentation");
    nh.param<int>("window_width", window_width, 10);
    nh.param<int>("window_height", window_height, 10);
    nh.param<int>("coeffs_to_cancel", coeffs_to_cancel, 2);

    if(argc < 2)
    {
        ROS_ERROR("Template segmentation needs a template to match. Please provide it as a command line argument.");
        return -1;
    }

    std::string template_path(argv[1]);
    ROS_INFO("%s", template_path.c_str());
    raw_template = cv::imread(template_path);
    pal_vision_util::dctNormalization(raw_template, target_template, coeffs_to_cancel);

    offset_x = (target_template.rows-1)/2;
    offset_y = (target_template.cols-1)/2;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rect_sub = it.subscribe("/image", 1, &imageCb);
    image_pub = it.advertise("image_masked", 1);
    mask_pub = it.advertise("mask", 1);
    debug_pub = it.advertise("debug",1);

    dynamic_reconfigure::Server<pal_vision_segmentation::TemplateSegmentConfig> server;
    dynamic_reconfigure::Server<pal_vision_segmentation::TemplateSegmentConfig>::CallbackType f;
    f = boost::bind(&reconf_callback, _1, _2);
    server.setCallback(f);

    ros::spin();
}

