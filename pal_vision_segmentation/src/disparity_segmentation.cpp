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
  * @file disparity_segmentation.cpp
  * @author Bence Magyar
  * @date May 2012
  * @brief Disparity based segmentation node.
  */

// PAL headers
#include <pal_vision_segmentation/DisparitySegmentConfig.h>

// ROS headers
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <dynamic_reconfigure/server.h>

// OpenCV headers
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


/***Variables used in callbacks***/
image_transport::Publisher mask_pub;
int threshold;
int dilate_iterations;
int dilate_size;
int erode_iterations;
int erode_size;
int scale_factor;
cv::Mat image_rect;
image_transport::CameraPublisher cam_pub;
sensor_msgs::CameraInfo camera_info;
ros::Subscriber cam_info_sub;
cv::Mat mask;
/***end of callback section***/

void disparityCb(const stereo_msgs::DisparityImageConstPtr& msg)
{
    if(cam_pub.getNumSubscribers() > 0 ||
       mask_pub.getNumSubscribers() > 0)
    {
        //    double ticksBefore = cv::getTickCount();
        // Check for common errors in input
        if (msg->min_disparity == 0.0 && msg->max_disparity == 0.0)
        {
            ROS_ERROR("Disparity image fields min_disparity and max_disparity are not set");
            return;
        }
        if (msg->image.encoding != sensor_msgs::image_encodings::TYPE_32FC1)
        {
            ROS_ERROR("Disparity image must be 32-bit floating point (encoding '32FC1'), but has encoding '%s'",
                      msg->image.encoding.c_str());
            return;
        }

        // code taken from image_view / disparity_view
        // Colormap and display the disparity image
        float min_disparity = msg->min_disparity;
        float max_disparity = msg->max_disparity;
        float multiplier = 255.0f / (max_disparity - min_disparity);

        const cv::Mat_<float> dmat(msg->image.height, msg->image.width,
                                   (float*)&msg->image.data[0], msg->image.step);
        cv::Mat disparity_greyscale;
        disparity_greyscale.create(msg->image.height, msg->image.width, CV_8UC1);

        for (int row = 0; row < disparity_greyscale.rows; ++row) {
            const float* d = dmat[row];
            for (int col = 0; col < disparity_greyscale.cols; ++col) {
                int index = (d[col] - min_disparity) * multiplier + 0.5;

                //index = std::min(255, std::max(0, index));
                // pushing it into the interval does not matter because of the threshold afterwards
                if(index >= threshold)
                    disparity_greyscale.at<uchar>(row, col) = 255;
                else
                    disparity_greyscale.at<uchar>(row, col) = 0;
            }
        }

        cv::Mat tmp1, mask;
        cv::erode(disparity_greyscale, tmp1,
                  cv::Mat::ones(erode_size, erode_size, CV_8UC1),
                  cv::Point(-1, -1), erode_iterations);
        cv::dilate(tmp1, mask,
                   cv::Mat::ones(dilate_size, dilate_size, CV_8UC1),
                   cv::Point(-1, -1), dilate_iterations);

        if(mask_pub.getNumSubscribers() > 0)
        {
            cv_bridge::CvImage mask_msg;
            mask_msg.header = msg->header;
            mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
            mask_msg.image = mask;
            mask_pub.publish(mask_msg.toImageMsg());
        }

        if(!image_rect.empty() && cam_pub.getNumSubscribers() > 0)
        {
            cv::Mat masked;
            image_rect.copyTo(masked, mask);
            cv_bridge::CvImage masked_msg;
            masked_msg.header = msg->header;
            masked_msg.encoding = sensor_msgs::image_encodings::BGR8;

            //if we want rescale then rescale
            if(scale_factor > 1)
            {
                cv::Mat masked_tmp = masked;
                cv::resize(masked_tmp, masked,
                           cv::Size(masked.cols*scale_factor, masked.rows*scale_factor));
            }
            masked_msg.image = masked;
            // to provide a synchronized output we publish both topics with the same timestamp
            masked_msg.header.stamp  = msg->header.stamp;
            camera_info.header.stamp = msg->header.stamp;
            cam_pub.publish(*masked_msg.toImageMsg(), camera_info);
        }

        //    ROS_INFO("disparityCb runtime: %f ms",
        //             1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());
    }
}

void constMaskCb(const stereo_msgs::DisparityImageConstPtr& msg)
{
    if(!image_rect.empty())
    {
        cv::Mat masked;
        image_rect.copyTo(masked, mask);
        cv_bridge::CvImage masked_msg;
        masked_msg.header = msg->header;
        masked_msg.encoding = sensor_msgs::image_encodings::BGR8;

        //if we want rescale then rescale
        if(scale_factor > 1)
        {
            cv::Mat masked_tmp = masked;
            cv::resize(masked_tmp, masked,
                       cv::Size(masked.cols*scale_factor, masked.rows*scale_factor));
        }
        masked_msg.image = masked;
        cam_pub.publish(*masked_msg.toImageMsg(), camera_info);
    }

    //    ROS_INFO("disparityCb runtime: %f ms",
    //             1000*(cv::getTickCount() - ticksBefore)/cv::getTickFrequency());
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
}

void reconf_callback(pal_vision_segmentation::DisparitySegmentConfig &config, uint32_t level)
{
    threshold = config.threshold;
    dilate_iterations = config.dilate_iterations;
    dilate_size = config.dilate_size;
    erode_iterations = config.erode_iterations;
    erode_size = config.erode_size;
}

void camInfoCallback(const sensor_msgs::CameraInfo &msg)
{
    camera_info = msg;
    // the image is rectified so by default there's no distortion
    for(unsigned int i=0; i<camera_info.D.size(); ++i)
        camera_info.D[i] = 0.0;

    //rescale the parameters if the image is rescaled
    if(scale_factor > 1)
    {
        camera_info.width *= scale_factor;
        camera_info.height *= scale_factor;
        for(int i=0; i<12; i++)
            camera_info.P[i] *= scale_factor;
        camera_info.P[10] = 1.0;
    }

    // Copy the top 3x3 matrix of P into K
    for(int i=0; i<3; ++i)
        for(int j=0; j<3; ++j)
            camera_info.K[i*3+j] = camera_info.P[i*4+j];
}

int main(int argc, char *argv[] )
{
    ros::init(argc, argv, "disparity_segmentation");
    ros::NodeHandle nh("disparity_segmentation");
    nh.param<int>("threshold", threshold, 89);
    nh.param<int>("dilate_iterations", dilate_iterations, 9);
    nh.param<int>("dilate_size", dilate_size, 7);
    nh.param<int>("erode_iterations", erode_iterations, 1);
    nh.param<int>("erode_size", erode_size, 3);
    nh.param<int>("scale_factor", scale_factor, 1);

    std::string mask_path;
    nh.param<std::string>("mask_path", mask_path, "");

    image_transport::ImageTransport it(nh);
    ros::Subscriber disp_sub;
    if(mask_path == "")
    {
        disp_sub = nh.subscribe<stereo_msgs::DisparityImage>("/disparity_image", 1, &disparityCb);
    } else {
        mask = cv::imread(mask_path);
        disp_sub = nh.subscribe<stereo_msgs::DisparityImage>("/disparity_image", 1, &constMaskCb);
    }
    image_transport::Subscriber rect_sub = it.subscribe("/image", 1, &imageCb);
    cam_pub = it.advertiseCamera("image_masked", 1);
    cam_info_sub = nh.subscribe("/camera_info", 1, &camInfoCallback);
    mask_pub = it.advertise("mask", 1);

    dynamic_reconfigure::Server<pal_vision_segmentation::DisparitySegmentConfig> server;
    dynamic_reconfigure::Server<pal_vision_segmentation::DisparitySegmentConfig>::CallbackType f;
    f = boost::bind(&reconf_callback, _1, _2);
    server.setCallback(f);

    ros::spin();
}

