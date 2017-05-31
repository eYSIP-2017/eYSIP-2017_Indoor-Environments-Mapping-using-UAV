#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <pal_vision_segmentation/histogram.h>

bool isInteractive;

void backProjectionTest(const std::string& imageFileName,
                        const std::string& templateFileName,
                        int dilateIterations, int dilateSize,
                        int erodeIterations, int erodeSize,
                        cv::Mat& mask)
{
  cv::Mat templateImg;

  std::cout << "Trying to read image from file: " << imageFileName << std::endl;
  std::cout << "Trying to read template from file: " << templateFileName << std::endl;

  templateImg = cv::imread(templateFileName, 1);

  cv::Mat image;
  image = cv::imread(imageFileName, 1);

  cv::MatND hist;
  pal_vision_util::calcHSVHist(templateImg, hist);
  cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX, -1, cv::Mat()); //set histogram bin values in [0, 255]
  cv::threshold(hist, hist, 128, 255, cv::THRESH_BINARY);            //all those bin values > threshold are set to 255, otherwise 0

  if ( isInteractive )
    pal_vision_util::showHist(hist, false);

  int threshold = 254;
  pal_vision_util::backProject(image,
                               hist,
                               threshold,
                               mask,
                               dilateIterations, dilateSize, //dilate iterations and size
                               erodeIterations, erodeSize);

  if ( isInteractive )
  {
    cv::Mat imgMasked;
    image.copyTo(imgMasked, mask);

    cv::namedWindow("image");
    cv::imshow("image", image);

    cv::namedWindow("back-projection");
    cv::imshow("back-projection", mask);

    cv::namedWindow("image masked");
    cv::imshow("image masked", imgMasked);

    cv::waitKey(5000);
  }

  bool ok = true;
  EXPECT_TRUE( ok );
}


TEST(histogram, test_back_projection)
{
  cv::Mat mask;

  //////////////////////////////////////////////////////////////////////////////////////
  // Synthetic example
  //////////////////////////////////////////////////////////////////////////////////////
  backProjectionTest(ros::package::getPath("pal_vision_segmentation") + "/test/etc/histogram_segmentation_sample.jpg",
                     ros::package::getPath("pal_vision_segmentation") + "/test/etc/histogram_segmentation_blue_template.jpg",
                     0, 3, 0, 3,
                     mask);

  //check some pixels of the resulting mask
  EXPECT_TRUE( mask.at<unsigned char>(45, 54)   == 0 );
  EXPECT_TRUE( mask.at<unsigned char>(52, 208)  == 0 );
  EXPECT_TRUE( mask.at<unsigned char>(165, 90)  == 0 );
  EXPECT_TRUE( mask.at<unsigned char>(158, 230) == 255 );

  //////////////////////////////////////////////////////////////////////////////////////
  // Real example
  //////////////////////////////////////////////////////////////////////////////////////
  backProjectionTest(ros::package::getPath("pal_vision_segmentation") + "/test/etc/pringles_frame.jpg",
                     ros::package::getPath("pal_vision_segmentation") + "/test/etc/pringles_template2.png",
                     9, 5, 0, 3,
                     mask);

  //check some pixels of the resulting mask
  EXPECT_TRUE( mask.at<unsigned char>(55, 55)   == 0 );
  EXPECT_TRUE( mask.at<unsigned char>(221, 246) == 255 );
  EXPECT_TRUE( mask.at<unsigned char>(460, 630) == 255 );
  EXPECT_TRUE( mask.at<unsigned char>(240, 450) == 0 );
  EXPECT_TRUE( mask.at<unsigned char>(365, 8)   == 255 );
  EXPECT_TRUE( mask.at<unsigned char>(208, 142) == 255 );
  EXPECT_TRUE( mask.at<unsigned char>(220, 140) == 255 );
  EXPECT_TRUE( mask.at<unsigned char>(20, 220) == 255 );

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    
    isInteractive = argc > 1 && std::string(argv[1]) == "interactive";
    
    return RUN_ALL_TESTS();
}
