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

/** \author Jordi Pages */

#include <pal_vision_segmentation/image_processing.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <iostream>

namespace pal_vision_util
{

  void getConnectedComponents(const cv::Mat& binaryImage,
                              std::vector< std::vector<cv::Point2i> >& components,
                              std::vector< cv::Rect >& boundingBoxes)
  {
    components.clear();

    //make sure to use a binary image where background pixels are set to 0 and the remaining ones to 1
    cv::Mat tempBinaryImage;
    cv::threshold(binaryImage, tempBinaryImage, 1, 1, cv::THRESH_BINARY );

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    cv::Mat label_image;
    tempBinaryImage.convertTo(label_image, CV_32FC1); // weird it doesn't support CV_32S!

    int label_count = 2; // starts at 2 because 0,1 are used already

    for(int y=0; y < binaryImage.rows; y++)
    {
      for(int x=0; x < binaryImage.cols; x++)
      {
        if ( static_cast<int>(label_image.at<float>(y,x)) != 1 )
        {
          continue;
        }

        cv::Rect rect;
        cv::floodFill(label_image, cv::Point(x,y), cv::Scalar(label_count), &rect, cv::Scalar(0), cv::Scalar(0), 4);

        std::vector <cv::Point2i> component;

        for(int i=rect.y; i < (rect.y+rect.height); i++)
        {
          for(int j=rect.x; j < (rect.x+rect.width); j++)
          {
            if ( static_cast<int>(label_image.at<float>(i,j)) != label_count )
            {
              continue;
            }

            component.push_back(cv::Point2i(j,i));
            boundingBoxes.push_back(rect);
          }
        }

        components.push_back(component);

        label_count++;
      }
    }
  }

  ImageRoi::ImageRoi()
  {
    clear();
  }

  ImageRoi::ImageRoi(const ImageRoi& roi)
  {
    x = roi.x;
    y = roi.y;
    width = roi.width;
    height = roi.height;
  }

  ImageRoi::ImageRoi(int left, int top, int w, int h): x(left), y(top), width(w), height(h)
  {
  }

  ImageRoi::ImageRoi(CvRect rect)
  {
    set(rect);
  }

  ImageRoi::~ImageRoi()
  {
  }

  void ImageRoi::set(CvRect rect)
  {
    x      = rect.x;
    y      = rect.y;
    width  = rect.width;
    height = rect.height;
  }

  void ImageRoi::clear()
  {
    x = 0;
    y = 0;
    width = 0;
    height = 0;
  }

  bool ImageRoi::empty() const
  {
    return x == 0 && y == 0 && width == 0 && height == 0;
  }

  CvRect ImageRoi::toCvRect() const
  {
    return cvRect(x, y, width, height);
  }

  /**
        * Clips the current roi so that it does not fall outside an image
        * of given width and height
        */
  void ImageRoi::keepInside(int imgWidth, int imgHeight)
  {
    if ( !empty() )
    {
      if ( x < 0 )
      {
        width += x;
        x = 0;
      }
      if ( y < 0 )
      {
        height += y;
        y = 0;
      }
      if ( x + width - 1 >= imgWidth )
      {
        width = - x + imgWidth;
      }
      if ( y + height - 1 >= imgHeight )
      {
        height = - y + imgHeight;
      }
    }
  }

  /**
        * Resize the roi width and height by the given factors and keep it centered in the same pixel
        */
  const ImageRoi& ImageRoi::resize(double factorHorizontal, double factorVertical)
  {
    int cx = x + width/2;
    int cy = y + height/2;

    int newWidth  = static_cast<int>( factorHorizontal * static_cast<double>(width) );
    int newHeight = static_cast<int>( factorVertical * static_cast<double>(height) );

    x = cx - newWidth/2;
    y = cy - newHeight/2;
    width = newWidth;
    height = newHeight;

    return *this;
  }

  Image::Image()
  {
    _img = NULL;
    _img = cvCreateImage(cvSize(10, 10), 8, 3);
    _releaseOnDestroyer = true;
  }

  Image::Image(IplImage *img)
  {
    _img = img;
    _releaseOnDestroyer = true;
  }

  Image::Image(const cv::Mat& mat)
  {
    _iplFromMat = mat;
    _img = &_iplFromMat;
    _releaseOnDestroyer = false;
  }

  Image::~Image()
  {
    freeImage();
  }

  void Image::freeImage()
  {
    if ( _img != NULL && _releaseOnDestroyer )
    {
      cvReleaseImage(&_img);
      _img = NULL;
    }
  }

  void Image::loadImage(const std::string& fileName)
  {

  }

  void Image::set(IplImage *img)
  {
    freeImage();
    _img = img;
    _releaseOnDestroyer = true;
  }

  IplImage* Image::release()
  {
    IplImage *img = _img;
    _img = NULL;
    return img;
  }

  void Image::resizeImage(int width, int height, int channels, int depth)
  {
    freeImage();
    _img = cvCreateImage(cvSize(width, height), depth, channels);
  }

  unsigned char Image::getPixelB(int col, int row, int ch) const
  {
    unsigned char value = 0;

    value = *((unsigned char *)((unsigned long)_img->imageData + (unsigned long)ch + (unsigned long)((col)*_img->nChannels) + (unsigned long)((row)*_img->widthStep)));

    return value;
  }

  float Image::getPixelF(int col, int row, int ch) const
  {
    float value = 0;

    value = *((float *)((unsigned long)(_img->imageData) + (unsigned long)((ch)*sizeof(float)) + (unsigned long)((col)*_img->nChannels*sizeof(float)) + (unsigned long)((row)*_img->widthStep)));

    return value;
  }

  short Image::getPixelS(int col, int row, int ch) const
  {
    short value = 0;

    value = *((short *)((unsigned long)_img->imageData + (unsigned long)((ch)*sizeof(short)) + (unsigned long)((col)*_img->nChannels*sizeof(short)) + (unsigned long)((row)*_img->widthStep)));

    return value;
  }


  void Image::setPixelB(int col, int row, int ch, unsigned char value)
  {
    *((unsigned char *)((unsigned long)(_img->imageData) + (unsigned long)ch + (unsigned long)(col*_img->nChannels) + (unsigned long)(row*_img->widthStep))) = value;
  }

  void Image::setPixelS(int col, int row, int ch, short value)
  {
    *((short *)((unsigned long)(_img->imageData) + (unsigned long)(ch*sizeof(short)) + (unsigned long)(col*_img->nChannels*sizeof(short)) + (unsigned long)((row)*_img->widthStep))) = value;
  }

  void Image::setPixelF(int col, int row, int ch, float value)
  {
    *((float *)((unsigned long)(_img->imageData) + (unsigned long)((ch)*sizeof(float)) + (unsigned long)((col)*(_img->nChannels)*sizeof(float)) + (unsigned long)((row)*_img->widthStep))) = value;

  }

  void getLab(const Image& img, Image& L, Image& a, Image& b)
  {
    if ( img.getChannel() != 3 || img.getDepth() != IPL_DEPTH_8U )
    {
      std::cout << std::endl << "Error in getLightnessFromLab: the input image is not 3-channel 8 bit image" << std::endl << std::endl;
      exit(0);
    }

    Image labImg;
    labImg.resizeImage(cvGetImageROI(img).width, cvGetImageROI(img).height, 3, 8);
    L.resizeImage(cvGetImageROI(img).width, cvGetImageROI(img).height, 1, 8);
    a.resizeImage(cvGetImageROI(img).width, cvGetImageROI(img).height, 1, 8);
    b.resizeImage(cvGetImageROI(img).width, cvGetImageROI(img).height, 1, 8);

    cvCvtColor(img, labImg, CV_BGR2Lab);
    cvSplit(labImg, L, a, b, NULL);
  }

  void setLab(const Image& L, const Image& a, const Image& b, Image& img)
  {
    if ( L.getChannel() != 1 || a.getChannel() != 1 || b.getChannel() != 1 ||
         L.getDepth() != 8 || a.getDepth() != 8 || b.getDepth() != 8 )
    {
      std::cout << std::endl << "Error in setLab: L, a, b images must be 1-channel 8 bit" << std::endl << std::endl;
    }

    img.resizeImage(cvGetImageROI(L).width, cvGetImageROI(L).height, 3, 8);
    cvMerge(L, a, b, NULL, img);
  }


  void dctNormalization(const Image& img, Image& normalizedImg, int coefficientsToCancel, const ImageRoi& roi)
  {
    //////////////////////////////////////////////////////////////////////////////////
    // Unfortunately, DCT may only be computed on even-sized images
    // First of all, define image as a suitable sized copy of img
    ///////////////////////////////////////////////////////////////////////////////////
    bool oddSize = false;
    int oddWidth = img.getWidth(), oddHeight = img.getHeight();
    ImageRoi evenRoi(roi);

    //check if no roi is provided but image size is odd
    if ( roi.width == 0 && roi.height == 0 && (img.getWidth() % 2 != 0 || img.getHeight() % 2 != 0) )
    {
      oddSize   = true;
      oddWidth  = img.getWidth();
      oddHeight = img.getHeight();
    }
    //check if roi is provided and it has odd size
    else if ( roi.width != 0 && roi.height != 0 && (roi.width % 2 != 0 || roi.height % 2 != 0) )
    {
      oddSize   = true;
      oddWidth  = roi.width;
      oddHeight = roi.height;
    }

    //if either the roi or the image is odd-sized, use an even-sized roi to fix it
    if ( oddSize )
    {
      int evenWidth = oddWidth, evenHeight = oddHeight;
      if ( evenWidth % 2 != 0 )
        --evenWidth;
      if ( evenHeight % 2 != 0 )
        --evenHeight;

      evenRoi.width  = evenWidth;
      evenRoi.height = evenHeight;
      dctNormalization(img, normalizedImg, coefficientsToCancel, evenRoi);
      return;
    }

    if ( img.getDepth() != 8 )
    {
      std::cout << std::endl << "Error in dctNormalization: input image must have 8 bit depth " << std::endl << std::endl;
      exit(0);
    }

    if ( img.getChannel() == 3 )
    {
      Image L, a, b;

      normalizedImg.resizeImage(img.getWidth(), img.getHeight(), 3, 8);
      cvZero(normalizedImg);

      if ( roi.width != 0 && roi.height != 0 )
      {
        cvSetImageROI(img, cvRect(roi.x, roi.y, roi.width, roi.height));
        cvSetImageROI(normalizedImg, cvRect(roi.x, roi.y, roi.width, roi.height));
      }

      getLab(img, L, a, b);

      //L.seeWindow("L", 5);
      Image normalizedL;
      dctNormalization(L, normalizedL, coefficientsToCancel);
      //normalizedL.seeWindow("L normalized", 5);
      Image LabImg;
      setLab(normalizedL, a, b, LabImg);
      cvCvtColor(LabImg, normalizedImg, CV_Lab2BGR);

      if ( roi.width != 0 && roi.height != 0 )
      {
        cvResetImageROI(img);
        cvResetImageROI(normalizedImg);
      }
    }
    else if ( img.getChannel() ==  1 )
    {
      Image X32, Y32;
      double minVal1, maxVal1;
      cvMinMaxLoc(img, &minVal1, &maxVal1);

      normalizedImg.resizeImage(img.getWidth(), img.getHeight(), img.getChannel(), img.getDepth());
      cvZero(normalizedImg);

      if ( roi.width != 0 && roi.height != 0 )
      {
        cvSetImageROI(img, cvRect(roi.x, roi.y, roi.width, roi.height));
        cvSetImageROI(normalizedImg, cvRect(roi.x, roi.y, roi.width, roi.height));
      }

      X32.resizeImage(cvGetImageROI(img).width, cvGetImageROI(img).height, 1, IPL_DEPTH_32F);
      Y32.resizeImage(cvGetImageROI(img).width, cvGetImageROI(img).height, 1, IPL_DEPTH_32F);

      cvZero(Y32);
      cvConvert(img, X32);

      for (int row = 0; row < X32.getHeight(); ++row)
        for (int col = 0; col < X32.getWidth(); ++col)
        {
          float f = X32.getPixelF(col, row, 0);
          if ( f == 0 ) f = 1; //to avoid log(0)
          X32.setPixelF(col, row, 0, log(f));
        }

      cvDCT(X32, Y32, CV_DXT_FORWARD);

      //cancel low frequency coefficients
      for (int row = 0; row < coefficientsToCancel && row < Y32.getHeight(); ++row)
        for (int col = 0; col < coefficientsToCancel && col < Y32.getWidth(); ++col)
        {
          Y32.setPixelF(col, row, 0, 0);
        }

      cvZero(X32);
      cvDCT(Y32, X32, CV_DXT_INVERSE);

      double minVal2, maxVal2, m = 0, M = 255;
      cvMinMaxLoc(X32, &minVal2, &maxVal2);
      cvConvertScaleAbs(X32, normalizedImg, (M-m)/(maxVal2-minVal2), (m-M)*minVal2/(maxVal2-minVal2));
      //cvConvertScaleAbs(X32, normalizedImg, 255.0/(maxVal2-minVal2), -255.0*minVal2/(maxVal2-minVal2));
      //cvConvertScaleAbs(X32, normalizedImg, (maxVal1-minVal1)/(maxVal2-minVal2), -minVal2*(maxVal1-minVal1)/(maxVal2-minVal2));

      if ( roi.width != 0 && roi.height != 0 )
      {
        cvResetImageROI(img);
        cvResetImageROI(normalizedImg);
      }
    }
    else
    {
      std::cout << std::endl << "Error in dctNormalization: input image format not supported" << std::endl << std::endl;
    }
  }

  void dctNormalization(const cv::Mat& img, cv::Mat& normalizedImg, int coefficientsToCancel, const ImageRoi& roi)
  {
    IplImage ipl = img;
    Image input(&ipl);
    Image output;
    dctNormalization(input, output, coefficientsToCancel, roi);
    normalizedImg = cv::Mat(output.getIplImg(), true);
    input.release();
  }

} // namespace pal_vision_util
