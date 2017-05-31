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

#ifndef __PAL_IMAGE_PROCESSING_H__
#define __PAL_IMAGE_PROCESSING_H__

//#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

#include <vector>

namespace pal_vision_util
{
  /**
   * @brief getConnectedComponents finds connected components in the given binary image
   *
   * @param[in] binaryImage. Pixels set at 0 will be considered background, otherwise foreground.
   * @param[out] components for every connected component found this vector contains a vector of pixel coordinates
   *            that belong to the component (non black pixels).
   **@param[out] boundingBoxes bounding rectangle of every connected component
   */
  void getConnectedComponents(const cv::Mat& binaryImage,
                              std::vector< std::vector<cv::Point2i> >& components,
                              std::vector< cv::Rect >& boundingBoxes);


  /**
      * @brief 2D roi
      *
      * @author Jordi
      *
      */
  class ImageRoi
  {
    public:
    ImageRoi();

    ImageRoi(const ImageRoi& roi);

    ImageRoi(int left, int top, int w, int h);

    ImageRoi(CvRect rect);

    ~ImageRoi();

    void set(CvRect rect);

    void clear();

    bool empty() const;

    CvRect toCvRect() const;

    /**
          * Clips the current roi so that it does not fall outside an image
          * of given width and height
          */
    void keepInside(int imgWidth, int imgHeight);

    /**
          * Resize the roi width and height by the given factors and keep it centered in the same pixel
          */
    const ImageRoi& resize(double factorHorizontal, double factorVertical);

    int x, y, width, height;
  };

  /**
      * @brief  Wrapper of old C-style OpenCV IplImage with some processing functionalities
      *
      * @author Jordi
      *
      */
  class Image
  {
  public:

    /**
          * @brief Default constructor
          */
    Image();

    /**
          * @brief constructor from a IplImage pointer. The ownership of the pointer object is taken.
          */
    Image(IplImage *img);

    /**
     * @brief Image constructor from a cv::Mat object. The ownership is shared but no memory will be released in the destructor.
     * @param mat
     */
    Image(const cv::Mat& mat);

    /**
          * @brief destructor. If the ownership of an IplImage has been taken it is released.
          */
    ~Image();

    /**
          * @brief Explicitely delete the pointed IplImage
          */
    void freeImage();

    void loadImage(const std::string& fileName);

    /**
          * @brief take the ownership of the given pointed object. If it already owned another pointed IplImage it is first realeased.
          */
    void set(IplImage *img);

    /**
          * @brief release the ownership of the returned IplImage.
          * @return NULL if no IplImage was owned. The pointer to the IplImage otherwise.
          */
    IplImage *release();

    void resizeImage(int width, int height, int channels, int depth);

    int getWidth() const { return _img->width; };
    int getHeight() const { return _img->height; };
    int getChannel() const { return _img->nChannels; };
    int getDepth() const { return _img->depth; };

    unsigned char getPixelB(int col, int row, int ch) const;

    float getPixelF(int col, int row, int ch) const;

    short getPixelS(int col, int row, int ch) const;

    void setPixelB(int col, int row, int ch, unsigned char value);

    void setPixelS(int col, int row, int ch, short value);

    void setPixelF(int col, int row, int ch, float value);

    IplImage* getIplImg() { return _img; };

    IplImage* operator->(void) const { return _img; }

    operator IplImage*()  const { return _img; }   //overloads cast to IplImage* operator

  private:

    IplImage _iplFromMat;
    IplImage *_img;
    bool _releaseOnDestroyer;
  };

  /**
      * @brief return the Lab components of the given image
      */
  void getLab(const Image& img, Image& L, Image& a, Image& b);

  /**
      * @brief build up a new image from the three Lab components provided.
      */
  void setLab(const Image& L, const Image& a, const Image& b, Image& img);

  /**
      * @brief illumination normalization using the discrete cosine transform (DCT).
      *
      * @param[in]  img input image
      * @param[out] normalizedImg resulting normalized image
      * @param[in]  coefficientsToCancel number of DCT coefficients that will be truncated
      * @param[in]  ImageRoi optional. Image region in which normalization will be carried on.
      */
  void dctNormalization(const Image& img, Image& normalizedImg, int coefficientsToCancel, const ImageRoi& roi = ImageRoi(0,0,0,0));

  /**
      * @brief @see dctNormalization(). Overloaded function accepting OpenCV C++ image as input and output.
      */
  void dctNormalization(const cv::Mat& img, cv::Mat& normalizedImg, int coefficientsToCancel, const ImageRoi& roi = ImageRoi(0,0,0,0));

} // namespace pal_vision_util

#endif // __PAL_IMAGE_PROCESSING_H__
