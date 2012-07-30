#ifndef HOMINGTOOLS
#define HOMINGTOOLS

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>

class HomingTools
{
  public:
  static float rms(cv::Mat& img1, cv::Mat& img2);
  static cv::Mat rotateImage(cv::Mat& img, float angle);
};

#endif
