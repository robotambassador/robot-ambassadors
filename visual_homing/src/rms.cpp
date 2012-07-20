#include <opencv2/core/core.hpp>
#include <math.h>

using namespace cv;

class RMS_Error
{
  public:
  float rms(Mat img1, Mat img2);
};

float RMS_Error::rms(Mat img1, Mat img2)
{
  float pixels = img1.rows*img1.cols;
  float sum = 0;
  for (int i = 0; i < img1.rows; i++)
  {
    for (int j = 0; j < img1.cols; j++)
    {
      int t = img1.at<uchar>(i,j) - img2.at<uchar>(i,j);
      sum += t*t;
    }
  }
  return sqrt(sum/pixels);
}
