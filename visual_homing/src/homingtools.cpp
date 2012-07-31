#include <visual_homing/homingtools.h>
#include <algorithm>
#include <ros/ros.h>

using namespace cv;

float HomingTools::rms(Mat& img1, Mat& img2)
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

Mat HomingTools::rotateImage(Mat& img, float angle)
{
  //ROS_INFO("rotating image");
	angle = (angle/180) * M_PI;
	Point2f src_center(img.cols/2.0F, img.rows/2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
	Mat dst;
	warpAffine(img, dst, rot_mat, img.size());

  int min = std::min(img.rows, dst.cols);

	int a = min/sqrt(2);
	int y = (img.rows - a)/2;
	int x = (img.cols - a)/2;

  //ROS_INFO("cols: %d, rows: %d, x: %d, y: %d, a: %d", img.cols, img.rows, x, y, a);
	Rect roi(x, y, a, a);

	Mat final = dst(roi);
	return final;
}
