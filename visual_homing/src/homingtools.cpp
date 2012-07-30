#include <visual_homing/homingtools.h>


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
	angle = (angle/180) * M_PI;
	Point2f src_center(img.cols/2.0F, img.rows/2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
	Mat dst;
	warpAffine(img, dst, rot_mat, img.size());

	int a = img.rows/sqrt(2);
	int x = (img.rows - a)/2;
	int y = (img.cols - a)/2;

	Rect sub(x, y, a, a);

	Mat final = dst(sub);
	return final;
}
