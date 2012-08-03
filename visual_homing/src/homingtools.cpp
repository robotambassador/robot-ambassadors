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
  //ROS_INFO("rotating image");
	angle = (angle/M_PI)*180;
	Point2f src_center(img.cols/2.0F, img.rows/2.0F);
	Mat rot_mat = getRotationMatrix2D(src_center, angle, 1.0);
	Mat dst;
	warpAffine(img, dst, rot_mat, img.size());

  int min = std::min(img.rows, dst.cols);

  

  ROS_INFO("angle: %f",angle);

	int a = min/sqrt(2);
	int y = (dst.rows - a)/2;
	int x = (dst.cols - a)/2;

  //ROS_INFO("cols: %d, rows: %d, x: %d, y: %d, a: %d", img.cols, img.rows, x, y, a);
	Rect roi(x, y, a, a);

	Mat final = dst(roi);
	
	cv::imshow("rotated  image", final);
  cv::waitKey(50);
  
	return final;
}

void HomingTools::turnAngle(bool clockwise, double radians, ros::NodeHandle& nh_, ros::Publisher& cmd_pub)
{
  /*bool turning = true;
  while(radians < 0) radians += 2*M_PI;
  while(radians > 2*M_PI) radians -= 2*M_PI;
  
  tf::TransformListener listener;
  tf::Transform start_transform;

  //wait for the listener to get the first message
  listener.waitForTransform("base_link", "odom", 
                             ros::Time(0), ros::Duration(1.0));
  
  

  //record the starting transform from the odometry to the base frame
  listener.lookupTransform("base_link", "odom", 
                            ros::Time(0), start_transform);
                            
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0;
  cmd.angular.z = 0.5;
  
  if (clockwise) cmd.angular.z = -cmd.angular.z;
  
  //the axis we want to be rotating by
  
  tf::Vector3 turn_axis(0,0,1);
  if (!clockwise) turn_axis = -turn_axis;
  
  double angle_turned;
  
  cmd_pub.publish(cmd);
  
  ros::Rate loop_rate(10);
    
  while (turning && nh_.ok())
  {
    tf::Transform current_transform;
    //get the current transform
    try
    {
      listener.lookupTransform("base_link", "odom", 
                                ros::Time(0), current_transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      turning = false;
      return true;
    }
    tf::Transform relative_transform = 
      start_transform.inverse() * current_transform;
    tf::Vector3 actual_turn_axis = 
      relative_transform.getRotation().getAxis();
    angle_turned = relative_transform.getRotation().getAngle();
    if ( fabs(angle_turned) > 1.0e-2){
      if ( actual_turn_axis.dot( turn_axis ) < 0 ) 
        angle_turned = 2 * M_PI - angle_turned;

      if (angle_turned > radians) turning = false;
    }
  } 
  
  cmd.angular.z = 0;
  cmd_pub.publish(cmd);
  
  ros::spinOnce();
  loop_rate.sleep();*/
}




