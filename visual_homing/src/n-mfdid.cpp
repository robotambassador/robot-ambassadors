#include <ros.h>



class NewtonMFDID
{
  NodeHandle nh_;
  Subscriber image_sub_;
  Publisher hv_pub_;
  
  void calculateHomingVector();
  
  
  public:
  NewtonMFDID();
}

NewtonMFDID::NewtonMFDID() 
{
  nh_ = 
  image_sub_ = subscribe("unwrapped_image", 1, &NewtonMFDID::calculateHomingVector());
  hv_pub_ = advertise("homing_vector", 1);
}

void calculateHomingVector()
{
  
  
  hv_pub_.publish();
}


int main(int argc, char *argv[])
{
  NewtonMFDID n;
 
  ros::spin(); 
  return 0;
}
