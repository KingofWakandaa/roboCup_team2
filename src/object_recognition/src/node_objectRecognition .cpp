#include <object_recognition/object_recognition.h>



int main(int argc,char **argv)
{
  
  ros::init(argc,argv,"object");
  ros::NodeHandle n;
  ObjectRecognition node(n);

  ros::spin();
  

  
  return 0;
}


