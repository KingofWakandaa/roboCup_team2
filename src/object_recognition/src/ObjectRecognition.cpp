#include <object_recognition/object_recognition.h>

  ObjectRecognition::ObjectRecognition(ros::NodeHandle n): n(n){
  
  pub = n.advertise<perception_msgs::Rect>("rect",100);
  sub = n.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes",100,&ObjectRecognition::callback,this);
  ROS_INFO_STREAM("you are detecting bottle");
  }

void ObjectRecognition::callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{

  for(int i=0; i<msg->bounding_boxes.size();i++)
  {
    if (msg->bounding_boxes[i].Class=="bottle")
    {
    percep_msg.x = (msg->bounding_boxes[i].xmin + msg->bounding_boxes[i].xmax)/2;
    percep_msg.y = (msg->bounding_boxes[i].ymin + msg->bounding_boxes[i].ymax)/2;
    percep_msg.height = (msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin);
    percep_msg.width = (msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin);
    pub.publish(percep_msg);
    }
  }
  
}



int main(int argc,char **argv)
{
  
  ros::init(argc,argv,"object");
  ros::NodeHandle n;
  ObjectRecognition node(n);

  ros::spin();
  

  
  return 0;
}


