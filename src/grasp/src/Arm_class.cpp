#include "grasp/Arm_class.h"
Arm_class::Arm_class()
{
  //pointstamped_local.header.stamp = pose->header.stamp;
  pointstamped_local.header.frame_id = "base_footprint";
  pointstamped_local.point.x = 0;
  pointstamped_local.point.y = 0;
  pointstamped_local.point.z = 0;
}
//Arm_class::~Arm_class()
//{
//}
void Arm_class::getPose(const geometry_msgs::PointStampedConstPtr &pose)
{
  pthread_mutex_lock( &this->count_mutex );
  //COPY THE MSG TO A LOCAL VARIABLE
  pointstamped_local.header.stamp = pose->header.stamp;
  pointstamped_local.header.frame_id = pose->header.frame_id;
  pointstamped_local.point.x = pose->point.x;
  pointstamped_local.point.y = pose->point.y;
  pointstamped_local.point.z = pose->point.z;
  //orientation = pose->pose.orientation;
  pthread_mutex_unlock( &this->count_mutex );
}
