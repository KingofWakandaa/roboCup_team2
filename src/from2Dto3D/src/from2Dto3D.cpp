#include <from2Dto3D/from2Dto3D.h>
#include <tf/transform_listener.h>

      // Subscribes to and advertises topics
From2Dto3D::From2Dto3D(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
{

//#>>>>TODO: Set publishers and subscribers.

// subscribers to the bounding boxes and the point cloud
// format:
// sub_name = nh_.subscribe<Type>("topic", queuesize, Function_of_the_class, this);
sub_pc_=nh_.subscribe<sensor_msgs::PointCloud2>("/xtion/depth_registered/points",1,&From2Dto3D::processCloud,this);
sub_rec_=nh_.subscribe<perception_msgs::Rect>("rect",1,&From2Dto3D::processRect,this);
pub_pc_ = nh_.advertise<geometry_msgs::PointStamped>("segmentation/Point3D",100);
// Publishers
// format:
//pub_name = nh_.advertise< Type >("topic", queuesize);
c_x = 0;
c_y = 0;
position = 0;
position_al =0;
o_x = 0;
o_y = 0;
ROS_INFO("from2Dto3D initialized ...");

}

void From2Dto3D::processCloud(const sensor_msgs::PointCloud2ConstPtr& pc)
{    
    //#>>>>TODO: store local data copy or shared, depending on the message
    ROS_INFO("########################################### Process Cloud ###################################");
    pcl::fromROSMsg(*pc,pointcloud);
    position = pointcloud.width*c_y + c_x;
    position_al = pointcloud.width*o_y + o_x;
    if (isnan(pointcloud.points[position].x)||isnan(pointcloud.points[position].y)||isnan(pointcloud.points[position].z))
    {
    msg.point.x = pointcloud.points[position_al].x;
    msg.point.y = pointcloud.points[position_al].y;
    msg.point.z = pointcloud.points[position_al].z;
    }
    else
    {
    msg.point.x = pointcloud.points[position].x;
    msg.point.y = pointcloud.points[position].y;
    msg.point.z = pointcloud.points[position].z;}
	

    ROS_INFO("Starting transform...");
    msg.header.frame_id = "xtion_rgb_optical_frame";
    msg.header.stamp = ros::Time();

    try{
      listener_.transformPoint("base_footprint", msg, base_point);

      ROS_INFO("image: (%.2f, %.2f. %.2f) -----> base_footprint: (%.2f, %.2f, %.2f) at time %.2f",
          msg.point.x, msg.point.y, msg.point.z,
          base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
      }
      catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"segmentation/Point3D\" to \"base_footprint\": %s", ex.what());
      }

    //onMouse(msg);

    
}


void From2Dto3D::processRect(const perception_msgs::RectConstPtr& r)
{
    ROS_INFO("########################################### Process Rect ###################################");
    //#>>>>TODO: process bounding box and send 3D position to the topic
    // tip: take a look at the pcl::PointXYZRGB structure
    c_x = r->x;
    c_y = r->y;
    o_x = r->x - r->width/2;
    o_y = r->y - r->height/2;
    
    ROS_INFO_STREAM("Publishing transformed data to topic segmentation/Point3D");
    pub_pc_.publish(base_point);
}

//void From2Dto3D::transformPoint(const tf::TransformListener& listener_)
//{
//    ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
//    msg.header.frame_id = "xtion_rgb_optical_frame";
//    msg.header.stamp = ros::Time();
//
//    try{
//      listener_.transformPoint("base_link", msg, base_point);

//      ROS_INFO("image: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
//          msg.point.x, msg.point.y, msg.point.z,
//          base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
//      }
//      catch(tf::TransformException& ex){
//        ROS_ERROR("Received an exception trying to transform a point from \"segmentation/Point3D\" to \"base_link\": %s", ex.what());
//      }
//}


// OpenCV callback function for mouse events on a window
void onMouse(geometry_msgs::PointStamped msg)
{
  //build the action goal
  control_msgs::PointHeadGoal goal;
  //the goal consists in making the Z axis of the cameraFrame to point towards the pointStamped
  goal.pointing_frame = "/xtion_rgb_optical_frame";
  goal.pointing_axis.x = 0.0;
  goal.pointing_axis.y = 0.0;
  goal.pointing_axis.z = 1.0;
  goal.min_duration = ros::Duration(1.0);
  goal.max_velocity = 0.25;
  goal.target = msg;

  pointHeadClient->sendGoal(goal);
  ros::Duration(0.5).sleep();
}

// Create a ROS action client to move TIAGo's head
void createPointHeadClient(PointHeadClientPtr& actionClient)
{
  ROS_INFO("Creating action client to head controller ...");

  actionClient.reset( new PointHeadClient("/head_controller/point_head_action") );

  int iterations = 0, max_iterations = 3;
  // Wait for head controller action server to come up
  while( !actionClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations )
  {
    ROS_DEBUG("Waiting for the point_head_action server to come up");
    ++iterations;
  }

  if ( iterations == max_iterations )
    throw std::runtime_error("Error in createPointHeadClient: head controller action server not available");
}





