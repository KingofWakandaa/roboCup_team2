#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>//????????
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <actionlib/client/simple_action_client.h>
#include <play_motion_msgs/PlayMotionAction.h>


// Std C++ headers
#include <string>
#include <vector>
#include <map>

class Arm_class
{
  public:
    ros::ServiceClient client_gripper;
    ros::Subscriber sub_goal_pos;

    pthread_mutex_t count_mutex;
   
    geometry_msgs::PointStamped pointstamped_local;//the recieved data
    geometry_msgs::PoseStamped goal_pose;//the goal position
   

    play_motion_msgs::PlayMotionGoal goal;

    Arm_class(ros::NodeHandle nh);
    void getPose(const geometry_msgs::PointStampedConstPtr &pose);//CREATE A CALLBACK FUNCTION FOR THE TOPIC  position
    void gotoTarget(double delta_x,double delta_y, double delta_z);
    void home();
};
