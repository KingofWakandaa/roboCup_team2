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
#include <trajectory_msgs/JointTrajectory.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "placethings");
  /*if ( argc < 4 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun tiago_moveit_tutorial plan_arm_torso_ik  x y z");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments specify the target pose of /arm_tool_link expressed in /base_footprint");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }*/
  const double SAFE_DIST_X = 0.05; //in oder to not touch the table
  const double SAFE_DIST_Z = 0.3; //in oder to not go from bottom to up

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //define the publisher for release gripper and massage
  ros::Publisher pub_release=nh.advertise<trajectory_msgs::JointTrajectory>("/parallel_gripper_controller/command",100);//publisher for gripper control
  trajectory_msgs::JointTrajectory gripper_msg;
  gripper_msg.joint_names.push_back("parallel_gripper_joint");
  gripper_msg.points.resize(1);
  gripper_msg.points[0].positions.push_back(0.1);
  gripper_msg.points[0].time_from_start = ros::Duration(1);

  //****************** motion begin ***************//
  //////////////////////////////////////////////////////////////
  //////////move to a higher position with same y //////////////
  //////////////////////////////////////////////////////////////
  geometry_msgs::PoseStamped goal_pose;
  double input_value[3] = {0.6,-0.06,0.82};//the place we need to place the bottle
  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position.x = input_value[0]-SAFE_DIST_X;
  goal_pose.pose.position.y = input_value[1];
  goal_pose.pose.position.z = input_value[2]+SAFE_DIST_Z;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57,0,0);
  //print the goal position
  ROS_INFO_STREAM("High Point x : " << goal_pose.pose.position.x);
  ROS_INFO_STREAM("High Point y : " << goal_pose.pose.position.y);
  ROS_INFO_STREAM("High Point z : " << goal_pose.pose.position.z);
  //select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  //choose preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);//give target
  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(0.4);
    group_arm_torso.setPlanningTime(10.0);//set maximum time to find a plan
  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());
  // set a plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = bool(group_arm_torso.plan(my_plan));
  if ( !success )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
  // Execute the plan
  ros::Time start = ros::Time::now();
  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  ros::WallDuration(2.0).sleep();
  //////////////////////////////////////////////////////////////
  //////////move to target position ////////////////////////////
  //////////////////////////////////////////////////////////////
  goal_pose.pose.position.x = input_value[0];
  //goal_pose.pose.position.y = input_value[1];
  goal_pose.pose.position.z = input_value[2];
  ROS_INFO_STREAM("Place Point x : " << goal_pose.pose.position.x);
  ROS_INFO_STREAM("Place Point y : " << goal_pose.pose.position.y);
  ROS_INFO_STREAM("Place Point z : " << goal_pose.pose.position.z);
  group_arm_torso.setPoseTarget(goal_pose);
  //set a plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  success = bool(group_arm_torso.plan(my_plan2));
  if ( !success )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << my_plan2.planning_time_ << " seconds");
  // Execute the plan
  start = ros::Time::now();
  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  // Wait a bit for ROS things to initialize
  ros::WallDuration(2.0).sleep();
  //////////////////////////////////////////////////////////////
  //////////  release the Gripper //////////////////////////////
  //////////////////////////////////////////////////////////////
  pub_release.publish(gripper_msg);
  ros::WallDuration(2.0).sleep();
  //////////////////////////////////////////////////////////////
  //////////  go back to a high position ///////////////////////
  //////////////////////////////////////////////////////////////
  //goal_pose.pose.position.x = input_value[0];
  //goal_pose.pose.position.y = input_value[1];
  goal_pose.pose.position.z = input_value[2]+SAFE_DIST_Z;
  ROS_INFO_STREAM("High Point2 x : " << goal_pose.pose.position.x);
  ROS_INFO_STREAM("High Point2 y : " << goal_pose.pose.position.y);
  ROS_INFO_STREAM("High Point2 z : " << goal_pose.pose.position.z);
  group_arm_torso.setPoseTarget(goal_pose);
  //plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
  success = bool(group_arm_torso.plan(my_plan3));
  if ( !success )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << my_plan3.planning_time_ << " seconds");
  // Execute the plan
  start = ros::Time::now();
  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
  ros::WallDuration(2.0).sleep();
  //////////////////////////////////////////////////////////////
  ////////////////////  go home position ///////////////////////
  //////////////////////////////////////////////////////////////
  actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> client("/play_motion", true);
  ROS_INFO("Waiting for Action Server ...");
  client.waitForServer();
  play_motion_msgs::PlayMotionGoal goal;
  goal.motion_name = "home";
  goal.skip_planning = false;
  goal.priority = 0;
  client.sendGoal(goal);//call service through "play_motion"
  ROS_INFO("Waiting for result ...");
  bool actionOk = client.waitForResult(ros::Duration(30.0));
  actionlib::SimpleClientGoalState state = client.getState();
  if ( actionOk )
  {
      ROS_INFO_STREAM("Action finished successfully with state: " << state.toString());
  }
  else
  {
      ROS_ERROR_STREAM("Action failed with state: " << state.toString());
  }
  //************** motion end **************//
  spinner.stop();
  return EXIT_SUCCESS;
}
