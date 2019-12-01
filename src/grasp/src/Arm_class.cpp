#include "grasp/Arm_class.h"
Arm_class::Arm_class(ros::NodeHandle nh)
{
  //pointstamped_local.header.stamp = pose->header.stamp;
  pointstamped_local.header.frame_id = "base_footprint";
  //pointstamped_local.point.x = 0;
  //pointstamped_local.point.y = 0;
  //pointstamped_local.point.z = 0;

  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);

  client_gripper=nh.serviceClient<std_srvs::Empty>("/gripper_controller/grasp");
  sub_goal_pos = nh.subscribe("segmentation/Point3D",100,
                                               &Arm_class::getPose,
                                               this);

  goal.motion_name = "home";
  goal.skip_planning = false;
  goal.priority = 0;
}

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

void Arm_class::gotoTarget(double delta_x,double delta_y, double delta_z)
{
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  group_arm_torso.setPlannerId("SBLkConfigDefault");//choose the planner
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(0.3);
  group_arm_torso.setPlanningTime(10.0);//set maximum time to find a plan

  goal_pose.header.stamp = pointstamped_local.header.stamp;
  goal_pose.header.frame_id = pointstamped_local.header.frame_id;
  goal_pose.pose.position.x = pointstamped_local.point.x - delta_x;
  goal_pose.pose.position.y = pointstamped_local.point.y - delta_y;
  goal_pose.pose.position.z = pointstamped_local.point.z - delta_z;

  ROS_INFO_STREAM("the frame is:"<<goal_pose.header.frame_id);
  ROS_INFO_STREAM("Point x : " << goal_pose.pose.position.x);
  ROS_INFO_STREAM("Point y : " << goal_pose.pose.position.y);
  ROS_INFO_STREAM("Point z : " << goal_pose.pose.position.z);
  //select group of joints
  group_arm_torso.setPoseTarget(goal_pose);//give the position
  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());
  //set the plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = bool(group_arm_torso.plan(my_plan));
  if ( !success )
    throw std::runtime_error("No plan found");
  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");
  // Execute the plan
  ros::Time start = ros::Time::now();
  group_arm_torso.move();
  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
}

void Arm_class::home()
{
  actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> client("/play_motion", true);
  ROS_INFO("Waiting for Action Server ...");
  client.waitForServer();
  client.sendGoal(goal);
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
}
