#include <grasp/Arm_class.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takethings");
  
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //****************** Definition **************//
  Arm_class Arm_tiago;
  
  ros::ServiceClient client_gripper=nh.serviceClient<std_srvs::Empty>("/gripper_controller/grasp");
  ros::Subscriber sub_goal_pos = nh.subscribe("segmentation/Point3D",100,
                                               &Arm_class::getPose,
                                               &Arm_tiago);
  std_srvs::Empty empty_msg;
  geometry_msgs::PoseStamped goal_pose;
  //the distance between table and tiago must be more than MIN_DIST_TIAGO_TABEL
  const double SAFE_DIST = 0.3; //the place of bottle to the edge of the table should be less than SAFE_DIST
  const double GRIPPER_LEN = 0.15; //the length of the tiago gripper


  //****************** Arm motion ***************//
  //////////////////////////////////////////////////////////////
  //////////move to a near position with same y and z///////////
  //////////////////////////////////////////////////////////////
  ros::WallDuration(1.0).sleep();
  goal_pose.header.stamp = Arm_tiago.pointstamped_local.header.stamp;
  goal_pose.header.frame_id = Arm_tiago.pointstamped_local.header.frame_id;
  ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
  ROS_INFO_STREAM(goal_pose.header.frame_id);
  ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");

  goal_pose.pose.position.x = Arm_tiago.pointstamped_local.point.x - SAFE_DIST;
  goal_pose.pose.position.y = Arm_tiago.pointstamped_local.point.y;
  goal_pose.pose.position.z = Arm_tiago.pointstamped_local.point.z;
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57, 0, 0);
  ROS_INFO_STREAM("Get Point x : " << goal_pose.pose.position.x);
  ROS_INFO_STREAM("Get Point y : " << goal_pose.pose.position.y);
  ROS_INFO_STREAM("Get Point z : " << goal_pose.pose.position.z);
  ROS_INFO_STREAM("Get Point x : " << Arm_tiago.pointstamped_local.point.x);
  ROS_INFO_STREAM("Get Point y : " << Arm_tiago.pointstamped_local.point.y);
  ROS_INFO_STREAM("Get Point z : " << Arm_tiago.pointstamped_local.point.z);

  //select group of joints
  moveit::planning_interface::MoveGroupInterface group_arm_torso("arm_torso");
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(0.3);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm_torso.setPlanningTime(10.0);//set maximum time to find a plan
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
  //////////move to bottle position with same y and z///////////
  //////////////////////////////////////////////////////////////
  goal_pose.pose.position.x = Arm_tiago.pointstamped_local.point.x - GRIPPER_LEN;
  goal_pose.pose.position.y = Arm_tiago.pointstamped_local.point.y;
  goal_pose.pose.position.z = Arm_tiago.pointstamped_local.point.z - 0.05;
  ROS_INFO_STREAM("Get Point2 x : " << goal_pose.pose.position.x);
  ROS_INFO_STREAM("Get Point2 y : " << goal_pose.pose.position.y);
  ROS_INFO_STREAM("Get Point2 z : " << goal_pose.pose.position.z);
  ROS_INFO_STREAM("Get Point2 x : " << Arm_tiago.pointstamped_local.point.x);
  ROS_INFO_STREAM("Get Point2 y : " << Arm_tiago.pointstamped_local.point.y);
  ROS_INFO_STREAM("Get Point2 z : " << Arm_tiago.pointstamped_local.point.z);
  group_arm_torso.setPoseTarget(goal_pose);
  //plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  group_arm_torso.setPlanningTime(5.0);//set maximum time to find a plan
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
  //*********************** Arm motion ********************//

  //*********************** Graps ***********************//
  if(client_gripper.call(empty_msg))
  {
      //PLOT THE MESSAGE
      ROS_INFO_STREAM("call '/gripper_controller/grasp' successfully") ;
  }
  else
  {
      ROS_ERROR_STREAM("Failed to call the service '/gripper_controller/grasp'");
  }
  //*********************** Graps ***********************//

  //*********************** Homing ***********************//
  actionlib::SimpleActionClient<play_motion_msgs::PlayMotionAction> client("/play_motion", true);

  ROS_INFO("Waiting for Action Server ...");
  client.waitForServer();

  play_motion_msgs::PlayMotionGoal goal;

  goal.motion_name = "home";
  goal.skip_planning = false;
  goal.priority = 0;

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
  //*********************** Homing ***********************//
  spinner.stop();
  return EXIT_SUCCESS;
}
