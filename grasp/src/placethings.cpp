#include <grasp/Arm_class.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "placethings");

  const double SAFE_DIST_X = 0.05; //in oder to not touch the table
  const double SAFE_DIST_Z = 0.3; //in oder to not go from bottom to up

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  Arm_class Arm_tiago_2(nh);
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
  geometry_msgs::PointStamped pointstamped;
  double input_value[3] = {0.6,-0.06,0.82};//the place we need to place the bottle
  pointstamped.header.frame_id = "base_footprint";
  pointstamped.point.x = input_value[0];
  pointstamped.point.y = input_value[1];
  pointstamped.point.z = input_value[2];
  Arm_tiago_2.gotoTarget(pointstamped,SAFE_DIST_X,0,-SAFE_DIST_Z);
  ros::WallDuration(2.0).sleep();
  //////////////////////////////////////////////////////////////
  //////////move to target position ////////////////////////////
  //////////////////////////////////////////////////////////////
  Arm_tiago_2.gotoTarget(pointstamped,0,0,0);
  ros::WallDuration(2.0).sleep();
  //////////////////////////////////////////////////////////////
  //////////  release the Gripper //////////////////////////////
  //////////////////////////////////////////////////////////////
  pub_release.publish(gripper_msg);
  ros::WallDuration(2.0).sleep();
  //////////////////////////////////////////////////////////////
  //////////  go back to a high position ///////////////////////
  //////////////////////////////////////////////////////////////
  Arm_tiago_2.gotoTarget(pointstamped,0,0,-SAFE_DIST_Z);
  ros::WallDuration(2.0).sleep();
  //////////////////////////////////////////////////////////////
  ////////////////////  go home position ///////////////////////
  //////////////////////////////////////////////////////////////
  Arm_tiago_2.home();
  //************** motion end **************//
  spinner.stop();
  return EXIT_SUCCESS;
}
