#include <grasp/Arm_class.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "takethings");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  Arm_class Arm_tiago(nh);
  //ros::ServiceClient client_gripper=nh.serviceClient<std_srvs::Empty>("/gripper_controller/grasp");
  //ros::Subscriber sub_goal_pos = nh.subscribe("segmentation/Point3D",100,
  //                                             &Arm_class::getPose,
  //                                             &Arm_tiago);
  const double SAFE_DIST = 0.3; //the place of bottle to the edge of the table should be less than SAFE_DIST
  const double GRIPPER_LEN = 0.15; //the length of the tiago gripper
  std_srvs::Empty empty_msg;
  //****************** motion begins ***************//
  //////////////////////////////////////////////////////////////
  //////////move to a near position with same y and z///////////
  //////////////////////////////////////////////////////////////
  ros::WallDuration(1.0).sleep();
  Arm_tiago.gotoTarget(SAFE_DIST,0,0);
  ros::WallDuration(2.0).sleep();
  //////////////////////////////////////////////////////////////
  /////////////move to the target position//////////////////////
  //////////////////////////////////////////////////////////////
  ros::WallDuration(1.0).sleep();
  Arm_tiago.gotoTarget(GRIPPER_LEN,0,0);
  ros::WallDuration(2.0).sleep();
  //////////////////////////////////////////////////////////////
  ///////////////////////////// grasp  /////////////////////////
  //////////////////////////////////////////////////////////////
  if(Arm_tiago.client_gripper.call(empty_msg))
  {
      //PLOT THE MESSAGE
      ROS_INFO_STREAM("call '/gripper_controller/grasp' successfully") ;
  }
  else
  {
      ROS_ERROR_STREAM("Failed to call the service '/gripper_controller/grasp'");
  }
  //////////////////////////////////////////////////////////////
  ////////////////////////  homing     /////////////////////////
  //////////////////////////////////////////////////////////////
  Arm_tiago.home();
  //*********************** motion end ***********************//
  spinner.stop();
  return EXIT_SUCCESS;
}
