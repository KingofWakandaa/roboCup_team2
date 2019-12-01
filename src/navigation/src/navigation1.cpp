#include <navigation/navigationClass.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


navigationSpace::navigationClass::navigationClass()
{
    //INITIALIZE MEMBER VARIABLES
    count_mutex = PTHREAD_MUTEX_INITIALIZER;
    loc_acc = 1.0;
}

navigationSpace::navigationClass::~navigationClass()
{

}

void navigationSpace::navigationClass::getLocCov(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    pthread_mutex_lock( &this->count_mutex );
    //COPY THE MSG TO A LOCAL VARIABLE
    this->loc_acc=msg->pose.covariance[0]+msg->pose.covariance[7];
    pthread_mutex_unlock( &this->count_mutex );
}

void navigationSpace::navigationClass::arm_move(moveit::planning_interface::MoveGroupInterface& group_arm_torso, double *pos)
{
  //set parameter
  std::map<std::string, double> target_position;
  target_position["torso_lift_joint"] = pos[0];
  target_position["arm_1_joint"] = pos[1];
  target_position["arm_2_joint"] = pos[2];
  target_position["arm_3_joint"] = pos[3];
  target_position["arm_4_joint"] = pos[4];
  target_position["arm_5_joint"] = pos[5];
  target_position["arm_6_joint"] = pos[6];
  target_position["arm_7_joint"] = pos[7];

  //choose planer
  ROS_INFO("Choose planner");
  group_arm_torso.setPlannerId("SBLkConfigDefault");

  //set target
  std::vector<std::string> torso_arm_joint_names;
  torso_arm_joint_names = group_arm_torso.getJoints();

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  for (unsigned int i = 0; i < torso_arm_joint_names.size(); ++i)
  {
    if ( target_position.count(torso_arm_joint_names[i]) > 0 )
    {
      ROS_INFO_STREAM("\t" << torso_arm_joint_names[i] << " goal position: " << target_position[torso_arm_joint_names[i]]);
      group_arm_torso.setJointValueTarget(torso_arm_joint_names[i], target_position[torso_arm_joint_names[i]]);
    }
  }

  //set plan
  ROS_INFO("Set plan");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //set maximum time to find a plan
  group_arm_torso.setPlanningTime(5.0);
  bool success = group_arm_torso.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  if ( !success )
  {
    ROS_INFO("No plan found");
    return;
  }

  ROS_INFO("Move start");
  group_arm_torso.move();
  ROS_INFO("Move end");
}


//##################################################################################################################

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigation1");
  ros::NodeHandle n;
  ros::Duration(15).sleep();
  ros::Rate r(60);

  //prepare for localization
  navigationSpace::navigationClass navigationF;

  ros::Subscriber loc_sub = n.subscribe("/amcl_pose",100,
                                      &navigationSpace::navigationClass::getLocCov,
                                      &navigationF);

  ros::Publisher key_pub=n.advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel",100);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  geometry_msgs::Twist msg;
  move_base_msgs::MoveBaseGoal goal;
  std_srvs::Empty srv;

  //start localization
  ROS_INFO("Localization start");
  ros::service::call("/global_localization", srv);

  //set rotation
  msg.angular.z = 1;
  /*goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose.orientation.z = 2;
  ros::Duration(0.5).sleep();

  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Initialize position");
  else
  {
    ROS_INFO("Initialize failed");
  }*/
  

  ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
  //rotate until localization is done
  while(navigationF.loc_acc>0.01)
  {
    key_pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }

  ros::service::call("/move_base/clear_costmaps", srv);
  ROS_INFO("Localization end");

  //start move
  //ros::Duration(2).sleep();
  ROS_INFO("The first navigation start");
  //reference frame is map
  goal.target_pose.header.frame_id = "map";

  //move to the table
  ROS_INFO("Move to the table");
  //goal.target_pose.header.stamp = ros::Time::now();
  //set goal as 2D coordinate
  goal.target_pose.pose.position.x = 3.02521014214;
  goal.target_pose.pose.position.y = -1.65591907501;
  //set goal orientation
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = -0.755716192043;
  goal.target_pose.pose.orientation.w = 0.654899257202;
  ac.sendGoal(goal);
  ac.waitForResult();






















}
