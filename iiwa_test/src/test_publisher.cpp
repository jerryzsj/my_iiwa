#include "ros/ros.h"
#include <iiwa_ros.h>
#include <conversions.h>
#include <time_to_destination_service.h>
#include <iiwa_msgs/JointQuantity.h>
#include <iiwa_msgs/CartesianVelocity.h>
#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointStiffness.h>
#include <iiwa_msgs/DOF.h>
#include <iiwa_msgs/JointTorque.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_msgs/JointDamping.h>
#include <std_msgs/Time.h>
#include <smart_servo_service.h>
#include <iiwa_services.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>

//using namespace ros;
//using namespace iiwa_ros;
//using namespace iiwa_msgs;
using namespace std;

void waitingToDestination()
{
	
}

int main (int argc, char **argv) {
  
  //cerr<< argc <<endl<<argv <<endl;
  // Initialize ROS
  ros::init(argc, argv, "iiwa_test");
  ros::NodeHandle nh("my_iiwa");
	
  // ROS spinner.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  //init loop_rate, and other robot statement massages
	ros::Rate loop_rate(10);
	double my_time_to_destination=0;
  iiwa_msgs::JointPosition my_joint_position;
  geometry_msgs::PoseStamped my_cartesian_pose;
  iiwa_msgs::JointTorque my_joint_torque;
  iiwa_msgs::JointStiffness my_joint_stiffness;
  geometry_msgs::WrenchStamped my_cartesian_wrench;
  iiwa_msgs::JointVelocity my_joint_velocity;
  
  //init iiwa
  iiwa_ros::iiwaRos my_iiwa;
  my_iiwa.init_iiwa();
  ros::Duration(1.0).sleep();
  
  //to see if robot is connected
  if(my_iiwa.getRobotIsConnected())
  {
  	ROS_INFO_STREAM("Robot is connected");
  	
  	
  
  double a1,a2,a3,a4,a5,a6,a7;
  a1=a2=a3=a4=a5=a6=a7=0.0;
  
  my_joint_position.position = iiwa_ros::jointQuantityFromDouble (0.0);
  my_cartesian_pose=iiwa_ros::CartesianPoseFromDouble(0.2,0.2,0.8,0.0,0.0,0.0,1.0);
  ROS_INFO_STREAM("Go to Home position");
  
  my_iiwa.setJointPosition(my_joint_position);
  ros::Duration(1.0).sleep();
  my_time_to_destination=my_iiwa.getTimeToDestinationService().getTimeToDestination();
  ROS_INFO_STREAM(my_time_to_destination);
  while(my_time_to_destination>0)
  {
  	my_time_to_destination=my_iiwa.getTimeToDestinationService().getTimeToDestination();
  	ROS_INFO_STREAM(my_time_to_destination);
  	loop_rate.sleep();
  }
  
  ROS_INFO_STREAM("Go to all joint 1 position");
  a1=a2=a3=a4=a5=a6=a7=1.0;
  my_joint_velocity.velocity=iiwa_ros::jointQuantityFromDouble(0.1);
  //my_iiwa.setJointVelocity(my_joint_velocity);
  my_joint_position.position = iiwa_ros::jointQuantityFromDouble(a1, a2, a3, a4,a5, a6, a7);
  
  my_iiwa.setJointPosition(my_joint_position);
  ros::Duration(2.0).sleep();
  my_iiwa.getJointVelocity(my_joint_velocity);
  ROS_INFO_STREAM("i got JointVelocity is :" << my_joint_velocity);
  ros::Duration(1.0).sleep();
  my_iiwa.getJointVelocity(my_joint_velocity);
  ROS_INFO_STREAM("i got JointVelocity is :" << my_joint_velocity);
  ros::Duration(1.0).sleep();
  
  ROS_INFO_STREAM("Go to cartesian position");
  my_iiwa.setCartesianPose(my_cartesian_pose);
	ros::Duration(6.0).sleep();
	//my_iiwa.getSmartServoService().setDesiredForceMode(iiwa_msgs::DOF::Z, 2.0, 5);
	
	/*while(ros::ok())
  {
  	my_iiwa.getCartesianPose(my_cartesian_pose);
  	ROS_INFO_STREAM("i got cartesian pose is :" << my_cartesian_pose);
  	loop_rate.sleep();
  }*/
  }
  
  else
  {
  	ROS_INFO_STREAM("Robot not connected!!");
  }
  /*
  //--get robot statement
  //--joint position
  if(my_iiwa.getJointPosition(my_joint_position))
  	{ROS_INFO_STREAM("i got a joint message and its contests is :" << my_joint_position );}
  	
  //--cartesian pose, x y z, x y z w
  if(my_iiwa.getCartesianPose(my_cartesian_pose))
  	{ROS_INFO_STREAM("i got cartesian pose is :" << my_cartesian_pose);}
  
  //--joint torque
  if(my_iiwa.getJointTorque(my_joint_torque))
  	{ROS_INFO_STREAM("i got JointTorque is :" << my_joint_torque);}
  
  //--joint stiffness
  if(my_iiwa.getJointStiffness(my_joint_stiffness))
  	{ROS_INFO_STREAM("i got JointStiffness is :" << my_joint_stiffness);}
  
  //--cartesian wrench
  if(my_iiwa.getCartesianWrench(my_cartesian_wrench))
  	{ROS_INFO_STREAM("i got CartesianWrenchis :" << my_cartesian_wrench);}
  
  //--joint velocity
  if(my_iiwa.getJointVelocity(my_joint_velocity))
  	{ROS_INFO_STREAM("i got JointVelocity is :" << my_joint_velocity);}
  */
  
  
  //--set joint velocity
  //my_joint_velocity.velocity=iiwa_ros::jointQuantityFromDouble(0.1);
  //my_joint_velocity.velocity=iiwa_ros::jointQuantityFromDouble (0.1,0.2,0.3,0.4,0.5,0.6,0.7);
  //my_iiwa.setJointVelocity(my_joint_velocity);
  
  
  //--set joint position
  //my_joint_position.position = iiwa_ros::jointQuantityFromDouble (0.1,0.2,0.3,0.4,0.5,0.6,0.7);
  //my_joint_position.position = iiwa_ros::jointQuantityFromDouble (0.0);
  //my_iiwa.setJointPosition(my_joint_position);
  
  //--set cartesian pose, x y z, x y z w
  //my_cartesian_pose=iiwa_ros::CartesianPoseFromDouble(0.2,0.2,0.8,0.0,0.0,0.0,1.0);
  //my_iiwa.setCartesianPose(my_cartesian_pose);
  
  
  //--waiting to destination
  /*
  ros::Duration(1.0).sleep();
  my_time_to_destination=my_iiwa.getTimeToDestinationService().getTimeToDestination();
  while(my_time_to_destination>0)
  {
  	my_time_to_destination=my_iiwa.getTimeToDestinationService().getTimeToDestination();
  	//ROS_INFO_STREAM(my_time_to_destination);
  	loop_rate.sleep();
  }
  */
}; 
