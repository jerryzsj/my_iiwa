	
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
  
  
  
  //--set joint velocity
  my_joint_velocity.velocity=iiwa_ros::jointQuantityFromDouble(0.1);
  my_joint_velocity.velocity=iiwa_ros::jointQuantityFromDouble (0.1,0.2,0.3,0.4,0.5,0.6,0.7);
  my_iiwa.setJointVelocity(my_joint_velocity);
  
  
  //--set joint position
  my_joint_position.position = iiwa_ros::jointQuantityFromDouble (0.1,0.2,0.3,0.4,0.5,0.6,0.7);
  my_joint_position.position = iiwa_ros::jointQuantityFromDouble (0.0);
  my_iiwa.setJointPosition(my_joint_position);
  
  //--set cartesian pose, x y z, x y z w
  my_cartesian_pose=iiwa_ros::CartesianPoseFromDouble(0.2,0.2,0.8,0.0,0.0,0.0,1.0);
  my_iiwa.setCartesianPose(my_cartesian_pose);
  
  
  //--waiting to destination
  ros::Duration(1.0).sleep();
  my_time_to_destination=my_iiwa.getTimeToDestinationService().getTimeToDestination();
  while(my_time_to_destination>0)
  {
  	my_time_to_destination=my_iiwa.getTimeToDestinationService().getTimeToDestination();
  	//ROS_INFO_STREAM(my_time_to_destination);
  	loop_rate.sleep();
  }
  
  
  
  
