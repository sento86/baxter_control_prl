/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, CU Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of CU Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 * \brief   Helper functions for controlling baxter
 * \author  Vicent Girbes Juan
 */

//#include <baxter_control/baxter_utilities.h>
#include <baxter_pick_place/baxter_utilities.h>

namespace baxter_control
{

BaxterUtilities::BaxterUtilities()
{
  ros::NodeHandle nh;

  // Preload Messages
  disable_msg_.data = false;
  enable_msg_.data = true;

  // ---------------------------------------------------------------------------------------------
  // Advertise services
  pub_baxter_enable_ = nh.advertise<std_msgs::Bool>("/robot/set_super_enable",10);
  pub_baxter_reset_ = nh.advertise<std_msgs::Empty>("/robot/set_super_reset",10);

  pub_left_end_effector_ = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command",10);
  pub_right_end_effector_ = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",10);

  pub_left_cmd_ = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);
  pub_right_cmd_ = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);

  //pub_left_action_cmd_ = nh.advertise<control_msgs::JointTrajectoryActionGoal>("/robot/limb/left/joint_trajectory_action/goal", 1);
  //pub_right_action_cmd_ = nh.advertise<control_msgs::JointTrajectoryActionGoal>("/robot/limb/right/joint_trajectory_action/goal", 1);

  //pub_left_action_cmd_ = nh.advertise<control_msgs::FollowJointTrajectoryAction>("/robot/limb/left/follow_joint_trajectory/goal", 1);
  //pub_right_action_cmd_ = nh.advertise<control_msgs::FollowJointTrajectoryAction>("/robot/limb/right/follow_joint_trajectory/goal", 1);

  left_action_client_ = new joint_action_client("robot/limb/left/follow_joint_trajectory",true);
  right_action_client_ = new joint_action_client("robot/limb/right/follow_joint_trajectory",true);

  left_action_client_->waitForServer();
  right_action_client_->waitForServer();

  // ---------------------------------------------------------------------------------------------
  // Start the state subscriber
  //sub_baxter_state_ = nh.subscribe<baxter_msgs::AssemblyState>(BAXTER_STATE_TOPIC, 1, &BaxterUtilities::stateCallback, this);
  sub_baxter_state_ = nh.subscribe<baxter_core_msgs::AssemblyState>(BAXTER_STATE_TOPIC, 1, &BaxterUtilities::stateCallback, this);

  sub_left_gripper_state_ = nh.subscribe<baxter_core_msgs::EndEffectorState>(BAXTER_LEFT_GRIPPER_TOPIC, 1, &BaxterUtilities::leftGripperCallback, this);
  sub_right_gripper_state_ = nh.subscribe<baxter_core_msgs::EndEffectorState>(BAXTER_RIGHT_GRIPPER_TOPIC, 1, &BaxterUtilities::rightGripperCallback, this);
  sub_left_endpoint_state_ = nh.subscribe<baxter_core_msgs::EndpointState>(BAXTER_LEFT_ENDPOINT_TOPIC, 1, &BaxterUtilities::leftEndpointCallback, this);
  sub_right_endpoint_state_ = nh.subscribe<baxter_core_msgs::EndpointState>(BAXTER_RIGHT_ENDPOINT_TOPIC, 1, &BaxterUtilities::rightEndpointCallback, this);
  sub_joint_state_ = nh.subscribe<sensor_msgs::JointState>(BAXTER_JOINT_STATE_TOPIC, 1, &BaxterUtilities::jointCallback, this);

  srv_left_ik_ = nh.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/left/PositionKinematicsNode/IKService");
  srv_right_ik_ = nh.serviceClient<baxter_core_msgs::SolvePositionIK>("/ExternalTools/right/PositionKinematicsNode/IKService");

  // ---------------------------------------------------------------------------------------------
  // command in effort/velocity/position mode
  //left_msg_.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
  //left_msg_.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
  left_msg_.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

  // command joints in the order shown in baxter_interface
  left_msg_.names.push_back("left_s0");
  left_msg_.names.push_back("left_s1");
  left_msg_.names.push_back("left_e0");
  left_msg_.names.push_back("left_e1");
  left_msg_.names.push_back("left_w0");
  left_msg_.names.push_back("left_w1");
  left_msg_.names.push_back("left_w2");
  left_msg_.names.push_back("left_measure_joint");

  left_msg_.command.resize(left_msg_.names.size());

  left_msg_.command[0] = 0.0;
  left_msg_.command[1] = 0.0;
  left_msg_.command[2] = -M_PI/2;
  left_msg_.command[3] = M_PI/2;
  left_msg_.command[4] = M_PI/2;
  left_msg_.command[5] = M_PI/2;
  left_msg_.command[6] = 0.0;

  left_initial_msg_=left_msg_;

  for(int i=0;i<left_msg_.names.size();i++)
	  left_joints_.name.push_back(left_msg_.names[i]);

  left_joints_.position.resize(left_joints_.name.size());

  // Action
  for(int i=0;i<left_msg_.names.size();i++)
	  left_action_msg_.trajectory.joint_names.push_back(left_msg_.names[i]);

  trajectory_msgs::JointTrajectoryPoint left_points_;
  for(int i=0;i<left_msg_.command.size();i++)
	  left_points_.positions.push_back(left_msg_.command[i]);

  left_action_msg_.trajectory.points.push_back(left_points_);

  // ---------------------------------------------------------------------------------------------
  // command in effort/velocity/position mode
  //right_msg_.mode = baxter_core_msgs::JointCommand::TORQUE_MODE;
  //right_msg_.mode = baxter_core_msgs::JointCommand::VELOCITY_MODE;
  right_msg_.mode = baxter_core_msgs::JointCommand::POSITION_MODE;

  // command joints in the order shown in baxter_interface
  right_msg_.names.push_back("right_s0");
  right_msg_.names.push_back("right_s1");
  right_msg_.names.push_back("right_e0");
  right_msg_.names.push_back("right_e1");
  right_msg_.names.push_back("right_w0");
  right_msg_.names.push_back("right_w1");
  right_msg_.names.push_back("right_w2");
  right_msg_.names.push_back("right_measure_joint");

  right_msg_.command.resize(right_msg_.names.size());

  right_msg_.command[0] = 0.0;
  right_msg_.command[1] = 0.0;
  right_msg_.command[2] = M_PI/2;
  right_msg_.command[3] = M_PI/2;
  right_msg_.command[4] = -M_PI/2;
  right_msg_.command[5] = M_PI/2;
  right_msg_.command[6] = 0.0;

  right_initial_msg_=right_msg_;

  for(int i=0;i<right_msg_.names.size();i++)
	  right_joints_.name.push_back(right_msg_.names[i]);

  right_joints_.position.resize(right_joints_.name.size());

 // Action
  for(int i=0;i<right_msg_.names.size();i++)
	  right_action_msg_.trajectory.joint_names.push_back(right_msg_.names[i]);

  trajectory_msgs::JointTrajectoryPoint right_points_;
  for(int i=0;i<right_msg_.command.size();i++)
	  right_points_.positions.push_back(right_msg_.command[i]);
  right_points_.time_from_start=ros::Duration(5.0);
  right_action_msg_.trajectory.points.push_back(right_points_);

  for(int i=0;i<right_msg_.command.size();i++)
	  right_points_.positions.push_back(0.75*right_msg_.command[i]);
  right_points_.time_from_start=ros::Duration(3.0);
  right_action_msg_.trajectory.points.push_back(right_points_);

  for(int i=0;i<right_msg_.command.size();i++)
	  right_points_.positions.push_back(1.25*right_msg_.command[i]);
  right_points_.time_from_start=ros::Duration(8.0);
  right_action_msg_.trajectory.points.push_back(right_points_);

  // ---------------------------------------------------------------------------------------------
}

bool BaxterUtilities::communicationActive()
{
  int count = 0;
  while( ros::ok() && baxter_state_timestamp_.toSec() == 0 )
  {
    if( count > 40 ) // 40 is an arbitrary number for when to assume no state is being published
    {
      ROS_WARN_STREAM_NAMED("utilities","No state message has been recieved on topic "
        << BAXTER_STATE_TOPIC);
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }

  // Check that the message timestamp is no older than 1 second
  if(ros::Time::now() > baxter_state_timestamp_ + ros::Duration(1.0))
  {
    ROS_ERROR_STREAM_NAMED("utilities","Baxter state expired. State: \n" << *baxter_state_ );
    return false;
  }

  return true;
}

bool BaxterUtilities::isEnabled(bool verbose)
{
  // Check communication
  if( !communicationActive() )
  {
    // Error message aready outputed
    return false;
  }

  // Check for estop
  if( baxter_state_->stopped == true )
  {
    // Skip the switch statments if we are not wanting verbose output
    if(!verbose)
      return false;

    std::string estop_button;
    switch( baxter_state_->estop_button )
    {
    //case baxter_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED:
    case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNPRESSED:
      estop_button = "Robot is not stopped and button is not pressed";
      break;
    //case baxter_msgs::AssemblyState::ESTOP_BUTTON_PRESSED:
    case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_PRESSED:
      estop_button = "Pressed";
      break;
    //case baxter_msgs::AssemblyState::ESTOP_BUTTON_UNKNOWN:
    case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_UNKNOWN:
      estop_button = "STATE_UNKNOWN when estop was asserted by a non-user source";
      break;
    //case baxter_msgs::AssemblyState::ESTOP_BUTTON_RELEASED:
    case baxter_core_msgs::AssemblyState::ESTOP_BUTTON_RELEASED:
      estop_button = "Was pressed, is now known to be released, but robot is still stopped.";
      break;
    default:
      estop_button = "Unkown button state code";
    }

    std::string estop_source;
    switch( baxter_state_->estop_source )
    {
    //case baxter_msgs::AssemblyState::ESTOP_SOURCE_NONE:
    case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_NONE:
      estop_source = "e-stop is not asserted";
      break;
    //case baxter_msgs::AssemblyState::ESTOP_SOURCE_USER:
    case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_USER:
      estop_source = "e-stop source is user input (the red button)";
      break;
    //case baxter_msgs::AssemblyState::ESTOP_SOURCE_UNKNOWN:
    case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_UNKNOWN:
      estop_source = "e-stop source is unknown";
      break;
    //case baxter_msgs::AssemblyState::ESTOP_SOURCE_FAULT:
    case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_FAULT:
      estop_source = "MotorController asserted e-stop in response to a joint fault";
      break;
    //case baxter_msgs::AssemblyState::ESTOP_SOURCE_BRAIN:
    case baxter_core_msgs::AssemblyState::ESTOP_SOURCE_BRAIN:
      estop_source = "MotorController asserted e-stop in response to a lapse of the brain heartbeat";
      break;
    default:
      estop_source = "Unkown button source code";

    }

    ROS_ERROR_STREAM_NAMED("utilities","ESTOP Button State: '" << estop_button << "'. Source: '" << estop_source << "'");
    return false;
  }

  // Check for error
  if( baxter_state_->error == true )
  {
    if(verbose)
      ROS_ERROR_STREAM_NAMED("utilities","Baxter has an error :(  State: \n" << *baxter_state_ );
    return false;
  }

  // Check enabled
  if( baxter_state_->enabled == false )
  {
    if(verbose)
      ROS_ERROR_STREAM_NAMED("utilities","Baxter is not enabled.  State: \n" << *baxter_state_ );
    return false;
  }

  return true;
}

//void BaxterUtilities::stateCallback(const baxter_msgs::AssemblyStateConstPtr& msg)
void BaxterUtilities::stateCallback(const baxter_core_msgs::AssemblyStateConstPtr& msg)
{
  baxter_state_ = msg;
  baxter_state_timestamp_ = ros::Time::now();

  /*
  // Check for errors every 50 refreshes
  static std::size_t counter = 0;
  counter++;
  if( counter % 50 == 0 )
  {
  if( !isEnabled() )
  {
  ROS_ERROR_STREAM_THROTTLE_NAMED(2,"utility","Baxter in error state.");
  }
  }
  // Reset counter when it reaches max value
  if( counter >= (size_t)-1 )
  counter = 0;
  */
}

void BaxterUtilities::leftGripperCallback(const baxter_core_msgs::EndEffectorStateConstPtr& msg)
{
  left_gripper_state_ = msg;
  left_gripper_state_timestamp_ = ros::Time::now();
}

void BaxterUtilities::rightGripperCallback(const baxter_core_msgs::EndEffectorStateConstPtr& msg)
{
  right_gripper_state_ = msg;
  right_gripper_state_timestamp_ = ros::Time::now();
}

void BaxterUtilities::leftEndpointCallback(const baxter_core_msgs::EndpointStateConstPtr& msg)
{
  left_endpoint_state_ = msg;
  left_endpoint_state_timestamp_ = ros::Time::now();
}

void BaxterUtilities::rightEndpointCallback(const baxter_core_msgs::EndpointStateConstPtr& msg)
{
  right_endpoint_state_ = msg;
  right_endpoint_state_timestamp_ = ros::Time::now();
}

void BaxterUtilities::jointCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  joint_state_ = msg;
  joint_state_timestamp_ = ros::Time::now();
}

bool BaxterUtilities::enableBaxter()
{
  ROS_INFO_STREAM_NAMED("utility","Enabling Baxter");

  // Check if we need to do anything
  if( isEnabled(false) )
    return true;

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  // Reset Baxter
  if( !resetBaxter() )
    return false;

  // Attempt to enable baxter
  pub_baxter_enable_.publish(enable_msg_);
  ros::Duration(0.5).sleep();

  // Check if enabled
  int count = 0;
  while( ros::ok() && !isEnabled(true) )
  {
    if( count > 20 ) // 20 is an arbitrary number for when to assume its not going to enable
    {
      ROS_ERROR_STREAM_NAMED("utilities","Giving up on waiting");
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }
  ROS_INFO_STREAM_NAMED("utility","Robot Enabled");

  return true;
}

bool BaxterUtilities::disableBaxter()
{
  ROS_INFO_STREAM_NAMED("utility","Disabling Baxter");

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  pub_baxter_enable_.publish(disable_msg_);
  ros::Duration(0.5).sleep();

  // Check it enabled
  int count = 0;
  while( ros::ok() && baxter_state_->enabled == true )
  {
    if( count > 20 ) // 20 is an arbitrary number for when to assume its not going to enable
    {
      ROS_ERROR_STREAM_NAMED("utilities","Failed to disable Baxter");
      return false;
    }

    ++count;
    ros::Duration(0.05).sleep();
  }
  ROS_INFO_STREAM_NAMED("utility","Robot Disabled");

  return true;
}

bool BaxterUtilities::resetBaxter()
{
  ROS_INFO_STREAM_NAMED("utility","Resetting Baxter");

  // Wait for state msg to be recieved
  if( !communicationActive() )
    return false;

  // Attempt to reset and enable robot
  pub_baxter_reset_.publish(empty_msg_);
  ros::Duration(0.5).sleep();

  return true;
}

bool BaxterUtilities::resetGripper(std::string limb){

  gripper_msg_.id=131073;
  gripper_msg_.sender="baxter_control";
  gripper_msg_.command=gripper_msg_.CMD_RESET;
  if(limb==std::string("left")){
	ROS_ERROR("Reset left");
    pub_left_end_effector_.publish(gripper_msg_);
  }else if(limb==std::string("right")){
	ROS_ERROR("Reset right");
    pub_right_end_effector_.publish(gripper_msg_);
  }else{
    ROS_ERROR("Wrong limb name");
    return false;
  }

  return true;
}

bool BaxterUtilities::clearGripper(std::string limb){

  gripper_msg_.id=131073;
  gripper_msg_.sender="baxter_control";
  gripper_msg_.command=gripper_msg_.CMD_CLEAR_CALIBRATION;
  if(limb==std::string("left")){
	ROS_ERROR("Clear left");
    pub_left_end_effector_.publish(gripper_msg_);
  }else if(limb==std::string("right")){
	ROS_ERROR("Clear right");
    pub_right_end_effector_.publish(gripper_msg_);
  }else{
    ROS_ERROR("Wrong limb name");
    return false;
  }

  return true;
}

bool BaxterUtilities::calibrateGripper(std::string limb){

  gripper_msg_.id=131073;
  gripper_msg_.sender="baxter_control";
  gripper_msg_.command=gripper_msg_.CMD_CALIBRATE;
  if(limb==std::string("left")){
	ROS_ERROR("Calibrate left");
    pub_left_end_effector_.publish(gripper_msg_);
  }else if(limb==std::string("right")){
	ROS_ERROR("Calibrate right");
    pub_right_end_effector_.publish(gripper_msg_);
  }else{
    ROS_ERROR("Wrong limb name");
    return false;
  }

  return true;
}

bool BaxterUtilities::configureGripper(std::string limb, double holding_force, double velocity, double dead_zone, double moving_force){

	std::stringstream ss1;
	ss1 << std::fixed << std::setprecision(1) << holding_force;
	std::string holding_force_string = ss1.str();
	//std::string holding_force_string = std::to_string(holding_force);

	std::stringstream ss2;
	ss2 << std::fixed << std::setprecision(1) << velocity;
	std::string velocity_string = ss2.str();
	//std::string velocity_string = std::to_string(velocity);

	std::stringstream ss3;
	ss3 << std::fixed << std::setprecision(1) << dead_zone;
	std::string dead_zone_string = ss3.str();
	//std::string dead_zone_string = std::to_string(dead_zone);

	std::stringstream ss4;
	ss4 << std::fixed << std::setprecision(1) << moving_force;
	std::string moving_force_string = ss4.str();
	//std::string moving_force_string = std::to_string(moving_force);

  gripper_msg_.id=131073;
  gripper_msg_.sender="baxter_control";
  gripper_msg_.command=gripper_msg_.CMD_CONFIGURE;
  gripper_msg_.args=std::string("{\"holding_force\": "+holding_force_string+
		  ", \"velocity\": "+velocity_string+
		  ", \"dead_zone\": "+dead_zone_string+
		  ", \"moving_force\"\: "+moving_force_string+"}");
  if(limb==std::string("left")){
	ROS_ERROR("Configure left");
    pub_left_end_effector_.publish(gripper_msg_);
  }else if(limb==std::string("right")){
	ROS_ERROR("Configure right");
    pub_right_end_effector_.publish(gripper_msg_);
  }else{
    ROS_ERROR("Wrong limb name");
    return false;
  }

  return true;
}

bool BaxterUtilities::openGripper(std::string limb){

  gripper_msg_.id=131073;
  gripper_msg_.sender="baxter_control";
  //gripper_msg_.command=gripper_msg_.CMD_RELEASE;
  gripper_msg_.command=gripper_msg_.CMD_GO;
  gripper_msg_.args=std::string("{\"position\": 100.0}");
  if(limb==std::string("left")){
	ROS_ERROR("Open left");
    pub_left_end_effector_.publish(gripper_msg_);
  }else if(limb==std::string("right")){
	ROS_ERROR("Open right");
    pub_right_end_effector_.publish(gripper_msg_);
  }else{
    ROS_ERROR("Wrong limb name");
    return false;
  }

  return true;
}

bool BaxterUtilities::closeGripper(std::string limb){

  gripper_msg_.id=131073;
  gripper_msg_.sender="baxter_control";
  //gripper_msg_.command=gripper_msg_.CMD_GRIP;
  gripper_msg_.command=gripper_msg_.CMD_GO;
  gripper_msg_.args=std::string("{\"position\": 0.0}");
  if(limb==std::string("left")){
	ROS_ERROR("Close left");
    pub_left_end_effector_.publish(gripper_msg_);
  }else if(limb==std::string("right")){
	ROS_ERROR("Close right");
    pub_right_end_effector_.publish(gripper_msg_);
  }else{
    ROS_ERROR("Wrong limb name");
    return false;
  }

  return true;
}

bool BaxterUtilities::positionGripper(std::string limb, double position){

	std::stringstream ss;
	ss << std::fixed << std::setprecision(1) << position;
	std::string position_string = ss.str();
	//std::string position_string = std::to_string(position);

  gripper_msg_.id=131073;
  gripper_msg_.sender="baxter_control";
  gripper_msg_.command=gripper_msg_.CMD_GO;
  gripper_msg_.args=std::string("{\"position\": "+position_string+"}");
  if(limb==std::string("left")){
	ROS_ERROR("Close left");
    pub_left_end_effector_.publish(gripper_msg_);
  }else if(limb==std::string("right")){
	ROS_ERROR("Close right");
    pub_right_end_effector_.publish(gripper_msg_);
  }else{
    ROS_ERROR("Wrong limb name");
    return false;
  }

  return true;
}

bool BaxterUtilities::leftLimbInitial(){

  pub_left_cmd_.publish(left_initial_msg_);

  return true;
}

bool BaxterUtilities::rightLimbInitial(){

  pub_right_cmd_.publish(right_initial_msg_);

  return true;
}

bool BaxterUtilities::leftLimbInitialAction(){

  left_action_msg_.trajectory.header.stamp=ros::Time::now();

  left_action_client_->sendGoal(left_action_msg_);
  left_action_client_->waitForResult(ros::Duration(10.0));

  if (left_action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  printf("Yay! The left action was done!");

  //pub_left_action_cmd_.publish(left_initial_action_msg_);

  return true;
}

bool BaxterUtilities::rightLimbInitialAction(){

  right_action_client_->sendGoal(right_action_msg_);
  right_action_client_->waitForResult(ros::Duration(5.0));

  //pub_right_action_cmd_.publish(right_initial_action_msg_);

  return true;
}

bool BaxterUtilities::leftLimbCommand(baxter_core_msgs::JointCommand &msg_){

  left_msg_=msg_;

  pub_left_cmd_.publish(left_msg_);

  return true;
}

bool BaxterUtilities::rightLimbCommand(baxter_core_msgs::JointCommand &msg_){

  right_msg_=msg_;

  pub_right_cmd_.publish(right_msg_);

  return true;
}

bool BaxterUtilities::leftLimbCommand(std::vector<double> &left_joints_){

	left_msg_.command[0] = left_joints_[0];
	left_msg_.command[1] = left_joints_[1];
	left_msg_.command[2] = left_joints_[2];
	left_msg_.command[3] = left_joints_[3];
	left_msg_.command[4] = left_joints_[4];
	left_msg_.command[5] = left_joints_[5];
	left_msg_.command[6] = left_joints_[6];

  pub_left_cmd_.publish(left_msg_);

  return true;
}

bool BaxterUtilities::rightLimbCommand(std::vector<double> &right_joints_){

	right_msg_.command[0] = right_joints_[0];
	right_msg_.command[1] = right_joints_[1];
	right_msg_.command[2] = right_joints_[2];
	right_msg_.command[3] = right_joints_[3];
	right_msg_.command[4] = right_joints_[4];
	right_msg_.command[5] = right_joints_[5];
	right_msg_.command[6] = right_joints_[6];

  pub_right_cmd_.publish(right_msg_);

  return true;
}

bool BaxterUtilities::leftIKService(baxter_core_msgs::SolvePositionIK &srv_left){
	if(srv_left_ik_.call(srv_left))
	{
	  ROS_INFO("Service inverse kinematics successful!");
      return true;
	}
	else
	{
	  ROS_ERROR("Failed to call service inverse kinematics");
	  return false;
	}
}

bool BaxterUtilities::rightIKService(baxter_core_msgs::SolvePositionIK &srv_right){
	if(srv_right_ik_.call(srv_right))
	{
	  ROS_INFO("Service inverse kinematics successful!");
      return true;
	}
	else
	{
	  ROS_ERROR("Failed to call service inverse kinematics");
	  return false;
	}
}

baxter_core_msgs::JointCommand BaxterUtilities::getLeftJointInitial(){

  return left_initial_msg_;
}

baxter_core_msgs::JointCommand BaxterUtilities::getRightJointInitial(){

  return right_initial_msg_;
}

void BaxterUtilities::getLeftJointInitial(baxter_core_msgs::JointCommand &left_msg){

  left_msg=left_initial_msg_;
}

void BaxterUtilities::getRightJointInitial(baxter_core_msgs::JointCommand &right_msg){

  right_msg=right_initial_msg_;
}

baxter_core_msgs::JointCommand BaxterUtilities::getLeftJointCommand(){

  return left_msg_;
}

baxter_core_msgs::JointCommand BaxterUtilities::getRightJointCommand(){

  return right_msg_;
}

sensor_msgs::JointState BaxterUtilities::getLeftJoints(){

	left_joints_.header=joint_state_->header;
	left_joints_.header.frame_id="base";
	for(int i=0;i<left_joints_.name.size();i++){
		for(int j=0;j<joint_state_->name.size();j++){
			if(left_joints_.name[i]==joint_state_->name[j]){
				left_joints_.position[i]=joint_state_->position[j];
				break;
			}
		}
	}

	return left_joints_;
}

sensor_msgs::JointState BaxterUtilities::getRightJoints(){

	right_joints_.header=joint_state_->header;
	right_joints_.header.frame_id="base";
	for(int i=0;i<right_joints_.name.size();i++){
		for(int j=0;j<joint_state_->name.size();j++){
			if(right_joints_.name[i]==joint_state_->name[j]){
				right_joints_.position[i]=joint_state_->position[j];
				break;
			}
		}
	}

	return right_joints_;
}

geometry_msgs::PoseStamped BaxterUtilities::getLeftPose(){

	left_pose_.header=left_endpoint_state_->header;
	left_pose_.header.frame_id="base";
	left_pose_.pose=left_endpoint_state_->pose;

	return left_pose_;
}

geometry_msgs::PoseStamped BaxterUtilities::getRightPose(){

	right_pose_.header=right_endpoint_state_->header;
	right_pose_.header.frame_id="base";
	right_pose_.pose=right_endpoint_state_->pose;

	return right_pose_;
}

bool BaxterUtilities::positionBaxterReady()
{
  return sendToPose("both_ready");
}

bool BaxterUtilities::positionBaxterNeutral()
{
  return sendToPose(NEUTRAL_POSE_NAME);
}

bool BaxterUtilities::sendToPose(const std::string &pose_name)
{
  // Check if move group has been loaded yet
  // We only load it here so that applications that don't need this aspect of baxter_utilities 
  // don't have to load it every time.
  if( !move_group_ )
  {
    //move_group_.reset(new move_group_interface::MoveGroup(PLANNING_GROUP_NAME));
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_NAME));
  }

  // Send to ready position
  ROS_INFO_STREAM_NAMED("pick_place","Sending to right and left arm ready positions...");
  move_group_->setNamedTarget(pose_name);
  //bool result = move_group_->move();
  bool result = (bool) move_group_->move();

  if( !result )
    ROS_ERROR_STREAM_NAMED("utilities","Failed to send Baxter to pose '" << pose_name << "'");

  return result;
}


} //namespace

