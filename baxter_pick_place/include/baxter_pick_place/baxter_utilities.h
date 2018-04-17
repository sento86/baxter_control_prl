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

#ifndef BAXTER_CONTROL__BAXTER_UTILITIES_
#define BAXTER_CONTROL__BAXTER_UTILITIES_

// ROS
#include <ros/ros.h>

// Boost
#include <boost/scoped_ptr.hpp>

// MoveIt!
//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Msgs
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
//#include <baxter_msgs/AssemblyState.h>
#include <baxter_core_msgs/AssemblyState.h>

#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndpointState.h>
#include <sensor_msgs/JointState.h>

#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/JointCommand.h>

#include <baxter_core_msgs/SolvePositionIK.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <control_msgs/JointTrajectoryActionGoal.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>

#include <actionlib/client/simple_action_client.h>

#include <string> // To use std::to_string()

namespace baxter_control
{

//static const std::string BAXTER_STATE_TOPIC = "/sdk/robot/state";
static const std::string BAXTER_STATE_TOPIC = "/robot/state";

static const std::string BAXTER_LEFT_GRIPPER_TOPIC = "/robot/end_effector/left_gripper/state";
static const std::string BAXTER_RIGHT_GRIPPER_TOPIC = "/robot/end_effector/right_gripper/state";

static const std::string BAXTER_LEFT_ENDPOINT_TOPIC = "/robot/limb/left/endpoint_state";
static const std::string BAXTER_RIGHT_ENDPOINT_TOPIC = "/robot/limb/right/endpoint_state";

static const std::string BAXTER_JOINT_STATE_TOPIC = "/robot/joint_states";

// Needed?
static const std::string ROBOT_DESCRIPTION="robot_description";
static const std::string PLANNING_GROUP_NAME = "both_arms";
static const std::string BASE_LINK = "base"; //"/base";
//static const std::string EE_GROUP = "right_hand";
//static const std::string EE_JOINT = "right_gripper_l_finger_joint";
//static const std::string EE_PARENT_LINK = "right_wrist";
static const std::string NEUTRAL_POSE_NAME = "both_neutral";


class BaxterUtilities
{
public:

  ros::Publisher pub_baxter_enable_;
  ros::Publisher pub_baxter_reset_;
  ros::Publisher pub_left_end_effector_;
  ros::Publisher pub_right_end_effector_;

  ros::Publisher pub_left_cmd_;
  ros::Publisher pub_right_cmd_;

  ros::Publisher pub_left_action_cmd_;
  ros::Publisher pub_right_action_cmd_;

  ros::Subscriber sub_baxter_state_;
  ros::Subscriber sub_left_gripper_state_;
  ros::Subscriber sub_right_gripper_state_;
  ros::Subscriber sub_left_endpoint_state_;
  ros::Subscriber sub_right_endpoint_state_;
  ros::Subscriber sub_joint_state_;

  ros::ServiceClient srv_left_ik_;
  ros::ServiceClient srv_right_ik_;

  baxter_core_msgs::SolvePositionIK srv_msg;

  baxter_core_msgs::EndEffectorCommand gripper_msg_;

  baxter_core_msgs::JointCommand left_initial_msg_;
  baxter_core_msgs::JointCommand right_initial_msg_;
  baxter_core_msgs::JointCommand left_msg_;
  baxter_core_msgs::JointCommand right_msg_;

  control_msgs::JointTrajectoryActionGoal left_initial_action_msg_;
  control_msgs::JointTrajectoryActionGoal right_initial_action_msg_;

  //control_msgs::FollowJointTrajectoryAction left_action_msg_;
  //control_msgs::FollowJointTrajectoryAction right_action_msg_;
  control_msgs::FollowJointTrajectoryGoal left_action_msg_;
  control_msgs::FollowJointTrajectoryGoal right_action_msg_;

  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> joint_action_client;

  joint_action_client* left_action_client_;
  joint_action_client* right_action_client_;

  // Interface with MoveIt
  //boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;
  boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // Remember the last baxter state and time
  //baxter_msgs::AssemblyStateConstPtr baxter_state_;
  baxter_core_msgs::AssemblyStateConstPtr baxter_state_;
  ros::Time baxter_state_timestamp_;

  baxter_core_msgs::EndEffectorStateConstPtr left_gripper_state_;
  ros::Time left_gripper_state_timestamp_;

  baxter_core_msgs::EndEffectorStateConstPtr right_gripper_state_;
  ros::Time right_gripper_state_timestamp_;

  baxter_core_msgs::EndpointStateConstPtr left_endpoint_state_;
  ros::Time left_endpoint_state_timestamp_;

  baxter_core_msgs::EndpointStateConstPtr right_endpoint_state_;
  ros::Time right_endpoint_state_timestamp_;

  sensor_msgs::JointStateConstPtr joint_state_;
  ros::Time joint_state_timestamp_;

  // Cache messages
  std_msgs::Bool enable_msg_;
  std_msgs::Bool disable_msg_;
  std_msgs::Empty empty_msg_;
  
  // Pose and Joints of each Limb (left/right)
  sensor_msgs::JointState left_joints_;
  sensor_msgs::JointState right_joints_;
  geometry_msgs::PoseStamped left_pose_;
  geometry_msgs::PoseStamped right_pose_;


  BaxterUtilities();

  /**
   * \brief Wait for initial state to be recieved from Baxter
   * \return true if communication is ok
   */
  bool communicationActive();

  /**
   * \brief Check if there is no error, is not stopped, and is enabled
   * \return true if baxter is ready to use
   */
  bool isEnabled(bool verbose = false);

  //void stateCallback(const baxter_msgs::AssemblyStateConstPtr& msg);
  void stateCallback(const baxter_core_msgs::AssemblyStateConstPtr& msg);

  void leftGripperCallback(const baxter_core_msgs::EndEffectorStateConstPtr& msg);

  void rightGripperCallback(const baxter_core_msgs::EndEffectorStateConstPtr& msg);

  void leftEndpointCallback(const baxter_core_msgs::EndpointStateConstPtr& msg);

  void rightEndpointCallback(const baxter_core_msgs::EndpointStateConstPtr& msg);

  void jointCallback(const sensor_msgs::JointStateConstPtr& msg);

  bool enableBaxter();
  bool disableBaxter();
  bool resetBaxter();

  bool resetGripper(std::string limb);
  bool clearGripper(std::string limb);
  bool calibrateGripper(std::string limb);
  bool configureGripper(std::string limb, double holding_force, double velocity, double dead_zone, double moving_force);
  bool openGripper(std::string limb);
  bool closeGripper(std::string limb);
  bool positionGripper(std::string limb, double position);

  bool leftLimbInitial();
  bool rightLimbInitial();

  bool leftLimbInitialAction();
  bool rightLimbInitialAction();

  bool leftLimbCommand(baxter_core_msgs::JointCommand &cmd_left);
  bool rightLimbCommand(baxter_core_msgs::JointCommand &cmd_right);

  bool leftLimbCommand(std::vector<double> &left_joints_);
  bool rightLimbCommand(std::vector<double> &right_joints_);

  bool leftIKService(baxter_core_msgs::SolvePositionIK &srv_left);
  bool rightIKService(baxter_core_msgs::SolvePositionIK &srv_right);

  baxter_core_msgs::JointCommand getLeftJointInitial();
  baxter_core_msgs::JointCommand getRightJointInitial();

  void getLeftJointInitial(baxter_core_msgs::JointCommand &left_msg);
  void getRightJointInitial(baxter_core_msgs::JointCommand &right_msg);

  baxter_core_msgs::JointCommand getLeftJointCommand();
  baxter_core_msgs::JointCommand getRightJointCommand();

  sensor_msgs::JointState getLeftJoints();
  sensor_msgs::JointState getRightJoints();

  geometry_msgs::PoseStamped getLeftPose();
  geometry_msgs::PoseStamped getRightPose();

  bool positionBaxterReady();

  bool positionBaxterNeutral();

  /**
   * \brief Send baxter to a pose defined in the SRDF
   * \param pose_name - name of pose in SRDF
   * \return true if sucessful in planning and moving there
   */
  bool sendToPose(const std::string &pose_name);

};

} //namespace

#endif
