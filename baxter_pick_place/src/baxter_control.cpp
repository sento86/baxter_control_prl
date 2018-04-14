/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, CU Boulder
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
 * \brief   Simple pick place for blocks using Baxter
 * \author  Vicent Girbes Juan
 */

// ROS
#include <ros/ros.h>

// MoveIt!
//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Baxter Utilities
//#include <baxter_control/baxter_utilities.h>
#include <baxter_pick_place/baxter_utilities.h>

// Grasp generation
//#include <moveit_grasps/grasps.h>
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // simple tool for showing grasps

// Baxter specific properties
//#include <moveit_grasps/grasp_data.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <baxter_pick_place/custom_environment5.h>

namespace baxter_pick_place
{

struct MetaBlock
{
  std::string name;
  geometry_msgs::Pose start_pose;
  geometry_msgs::Pose goal_pose;
};

class SimplePickPlace
{
public:

  // A shared node handle
  ros::NodeHandle nh_;

  // grasp generator
  //moveit_grasps::GraspsPtr simple_grasps_;
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // our interface with MoveIt
  //boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;
  boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // baxter helper
  baxter_control::BaxterUtilities baxter_util_;

  // which baxter arm are we using
  std::string arm_;
  std::string planning_group_name_;

  // settings
  bool auto_reset_;
  int auto_reset_sec_;
  int pick_place_count_; // tracks how many pick_places have run 

  // Show more visual and console output, with general slower run time.
  bool verbose_;

  SimplePickPlace(bool verbose)
    : nh_("~"),
      verbose_(verbose),
      auto_reset_(false),
      auto_reset_sec_(4),
      arm_("right"),
      planning_group_name_(arm_+"_arm"),
      pick_place_count_(0)
  {

    // Let everything load
    ros::Duration(1.0).sleep();

    // Do it.
    startRoutine();

    return;

  }

  bool startRoutine()
  {

    // Enable baxter
    if( !baxter_util_.enableBaxter() )
      return false;

	  //###############################################################

		/*baxter_util_.clearGripper(std::string("right"));
	  	ros::Duration(0.5).sleep();

	  	baxter_util_.calibrateGripper(std::string("right"));
	  	ros::Duration(5.0).sleep();*/

	  	double holding_force=30.0;
	  	double velocity=50.0;
	  	double dead_zone=5.0;
	  	double moving_force=40.0;

	  	baxter_util_.configureGripper(std::string("right"), holding_force, velocity, dead_zone, moving_force);
		ros::Duration(0.5).sleep();

	  	baxter_util_.openGripper(std::string("right"));
		ros::Duration(0.5).sleep();

	  	baxter_util_.closeGripper(std::string("right"));
		ros::Duration(0.5).sleep();

	  //###############################################################

	    for(int i=0;i<20;i++){
		    baxter_util_.leftLimbInitial();
		    baxter_util_.rightLimbInitial();
		    ros::Duration(0.1).sleep();
	    }

	    //###############################################################

    	baxter_core_msgs::SolvePositionIK srv_left;
    	baxter_core_msgs::SolvePositionIK srv_right;
	    std::vector<geometry_msgs::PoseStamped> poses_left;
	    std::vector<geometry_msgs::PoseStamped> poses_right;
	    poses_left.resize(1);
	    poses_right.resize(1);

	    std::vector<sensor_msgs::JointState> joints_left;
	    std::vector<sensor_msgs::JointState> joints_right;
	    joints_left.resize(1);
	    joints_right.resize(1);

	    joints_left[0] = baxter_util_.getLeftJoints();
	    joints_right[0] = baxter_util_.getRightJoints();

	  //###############################################################

	    //poses_left[0]=baxter_util_.getLeftPose();

	    poses_left[0].header.stamp=ros::Time::now();
	    poses_left[0].header.frame_id="base";

	    poses_left[0].pose.position.x = 0.6;
	    poses_left[0].pose.position.y = 0.3;
	    poses_left[0].pose.position.z = -0.1;
	    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,M_PI,0),poses_left[0].pose.orientation);

	    srv_left.request.pose_stamp = poses_left;
	    srv_left.request.seed_angles = joints_left;
	    srv_left.request.seed_mode = srv_left.request.SEED_AUTO;
	    baxter_util_.leftIKService(srv_left);

	  //###############################################################

	    //poses_right[0]=baxter_util_.getRightPose();

	    poses_right[0].header.stamp=ros::Time::now();
	    poses_right[0].header.frame_id="base";

	    poses_right[0].pose.position.x = 0.8;
		poses_right[0].pose.position.y = -0.3;
		poses_right[0].pose.position.z = 0.2;
	    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0,M_PI,0),poses_right[0].pose.orientation);

	    srv_right.request.pose_stamp = poses_right;
	    srv_right.request.seed_angles = joints_right;
	    srv_right.request.seed_mode = srv_right.request.SEED_AUTO;
	    baxter_util_.rightIKService(srv_right);

	  //###############################################################

	    std::cout << "LEFT:" << std::endl;
	    std::cout << srv_left.request << std::endl;
	    std::cout << srv_left.response << std::endl;

	    std::cout << "RIGHT:" << std::endl;
	    std::cout << srv_right.request << std::endl;
	    std::cout << srv_right.response << std::endl;

	  //###############################################################

	    /*baxter_core_msgs::JointCommand cmd_left, cmd_right;

	    cmd_left = baxter_util_.getLeftJointInitial();
	    cmd_right = baxter_util_.getRightJointInitial();

	    if(srv_left.response.isValid[0]!=0){
			cmd_left.command[0] = srv_left.response.joints[0].position[0];
			cmd_left.command[1] = srv_left.response.joints[0].position[1];
			cmd_left.command[2] = srv_left.response.joints[0].position[2];
			cmd_left.command[3] = srv_left.response.joints[0].position[3];
			cmd_left.command[4] = srv_left.response.joints[0].position[4];
			cmd_left.command[5] = srv_left.response.joints[0].position[5];
			cmd_left.command[6] = srv_left.response.joints[0].position[6];
	    	ROS_ERROR_STREAM_NAMED("LEFT","isValid");
	    }else{
		    cmd_left.command[0] = 0.0;
		    cmd_left.command[1] = 0.0;
		    cmd_left.command[2] = 0.0;
		    cmd_left.command[3] = 0.0;
		    cmd_left.command[4] = 0.0;
		    cmd_left.command[5] = 0.0;
		    cmd_left.command[6] = 0.0;
	    	ROS_ERROR_STREAM_NAMED("LEFT","isNotValid");
	    }
	    if(srv_right.response.isValid[0]!=0){
			cmd_right.command[0] = srv_right.response.joints[0].position[0];
			cmd_right.command[1] = srv_right.response.joints[0].position[1];
			cmd_right.command[2] = srv_right.response.joints[0].position[2];
			cmd_right.command[3] = srv_right.response.joints[0].position[3];
			cmd_right.command[4] = srv_right.response.joints[0].position[4];
			cmd_right.command[5] = srv_right.response.joints[0].position[5];
			cmd_right.command[6] = srv_right.response.joints[0].position[6];
	    	ROS_ERROR_STREAM_NAMED("RIGHT","isValid");
	    }else{
		    cmd_right.command[0] = 0.0;
		    cmd_right.command[1] = 0.0;
		    cmd_right.command[2] = 0.0;
		    cmd_right.command[3] = 0.0;
		    cmd_right.command[4] = 0.0;
		    cmd_right.command[5] = 0.0;
		    cmd_right.command[6] = 0.0;
	    	ROS_ERROR_STREAM_NAMED("RIGHT","isNotValid");
	    }*/

	  //###############################################################

	    // publish at least at 5 Hz, or else Baxter switches back to Position mode and holds position
	    ros::Rate loop_rate(100);
	    while(ros::ok()){

	      //update cmd.command commands here
	      //baxter_util_.leftLimbCommand(cmd_left);
	      //baxter_util_.rightLimbCommand(cmd_right);
	      if(srv_left.response.isValid[0]!=0)
	        baxter_util_.leftLimbCommand(srv_left.response.joints[0].position);
	      else
		    ROS_ERROR_STREAM_NAMED("LEFT","isNotValid");

	      if(srv_right.response.isValid[0]!=0)
	        baxter_util_.rightLimbCommand(srv_right.response.joints[0].position);
	      else
		    ROS_ERROR_STREAM_NAMED("RIGHT","isNotValid");

	      ros::spinOnce();
	      loop_rate.sleep();
	    }

//###############################################################

	    // Shutdown
	    baxter_util_.disableBaxter();

    // Everything worked!
    return true;
  }

};

} //namespace

int main(int argc, char **argv)
{
  ros::init (argc, argv, "baxter_pick_place");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
      }
    }
  }

  // Start the pick place node
  baxter_pick_place::SimplePickPlace server(verbose);

  ROS_INFO_STREAM_NAMED("main", "Shutting down.");
  ros::shutdown();

  return 0;
}
