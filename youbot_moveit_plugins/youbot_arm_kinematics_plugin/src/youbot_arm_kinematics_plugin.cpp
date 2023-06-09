/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Sachin Chitta
 */

#include <youbot_arm_kinematics/youbot_arm_kinematics_plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>
#include <youbot_arm_kinematics/pr2_arm_kinematics_utils.h>
#include "ros/ros.h"
#include <algorithm>
#include <numeric>

#include <youbot_arm_kinematics/arm_kdl_inverse_kinematics.h>
#include <youbot_arm_kinematics/arm_kdl_forward_kinematics.h>
#include <youbot_arm_kinematics/arm_analytical_inverse_kinematics.h>
#include <youbot_arm_kinematics/inverse_kinematics_solver.h>

#include <pluginlib/class_list_macros.h>

using namespace KDL;
using namespace tf;
using namespace std;
using namespace ros;

PLUGINLIB_EXPORT_CLASS(youbot_arm_kinematics::ArmKinematicsPlugin,
		kinematics::KinematicsBase);

namespace youbot_arm_kinematics {

ArmKinematicsPlugin::ArmKinematicsPlugin() :
		_active(false) {
}

bool ArmKinematicsPlugin::isActive() {
	if (_active)
		return true;
	return false;
}

bool ArmKinematicsPlugin::initialize(const std::string& robot_description,
		const std::string& group_name, const std::string& base_name,
		const std::string& tip_name, const double search_discretization) {
	setValues(robot_description, group_name, base_name, tip_name,
			search_discretization);
	_root_name = base_name;

	urdf::Model robot_model;
	std::string xml_string;
	ros::NodeHandle private_handle("~/" + group_name);

	_node_handle.param<int>("free_angle", _free_angle, 2);

	// load and process the robot description
	_dimension = 5;
	while (!pr2_arm_kinematics::loadRobotModel(private_handle, robot_model,
			xml_string) && _node_handle.ok()) {
		ROS_ERROR(
				"Could not load robot model. Are you sure the robot model is on the parameter server?");
		ros::Duration(0.5).sleep();
	}

	// setup the IK
	// _ik = boost::shared_ptr<InverseKinematics>(new ArmKdlInverseKinematics(robot_model, xml_string, base_name, tip_name));
	_ik = std::shared_ptr<InverseKinematics>(
			new ArmAnalyticalInverseKinematics(robot_model, xml_string,
					base_name, tip_name));

	// setup the IK solver
	_ik_solver = std::shared_ptr<InverseKinematicsSolver>(
			new InverseKinematicsSolver(*_ik, search_discretization, 
			_free_angle));
	_ik_solver->getSolverInfo(_ik_solver_info);

	// setup the FK solver
	_fk_solver = std::shared_ptr<ForwardKinematics>(
			new ArmKdlForwardKinematics(robot_model, xml_string, base_name, 
			tip_name));
	_fk_solver->getSolverInfo(_fk_solver_info);

	// print information on kinematics capabilities
	for (unsigned int i = 0; i < _ik_solver_info.joint_names.size(); i++) {
		ROS_INFO("youBotKinematics:: joint name: %s",
				_ik_solver_info.joint_names[i].c_str());
	}

	for (unsigned int i = 0; i < _ik_solver_info.link_names.size(); i++) {
		ROS_INFO("youBotKinematics can solve IK for %s",
				_ik_solver_info.link_names[i].c_str());
	}

	for (unsigned int i = 0; i < _fk_solver_info.link_names.size(); i++) {
		ROS_INFO("youBotKinematics can solve FK for %s",
				_fk_solver_info.link_names[i].c_str());
	}

	_active = true;

	ROS_INFO("YouBotArmKinematicsPlugin::active for %s", group_name.c_str());

	return _active;
}

bool ArmKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state, std::vector<double> &solution,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options) const {

	ROS_DEBUG("getPositionIK()\n");
	if (!_active) {
		ROS_ERROR("kinematics not active");
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}

	KDL::Frame pose_desired;
	tf::PoseMsgToKDL(ik_pose, pose_desired);

	//Do the IK
	KDL::JntArray jnt_pos_in;
	KDL::JntArray jnt_pos_out;
	jnt_pos_in.resize(_dimension);
	for (int i = 0; i < _dimension; i++) {
		jnt_pos_in(i) = ik_seed_state[i];
	}

	int ik_valid = _ik_solver->CartToJnt(jnt_pos_in, pose_desired, jnt_pos_out);

	if (ik_valid < 0) {
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}

	if (ik_valid >= 0) {
		solution.resize(_dimension);
		for (int i = 0; i < _dimension; i++) {
			solution[i] = jnt_pos_out(i);
		}
		error_code.val = error_code.SUCCESS;
		return true;
	} else {
		ROS_DEBUG("An IK solution could not be found");
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}
}

bool ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state, double timeout,
		std::vector<double> &solution,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options) const {
	ROS_DEBUG("searchPositionIK(nocall)\n");

	if (!_active) {
		ROS_ERROR("kinematics not active");
		error_code.val = error_code.FAILURE;
		return false;
	}

	KDL::Frame pose_desired;
	tf::PoseMsgToKDL(ik_pose, pose_desired);

	//Do the IK
	KDL::JntArray jnt_pos_in;
	KDL::JntArray jnt_pos_out;
	jnt_pos_in.resize(_dimension);
	for (int i = 0; i < _dimension; i++) {
		jnt_pos_in(i) = ik_seed_state[i];
	}

	int ik_valid = _ik_solver->CartToJntSearch(jnt_pos_in, pose_desired,
			jnt_pos_out, timeout);

	if (ik_valid == error_code.NO_IK_SOLUTION) {
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}

	if (ik_valid >= 0) {
		solution.resize(_dimension);
		for (int i = 0; i < _dimension; i++) {
			solution[i] = jnt_pos_out(i);
		}
		error_code.val = error_code.SUCCESS;
		return true;
	} else {
		ROS_DEBUG("An IK solution could not be found");
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}

	error_code.val = error_code.NO_IK_SOLUTION;
	return false;
}

bool ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state, double timeout,
		const std::vector<double> &consistency_limits,
		std::vector<double> &solution,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options) const {
	ROS_DEBUG("searchPositionIK(consistency,nocall)\n");
	if (!_active) {
		ROS_ERROR("kinematics not active");
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}
	KDL::Frame pose_desired;
	tf::PoseMsgToKDL(ik_pose, pose_desired);

	//Do the IK
	KDL::JntArray jnt_pos_in;
	KDL::JntArray jnt_pos_out;
	jnt_pos_in.resize(_dimension);
	for (int i = 0; i < _dimension; i++) {
		jnt_pos_in(i) = ik_seed_state[i];
	}

	unsigned int old_free_angle = _ik_solver->getFreeAngle();
	//_ik_solver->setFreeAngle(redundancy);
	//TODO(arne) find out about "consistency_limits"
	int ik_valid = _ik_solver->CartToJntSearch(jnt_pos_in, pose_desired,
			consistency_limits[0], jnt_pos_out, timeout);

	_ik_solver->setFreeAngle(old_free_angle);

	if (ik_valid == error_code.NO_IK_SOLUTION) {
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}

	if (ik_valid >= 0) {
		solution.resize(_dimension);
		for (int i = 0; i < _dimension; i++) {
			solution[i] = jnt_pos_out(i);
		}
		error_code.val = error_code.SUCCESS;
		return true;
	} else {
		ROS_DEBUG("An IK solution could not be found");
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}
}

void ArmKinematicsPlugin::desiredPoseCallback(const KDL::JntArray& jnt_array,
		const KDL::Frame& ik_pose, moveit_msgs::MoveItErrorCodes& error_code) {
	std::vector<double> ik_seed_state;
	ik_seed_state.resize(_dimension);
	int int_error_code;

	for (int i = 0; i < _dimension; i++) {
		ik_seed_state[i] = jnt_array(i);
	}

	geometry_msgs::Pose ik_pose_msg;
	tf::PoseKDLToMsg(ik_pose, ik_pose_msg);

	_desiredPoseCallback(ik_pose_msg, ik_seed_state, int_error_code);
	if (int_error_code) {
		error_code.val = error_code.SUCCESS;
	} else {
		error_code.val = error_code.NO_IK_SOLUTION;
	}
}

void ArmKinematicsPlugin::jointSolutionCallback(const KDL::JntArray& jnt_array,
		const KDL::Frame& ik_pose, moveit_msgs::MoveItErrorCodes& error_code) {
	std::vector<double> ik_seed_state;
	ik_seed_state.resize(_dimension);
	int int_error_code;

	for (int i = 0; i < _dimension; i++) {
		ik_seed_state[i] = jnt_array(i);
	}

	geometry_msgs::Pose ik_pose_msg;
	tf::PoseKDLToMsg(ik_pose, ik_pose_msg);

	_solutionCallback(ik_pose_msg, ik_seed_state, int_error_code);
	if (int_error_code > 0) {
		error_code.val = error_code.SUCCESS;
	} else {
		error_code.val = error_code.NO_IK_SOLUTION;
	}
}

bool ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state, double timeout,
		std::vector<double> &solution, const IKCallbackFn &solution_callback,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options) const {
	static std::vector<double> consistency_limits;
	return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits,
			solution, solution_callback, error_code);
}

bool ArmKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
		const std::vector<double> &ik_seed_state, double timeout,
		const std::vector<double> &consistency_limits,
		std::vector<double> &solution, const IKCallbackFn &solution_callback,
		moveit_msgs::MoveItErrorCodes &error_code,
		const kinematics::KinematicsQueryOptions &options) const {

	ROS_DEBUG("searchPositionIK(consistency,call)\n");
	if (!_active) {
		ROS_ERROR("kinematics not active");
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}

	KDL::Frame pose_desired;
	tf::PoseMsgToKDL(ik_pose, pose_desired);

	//Do the IK
	KDL::JntArray jnt_pos_in;
	KDL::JntArray jnt_pos_out;
	jnt_pos_in.resize(_dimension);
	for (int i = 0; i < _dimension; i++) {
		jnt_pos_in(i) = ik_seed_state[i];
	}

	moveit_msgs::MoveItErrorCodes error_code_int;
	int ik_valid;
	if (consistency_limits.empty()) {
		//TODO(arne) find out about "consistency_limits"
		ik_valid = _ik_solver->CartToJntSearch(jnt_pos_in, pose_desired,
				jnt_pos_out, timeout, error_code,
				solution_callback ?
						boost::bind(solution_callback, _1, _2, _3) :
						IKCallbackFn());
	} else {
		//TODO(arne) find out about "consistency_limits"
		ik_valid = _ik_solver->CartToJntSearch(jnt_pos_in, pose_desired,
				jnt_pos_out, timeout, consistency_limits[0], error_code,
				solution_callback ?
						boost::bind(solution_callback, _1, _2, _3) :
						IKCallbackFn());
	}

	if (ik_valid == error_code.NO_IK_SOLUTION) {
		error_code.val = error_code.NO_IK_SOLUTION;
		return false;
	}

	if (ik_valid >= 0) {
		solution.resize(_dimension);
		for (int i = 0; i < _dimension; i++) {
			solution[i] = jnt_pos_out(i);
		}
		error_code.val = error_code.SUCCESS;
		return true;
	} else {
		ROS_DEBUG("An IK solution could not be found");
		error_code_int = error_code;
		return false;
	}

	return false;
}

bool ArmKinematicsPlugin::getPositionFK(
		const std::vector<std::string> &link_names,
		const std::vector<double> &joint_angles,
		std::vector<geometry_msgs::Pose> &poses) const {
	ROS_DEBUG("getPositionFK()\n");

	if (!_active) {
		ROS_ERROR("kinematics not active");
		return false;
	}

	KDL::Frame p_out;
	KDL::JntArray jnt_pos_in;
	geometry_msgs::PoseStamped pose;
	tf::Stamped<tf::Pose> tf_pose;

	jnt_pos_in.resize(_dimension);
	for (int i = 0; i < _dimension; i++) {
		jnt_pos_in(i) = joint_angles[i];
	}

	poses.resize(link_names.size());

	bool valid = true;
	for (unsigned int i = 0; i < poses.size(); i++) {
		ROS_DEBUG("End effector index: %d",
				_fk_solver->getSegmentIndex(link_names[i]));
		if (_fk_solver->JntToCart(jnt_pos_in, p_out,
				_fk_solver->getSegmentIndex(link_names[i])) >= 0) {
			tf::PoseKDLToMsg(p_out, poses[i]);
		} else {
			ROS_ERROR("Could not compute FK for %s", link_names[i].c_str());
			valid = false;
		}
	}

	return valid;
}

std::string ArmKinematicsPlugin::getBaseFrame() {
	if (!_active) {
		ROS_ERROR("kinematics not active");
		return std::string("");
	}

	return _root_name;
}

std::string ArmKinematicsPlugin::getToolFrame() {
	if (!_active || _ik_solver_info.link_names.empty()) {
		ROS_ERROR("kinematics not active");
		return std::string("");
	}

	return _ik_solver_info.link_names[0];
}

const std::vector<std::string>& ArmKinematicsPlugin::getJointNames() const {
	if (!_active) {
		ROS_ERROR("kinematics not active");
	}

	return _ik_solver_info.joint_names;
}

const std::vector<std::string>& ArmKinematicsPlugin::getLinkNames() const {
	if (!_active) {
		ROS_ERROR("kinematics not active");
	}

	return _fk_solver_info.link_names;
}

} // namespace
