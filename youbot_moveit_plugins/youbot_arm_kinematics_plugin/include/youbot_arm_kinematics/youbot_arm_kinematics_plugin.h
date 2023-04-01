/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*
* Author: Sachin Chitta
*********************************************************************/

#ifndef YOUBOT_ARM_KINEMATICS_PLUGIN_H
#define YOUBOT_ARM_KINEMATICS_PLUGIN_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <angles/angles.h>
#include <tf_conversions/tf_kdl.h>

#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <moveit/kinematics_base/kinematics_base.h>

#include <youbot_arm_kinematics/forward_kinematics.h>
#include <youbot_arm_kinematics/inverse_kinematics_solver.h>

namespace youbot_arm_kinematics
{
class ArmKinematicsPlugin : public kinematics::KinematicsBase
{
	public:
		/** @class
		 *  @brief Plugin-able interface to the PR2 arm kinematics
		 */
		ArmKinematicsPlugin();

		/**
		 *  @brief Specifies if the node is active or not
		 *  @return True if the node is active, false otherwise.
		 */
		bool isActive();

		/**
		 * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
		 * @param ik_link_name - the name of the link for which IK is being computed
		 * @param ik_pose the desired pose of the link
		 * @param ik_seed_state an initial guess solution for the inverse kinematics
		 * @return True if a valid solution was found, false otherwise
		 */
		bool getPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state,
				std::vector<double> &solution,
				moveit_msgs::MoveItErrorCodes &error_code,
				const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

		/**
		 * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
		 * This particular method is intended for "searching" for a solutions by stepping through the redundancy
		 * (or other numerical routines).
		 * @param ik_pose the desired pose of the link
		 * @param ik_seed_state an initial guess solution for the inverse kinematics
		 * @return True if a valid solution was found, false otherwise
		 */
		bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state,
				double timeout,
				std::vector<double> &solution,
				moveit_msgs::MoveItErrorCodes &error_code,
				const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

		/**
		 * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
		 * This particular method is intended for "searching" for a solutions by stepping through the redundancy
		 * (or other numerical routines).
		 * @param ik_pose the desired pose of the link
		 * @param ik_seed_state an initial guess solution for the inverse kinematics
		 * @param the distance that the redundancy can be from the current position
		 * @return True if a valid solution was found, false otherwise
		 */
		bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
				const std::vector<double> &ik_seed_state,
				double timeout,
				const std::vector<double> &consistency_limits,
				std::vector<double> &solution,
				moveit_msgs::MoveItErrorCodes &error_code,
				const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

		/**
		 * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
		 * This particular method is intended for "searching" for a solutions by stepping through the redundancy
		 * (or other numerical routines).
		 * @param ik_pose the desired pose of the link
		 * @param ik_seed_state an initial guess solution for the inverse kinematics
		 * @return True if a valid solution was found, false otherwise
		*/
		bool searchPositionIK(	const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

		/**
		 * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
		 * This particular method is intended for "searching" for a solutions by stepping through the redundancy
		 * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
		 * around those specified in the seed state are admissible and need to be searched.
		 * @param ik_pose the desired pose of the link
		 * @param ik_seed_state an initial guess solution for the inverse kinematics
		 * @param consistency_limit the distance that the redundancy can be from the current position
		 * @return True if a valid solution was found, false otherwise
		 */
		bool searchPositionIK(	const geometry_msgs::Pose &ik_pose,
                                const std::vector<double> &ik_seed_state,
                                double timeout,
                                const std::vector<double> &consistency_limits,
                                std::vector<double> &solution,
                                const IKCallbackFn &solution_callback,
                                moveit_msgs::MoveItErrorCodes &error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

		/**
		 * @brief Given a set of joint angles and a set of links, compute their pose
		 * @param request  - the request contains the joint angles, set of links for which poses are to be computed and a timeout
		 * @param response - the response contains stamped pose information for all the requested links
		 * @return True if a valid solution was found, false otherwise
		 */
		bool getPositionFK(const std::vector<std::string> &link_names,
				const std::vector<double> &joint_angles,
				std::vector<geometry_msgs::Pose> &poses) const;

		/**
		 * @brief  Initialization function for the kinematics
		 * @return True if initialization was successful, false otherwise
		 */
		bool initialize(const std::string& robot_description,
				const std::string& group_name,
				const std::string& base_name,
				const std::string& tip_name,
				const double search_discretization);

		/**
		 * @brief  Return the frame in which the kinematics is operating
		 * @return the string name of the frame in which the kinematics is operating
		 */
		std::string getBaseFrame();

		/**
		 * @brief  Return the links for which kinematics can be computed
		 */
		std::string getToolFrame();

		/**
		 * @brief  Return all the joint names in the order they are used internally
		 */
		const std::vector<std::string>& getJointNames() const;

		/**
		 * @brief  Return all the link names in the order they are represented internally
		 */
		const std::vector<std::string>& getLinkNames() const;


	protected: // functions
		void desiredPoseCallback(const KDL::JntArray& jnt_array,
				const KDL::Frame& ik_pose,
				moveit_msgs::MoveItErrorCodes& error_code);

		void jointSolutionCallback(const KDL::JntArray& jnt_array,
				const KDL::Frame& ik_pose,
				moveit_msgs::MoveItErrorCodes& error_code);


    protected: // variables
		std::shared_ptr<InverseKinematics>_ik;
		std::shared_ptr<InverseKinematicsSolver> _ik_solver;
		std::shared_ptr<ForwardKinematics> _fk_solver;
		moveit_msgs::KinematicSolverInfo _ik_solver_info;
		moveit_msgs::KinematicSolverInfo _fk_solver_info;
		ros::NodeHandle _node_handle;
		bool _active;
		int _dimension;
		int _free_angle;
		std::string _root_name;
		std::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> _desiredPoseCallback;
		std::function<void(const geometry_msgs::Pose &ik_pose,const std::vector<double> &ik_solution,int &error_code)> _solutionCallback;
};

}

#endif
