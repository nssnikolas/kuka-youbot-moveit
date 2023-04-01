/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Arne Hitzmann
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
 *   * Neither the name of Ioan A. Sucan nor the names of its
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

/* Author: Ioan Sucan */
/* Author: Arne Hitzmann */

#include <ros/ros.h>
#include <moveit/controller_manager/controller_manager.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <map>
#include <moveit/macros/class_forward.h>
#include <control_msgs/GripperCommandAction.h>
#include <set>

namespace moveit_youbot_controller_manager
{
  class ActionBasedControllerHandleBase : public moveit_controller_manager::MoveItControllerHandle
  {
  public:
    ActionBasedControllerHandleBase(const std::string &name) :
      moveit_controller_manager::MoveItControllerHandle(name)
    {
    }

    virtual void addJoint(const std::string &name) = 0;
    virtual void getJoints(std::vector<std::string> &joints) = 0;
  };

  MOVEIT_CLASS_FORWARD(ActionBasedControllerHandleBase);


  /*
   * This is a simple base class, which handles all of the action creation/etc
   */
  template<typename T>
  class ActionBasedControllerHandle : public ActionBasedControllerHandleBase
  {

  public:
    ActionBasedControllerHandle(const std::string &name, const std::string &ns) :
      ActionBasedControllerHandleBase(name),
      namespace_(ns),
      done_(true)
    {
      controller_action_client_.reset(new actionlib::SimpleActionClient<T>(getActionName(), true));
      unsigned int attempts = 0;
      while (ros::ok() && !controller_action_client_->waitForServer(ros::Duration(5.0)) && ++attempts < 3)
        ROS_INFO_STREAM("YouBotMoveItControllerManager: Waiting for " << getActionName() << " to come up");

      if (!controller_action_client_->isServerConnected())
      {
        ROS_ERROR_STREAM("YouBotMoveItControllerManager: Action client not connected: " << getActionName());
        controller_action_client_.reset();
      }

      last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
    }

    bool isConnected() const
    {
      return static_cast<bool>(controller_action_client_);
    }

    virtual bool cancelExecution()
    {
      if (!controller_action_client_)
        return false;
      if (!done_)
      {
        ROS_INFO_STREAM("YouBotMoveItControllerManager: Cancelling execution for " << name_);
        controller_action_client_->cancelGoal();
        last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
        done_ = true;
      }
      return true;
    }

    virtual bool waitForExecution(const ros::Duration &timeout = ros::Duration(0))
    {
      if (controller_action_client_ && !done_)
        return controller_action_client_->waitForResult(timeout);
      return true;
    }

    virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus()
    {
      return last_exec_;
    }

    virtual void addJoint(const std::string &name)
    {
      joints_.push_back(name);
    }

    virtual void getJoints(std::vector<std::string> &joints)
    {
      joints = joints_;
    }

  protected:

    std::string getActionName(void) const
    {
      if (namespace_.empty())
        return name_;
      else
        return name_ +"/" + namespace_;
    }

    void finishControllerExecution(const actionlib::SimpleClientGoalState& state)
    {
      ROS_DEBUG_STREAM("YouBotMoveItControllerManager: Controller " << name_ << " is done with state " << state.toString() << ": " << state.getText());
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        last_exec_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
      else
        if (state == actionlib::SimpleClientGoalState::ABORTED)
          last_exec_ = moveit_controller_manager::ExecutionStatus::ABORTED;
        else
          if (state == actionlib::SimpleClientGoalState::PREEMPTED)
            last_exec_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
          else
            last_exec_ = moveit_controller_manager::ExecutionStatus::FAILED;
      done_ = true;
    }

    /* execution status */
    moveit_controller_manager::ExecutionStatus last_exec_;
    bool done_;

    /* the controller namespace, for instance, topics will map to name/ns/goal, name/ns/result, etc */
    std::string namespace_;

    /* the joints controlled by this controller */
    std::vector<std::string> joints_;

    /* action client */
    boost::shared_ptr<actionlib::SimpleActionClient<T> > controller_action_client_;
  };


  class GripperControllerHandle :
        public ActionBasedControllerHandle<control_msgs::GripperCommandAction>
  {
  public:
    /* Topics will map to name/ns/goal, name/ns/result, etc */
    GripperControllerHandle(const std::string &name, const std::string &ns) :
      ActionBasedControllerHandle<control_msgs::GripperCommandAction>(name, ns),
      allow_failure_(false), parallel_jaw_gripper_(false)
    {
    }

    virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
    {
      ROS_DEBUG_STREAM_NAMED("GripperController",
                             "Received new trajectory for " << name_);

      if (!controller_action_client_)
        return false;

      if (!trajectory.multi_dof_joint_trajectory.points.empty())
      {
        ROS_ERROR_NAMED("GripperController",
                        "Gripper cannot execute multi-dof trajectories.");
        return false;
      }

      if (trajectory.joint_trajectory.points.empty())
      {
        ROS_ERROR_NAMED("GripperController",
                        "GripperController requires at least one joint trajectory point.");
        return false;
      }

      if (trajectory.joint_trajectory.points.size() > 1)
      {
        ROS_DEBUG_STREAM_NAMED("GripperController","Trajectory: " << trajectory.joint_trajectory);
      }

      if (trajectory.joint_trajectory.joint_names.empty())
      {
        ROS_ERROR_NAMED("GripperController", "No joint names specified");
        return false;
      }

      std::vector<int> gripper_joint_indexes;
      for (std::size_t i = 0 ; i < trajectory.joint_trajectory.joint_names.size() ; ++i)
      {
        if (command_joints_.find(trajectory.joint_trajectory.joint_names[i]) != command_joints_.end())
        {
          gripper_joint_indexes.push_back(i);
          if (!parallel_jaw_gripper_)
            break;
        }
      }

      if (gripper_joint_indexes.empty())
      {
        ROS_WARN_NAMED("GripperController",
                       "No command_joint was specified for the MoveIt controller gripper handle. \
                        Please see GripperControllerHandle::addCommandJoint() and \
                        GripperControllerHandle::setCommandJoint(). Assuming index 0.");
        gripper_joint_indexes.push_back(0);
      }

      // goal to be sent
      control_msgs::GripperCommandGoal goal;
      goal.command.position = 0.0;
      goal.command.max_effort = 0.0;

      // send last point
      int tpoint = trajectory.joint_trajectory.points.size() - 1;
      ROS_DEBUG_NAMED("GripperController",
                      "Sending command from trajectory point %d",
                      tpoint);

      // fill in goal from last point
      for (std::size_t i = 0; i < gripper_joint_indexes.size(); ++i)
      {
        int idx = gripper_joint_indexes[i];

        if (trajectory.joint_trajectory.points[tpoint].positions.size() <= idx)
        {
          ROS_ERROR_NAMED("GripperController",
                          "GripperController expects a joint trajectory with one \
                           point that specifies at least the position of joint \
                           '%s', but insufficient positions provided",
                          trajectory.joint_trajectory.joint_names[idx].c_str());
          return false;
        }
        goal.command.position += trajectory.joint_trajectory.points[tpoint].positions[idx];

        if (trajectory.joint_trajectory.points[tpoint].effort.size() > idx)
          goal.command.max_effort = trajectory.joint_trajectory.points[tpoint].effort[idx];
      }

      controller_action_client_->sendGoal(goal,
                      boost::bind(&GripperControllerHandle::controllerDoneCallback, this, _1, _2),
                      boost::bind(&GripperControllerHandle::controllerActiveCallback, this),
                      boost::bind(&GripperControllerHandle::controllerFeedbackCallback, this, _1));

      done_ = false;
      last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
      return true;
    }

    void setCommandJoint(const std::string& name)
    {
      command_joints_.clear();
      addCommandJoint(name);
    }

    void addCommandJoint(const std::string& name)
    {
      command_joints_.insert(name);
    }

    void allowFailure(bool allow)
    {
      allow_failure_ = allow;
    }

    void setParallelJawGripper(const std::string& left, const std::string& right)
    {
      addCommandJoint(left);
      addCommandJoint(right);
      parallel_jaw_gripper_ = true;
    }

  private:

    void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                const control_msgs::GripperCommandResultConstPtr& result)
    {
      if (state == actionlib::SimpleClientGoalState::ABORTED && allow_failure_)
        finishControllerExecution(actionlib::SimpleClientGoalState::SUCCEEDED);
      else
        finishControllerExecution(state);
    }

    void controllerActiveCallback()
    {
      ROS_DEBUG_STREAM_NAMED("GripperController", name_ << " started execution");
    }

    void controllerFeedbackCallback(const control_msgs::GripperCommandFeedbackConstPtr& feedback)
    {
    }

    /*
     * Some gripper drivers may indicate a failure if they do not close all the way when
     * an object is in the gripper.
     */
    bool allow_failure_;

    /*
     * A common setup is where there are two joints that each move
     * half the overall distance. Thus, the command to the gripper
     * should be the sum of the two joint distances.
     *
     * When this is set, command_joints_ should be of size 2,
     * and the command will be the sum of the two joints.
     */
    bool parallel_jaw_gripper_;

    /*
     * A GripperCommand message has only a single float64 for the
     * "command", thus only a single joint angle can be sent -- however,
     * due to the complexity of making grippers look correct in a URDF,
     * they typically have >1 joints. The "command_joint" is the joint
     * whose position value will be sent in the GripperCommand action. A
     * set of names is provided for acceptable joint names. If any of
     * the joints specified is found, the value corresponding to that
     * joint is considered the command.
     */
    std::set<std::string> command_joints_;
  };


  class FollowJointTrajectoryControllerHandle : public ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>
  {
  public:

    FollowJointTrajectoryControllerHandle(const std::string &name, const std::string &action_ns) :
      ActionBasedControllerHandle<control_msgs::FollowJointTrajectoryAction>(name, action_ns)
    {
    }

    virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &trajectory)
    {
      ROS_DEBUG_STREAM("FollowJointTrajectoryController: new trajectory to " << name_);

      if (!controller_action_client_)
        return false;

      if (!trajectory.multi_dof_joint_trajectory.points.empty())
      {
        ROS_WARN("FollowJointTrajectoryController: %s cannot execute multi-dof trajectories.", name_.c_str());
      }

      if (done_)
        ROS_DEBUG_STREAM("FollowJointTrajectoryController: sending trajectory to " << name_);
      else
        ROS_DEBUG_STREAM("FollowJointTrajectoryController: sending continuation for the currently executed trajectory to " << name_);

      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory = trajectory.joint_trajectory;
      controller_action_client_->sendGoal(goal,
                      boost::bind(&FollowJointTrajectoryControllerHandle::controllerDoneCallback, this, _1, _2),
                      boost::bind(&FollowJointTrajectoryControllerHandle::controllerActiveCallback, this),
                      boost::bind(&FollowJointTrajectoryControllerHandle::controllerFeedbackCallback, this, _1));
      done_ = false;
      last_exec_ = moveit_controller_manager::ExecutionStatus::RUNNING;
      return true;
    }

  protected:

    void controllerDoneCallback(const actionlib::SimpleClientGoalState& state,
                                const control_msgs::FollowJointTrajectoryResultConstPtr& result)
    {
      // Output custom error message for FollowJointTrajectoryResult if necessary
      switch( result->error_code )
      {
        case control_msgs::FollowJointTrajectoryResult::INVALID_GOAL:
          ROS_WARN_STREAM("Controller " << name_ << " failed with error code INVALID_GOAL");
          break;
        case control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS:
          ROS_WARN_STREAM("Controller " << name_ << " failed with error code INVALID_JOINTS");
          break;
        case control_msgs::FollowJointTrajectoryResult::OLD_HEADER_TIMESTAMP:
          ROS_WARN_STREAM("Controller " << name_ << " failed with error code OLD_HEADER_TIMESTAMP");
          break;
        case control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED:
          ROS_WARN_STREAM("Controller " << name_ << " failed with error code PATH_TOLERANCE_VIOLATED");
          break;
        case control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED:
          ROS_WARN_STREAM("Controller " << name_ << " failed with error code GOAL_TOLERANCE_VIOLATED");
          break;
      }

      finishControllerExecution(state);
    }

    void controllerActiveCallback()
    {
      ROS_DEBUG_STREAM("FollowJointTrajectoryController: " << name_ << " started execution");
    }

    void controllerFeedbackCallback(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback)
    {
    }
  };

class YouBotMoveItControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:

  YouBotMoveItControllerManager() : node_handle_("~")
  {
    if (!node_handle_.hasParam("controller_list"))
    {
      ROS_ERROR_STREAM("No controller_list specified.");
      return;
    }

    XmlRpc::XmlRpcValue controller_list;
    node_handle_.getParam("controller_list", controller_list);
    if (controller_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Parameter controller_list should be specified as an array");
      return;
    }

    /* actually create each controller */
    for (int i = 0 ; i < controller_list.size() ; ++i)
    {
      if (!controller_list[i].hasMember("name") || !controller_list[i].hasMember("joints"))
      {
        ROS_ERROR("Name and joints must be specifed for each controller");
        continue;
      }

      try
      {
        std::string name = std::string(controller_list[i]["name"]);

        std::string action_ns;
        if (controller_list[i].hasMember("action_ns")){
          action_ns = std::string(controller_list[i]["action_ns"]);
        } else {
          ROS_WARN("Please note that 'action_ns' no longer has a default value.");
        }

        if (controller_list[i]["joints"].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          ROS_ERROR_STREAM("The list of joints for controller " << name << " is not specified as an array");
          continue;
        }

        if (!controller_list[i].hasMember("type"))
        {
          ROS_ERROR_STREAM("No type specified for controller " << name);
          continue;
        }

        std::string type = std::string(controller_list[i]["type"]);


        ActionBasedControllerHandleBasePtr new_handle;
        if ( type == "GripperCommand" )
        {
          new_handle.reset(new GripperControllerHandle(name, action_ns));
          if (static_cast<GripperControllerHandle*>(new_handle.get())->isConnected())
          {
            if (controller_list[i].hasMember("parallel"))
            {
              if (controller_list[i]["joints"].size() != 2)
              {
                ROS_ERROR_STREAM("YouBotMoveItControllerManager: Parallel Gripper requires exactly two joints");
                continue;
              }
              static_cast<GripperControllerHandle*>(new_handle.get())->setParallelJawGripper(controller_list[i]["joints"][0], controller_list[i]["joints"][1]);
            }
            else
            {
              if (controller_list[i].hasMember("command_joint"))
                static_cast<GripperControllerHandle*>(new_handle.get())->setCommandJoint(controller_list[i]["command_joint"]);
              else
                static_cast<GripperControllerHandle*>(new_handle.get())->setCommandJoint(controller_list[i]["joints"][0]);
            }

            if (controller_list[i].hasMember("allow_failure"))
                static_cast<GripperControllerHandle*>(new_handle.get())->allowFailure(true);

            ROS_INFO_STREAM("Added GripperCommand controller for " << name );
            controllers_[name] = new_handle;
          }

        }
        else if ( type == "FollowJointTrajectory" )
        {
          new_handle.reset(new FollowJointTrajectoryControllerHandle(name, action_ns));
          if (static_cast<FollowJointTrajectoryControllerHandle*>(new_handle.get())->isConnected())
          {
            ROS_INFO_STREAM("Added FollowJointTrajectory controller for " << name );
            controllers_[name] = new_handle;
          }
        }
        else
        {
          ROS_ERROR("Unknown controller type: '%s'", type.c_str());
          continue;
        }


        if (!controllers_[name])
        {
          controllers_.erase(name);
          continue;
        }

        /* add list of joints, used by controller manager and moveit */
        for (int j = 0 ; j < controller_list[i]["joints"].size() ; ++j)
          controllers_[name]->addJoint(std::string(controller_list[i]["joints"][j]));
      }
      catch (...)
      {
        ROS_ERROR("Unable to parse controller information");
      }
    }
  }

  virtual ~YouBotMoveItControllerManager()
  {
  }

  /*
   * Get a controller, by controller name (which was specified in the controllers.yaml
   */
  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name)
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
      return it->second;
    else
      ROS_FATAL_STREAM("No such controller: " << name);
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /*
   * Get the list of controller names.
   */
  virtual void getControllersList(std::vector<std::string> &names)
  {
    for (std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.begin() ; it != controllers_.end() ; ++it)
      names.push_back(it->first);
    ROS_INFO_STREAM("Returned " << names.size() << " controllers in list");
  }

  /*
   * This plugin assumes that all controllers are already active -- and if they are not, well, it has no way to deal with it anyways!
   */
  virtual void getActiveControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints)
  {
    std::map<std::string, ActionBasedControllerHandleBasePtr>::const_iterator it = controllers_.find(name);
    if (it != controllers_.end())
    {
      it->second->getJoints(joints);
    }
    else
    {
      ROS_WARN("The joints for controller '%s' are not known. Perhaps the controller configuration is not loaded on the param server?", name.c_str());
      joints.clear();
    }
  }

  /*
   * Controllers are all active and default.
   */
  virtual moveit_controller_manager::MoveItControllerManager::ControllerState getControllerState(const std::string &name)
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate) { return false; }

protected:

  ros::NodeHandle node_handle_;
  std::map<std::string, ActionBasedControllerHandleBasePtr> controllers_;
};

}

PLUGINLIB_EXPORT_CLASS(moveit_youbot_controller_manager::YouBotMoveItControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
