/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Southwest Research Institute
* Copyright (c) 2012, Edward Venator, Case Western Reserve University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 	* Redistributions of source code must retain the above copyright
* 	notice, this list of conditions and the following disclaimer.
* 	* Redistributions in binary form must reproduce the above copyright
* 	notice, this list of conditions and the following disclaimer in the
* 	documentation and/or other materials provided with the distribution.
* 	* Neither the name of the Southwest Research Institute, nor the names 
*	of its contributors may be used to endorse or promote products derived
*	from this software without specific prior written permission.
* 	* Neither the name of Case Western Reserve University, nor the names 
*	of its contributors may be used to endorse or promote products derived
*	from this software without specific prior written permission.
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

#include "ros/ros.h"
#include <joint_relay_handler.h>
#include "simple_message/messages/joint_message.h"
#include "simple_message/log_wrapper.h"
#include "urdf/model.h"
#include "urdf_interface/joint.h"

using namespace industrial::joint_message;
using namespace industrial::simple_message;
using namespace industrial::shared_types;

namespace industrial_robot_client
{
namespace joint_relay_handler
{

JointRelayHandler::JointRelayHandler(ros::NodeHandle &n) :
        joint_control_state_(),
        node_(n)
{

  this->pub_joint_control_state_ =
          this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);

  this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states",1);
  
  std::string urdf;
  int num_joints = NUM_OF_JOINTS_;
  if(! this->node_.searchParam("robot_description", urdf)){
    //Set up joint names from URDF file
    urdf::Model model;
    if (!model.initFile(urdf)){
        ROS_ERROR("Failed to parse urdf file. Using default joint configuration.");
        // Set up the default joint names
        this->joint_control_state_.joint_names.push_back("joint1");
        this->joint_control_state_.joint_names.push_back("joint2");
        this->joint_control_state_.joint_names.push_back("joint3");
        this->joint_control_state_.joint_names.push_back("joint4");
        this->joint_control_state_.joint_names.push_back("joint5");
        this->joint_control_state_.joint_names.push_back("joint6");
    }
    else{
        ROS_INFO("Successfully parsed urdf file");
        for (std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator joint = model.joints_.begin(); joint != model.joints_.end(); joint++){
          this->joint_control_state_.joint_names.push_back((*joint).second->name);
        }
        num_joints = model.joints_.size();
    }
  }
  else{
      ROS_WARN("No urdf file in robot_description. Using default joint configuration.");
      // Set up the default joint names
        this->joint_control_state_.joint_names.push_back("joint1");
        this->joint_control_state_.joint_names.push_back("joint2");
        this->joint_control_state_.joint_names.push_back("joint3");
        this->joint_control_state_.joint_names.push_back("joint4");
        this->joint_control_state_.joint_names.push_back("joint5");
        this->joint_control_state_.joint_names.push_back("joint6");
  }
  this->joint_control_state_.actual.positions.resize(num_joints);
  this->joint_control_state_.desired.positions.resize(num_joints);
  this->joint_control_state_.error.positions.resize(num_joints);

  // Copy from control state to sensor state
  this->joint_sensor_state_.name = this->joint_control_state_.joint_names;
  this->joint_sensor_state_.position.resize(num_joints);
  this->joint_sensor_state_.velocity.resize(num_joints);
  this->joint_sensor_state_.effort.resize(num_joints);
}

bool JointRelayHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection)
{
  return this->init(StandardMsgTypes::JOINT, connection);
}

bool JointRelayHandler::internalCB(industrial::simple_message::SimpleMessage & in)
{
  bool rtn = false;
  JointMessage joint;
  SimpleMessage msg;

  LOG_DEBUG("Executing internal CB");

  if (joint.init(in))
  {
    shared_real value;
    for(int i =0; i < NUM_OF_JOINTS_; i++)
    {
      if (joint.getJoints().getJoint(i, value))
      {
        this->joint_control_state_.actual.positions[i] = value;
        this->joint_sensor_state_.position[i] = value;
      }
      else
      {
        this->joint_control_state_.actual.positions[i] = 0.0;
        LOG_ERROR("Failed to populate ith(%d) of controller state message", i);
      }
      // TODO: For now these values are not populated
      this->joint_control_state_.desired.positions[i] = 0.0;
      this->joint_control_state_.error.positions[i] = 0.0;
    }
    this->joint_control_state_.header.stamp = ros::Time::now();
    this->pub_joint_control_state_.publish(this->joint_control_state_);

    this->joint_sensor_state_.header.stamp = ros::Time::now();
    this->pub_joint_sensor_state_.publish(this->joint_sensor_state_);

    // Reply back to the controller if the sender requested it.
    if (CommTypes::SERVICE_REQUEST == in.getMessageType())
    {
      joint.toReply(msg, ReplyTypes::SUCCESS);
    }
  }
  else
  {
    LOG_ERROR("Failed to initialize joint message");
    rtn = false;
  }

  return rtn;
}



}//namespace joint_relay_handler
}//namespace industrial_robot_client




