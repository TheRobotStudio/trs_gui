/*
 * Copyright (c) 2014, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Oct 30, 2014
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com), based on ROS code
 */

/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date October 2014
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <sstream>
#include "../include/trs_gui/qnode.hpp"

#define LOOP_RATE				HEART_BEAT
#define BALANCE_FACTOR			3
#define BALANCE_DEADBAND		5
#define P_FACTOR				4

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace trs_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init()
{
	//Initialize ROS
	ros::init(init_argc,init_argv,"trs_qt_gui_node");

	if(!ros::master::check())
	{
		return false;
	}

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh;

	//Publishers
	pub_setArmsCommands = nh.advertise<trs_msgs::MotorCmdSet>("/setArmsCommands", 100, true);
	pub_setHandsCommands = nh.advertise<trs_msgs::MotorCmdSet>("/setHandsCommands", 100, true);
	pub_setHeadCommands = nh.advertise<trs_msgs::MotorCmdSet>("/setHeadCommands", 100, true);
	pub_setLegsCommands = nh.advertise<trs_msgs::MotorCmdSet>("/setLegsCommands", 100, true);

	//Service clients
	srvClt_switchConductor = nh.serviceClient<trs_control::switchNode>("switch_conductor");
	srvClt_switchRandomPosturePlayer = nh.serviceClient<trs_control::switchNode>("switch_random_posture_player");
	srvClt_switchFaceTracking = nh.serviceClient<trs_control::switchNode>("switch_face_tracking");
	srvClt_switchObjectTracking = nh.serviceClient<trs_control::switchNode>("switch_object_tracking");
	srvClt_switchKdTreeAngles = nh.serviceClient<trs_control::switchNode>("switch_kd_tree_angles");
	srvClt_switchKdTreeObject = nh.serviceClient<trs_control::switchNode>("switch_kd_tree_object");
	srvClt_switchKdTreeHead = nh.serviceClient<trs_control::switchNode>("switch_kd_tree_head");
	srvClt_switchHandshake = nh.serviceClient<trs_control::switchNode>("switch_handshake");
	srvClt_switchBalanceLegs = nh.serviceClient<trs_control::switchNode>("switch_balance_legs");
	srvClt_switchRandomLegsPlayer = nh.serviceClient<trs_control::switchNode>("switch_random_legs_player");

	srvClt_sendRobotZeroPositionCmd = nh.serviceClient<trs_control::getMotorCmdSet>("get_all_zero_position_cmd");
	srvClt_sendSlaveZeroPositionCmd = nh.serviceClient<trs_control::getSlaveLockCmd>("get_slave_zero_position_cmd");
	srvClt_sendSlaveCurrentCmd = nh.serviceClient<trs_control::getSlaveCurrentCmd>("get_slave_current_cmd");
	srvClt_sendSlaveLockCmd = nh.serviceClient<trs_control::getSlaveLockCmd>("get_slave_lock_cmd");
	srvClt_sendHeadPositionCmd = nh.serviceClient<trs_control::getHeadPositionCmd>("get_head_position_cmd");

	start();
	return true;
}

void QNode::run()
{
	ros::Rate r(LOOP_RATE);

	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}



//whole robot
void QNode::switchConductor(bool state)
{
	trs_control::switchNode srv_switchNode;
	srv_switchNode.request.state = state;

	//call service to switch
	if(srvClt_switchConductor.call(srv_switchNode))
	{

	}
	else
	{
		ROS_ERROR("Failed to call service switch_conductor");
	}
}

void QNode::sendRobotZeroPositionCmd()
{
	trs_control::getMotorCmdSet srv_getMotorCmdSet;

	//call service
	if(srvClt_sendRobotZeroPositionCmd.call(srv_getMotorCmdSet))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setArmsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
		pub_setArmsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
		pub_setLegsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
		pub_setLegsCommands.publish(srv_getMotorCmdSet.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_all_zero_position_cmd");
	}
}

void QNode::sendRobotLockPositionCmd()
{
	this->sendArmsLockPositionCmd();
	this->sendLegsLockPositionCmd();
}

void QNode::sendRobotCurrentCmd(int currVal)
{
	this->sendArmsCurrentCmd(currVal);
	this->sendLegsCurrentCmd(currVal);
}

void QNode::switchRandomPosturePlayer(bool state)
{
	trs_control::switchNode srv_switchNode;
	srv_switchNode.request.state = state;

	//call service to switch
	if(srvClt_switchRandomPosturePlayer.call(srv_switchNode))
	{

	}
	else
	{
		ROS_ERROR("Failed to call service switch_random_posture_player");
	}
}


//head
void QNode::switchFaceTracking(bool state)
{
	trs_control::switchNode srv_switchNode;
	srv_switchNode.request.state = state;

	//call service to switch
	if(srvClt_switchFaceTracking.call(srv_switchNode))
	{

	}
	else
	{
		ROS_ERROR("Failed to call service switch_face_tracking");
	}
}

void QNode::switchObjectTracking(bool state)
{
	trs_control::switchNode srv_switchNode;
	srv_switchNode.request.state = state;

	//call service to switch
	if(srvClt_switchObjectTracking.call(srv_switchNode))
	{

	}
	else
	{
		ROS_ERROR("Failed to call service switch_object_tracking");
	}
}


void QNode::sendRecenterHeadCmd()
{
	trs_control::getHeadPositionCmd srv_getHeadPositionCmd;
	srv_getHeadPositionCmd.request.recenter = true;

	//call service
	if(srvClt_sendHeadPositionCmd.call(srv_getHeadPositionCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setHeadCommands.publish(srv_getHeadPositionCmd.response.motorCmdSet);
		pub_setHeadCommands.publish(srv_getHeadPositionCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_head_position_cmd, recenter");
	}
}

void QNode::sendHeadPositionCmd(int headYaw, int headPitch, int eyeYaw, int eyePitch, int eyeRoll, int eyeIris)
{
	trs_control::getHeadPositionCmd srv_getHeadPositionCmd;
	srv_getHeadPositionCmd.request.headYaw = headYaw;
	srv_getHeadPositionCmd.request.headPitch = headPitch;
	srv_getHeadPositionCmd.request.eyeYaw = eyeYaw;
	srv_getHeadPositionCmd.request.eyePitch = eyePitch;
	srv_getHeadPositionCmd.request.eyeRoll = eyeRoll;
	srv_getHeadPositionCmd.request.eyeIris = eyeIris;
	srv_getHeadPositionCmd.request.recenter = false;

	//call service
	if(srvClt_sendHeadPositionCmd.call(srv_getHeadPositionCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setHeadCommands.publish(srv_getHeadPositionCmd.response.motorCmdSet);
		pub_setHeadCommands.publish(srv_getHeadPositionCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_head_position_cmd");
	}
}

//arms
void QNode::sendArmsLockPositionCmd()
{
	this->sendRightArmLockPositionCmd();
	this->sendLeftArmLockPositionCmd();
}

void QNode::sendArmsCurrentCmd(int currVal)
{
	this->sendRightArmCurrentCmd(currVal);
	this->sendLeftArmCurrentCmd(currVal);
}

void QNode::switchKdTreeAngles(bool state)
{
	trs_control::switchNode srv_switchNode;
	srv_switchNode.request.state = state;

	//call service to switch
	if(srvClt_switchKdTreeAngles.call(srv_switchNode))
	{

	}
	else
	{
		ROS_ERROR("Failed to call service switch_kd_tree_angles");
	}
}

void QNode::switchKdTreeObject(bool state)
{
	trs_control::switchNode srv_switchNode;
	srv_switchNode.request.state = state;

	//call service to switch
	if(srvClt_switchKdTreeObject.call(srv_switchNode))
	{

	}
	else
	{
		ROS_ERROR("Failed to call service switch_kd_tree_object");
	}
}

void QNode::switchKdTreeHead(bool state)
{
	trs_control::switchNode srv_switchNode;
	srv_switchNode.request.state = state;

	//call service to switch
	if(srvClt_switchKdTreeHead.call(srv_switchNode))
	{

	}
	else
	{
		ROS_ERROR("Failed to call service switch_kd_tree_head");
	}
}

//right arm
void QNode::sendRightArmZeroPositionCmd()
{
	trs_control::getSlaveLockCmd srv_getSlaveLockCmd;
	srv_getSlaveLockCmd.request.slaveNb = SLAVE_RIGHT_ARM_ID;

	//call service
	if(srvClt_sendSlaveZeroPositionCmd.call(srv_getSlaveLockCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setArmsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setArmsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_zero_position_cmd, SLAVE_RIGHT_ARM_ID");
	}
}

void QNode::sendRightArmLockPositionCmd()
{
	trs_control::getSlaveLockCmd srv_getSlaveLockCmd;
	srv_getSlaveLockCmd.request.slaveNb = SLAVE_RIGHT_ARM_ID;

	//call service
	if(srvClt_sendSlaveLockCmd.call(srv_getSlaveLockCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setArmsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setArmsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_lock_cmd, SLAVE_RIGHT_ARM_ID");
	}
}

void QNode::sendRightArmCurrentCmd(int currVal)
{
	trs_control::getSlaveCurrentCmd srv_getSlaveCurrentCmd;
	srv_getSlaveCurrentCmd.request.current = currVal;
	srv_getSlaveCurrentCmd.request.slaveNb = SLAVE_RIGHT_ARM_ID;

	//call service
	if(srvClt_sendSlaveCurrentCmd.call(srv_getSlaveCurrentCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setArmsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
		pub_setArmsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_current_cmd, SLAVE_RIGHT_ARM_ID");
	}
}

void QNode::switchHandshake(bool state)
{
	trs_control::switchNode srv_switchNode;
	srv_switchNode.request.state = state;

	//call service to switch
	if(srvClt_switchHandshake.call(srv_switchNode))
	{

	}
	else
	{
		ROS_ERROR("Failed to call service switch_handshake");
	}
}

//left arm
void QNode::sendLeftArmZeroPositionCmd()
{
	trs_control::getSlaveLockCmd srv_getSlaveLockCmd;
	srv_getSlaveLockCmd.request.slaveNb = SLAVE_LEFT_ARM_ID;

	//call service
	if(srvClt_sendSlaveZeroPositionCmd.call(srv_getSlaveLockCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setArmsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setArmsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_zero_position_cmd, SLAVE_LEFT_ARM_ID");
	}
}

void QNode::sendLeftArmLockPositionCmd()
{
	trs_control::getSlaveLockCmd srv_getSlaveLockCmd;
	srv_getSlaveLockCmd.request.slaveNb = SLAVE_LEFT_ARM_ID;

	//call service
	if(srvClt_sendSlaveLockCmd.call(srv_getSlaveLockCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setArmsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setArmsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_lock_cmd, SLAVE_LEFT_ARM_ID");
	}
}

void QNode::sendLeftArmCurrentCmd(int currVal)
{
	trs_control::getSlaveCurrentCmd srv_getSlaveCurrentCmd;
	srv_getSlaveCurrentCmd.request.current = currVal;
	srv_getSlaveCurrentCmd.request.slaveNb = SLAVE_LEFT_ARM_ID;

	//call service
	if(srvClt_sendSlaveCurrentCmd.call(srv_getSlaveCurrentCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setArmsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
		pub_setArmsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
		pub_setHandsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_current_cmd, SLAVE_LEFT_ARM_ID");
	}
}

//legs
void QNode::sendLegsLockPositionCmd()
{
	this->sendRightLegLockPositionCmd();
	this->sendLeftLegLockPositionCmd();
}

void QNode::sendLegsCurrentCmd(int currVal)
{
	this->sendRightLegCurrentCmd(currVal);
	this->sendLeftLegCurrentCmd(currVal);
}

void QNode::switchBalanceLegs(bool state)
{
	trs_control::switchNode srv_switchNode;
	srv_switchNode.request.state = state;

	//call service to switch
	if(srvClt_switchBalanceLegs.call(srv_switchNode))
	{

	}
	else
	{
		ROS_ERROR("Failed to call service switch_balance_legs");
	}
}

void QNode::switchRandomLegsPlayer(bool state)
{
	trs_control::switchNode srv_switchNode;
	srv_switchNode.request.state = state;

	//call service to switch
	if(srvClt_switchRandomLegsPlayer.call(srv_switchNode))
	{

	}
	else
	{
		ROS_ERROR("Failed to call service switch_random_legs_player");
	}
}

//right leg
void QNode::sendRightLegZeroPositionCmd()
{
	trs_control::getSlaveLockCmd srv_getSlaveLockCmd;
	srv_getSlaveLockCmd.request.slaveNb = SLAVE_RIGHT_LEG_ID;

	//call service
	if(srvClt_sendSlaveZeroPositionCmd.call(srv_getSlaveLockCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setLegsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setLegsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_zero_position_cmd, SLAVE_RIGHT_LEG_ID");
	}
}

void QNode::sendRightLegLockPositionCmd()
{
	trs_control::getSlaveLockCmd srv_getSlaveLockCmd;
	srv_getSlaveLockCmd.request.slaveNb = SLAVE_RIGHT_LEG_ID;

	//call service
	if(srvClt_sendSlaveLockCmd.call(srv_getSlaveLockCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setLegsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setLegsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_lock_cmd, SLAVE_RIGHT_LEG_ID");
	}
}

void QNode::sendRightLegCurrentCmd(int currVal)
{
	trs_control::getSlaveCurrentCmd srv_getSlaveCurrentCmd;
	srv_getSlaveCurrentCmd.request.current = currVal;
	srv_getSlaveCurrentCmd.request.slaveNb = SLAVE_RIGHT_LEG_ID;

	//call service
	if(srvClt_sendSlaveCurrentCmd.call(srv_getSlaveCurrentCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setLegsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
		pub_setLegsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_current_cmd, SLAVE_RIGHT_LEG_ID");
	}
}

//left leg
void QNode::sendLeftLegZeroPositionCmd()
{
	trs_control::getSlaveLockCmd srv_getSlaveLockCmd;
	srv_getSlaveLockCmd.request.slaveNb = SLAVE_LEFT_LEG_ID;

	//call service
	if(srvClt_sendSlaveZeroPositionCmd.call(srv_getSlaveLockCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setLegsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setLegsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_zero_position_cmd, SLAVE_LEFT_LEG_ID");
	}
}

void QNode::sendLeftLegLockPositionCmd()
{
	trs_control::getSlaveLockCmd srv_getSlaveLockCmd;
	srv_getSlaveLockCmd.request.slaveNb = SLAVE_LEFT_LEG_ID;

	//call service
	if(srvClt_sendSlaveLockCmd.call(srv_getSlaveLockCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setLegsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
		pub_setLegsCommands.publish(srv_getSlaveLockCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_lock_cmd, SLAVE_LEFT_LEG_ID");
	}
}

void QNode::sendLeftLegCurrentCmd(int currVal)
{
	trs_control::getSlaveCurrentCmd srv_getSlaveCurrentCmd;
	srv_getSlaveCurrentCmd.request.current = currVal;
	srv_getSlaveCurrentCmd.request.slaveNb = SLAVE_LEFT_LEG_ID;

	//call service
	if(srvClt_sendSlaveCurrentCmd.call(srv_getSlaveCurrentCmd))
	{
		//publish twice if an EPOS2 board needs to change mode
		pub_setLegsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
		pub_setLegsCommands.publish(srv_getSlaveCurrentCmd.response.motorCmdSet);
	}
	else
	{
		ROS_ERROR("Failed to call service get_slave_current_cmd, SLAVE_LEFT_LEG_ID");
	}
}

}  // namespace trs_gui
