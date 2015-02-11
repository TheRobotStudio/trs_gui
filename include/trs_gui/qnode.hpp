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
 * @file /include/trs_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date October 2014
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef trs_gui_QNODE_HPP_
#define trs_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <trs_msgs/MotorDataSet.h>
#include <trs_msgs/MotorCmdSet.h>
#include <std_msgs/Int32MultiArray.h>
#include "trs_control/switchNode.h"
#include "trs_control/getMotorCmdSet.h"
#include "trs_control/getSlaveCurrentCmd.h"
#include "trs_control/getSlaveLockCmd.h"
#include "trs_control/getHeadPositionCmd.h"
#include "robotDefines.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace trs_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

	//whole robot
	void switchConductor(bool state);
	void sendRobotZeroPositionCmd();
	void sendRobotLockPositionCmd();
	void sendRobotCurrentCmd(int currVal);
	void switchRandomPosturePlayer(bool state);

	//head
	void switchFaceTracking(bool state);
	void switchObjectTracking(bool state);
	void sendRecenterHeadCmd();
	void sendHeadPositionCmd(int headYaw, int headPitch, int eyeYaw, int eyePitch, int eyeRoll, int eyeIris);

	//arms
	void sendArmsLockPositionCmd();
	void sendArmsCurrentCmd(int currVal);
	void switchKdTreeAngles(bool state);
	void switchKdTreeObject(bool state);
	void switchKdTreeHead(bool state);

	//right arm
	void sendRightArmZeroPositionCmd();
	void sendRightArmLockPositionCmd();
	void sendRightArmCurrentCmd(int currVal);
	void switchHandshake(bool state);

	//left arm
	void sendLeftArmZeroPositionCmd();
	void sendLeftArmLockPositionCmd();
	void sendLeftArmCurrentCmd(int currVal);

	//legs
	void sendLegsLockPositionCmd();
	void sendLegsCurrentCmd(int currVal);
	void switchBalanceLegs(bool state);
	void switchRandomLegsPlayer(bool state);

	//right leg
	void sendRightLegZeroPositionCmd();
	void sendRightLegLockPositionCmd();
	void sendRightLegCurrentCmd(int currVal);

	//left leg
	void sendLeftLegZeroPositionCmd();
	void sendLeftLegLockPositionCmd();
	void sendLeftLegCurrentCmd(int currVal);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

public :

private:
	int init_argc;
	char** init_argv;

	//Publishers
	ros::Publisher pub_setArmsCommands;
	ros::Publisher pub_setHandsCommands;
	ros::Publisher pub_setHeadCommands;
	ros::Publisher pub_setLegsCommands;

	//Services
	ros::ServiceClient srvClt_switchConductor;
	ros::ServiceClient srvClt_switchRandomPosturePlayer;
	ros::ServiceClient srvClt_switchFaceTracking;
	ros::ServiceClient srvClt_switchObjectTracking;
	ros::ServiceClient srvClt_switchKdTreeAngles;
	ros::ServiceClient srvClt_switchKdTreeObject;
	ros::ServiceClient srvClt_switchKdTreeHead;
	ros::ServiceClient srvClt_switchHandshake;
	ros::ServiceClient srvClt_switchBalanceLegs;
	ros::ServiceClient srvClt_switchRandomLegsPlayer;

	ros::ServiceClient srvClt_sendRobotZeroPositionCmd;
	ros::ServiceClient srvClt_sendSlaveZeroPositionCmd;
	ros::ServiceClient srvClt_sendSlaveCurrentCmd;
	ros::ServiceClient srvClt_sendSlaveLockCmd;
	ros::ServiceClient srvClt_sendHeadPositionCmd;

    QStringListModel logging_model;
};

}  // namespace trs_gui

#endif /* trs_gui_QNODE_HPP_ */
