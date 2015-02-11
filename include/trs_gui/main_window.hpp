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
 * @file /include/trs_gui/main_window.hpp
 *
 * @brief Qt based gui for trs_gui.
 *
 * @date October 2014
 **/

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef trs_gui_MAIN_WINDOW_H
#define trs_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace trs_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();

	//whole robot
	void on_pb_switchConductor_clicked();
	void on_pb_sendRobotZeroPositionCmd_clicked();
	void on_pb_sendRobotLockPositionCmd_clicked();
	void on_pb_sendRobotCurrentCmd_clicked();
	void on_pb_switchRandomPosturePlayer_clicked();

	//head
	void on_pb_switchFaceTracking_clicked();
	void on_pb_switchObjectTracking_clicked();
	void on_pb_sendRecenterHeadCmd_clicked();
	void on_pb_sendHeadPositionCmd_clicked();

	//arms
	void on_pb_sendArmsLockPositionCmd_clicked();
	void on_pb_sendArmsCurrentCmd_clicked();
	void on_pb_switchKdTreeAngles_clicked();
	void on_pb_switchKdTreeObject_clicked();
	void on_pb_switchKdTreeHead_clicked();

	//right arm
	void on_pb_sendRightArmZeroPositionCmd_clicked();
	void on_pb_sendRightArmLockPositionCmd_clicked();
	void on_pb_sendRightArmCurrentCmd_clicked();
	void on_pb_switchHandshake_clicked();

	//left arm
	void on_pb_sendLeftArmZeroPositionCmd_clicked();
	void on_pb_sendLeftArmLockPositionCmd_clicked();
	void on_pb_sendLeftArmCurrentCmd_clicked();

	//legs
	void on_pb_sendLegsLockPositionCmd_clicked();
	void on_pb_sendLegsCurrentCmd_clicked();
	void on_pb_switchBalanceLegs_clicked();
	void on_pb_switchRandomLegsPlayer_clicked();

	//right leg
	void on_pb_sendRightLegZeroPositionCmd_clicked();
	void on_pb_sendRightLegLockPositionCmd_clicked();
	void on_pb_sendRightLegCurrentCmd_clicked();

	//left leg
	void on_pb_sendLeftLegZeroPositionCmd_clicked();
	void on_pb_sendLeftLegLockPositionCmd_clicked();
	void on_pb_sendLeftLegCurrentCmd_clicked();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace trs_gui

#endif // trs_gui_MAIN_WINDOW_H
