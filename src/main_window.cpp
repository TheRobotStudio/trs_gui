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
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date October 2014
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/trs_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace trs_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	setWindowIcon(QIcon(":/images/icon.png"));
	QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    //Connect the node to ROS
    qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

//whole robot
void MainWindow::on_pb_switchConductor_clicked()
{
	qnode.switchConductor(ui.pb_switchConductor->isChecked());
}

void MainWindow::on_pb_sendRobotZeroPositionCmd_clicked()
{
	qnode.sendRobotZeroPositionCmd();
}

void MainWindow::on_pb_sendRobotLockPositionCmd_clicked()
{
	qnode.sendRobotLockPositionCmd();
}

void MainWindow::on_pb_sendRobotCurrentCmd_clicked()
{
	qnode.sendRobotCurrentCmd(100);
}

void MainWindow::on_pb_switchRandomPosturePlayer_clicked()
{
	qnode.switchRandomPosturePlayer(ui.pb_switchRandomPosturePlayer->isChecked());
}

//head
void MainWindow::on_pb_switchFaceTracking_clicked()
{
	qnode.switchFaceTracking(ui.pb_switchFaceTracking->isChecked());
}

void MainWindow::on_pb_switchObjectTracking_clicked()
{
	qnode.switchObjectTracking(ui.pb_switchObjectTracking->isChecked());
}

void MainWindow::on_pb_sendRecenterHeadCmd_clicked()
{
	qnode.sendRecenterHeadCmd();
}

void MainWindow::on_pb_sendHeadPositionCmd_clicked()
{
	qnode.sendHeadPositionCmd(ui.hslid_headYaw->value(), ui.vslid_headPitch->value(), ui.hslid_eyeYaw->value(), ui.vslid_eyePitch->value(), ui.hslid_eyeRoll->value(), ui.hslid_eyeIris->value());
}

//arms
void MainWindow::on_pb_sendArmsLockPositionCmd_clicked()
{
	qnode.sendArmsLockPositionCmd();
}

void MainWindow::on_pb_sendArmsCurrentCmd_clicked()
{
	qnode.sendArmsCurrentCmd(ui.hslid_armsCurrent->value());
}

void MainWindow::on_pb_switchKdTreeAngles_clicked()
{
	qnode.switchKdTreeAngles(ui.pb_switchKdTreeAngles->isChecked());
}

void MainWindow::on_pb_switchKdTreeObject_clicked()
{
	qnode.switchKdTreeObject(ui.pb_switchKdTreeObject->isChecked());
}

void MainWindow::on_pb_switchKdTreeHead_clicked()
{
	qnode.switchKdTreeHead(ui.pb_switchKdTreeHead->isChecked());
}

//right arm
void MainWindow::on_pb_sendRightArmZeroPositionCmd_clicked()
{
	qnode.sendRightArmZeroPositionCmd();
}

void MainWindow::on_pb_sendRightArmLockPositionCmd_clicked()
{
	qnode.sendRightArmLockPositionCmd();
}

void MainWindow::on_pb_sendRightArmCurrentCmd_clicked()
{
	qnode.sendRightArmCurrentCmd(ui.hslid_rightArmCurrent->value());
}

void MainWindow::on_pb_switchHandshake_clicked()
{
	qnode.switchHandshake(ui.pb_switchHandshake->isChecked());
}


//left arm
void MainWindow::on_pb_sendLeftArmZeroPositionCmd_clicked()
{
	qnode.sendLeftArmZeroPositionCmd();
}

void MainWindow::on_pb_sendLeftArmLockPositionCmd_clicked()
{
	qnode.sendLeftArmLockPositionCmd();
}

void MainWindow::on_pb_sendLeftArmCurrentCmd_clicked()
{
	qnode.sendLeftArmCurrentCmd(ui.hslid_leftArmCurrent->value());
}

//legs
void MainWindow::on_pb_sendLegsLockPositionCmd_clicked()
{
	qnode.sendLegsLockPositionCmd();
}

void MainWindow::on_pb_sendLegsCurrentCmd_clicked()
{
	qnode.sendLegsCurrentCmd(ui.hslid_legsCurrent->value());
}

void MainWindow::on_pb_switchBalanceLegs_clicked()
{
	qnode.switchBalanceLegs(ui.pb_switchBalanceLegs->isChecked());
}

void MainWindow::on_pb_switchRandomLegsPlayer_clicked()
{
	qnode.switchRandomLegsPlayer(ui.pb_switchRandomLegsPlayer->isChecked());
}

//right leg
void MainWindow::on_pb_sendRightLegZeroPositionCmd_clicked()
{
	qnode.sendRightLegZeroPositionCmd();
}

void MainWindow::on_pb_sendRightLegLockPositionCmd_clicked()
{
	qnode.sendRightLegLockPositionCmd();
}

void MainWindow::on_pb_sendRightLegCurrentCmd_clicked()
{
	qnode.sendRightLegCurrentCmd(ui.hslid_rightLegCurrent->value());
}

//left leg
void MainWindow::on_pb_sendLeftLegZeroPositionCmd_clicked()
{
	qnode.sendLeftLegZeroPositionCmd();
}

void MainWindow::on_pb_sendLeftLegLockPositionCmd_clicked()
{
	qnode.sendLeftLegLockPositionCmd();
}

void MainWindow::on_pb_sendLeftLegCurrentCmd_clicked()
{
	qnode.sendLeftLegCurrentCmd(ui.hslid_leftLegCurrent->value());
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, tr("About ..."),tr("<h2>The Robot Studio Program 0.10</h2><p>Open Source Android Robot</p><p>This GUI has been made to manage the robot demo.</p>"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace trs_gui

