/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Foshan Huashu Robotics Co.,Ltd
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
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
/** 
* @version 	v1.0.0 
* @file		GripperWdg.h
* @brief	夹爪模块头文件
* @details	夹爪模块头文件，与GripperWdg.cpp文件对应
* @autor 	wanghaoqing
* @date		2018/10/10
*/

#ifndef GRIPPERWDG_H
#define GRIPPERWDG_H

#include <QWidget>

#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QTextCodec>

#include<QGridLayout>

class GripperWdg : public QWidget
{
    Q_OBJECT
public:
    explicit GripperWdg(QWidget *parent = 0);
    ~GripperWdg();

private:

public:
	QLabel *gripperTitleLabel;              /* 夹爪模块标题*/
	QLabel *gripperStatusLabel;             /* 夹爪状态显示标签 */
	QLabel *gripperSetLabel;                /* 夹爪设置标签*/
	QComboBox *gripperSetComoBox;           /* 夹爪设置项选择下拉框 */   
    QLabel *openMaxLabel;                   /* 夹爪最大开口标签*/
    QLineEdit *openMaxLineEdit;             /* 串口最大开口输入框*/
    QLabel *openMinLabel;                   /* 夹爪最小开口标签*/
    QLineEdit *openMinLineEdit;             /* 串口最小开口输入框*/
    QLabel *speedOrForceValLabel;           /* 速度值或力矩值标签*/
    QLineEdit *speedOrForceValLineEdit;     /* 速度值或力矩值输入框*/
    QPushButton *gripperSetBtn;             /* 夹爪设置按钮 */
    QLabel *gripperActLabel;                /* 夹爪动作标签*/
	QComboBox *gripperActComoBox;           /* 夹爪动作选择下拉框 */
    QPushButton *gripperActRunBtn;          /* 夹爪动作执行按钮 */

	QGridLayout *gripperLayout;             /* 夹爪模块布局 */

};

#endif // SERIALWDG_H
