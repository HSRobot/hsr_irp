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
* @file		SerialWdg.h
* @brief	串口模块头文件
* @details	串口模块头文件，与SerialWdg.cpp文件对应
* @autor 	wanghaoqing
* @date		2018/10/10
*/

#ifndef SERIALWDG_H
#define SERIALWDG_H

#include <QWidget>

#include "MyComboBox.h"    /* 里面包含 QComboBox */

#include <QLabel>
#include<QPushButton>
#include <QTextCodec>

#include<QHBoxLayout>

class SerialWdg : public QWidget
{
    Q_OBJECT
public:
    explicit SerialWdg(QWidget *parent = 0);
    ~SerialWdg();	

private:

public:
	QLabel *serialTitleLabel;               /* 串口模块标题 */
	QLabel *serialNoLabel;                  /* 串口设备号标题 */
	//QComboBox *serialNoComoBox;           /* 串口设备选择下拉框 */
    MyComboBox *serialNoComoBox;            /* 串口设备选择下拉框 */
    QLabel *baudrateLabel;                  /* 波特率设置标题 */
	QComboBox *baudrateComoBox;             /* 波特率选择下拉框 */
    QPushButton *serialConnectOrNotBtn;     /* 串口连接或断开按钮 */

	QHBoxLayout *serialLayout;	            /* 串口模块布局 */

};

#endif // SERIALWDG_H
