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

#include "SerialWdg.h"

SerialWdg::SerialWdg(QWidget *parent) : QWidget(parent)
{
    QTextCodec::setCodecForTr(QTextCodec::codecForLocale());	/* 处理中文字符 */
	
	/* 串口模块 */
    serialTitleLabel = new QLabel(tr("串口"));
	serialTitleLabel->setStyleSheet("background-color: rgb(227,114,0);color: rgb(255,255,255);");
		
	serialNoLabel = new QLabel(tr("串口设备号："));
	serialNoComoBox = new MyComboBox();
		
	baudrateLabel = new QLabel(tr("波特率："));
	baudrateComoBox = new QComboBox;
	baudrateComoBox->addItem(tr("115200"));
	baudrateComoBox->addItem(tr("9600"));
	serialConnectOrNotBtn = new QPushButton(tr("连接"));

	serialLayout = new QHBoxLayout;
	serialLayout->addWidget(serialNoLabel,2);
	serialLayout->addWidget(serialNoComoBox,2);
	serialLayout->addWidget(baudrateLabel,1);
	serialLayout->addWidget(baudrateComoBox,2);
	serialLayout->addStretch(4);
	serialLayout->addWidget(serialConnectOrNotBtn,1);
		
	setLayout(serialLayout);

}
SerialWdg::~SerialWdg()
{
	//
	delete serialLayout;
	delete serialConnectOrNotBtn;
	delete baudrateComoBox;
	delete baudrateLabel;
	delete serialNoComoBox;
	delete serialNoLabel;
	delete serialTitleLabel;
}
