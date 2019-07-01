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
#include "GripperWdg.h"

GripperWdg::GripperWdg(QWidget *parent) : QWidget(parent)
{
	QTextCodec::setCodecForTr(QTextCodec::codecForLocale());	/* 处理中文字符 */
	
    gripperTitleLabel = new QLabel(tr("夹爪"));
	gripperTitleLabel->setStyleSheet("background-color: rgb(227,114,0);color: rgb(255,255,255);");
		
	gripperStatusLabel = new QLabel(tr("当前夹爪关闭，速度为500，力矩为100，最大开口1000，最小开口0"));
	gripperSetLabel = new QLabel(tr("夹爪参数"));
	gripperSetLabel->setFixedSize(60,30);
		
	gripperSetComoBox = new QComboBox;
	gripperSetComoBox->setFixedSize(100,30);
    gripperSetComoBox->addItem(tr("开口大小:"));
	gripperSetComoBox->addItem(tr("夹爪速度:"));
	gripperSetComoBox->addItem(tr("夹爪力矩:"));
		
	openMaxLabel = new QLabel(tr("最大开口:"));
    openMaxLineEdit = new QLineEdit;
	openMaxLineEdit->setFixedSize(100,30);
	openMaxLineEdit->setText(tr("1000"));
	openMaxLineEdit->setValidator(new QIntValidator(0, 1000, this));
	openMaxLineEdit->setPlaceholderText(tr("输入0-1000的整数"));
	openMaxLineEdit->setToolTip(tr("输入0-1000的整数，数值越大开口越大，\r\n且该值需要大于【开口最小值】"));

	openMinLabel = new QLabel(tr("最小开口:"));
    openMinLineEdit = new QLineEdit;
	openMinLineEdit->setFixedSize(100,30);
	openMinLineEdit->setText(tr("0"));
	openMinLineEdit->setValidator(new QIntValidator(0, 1000, this));
	openMinLineEdit->setPlaceholderText(tr("输入0-1000的整数"));
	openMinLineEdit->setToolTip(tr("输入0-1000的整数，数值越大开口越大，\r\n且该值需要小于【开口最大值】"));
	
	speedOrForceValLabel = new QLabel(tr("大小:"));
    speedOrForceValLineEdit = new QLineEdit;
	gripperSetBtn = new QPushButton(tr("设置"));
	gripperActLabel = new QLabel(tr("夹爪动作:"));
		
    gripperActComoBox = new QComboBox;
	gripperActComoBox->addItem(tr("夹爪打开"));
	gripperActComoBox->addItem(tr("夹爪关闭"));
	gripperActComoBox->addItem(tr("夹爪急停"));
		
	gripperActRunBtn = new QPushButton(tr("执行"));

	gripperLayout = new QGridLayout;
	gripperLayout->addWidget(gripperStatusLabel,0,0,1,11);
	gripperLayout->addWidget(gripperSetLabel,1,0,1,2);
	gripperLayout->addWidget(gripperSetComoBox,1,2,1,2);
	gripperLayout->addWidget(openMaxLabel,1,4,1,2);
	gripperLayout->addWidget(openMaxLineEdit,1,6,1,1);
	gripperLayout->addWidget(openMinLabel,1,7,1,2);
	gripperLayout->addWidget(openMinLineEdit,1,9,1,1);
	gripperLayout->addWidget(gripperSetBtn,1,10,1,1);
	gripperLayout->addWidget(gripperActLabel,2,0,1,2);
	gripperLayout->addWidget(gripperActComoBox,2,2,1,2);
	gripperLayout->addWidget(gripperActRunBtn,2,10,1,1);
		
	setLayout(gripperLayout);
}
GripperWdg::~GripperWdg()
{
	//
	delete gripperLayout;
	delete gripperActRunBtn;
	delete gripperActComoBox;
	delete gripperActLabel;
	delete gripperSetBtn;
	delete speedOrForceValLineEdit;
	delete speedOrForceValLabel;
	delete openMinLineEdit;
	delete openMinLabel;
	delete openMaxLineEdit;
	delete openMaxLabel;
	delete gripperSetComoBox;	
	delete gripperSetLabel;
	delete gripperStatusLabel;
	delete gripperTitleLabel;
}

