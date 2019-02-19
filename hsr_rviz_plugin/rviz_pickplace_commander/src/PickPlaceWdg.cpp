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
#include "PickPlaceWdg.h"

PickPlaceWdg::PickPlaceWdg(QWidget *parent) : QWidget(parent)
{
	QTextCodec::setCodecForTr(QTextCodec::codecForLocale());	/* 处理中文字符 */
	
    /* 抓取与放置模块*/
    pickPlacetitleLabel = new QLabel(tr("抓取与放置"));
    pickPlacetitleLabel->setStyleSheet("background-color: rgb(227,114,0);color: rgb(255,255,255);");

	/* 抓取模块 */
	tobePickedPoseLabel = new QLabel(tr("待抓取物位姿"));
	getPickPoseMeans = new QComboBox;
	getPickPoseMeans->addItem(tr("自动识别"));
	getPickPoseMeans->addItem(tr("空间读取"));
	getPickPoseMeans->addItem(tr("手动设置"));
	getPickPoseMeans->setCurrentIndex(2);
	getPickPoseOkBtn = new QPushButton(tr("确定"));

    pickXlabel = new QLabel(tr("X"));
	pickXlineEdit = new QLineEdit(tr("0.47"));pickXlineEdit->setAlignment(Qt::AlignLeft);
	pickYlabel = new QLabel(tr("Y"));
	pickYlineEdit = new QLineEdit(tr("-0.4"));pickYlineEdit->setAlignment(Qt::AlignLeft);
	pickZlabel = new QLabel(tr("Z"));
	pickZlineEdit = new QLineEdit(tr("0.2885"));pickZlineEdit->setAlignment(Qt::AlignLeft);

	pickPoseWdg = new QTabWidget;
	pickEulerWdg = new QWidget;
	pickQuaterWdg = new QWidget;
		
	pickEulerRlabel = new QLabel(tr("R"));
	pickEulerRlineEdit = new QLineEdit;pickEulerRlineEdit->setAlignment(Qt::AlignLeft);
	pickEulerPlabel = new QLabel(tr("P"));
	pickEulerPlineEdit = new QLineEdit;pickEulerPlineEdit->setAlignment(Qt::AlignLeft);
	pickEulerYlabel = new QLabel(tr("Y"));
	pickEulerYlineEdit = new QLineEdit;pickEulerYlineEdit->setAlignment(Qt::AlignLeft);
		
	pickEulerLayout = new QHBoxLayout;
	pickEulerLayout->addWidget(pickEulerRlabel);
	pickEulerLayout->addWidget(pickEulerRlineEdit);
	pickEulerLayout->addWidget(pickEulerPlabel);
	pickEulerLayout->addWidget(pickEulerPlineEdit);
	pickEulerLayout->addWidget(pickEulerYlabel);
	pickEulerLayout->addWidget(pickEulerYlineEdit);

	pickEulerWdg->setLayout(pickEulerLayout);
		
	pickQuaterXlabel = new QLabel(tr("x"));
	pickQuaterXlineEdit = new QLineEdit(tr("0.0"));pickQuaterXlineEdit->setAlignment(Qt::AlignLeft);
	pickQuaterYlabel = new QLabel(tr("y"));
	pickQuaterYlineEdit = new QLineEdit(tr("0.0"));pickQuaterYlineEdit->setAlignment(Qt::AlignLeft);
	pickQuaterZlabel = new QLabel(tr("z"));
	pickQuaterZlineEdit = new QLineEdit(tr("0.0"));pickQuaterZlineEdit->setAlignment(Qt::AlignLeft);
	pickQuaterWlabel = new QLabel(tr("w"));
	pickQuaterWlineEdit = new QLineEdit(tr("1.0"));pickQuaterWlineEdit->setAlignment(Qt::AlignLeft);

	pickQuaterLayout = new QHBoxLayout;
	pickQuaterLayout->addWidget(pickQuaterXlabel);
	pickQuaterLayout->addWidget(pickQuaterXlineEdit);
	pickQuaterLayout->addWidget(pickQuaterYlabel);
	pickQuaterLayout->addWidget(pickQuaterYlineEdit);
	pickQuaterLayout->addWidget(pickQuaterZlabel);
	pickQuaterLayout->addWidget(pickQuaterZlineEdit);
	pickQuaterLayout->addWidget(pickQuaterWlabel);
	pickQuaterLayout->addWidget(pickQuaterWlineEdit);

	pickQuaterWdg->setLayout(pickQuaterLayout);

	pickPoseWdg->addTab(pickEulerWdg,tr("欧拉角"));
	pickPoseWdg->addTab(pickQuaterWdg,tr("四元数"));
	pickPoseWdg->setCurrentIndex(1);

	pickLayout = new QGridLayout;

    pickLayout->addWidget(tobePickedPoseLabel,0,0,1,4);
	pickLayout->addWidget(getPickPoseMeans,0,4,1,4);
	pickLayout->addWidget(getPickPoseOkBtn,0,8,1,1);
    pickLayout->addWidget(pickPoseWdg,0,9,2,9);
	pickLayout->addWidget(pickXlabel,1,0,1,1);
	pickLayout->addWidget(pickXlineEdit,1,1,1,2);
	pickLayout->addWidget(pickYlabel,1,3,1,1);
	pickLayout->addWidget(pickYlineEdit,1,4,1,2);
	pickLayout->addWidget(pickZlabel,1,6,1,1);
	pickLayout->addWidget(pickZlineEdit,1,7,1,2);

    /* 放置模块 */
    tobePlacedPoseLabel = new QLabel(tr("目标放置位姿"));
		
	getPlacePoseMeans = new QComboBox;
	getPlacePoseMeans->addItem(tr("空间读取"));
	getPlacePoseMeans->addItem(tr("手动设置"));
	getPlacePoseMeans->setCurrentIndex(1);
		
	getPlacePoseOkBtn = new QPushButton(tr("确定"));

    placeXlabel = new QLabel(tr("X"));
	placeXlineEdit = new QLineEdit(tr("0.43"));placeXlineEdit->setAlignment(Qt::AlignLeft);
	placeYlabel = new QLabel(tr("Y"));
	placeYlineEdit = new QLineEdit(tr("-0.18"));placeYlineEdit->setAlignment(Qt::AlignLeft);
	placeZlabel = new QLabel(tr("Z"));
	placeZlineEdit = new QLineEdit(tr("0.2885"));placeZlineEdit->setAlignment(Qt::AlignLeft);

	placePoseWdg = new QTabWidget;
	placeEulerWdg = new QWidget;
	placeQuaterWdg = new QWidget;
		
	placeEulerAlabel = new QLabel(tr("A"));
	placeEulerAlineEdit = new QLineEdit;placeEulerAlineEdit->setAlignment(Qt::AlignLeft);
	placeEulerBlabel = new QLabel(tr("B"));
	placeEulerBlineEdit = new QLineEdit;placeEulerBlineEdit->setAlignment(Qt::AlignLeft);
	placeEulerClabel = new QLabel(tr("C"));
	placeEulerClineEdit = new QLineEdit;placeEulerClineEdit->setAlignment(Qt::AlignLeft);
		
	placeEulerLayout = new QHBoxLayout;
	placeEulerLayout->addWidget(placeEulerAlabel);
	placeEulerLayout->addWidget(placeEulerAlineEdit);
	placeEulerLayout->addWidget(placeEulerBlabel);
	placeEulerLayout->addWidget(placeEulerBlineEdit);
	placeEulerLayout->addWidget(placeEulerClabel);
	placeEulerLayout->addWidget(placeEulerClineEdit);

	placeEulerWdg->setLayout(placeEulerLayout);
		
	placeQuaterXlabel = new QLabel(tr("x"));
	placeQuaterXlineEdit = new QLineEdit(tr("0.0"));placeQuaterXlineEdit->setAlignment(Qt::AlignLeft);
	placeQuaterYlabel = new QLabel(tr("y"));
	placeQuaterYlineEdit = new QLineEdit(tr("0.0"));placeQuaterYlineEdit->setAlignment(Qt::AlignLeft);
	placeQuaterZlabel = new QLabel(tr("z"));
	placeQuaterZlineEdit = new QLineEdit(tr("0.0"));placeQuaterZlineEdit->setAlignment(Qt::AlignLeft);
	placeQuaterWlabel = new QLabel(tr("w"));
	placeQuaterWlineEdit = new QLineEdit(tr("1.0"));placeQuaterWlineEdit->setAlignment(Qt::AlignLeft);

	placeQuaterLayout = new QHBoxLayout;
	placeQuaterLayout->addWidget(placeQuaterXlabel);
	placeQuaterLayout->addWidget(placeQuaterXlineEdit);
	placeQuaterLayout->addWidget(placeQuaterYlabel);
	placeQuaterLayout->addWidget(placeQuaterYlineEdit);
	placeQuaterLayout->addWidget(placeQuaterZlabel);
	placeQuaterLayout->addWidget(placeQuaterZlineEdit);
	placeQuaterLayout->addWidget(placeQuaterWlabel);
	placeQuaterLayout->addWidget(placeQuaterWlineEdit);

	placeQuaterWdg->setLayout(placeQuaterLayout);

	placePoseWdg->addTab(placeEulerWdg,tr("欧拉角"));
	placePoseWdg->addTab(placeQuaterWdg,tr("四元数"));
	placePoseWdg->setCurrentIndex(1);

	placeLayout = new QGridLayout;

    placeLayout->addWidget(tobePlacedPoseLabel,0,0,1,4);
	placeLayout->addWidget(getPlacePoseMeans,0,4,1,4);
	placeLayout->addWidget(getPlacePoseOkBtn,0,8,1,1);
    placeLayout->addWidget(placePoseWdg,0,9,2,9);
	placeLayout->addWidget(placeXlabel,1,0,1,1);
	placeLayout->addWidget(placeXlineEdit,1,1,1,2);
	placeLayout->addWidget(placeYlabel,1,3,1,1);
	placeLayout->addWidget(placeYlineEdit,1,4,1,2);
	placeLayout->addWidget(placeZlabel,1,6,1,1);
	placeLayout->addWidget(placeZlineEdit,1,7,1,2);

	/* 抓取与放置状态反馈信息栏 */
    pickPlaceStatuslabel = new QLabel(tr("抓取与放置状态反馈信息"));
	
	pickPlaceStatusTextEdit = new QTextEdit;
	pickPlaceStatusTextEdit->setWordWrapMode(QTextOption::NoWrap);
	
	runPickPlaceBtn = new QPushButton(tr("执行抓取与放置"));
    runPickPlaceBtn->setFixedSize(150,50);
	runPickPlaceBtn->setStyleSheet("font-size:20px;background-color: rgb(0,106,213);color: rgb(255,255,255);");
	
	pickPlaceInfoLayout = new QGridLayout;
    pickPlaceInfoLayout->addWidget(pickPlaceStatuslabel,0,0,1,6);
    pickPlaceInfoLayout->addWidget(pickPlaceStatusTextEdit,1,0,2,14);
    pickPlaceInfoLayout->addWidget(runPickPlaceBtn,1,14,2,4);

    /* 抓取与放置模块整体布局 */
	pickPlaceLayout = new QVBoxLayout;
	pickPlaceLayout->addLayout(pickLayout);
	pickPlaceLayout->addLayout(placeLayout);
	pickPlaceLayout->addLayout(pickPlaceInfoLayout);
		
	setLayout(pickPlaceLayout);

}
PickPlaceWdg::~PickPlaceWdg()
{
	//
	delete pickPlaceLayout;
	delete pickPlaceInfoLayout;
	delete runPickPlaceBtn;
	delete pickPlaceStatusTextEdit;
	delete pickPlaceStatuslabel;
	delete placeLayout;
	delete placeQuaterLayout;	
	delete placeQuaterWlineEdit;
	delete placeQuaterWlabel;
	delete placeQuaterZlineEdit;
	delete placeQuaterZlabel;
	delete placeQuaterYlineEdit;
	delete placeQuaterYlabel;
	delete placeQuaterXlineEdit;
	delete placeQuaterXlabel;
	delete placeEulerLayout;	
	delete placeEulerClineEdit;
	delete placeEulerClabel;
	delete placeEulerBlineEdit;
	delete placeEulerBlabel;
	delete placeEulerAlineEdit;
	delete placeEulerAlabel;
	delete placeQuaterWdg;
	delete placeEulerWdg;
	delete placePoseWdg;
	delete placeZlineEdit;
	delete placeZlabel;
	delete placeYlineEdit;
	delete placeYlabel;
	delete placeXlineEdit;
	delete placeXlabel;
	delete getPlacePoseOkBtn;
	delete getPlacePoseMeans;
	delete tobePlacedPoseLabel;
	delete pickLayout;
	delete pickQuaterLayout;	
	delete pickQuaterWlineEdit;
	delete pickQuaterWlabel;
	delete pickQuaterZlineEdit;
	delete pickQuaterZlabel;
	delete pickQuaterYlineEdit;
	delete pickQuaterYlabel;
	delete pickQuaterXlineEdit;
	delete pickQuaterXlabel;	
	delete pickEulerLayout;
	delete pickEulerYlineEdit;
	delete pickEulerYlabel;
	delete pickEulerPlineEdit;
	delete pickEulerPlabel;
	delete pickEulerRlineEdit;
	delete pickEulerRlabel;
	delete pickQuaterWdg;
	delete pickEulerWdg;
	delete pickPoseWdg;
	delete pickZlineEdit;
	delete pickZlabel;
	delete pickYlineEdit;
	delete pickYlabel;
	delete pickXlineEdit;	
	delete pickXlabel;
	delete getPickPoseOkBtn;
	delete getPickPoseMeans;
	delete tobePickedPoseLabel;
	delete pickPlacetitleLabel;
}
