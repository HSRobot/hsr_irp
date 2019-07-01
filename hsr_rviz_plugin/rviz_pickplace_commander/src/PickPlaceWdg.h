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
* @file		PickPlaceWdg.h
* @brief	抓取与放置模块头文件
* @details	抓取与放置模块头文件，与PickPlaceWdg.cpp文件对应
* @autor 	wanghaoqing
* @date		2018/10/10
*/

#ifndef PICKPLACEWDG_H
#define PICKPLACEWDG_H

#include <QWidget>
#include <QTabWidget>

#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QScrollBar>
#include <QLineEdit>
#include <QTextEdit>
#include <QTextCodec>


#include<QGridLayout>
#include<QHBoxLayout>
#include<QVBoxLayout>

class PickPlaceWdg : public QWidget
{
    Q_OBJECT
public:
    explicit PickPlaceWdg(QWidget *parent = 0);
	~PickPlaceWdg();

private:

public:

    QLabel *pickPlacetitleLabel;            /* 抓取与放置模块标题*/
	QLabel *tobePickedPoseLabel;            /* 待抓取物位姿显示标签 */
    QComboBox *getPickPoseMeans;            /* 获取抓取物位姿方式下拉选择框 */
    QPushButton *getPickPoseOkBtn;          /* 获取位姿确认按钮 */

	QLabel *pickXlabel;                     /* 待抓取物体坐标X 标签*/
    QLabel *pickYlabel;                     /* 待抓取物体坐标Y 标签*/
    QLabel *pickZlabel;                     /* 待抓取物体坐标Z 标签*/
    QLineEdit *pickXlineEdit;               /* 待抓取物体坐标X 输入框*/
	QLineEdit *pickYlineEdit;			    /* 待抓取物体坐标Y 输入框*/
	QLineEdit *pickZlineEdit;			    /* 待抓取物体坐标Z 输入框*/

	QTabWidget *pickPoseWdg;                /* 抓取模块姿态窗体 */
	QWidget *pickEulerWdg;                  /* 抓取模块欧拉角窗体 */
	QWidget *pickQuaterWdg;                 /* 抓取模块四元数窗体 */
	
    QLabel *pickEulerRlabel;                /* 抓取物体姿态R 标签*/
    QLabel *pickEulerPlabel;                /* 抓取物体姿态P 标签*/
    QLabel *pickEulerYlabel;                /* 抓取物体姿态Y 标签*/
    QLineEdit *pickEulerRlineEdit;          /* 抓取物体姿态R 输入框*/
	QLineEdit *pickEulerPlineEdit;			/* 抓取物体姿态P 输入框*/
	QLineEdit *pickEulerYlineEdit;			/* 抓取物体姿态Y 输入框*/
    QHBoxLayout *pickEulerLayout;           /* 抓取模块欧拉角窗口布局 */

    QLabel *pickQuaterXlabel;               /* 抓取物体姿态X 标签*/
    QLabel *pickQuaterYlabel;               /* 抓取物体姿态Y 标签*/
    QLabel *pickQuaterZlabel;               /* 抓取物体姿态Z 标签*/
	QLabel *pickQuaterWlabel;               /* 抓取物体姿态W 标签*/
    QLineEdit *pickQuaterXlineEdit;         /* 抓取物体姿态X 输入框*/
	QLineEdit *pickQuaterYlineEdit;			/* 抓取物体姿态Y 输入框*/
	QLineEdit *pickQuaterZlineEdit;			/* 抓取物体姿态Z 输入框*/
	QLineEdit *pickQuaterWlineEdit;			/* 抓取物体姿态W 输入框*/
    QHBoxLayout *pickQuaterLayout;          /* 抓取模块四元数窗口布局 */

    QGridLayout *pickLayout;                /* 抓取部分布局 */
	
	QLabel *tobePlacedPoseLabel;            /* 待放置物位姿显示标签 */
    QComboBox *getPlacePoseMeans;           /* 获取放置物位姿方式下拉选择框 */
    QPushButton *getPlacePoseOkBtn;         /* 获取位姿确认按钮 */

	QLabel *placeXlabel;                     /* 待放置物体坐标X 标签*/
    QLabel *placeYlabel;                     /* 待放置物体坐标Y 标签*/
    QLabel *placeZlabel;                     /* 待放置物体坐标Z 标签*/
    QLineEdit *placeXlineEdit;               /* 待放置物体坐标X 输入框*/
	QLineEdit *placeYlineEdit;			     /* 待放置物体坐标Y 输入框*/
	QLineEdit *placeZlineEdit;			     /* 待放置物体坐标Z 输入框*/

	QTabWidget *placePoseWdg;                /* 放置模块姿态窗体 */
	QWidget *placeEulerWdg;                  /* 放置模块欧拉角窗体 */
	QWidget *placeQuaterWdg;                 /* 放置模块四元数窗体 */
	
    QLabel *placeEulerAlabel;                /* 放置物体姿态A 标签*/
    QLabel *placeEulerBlabel;                /* 放置物体姿态B 标签*/
    QLabel *placeEulerClabel;                /* 放置物体姿态C 标签*/
    QLineEdit *placeEulerAlineEdit;          /* 放置物体姿态A 输入框*/
	QLineEdit *placeEulerBlineEdit;			 /* 放置物体姿态B 输入框*/
	QLineEdit *placeEulerClineEdit;			 /* 放置物体姿态C 输入框*/
    QHBoxLayout *placeEulerLayout;           /* 放置模块欧拉角窗口布局 */

    QLabel *placeQuaterXlabel;               /* 放置物体姿态X 标签*/
    QLabel *placeQuaterYlabel;               /* 放置物体姿态Y 标签*/
    QLabel *placeQuaterZlabel;               /* 放置物体姿态Z 标签*/
	QLabel *placeQuaterWlabel;               /* 放置物体姿态W 标签*/
    QLineEdit *placeQuaterXlineEdit;         /* 放置物体姿态X 输入框*/
	QLineEdit *placeQuaterYlineEdit;	     /* 放置物体姿态Y 输入框*/
	QLineEdit *placeQuaterZlineEdit;		 /* 放置物体姿态Z 输入框*/
	QLineEdit *placeQuaterWlineEdit;		 /* 放置物体姿态W 输入框*/
    QHBoxLayout *placeQuaterLayout;          /* 放置模块四元数窗口布局 */

    QGridLayout *placeLayout;                 /* 放置部分布局 */

    QLabel *pickPlaceStatuslabel;            /* 抓取与放置状态反馈信息 标签*/
    QTextEdit *pickPlaceStatusTextEdit;      /* 抓取与放置状态反馈信息文本输入框 */
	QPushButton *runPickPlaceBtn;            /* 执行抓取与放置按钮 */
    QGridLayout *pickPlaceInfoLayout;        /* 抓取与放置反馈信息布局 */
	
	QVBoxLayout *pickPlaceLayout;            /* 抓取与放置模块布局 */

};

#endif // SERIALWDG_H
