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
* @file		OrkWdg.h
* @brief	识别与定位模块头文件
* @details	识别与定位模块头文件，与OrkWdg.cpp文件对应
* @autor 	wanghaoqing
* @date		2018/10/10
*/

#ifndef ORKWDG_H
#define ORKWDG_H

#include <QWidget>
#include <QTabWidget>

#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QTextCodec>

#include<QGridLayout>
#include<QHBoxLayout>

class OrkWdg : public QWidget
{
    Q_OBJECT
public:
    explicit OrkWdg(QWidget *parent = 0);
    ~OrkWdg();

private:

public:

    QLabel *ORKtitleLabel;                  /* 识别与定位模块标题*/
	QLabel *newAddModelNameLabel;           /* 新增加训练模型名称标签 */
	QComboBox *addOrDeleteComoBox;          /* 新增或删除模型选择下拉框 */
	QLineEdit *newAddModelNameLineEdit;     /* 新增加训练模型名称输入框 */
	QComboBox *toSelectDeleteComoBox;       /* 待删除项选择下拉框 */
	QLabel *newAddModelDescripLabel;        /* 新增加训练模型描述标签 */
	QLineEdit *newAddModelDescripLineEdit;  /* 新增加训练模型描述输入框 */
	QPushButton *newAddOrDeleteOkBtn;       /* 新增加或删除训练模型确认按钮 */
	QLabel *tobeTrainModelLabel;            /* 待训练模型显示标签 */
	QLineEdit *modelPathLineEdit;           /* 待训练模型路径标签*/	
    QPushButton *modelPathOpenBtn;          /* 待训练模型路径按钮 */
	QPushButton *addMeshOkBtn;              /* 新增加模型Mesh确认按钮 */
	QPushButton *toTrainBtn;                /* 开始训练按钮 */

	QLabel *canRecognModelLabel;            /* 可识别模型标签 */
    QComboBox *canRecognModelComboBox;      /* 可识别模型选择下拉框 */
    QPushButton *toRecognBtn;               /* 开始识别按钮 */

	QLabel *poseLabel;                      /* 位置与姿态标签 */
    QComboBox *recognizedPoseComboBox;      /* 已识别出物体位姿选择下拉框 */
    QLabel *ORKposeXlabel;                  /* 识别出的物体坐标X 标签*/
    QLabel *ORKposeYlabel;                  /* 识别出的物体坐标Y 标签*/
    QLabel *ORKposeZlabel;                  /* 识别出的物体坐标Z 标签*/
    QLineEdit *ORKposeXlineEdit;            /* 识别出的物体坐标X 输入框*/
	QLineEdit *ORKposeYlineEdit;			/* 识别出的物体坐标Y 输入框*/
	QLineEdit *ORKposeZlineEdit;			/* 识别出的物体坐标Z 输入框*/

    QTabWidget *ORKposeWdg;                 /* 识别与定位模块姿态窗体 */
	QWidget *ORKeulerWdg;                   /* 识别与定位模块欧拉角窗体 */
	QWidget *ORKquaterWdg;                  /* 识别与定位模块四元数窗体 */
	
    QLabel *ORKeulerRlabel;                 /* 识别出的物体姿态R 标签*/
    QLabel *ORKeulerPlabel;                 /* 识别出的物体姿态P 标签*/
    QLabel *ORKeulerYlabel;                 /* 识别出的物体姿态Y 标签*/
    QLineEdit *ORKeulerRlineEdit;           /* 识别出的物体姿态R 输入框*/
	QLineEdit *ORKeulerPlineEdit;			/* 识别出的物体姿态P 输入框*/
	QLineEdit *ORKeulerYlineEdit;			/* 识别出的物体姿态Y 输入框*/
    QHBoxLayout *ORKeulerLayout;            /* 识别与定位模块欧拉角窗口布局 */
	
    QLabel *ORKquaterXlabel;                 /* 识别出的物体姿态X 标签*/
    QLabel *ORKquaterYlabel;                 /* 识别出的物体姿态Y 标签*/
    QLabel *ORKquaterZlabel;                 /* 识别出的物体姿态Z 标签*/
	QLabel *ORKquaterWlabel;                 /* 识别出的物体姿态W 标签*/
    QLineEdit *ORKquaterXlineEdit;           /* 识别出的物体姿态X 输入框*/
	QLineEdit *ORKquaterYlineEdit;			 /* 识别出的物体姿态Y 输入框*/
	QLineEdit *ORKquaterZlineEdit;			 /* 识别出的物体姿态Z 输入框*/
	QLineEdit *ORKquaterWlineEdit;			 /* 识别出的物体姿态W 输入框*/
    QHBoxLayout *ORKquaterLayout;            /* 识别与定位模块四元数窗口布局 */

	QGridLayout *ORKLayout;                  /* 识别与定位模块布局 */

};

#endif // SERIALWDG_H
