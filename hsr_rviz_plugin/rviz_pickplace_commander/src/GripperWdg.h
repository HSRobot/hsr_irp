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
