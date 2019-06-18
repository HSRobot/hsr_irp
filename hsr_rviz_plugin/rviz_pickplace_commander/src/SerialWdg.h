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
