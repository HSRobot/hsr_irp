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
