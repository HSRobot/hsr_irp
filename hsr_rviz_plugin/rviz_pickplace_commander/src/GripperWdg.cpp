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

