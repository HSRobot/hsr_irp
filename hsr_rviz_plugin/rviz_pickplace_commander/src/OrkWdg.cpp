#include "OrkWdg.h"

OrkWdg::OrkWdg(QWidget *parent) : QWidget(parent)
{
	QTextCodec::setCodecForTr(QTextCodec::codecForLocale());	/* 处理中文字符 */
	
    ORKtitleLabel = new QLabel(tr("识别与定位"));
	ORKtitleLabel->setStyleSheet("background-color: rgb(227,114,0);color: rgb(255,255,255);");

    newAddModelNameLabel = new QLabel(tr("新增模型名:"));
		
	addOrDeleteComoBox = new QComboBox;
	addOrDeleteComoBox->setFixedSize(100,30);
    addOrDeleteComoBox->addItem(tr("新增模型:"));
	addOrDeleteComoBox->addItem(tr("删除模型:"));
		
	newAddModelNameLineEdit = new QLineEdit;
	toSelectDeleteComoBox = new QComboBox; 
	newAddModelDescripLabel = new QLabel(tr("描述:"));
	newAddModelDescripLineEdit = new QLineEdit;
	newAddOrDeleteOkBtn = new QPushButton(tr("确认添加"));
		
	tobeTrainModelLabel = new QLabel(tr("待训练模型:"));
    modelPathLineEdit = new QLineEdit;
	modelPathOpenBtn = new QPushButton(tr("路径"));
	addMeshOkBtn = new QPushButton(tr("确认"));
	toTrainBtn = new QPushButton(tr("开始训练"));
	
	canRecognModelLabel = new QLabel(tr("可识别模型:"));
    canRecognModelComboBox = new QComboBox;
	toRecognBtn = new QPushButton(tr("开始识别"));

	poseLabel = new QLabel(tr("位置与姿态"));
	recognizedPoseComboBox = new QComboBox;

	ORKposeXlabel = new QLabel(tr("X"));
	ORKposeXlineEdit = new QLineEdit;ORKposeXlineEdit->setAlignment(Qt::AlignLeft);ORKposeXlineEdit->setReadOnly(true);
	ORKposeYlabel = new QLabel(tr("Y"));
	ORKposeYlineEdit = new QLineEdit;ORKposeYlineEdit->setAlignment(Qt::AlignLeft);ORKposeYlineEdit->setReadOnly(true);
	ORKposeZlabel = new QLabel(tr("Z"));
	ORKposeZlineEdit = new QLineEdit;ORKposeZlineEdit->setAlignment(Qt::AlignLeft);ORKposeZlineEdit->setReadOnly(true);

	ORKposeWdg = new QTabWidget;
	ORKeulerWdg = new QWidget;
	ORKquaterWdg = new QWidget;
		
	ORKeulerRlabel = new QLabel(tr("R"));
	ORKeulerRlineEdit = new QLineEdit;ORKeulerRlineEdit->setAlignment(Qt::AlignLeft);ORKeulerRlineEdit->setReadOnly(true);
	ORKeulerPlabel = new QLabel(tr("P"));
	ORKeulerPlineEdit = new QLineEdit;ORKeulerPlineEdit->setAlignment(Qt::AlignLeft);ORKeulerPlineEdit->setReadOnly(true);
	ORKeulerYlabel = new QLabel(tr("Y"));
	ORKeulerYlineEdit = new QLineEdit;ORKeulerYlineEdit->setAlignment(Qt::AlignLeft);ORKeulerYlineEdit->setReadOnly(true);
		
	ORKeulerLayout = new QHBoxLayout;
	ORKeulerLayout->addWidget(ORKeulerRlabel);
	ORKeulerLayout->addWidget(ORKeulerRlineEdit);
	ORKeulerLayout->addWidget(ORKeulerPlabel);
	ORKeulerLayout->addWidget(ORKeulerPlineEdit);
	ORKeulerLayout->addWidget(ORKeulerYlabel);
	ORKeulerLayout->addWidget(ORKeulerYlineEdit);

	ORKeulerWdg->setLayout(ORKeulerLayout);
		
	ORKquaterXlabel = new QLabel(tr("x"));
	ORKquaterXlineEdit = new QLineEdit;ORKquaterXlineEdit->setAlignment(Qt::AlignLeft);ORKquaterXlineEdit->setReadOnly(true);
	ORKquaterYlabel = new QLabel(tr("y"));
	ORKquaterYlineEdit = new QLineEdit;ORKquaterYlineEdit->setAlignment(Qt::AlignLeft);ORKquaterYlineEdit->setReadOnly(true);
	ORKquaterZlabel = new QLabel(tr("z"));
	ORKquaterZlineEdit = new QLineEdit;ORKquaterZlineEdit->setAlignment(Qt::AlignLeft);ORKquaterZlineEdit->setReadOnly(true);
	ORKquaterWlabel = new QLabel(tr("w"));
	ORKquaterWlineEdit = new QLineEdit;ORKquaterWlineEdit->setAlignment(Qt::AlignLeft);ORKquaterWlineEdit->setReadOnly(true);

	ORKquaterLayout = new QHBoxLayout;
	ORKquaterLayout->addWidget(ORKquaterXlabel);
	ORKquaterLayout->addWidget(ORKquaterXlineEdit);
	ORKquaterLayout->addWidget(ORKquaterYlabel);
	ORKquaterLayout->addWidget(ORKquaterYlineEdit);
	ORKquaterLayout->addWidget(ORKquaterZlabel);
	ORKquaterLayout->addWidget(ORKquaterZlineEdit);
	ORKquaterLayout->addWidget(ORKquaterWlabel);
	ORKquaterLayout->addWidget(ORKquaterWlineEdit);

	ORKquaterWdg->setLayout(ORKquaterLayout);

	ORKposeWdg->addTab(ORKeulerWdg,tr("欧拉角"));
	ORKposeWdg->addTab(ORKquaterWdg,tr("四元数"));
	ORKposeWdg->setCurrentIndex(1);

	ORKLayout = new QGridLayout;
    ORKLayout->addWidget(addOrDeleteComoBox,0,0,1,4);
	ORKLayout->addWidget(newAddModelNameLineEdit,0,4,1,4);newAddModelNameLineEdit->setVisible(true);
	ORKLayout->addWidget(toSelectDeleteComoBox,0,4,1,4);toSelectDeleteComoBox->setVisible(false);
	ORKLayout->addWidget(newAddModelDescripLabel,0,8,1,2);
	ORKLayout->addWidget(newAddModelDescripLineEdit,0,10,1,6);
	ORKLayout->addWidget(newAddOrDeleteOkBtn,0,16,1,2);
		
    ORKLayout->addWidget(tobeTrainModelLabel,1,0,1,4);
	ORKLayout->addWidget(modelPathLineEdit,1,4,1,10);
    ORKLayout->addWidget(modelPathOpenBtn,1,14,1,1);
	ORKLayout->addWidget(addMeshOkBtn,1,15,1,1);
    ORKLayout->addWidget(toTrainBtn,1,16,1,2);
    ORKLayout->addWidget(canRecognModelLabel,2,0,1,4);
    ORKLayout->addWidget(canRecognModelComboBox,2,4,1,12);
    ORKLayout->addWidget(toRecognBtn,2,16,1,2);
    ORKLayout->addWidget(poseLabel,3,0,1,4);
    ORKLayout->addWidget(recognizedPoseComboBox,3,4,1,4);
	ORKLayout->addWidget(ORKposeWdg,3,9,2,9);
	ORKLayout->addWidget(ORKposeXlabel,4,0,1,1);
	ORKLayout->addWidget(ORKposeXlineEdit,4,1,1,2);
	ORKLayout->addWidget(ORKposeYlabel,4,3,1,1);
	ORKLayout->addWidget(ORKposeYlineEdit,4,4,1,2);
	ORKLayout->addWidget(ORKposeZlabel,4,6,1,1);
	ORKLayout->addWidget(ORKposeZlineEdit,4,7,1,2);
		
	setLayout(ORKLayout);

}
OrkWdg::~OrkWdg()
{
	//
	delete ORKLayout;
	delete ORKquaterLayout;
	delete ORKquaterWlineEdit;
	delete ORKquaterWlabel;
	delete ORKquaterZlineEdit;
	delete ORKquaterZlabel;
	delete ORKquaterYlineEdit;
	delete ORKquaterYlabel;
	delete ORKquaterXlineEdit;
	delete ORKquaterXlabel;
	delete ORKeulerLayout;
	delete ORKeulerYlineEdit;
	delete ORKeulerYlabel;
	delete ORKeulerPlineEdit;
	delete ORKeulerPlabel;
	delete ORKeulerRlineEdit;
	delete ORKeulerRlabel;
	delete ORKquaterWdg;
	delete ORKeulerWdg;
	delete ORKposeWdg;
	delete ORKposeZlineEdit;
	delete ORKposeZlabel;
	delete ORKposeYlineEdit;
	delete ORKposeYlabel;
	delete ORKposeXlineEdit;
	delete ORKposeXlabel;
	delete poseLabel;
	delete toRecognBtn;
	delete canRecognModelComboBox;
	delete canRecognModelLabel;
	delete toTrainBtn;
	delete addMeshOkBtn;
	delete modelPathOpenBtn;
	delete modelPathLineEdit;
	delete tobeTrainModelLabel;
	delete newAddOrDeleteOkBtn;
	delete newAddModelDescripLineEdit;
	delete newAddModelDescripLabel;
	delete toSelectDeleteComoBox;
	delete newAddModelNameLineEdit;
	delete addOrDeleteComoBox;
	delete newAddModelNameLabel;
	delete ORKtitleLabel;
	
}
