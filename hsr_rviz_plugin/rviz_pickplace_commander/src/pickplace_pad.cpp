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

#include "pickplace_pad.h"

namespace rviz_pickplace_commander
{
<<<<<<< HEAD
    /* 构造函数，初始化变量 */
    PickPlacePanel::PickPlacePanel(QWidget* parent) : rviz::Panel(parent),openSizeMax(1000),\
                                        openSizeMin(0),gripperSpeed(500),gripperForce(100)
    {	
		//QTextCodec::setCodecForTr(QTextCodec::codecForLocale());	/* 处理中文字符 */
		QTextCodec::setCodecForCStrings(QTextCodec::codecForName("GB2312")); /* 处理中文字符 */
	
	/******************相关数据初始化*********************************************************************/
		m_trainingMdoel = new TRAINED_MODEL[1];
   
    /******************界面相关*********************************************************************/
=======
    /* ���캯������ʼ������ */
    PickPlacePanel::PickPlacePanel(QWidget* parent) : rviz::Panel(parent),openSizeMax(1000),\
                                        openSizeMin(0),gripperSpeed(500),gripperForce(100)
    {	
		//QTextCodec::setCodecForTr(QTextCodec::codecForLocale());	/* ���������ַ� */
		QTextCodec::setCodecForCStrings(QTextCodec::codecForName("GB2312")); /* ���������ַ� */
	
	/******************������ݳ�ʼ��*********************************************************************/
		m_trainingMdoel = new TRAINED_MODEL[1];
   
    /******************�������*********************************************************************/
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
        m_serialWdg = new SerialWdg;
	m_gripperWdg = new GripperWdg;
	m_orkWdg = new OrkWdg;
	m_pickPlaceWdg = new PickPlaceWdg;
					    		
<<<<<<< HEAD
        /* 整体布局 */
=======
        /* ���岼�� */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
        mainLayout = new QVBoxLayout;
		mainLayout->addWidget(m_serialWdg->serialTitleLabel,1);
        mainLayout->addWidget(m_serialWdg,1);
		mainLayout->addWidget(m_gripperWdg->gripperTitleLabel,1);
        mainLayout->addWidget(m_gripperWdg,3);
		mainLayout->addWidget(m_orkWdg->ORKtitleLabel,1);
        mainLayout->addWidget(m_orkWdg,4);
		mainLayout->addWidget(m_pickPlaceWdg->pickPlacetitleLabel,1);
        mainLayout->addWidget(m_pickPlaceWdg,7);
		
        setLayout(mainLayout);
		
<<<<<<< HEAD
    /******************槽函数*********************************************************************/
	    connect(m_serialWdg->serialNoComoBox,SIGNAL(clicked(bool)),this,SLOT(slotSerialNoComoBoxClicked()));
=======
    /******************�ۺ���*********************************************************************/
	        connect(m_serialWdg->serialNoComoBox,SIGNAL(clicked(bool)),this,SLOT(slotSerialNoComoBoxClicked()));
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
		connect(m_serialWdg->serialConnectOrNotBtn,SIGNAL(pressed()),this,SLOT(slotSerialConnectBtn()));
                connect(m_gripperWdg->gripperSetComoBox,SIGNAL(currentIndexChanged(int)),this,SLOT(slotGripperParmSetComboBox(int)));
                connect(m_gripperWdg->gripperSetBtn,SIGNAL(pressed()),this,SLOT(slotGripperSetBtn()));
		connect(m_gripperWdg->gripperActRunBtn,SIGNAL(clicked(bool)),this,SLOT(slotGripperActRunBtn()));
		
		connect(m_orkWdg->addOrDeleteComoBox,SIGNAL(currentIndexChanged(int)),this,SLOT(slotAddOrDeleteComboBox(int)));
		connect(m_orkWdg->toSelectDeleteComoBox,SIGNAL(currentIndexChanged(int)),this,SLOT(slotToSelectDeleteComoBox(int)));
		//connect(canRecognModelComboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(slotCanRecognModelComboBox(int)));
		
		connect(m_orkWdg->newAddOrDeleteOkBtn,SIGNAL(pressed()),this,SLOT(slotNewAddOrDeleteOkBtn()));
		connect(m_orkWdg->modelPathOpenBtn,SIGNAL(pressed()),this,SLOT(slotModelPathOpenBtn()));
		connect(m_orkWdg->addMeshOkBtn,SIGNAL(pressed()),this,SLOT(slotAddMeshOkBtn()));
		connect(m_orkWdg->toTrainBtn,SIGNAL(clicked(bool)),this,SLOT(slotToTrainBtn()));
		connect(m_orkWdg->toRecognBtn,SIGNAL(pressed()),this,SLOT(slotToRecognBtn()));

		connect(m_orkWdg->recognizedPoseComboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(slotToSelectDetectComoBox(int)));
		
		connect(m_pickPlaceWdg->getPickPoseMeans,SIGNAL(currentIndexChanged(int)),this,SLOT(slotGetPickPoseMeansComoBox(int)));
		connect(m_pickPlaceWdg->getPlacePoseMeans,SIGNAL(currentIndexChanged(int)),this,SLOT(slotGetPlacePoseMeansComoBox(int)));
		connect(m_pickPlaceWdg->getPlacePoseOkBtn,SIGNAL(pressed()),this,SLOT(slotGetPlacePoseOkBtn()));
		
		connect(m_pickPlaceWdg->runPickPlaceBtn,SIGNAL(clicked(bool)),this,SLOT(slotRunPickPlaceBtn()));
<<<<<<< HEAD
	/******************ROS相关*********************************************************************/
        /* 创建串口打开客户端 */
=======
               
                connect(m_pickPlaceWdg->voicePickPlaceBtn,SIGNAL(clicked(bool)),this,SLOT(slotVoicePickPlaceBtn()));
              
	/******************ROS���*********************************************************************/
        /* �������ڴ򿪿ͻ��� */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
	    client_serialOpen = n_pickPlace.serviceClient<hsr_gripper_driver::serial_open_srv>("serial_open");
        client_gripperOpenSize = n_pickPlace.serviceClient<hsr_gripper_driver::open_size_srv>("gripper_set_open_size");
		client_gripperOpen = n_pickPlace.serviceClient<hsr_gripper_driver::open_srv>("gripper_open");
		client_gripperClose = n_pickPlace.serviceClient<hsr_gripper_driver::close_srv>("gripper_close");
		client_gripperStop = n_pickPlace.serviceClient<hsr_gripper_driver::stop_srv>("gripper_stop");
		
<<<<<<< HEAD
		/* 创建ORK模块相关客户端 */
=======
		/* ����ORKģ����ؿͻ��� */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
		client_objAdd = n_pickPlace.serviceClient<ork_interface::objAdd>("object_add");
		client_objDelete = n_pickPlace.serviceClient<ork_interface::objDelete>("object_delete");
	    client_meshAdd = n_pickPlace.serviceClient<ork_interface::meshAdd>("mesh_add");
		client_objSearch = n_pickPlace.serviceClient<ork_interface::objSearch>("object_search");
		client_training = n_pickPlace.serviceClient<ork_interface::training_srv>("training");
		client_detection = n_pickPlace.serviceClient<ork_interface::detection_srv>("detection");
		
<<<<<<< HEAD
		/* 创建抓取与放置模块相关客户端 */
		client_pickPlace = n_runPickPlace.serviceClient<hsr_pick::pickPlace>("pick_and_place");
	/*******************定时器**************************************************************/
=======
		/* ����ץȡ�����ģ����ؿͻ��� */
		client_pickPlace = n_runPickPlace.serviceClient<hsr_pick::pickPlace>("pick_and_place");
	/*******************��ʱ��**************************************************************/
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
	    getPickPoseFromORK_Timer = new QTimer(this);
	    connect(getPickPoseFromORK_Timer,SIGNAL(timeout()),this,SLOT(slotGetPickPoseFromOrk()));
		
		//showPoseFromCamera_Timer = new QTimer(this);
        //connect(showPoseFromCamera_Timer,SIGNAL(timeout()),this,SLOT(slotShowPoseFromCameraTimer()));

		
<<<<<<< HEAD
		//其他初始化
=======
		//������ʼ��
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
		canRecognizedModelListInit();
        path_detect_config = "";
        num_detectedObj = 0;
	
    }
	
	PickPlacePanel::~PickPlacePanel()
	{
		//
		delete mainLayout;
		delete m_pickPlaceWdg;
		delete m_orkWdg;
		delete m_gripperWdg;
		delete m_serialWdg;
		delete[] m_trainingMdoel;
		delete[] m_poseFromCamera;
	}
	
	void PickPlacePanel::slotSerialNoComoBoxClicked()
	{
		//
		m_pickPlaceWdg->pickPlaceStatusTextEdit->append(tr("join the function of slotSerialNoComoBoxClicked"));
		
		glob_t ttyUSBpath_buf;
        glob_t ttySpath_buf;
        int i;
        glob("/dev/ttyUSB*",GLOB_NOSORT, NULL, &ttyUSBpath_buf);		
        //glob("/dev/ttyS*",GLOB_NOSORT, NULL, &ttySpath_buf);
        
<<<<<<< HEAD
		m_serialWdg->serialNoComoBox->clear();           /* 触发该槽函数后，清除设备备选号，防止在旧的设备号后追加 */
=======
		m_serialWdg->serialNoComoBox->clear();           /* �����òۺ���������豸��ѡ�ţ���ֹ�ھɵ��豸�ź�׷�� */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
		

        for(i=0; i < ttyUSBpath_buf.gl_pathc; i++)
        {
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append(QString::fromStdString(ttyUSBpath_buf.gl_pathv[i]));
			
			QFileInfo ttyUSBpath_Info = QFileInfo(QString::fromStdString(ttyUSBpath_buf.gl_pathv[i]));			
<<<<<<< HEAD
			// 将搜索到的ttyUSB添加到备选设备号下拉框中 
=======
			// ����������ttyUSB��ӵ���ѡ�豸���������� 
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			m_serialWdg->serialNoComoBox->addItem(ttyUSBpath_Info.fileName()); 
        }

        m_serialWdg->serialNoComoBox->addItem("ttyS0"); 
        m_serialWdg->serialNoComoBox->addItem("ttyS1"); 
        m_serialWdg->serialNoComoBox->addItem("ttyS2"); 
        m_serialWdg->serialNoComoBox->addItem("ttyS3"); 
        m_serialWdg->serialNoComoBox->addItem("ttyS4"); 
        m_serialWdg->serialNoComoBox->addItem("ttyS5"); 
        /*
        for(i=ttySpath_buf.gl_pathc; i >= 0;i--)
        {
			//m_pickPlaceWdg->pickPlaceStatusTextEdit->append(QString::fromStdString(ttySpath_buf.gl_pathv[i]));
			
			//QFileInfo ttySpath_Info = QFileInfo(QString::fromStdString(ttySpath_buf.gl_pathv[i]));			
<<<<<<< HEAD
			// 将搜索到的ttyS添加到备选设备号下拉框中 
=======
			// ����������ttyS��ӵ���ѡ�豸���������� 
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			m_serialWdg->serialNoComoBox->addItem(ttySpath_Info.fileName()); 
        }
        */
        
        globfree(&ttyUSBpath_buf);
        //globfree(&ttySpath_buf);
	}
	
	void PickPlacePanel::slotSerialConnectBtn()
	{
		//`rospack find object_recognition_linemod`/conf/detection.ros.ork
        //const char *cmd = "rospack find object_recognition_linemod";
        //char result[1024]={0};
       
        //executeCMD(cmd,result);
		//printf("%s", result);
		
		hsr_gripper_driver::serial_open_srv serialOpen_srv;

<<<<<<< HEAD
		/* 当前选中的串口设备号和波特率转成string类型后 传入服务参数*/
=======
		/* ��ǰѡ�еĴ����豸�źͲ�����ת��string���ͺ� ����������*/
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
		serialOpen_srv.request.serialNo = (std::string)("/dev/")+ m_serialWdg->serialNoComoBox->currentText().toStdString();
        //serialOpen_srv.request.serialNo = "/dev/ttyUSB0";
		serialOpen_srv.request.baudrate = m_serialWdg->baudrateComoBox->currentText().toInt();
        m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Baudrate:");
		m_pickPlaceWdg->pickPlaceStatusTextEdit->append(QString::number(serialOpen_srv.request.baudrate, 10));
		m_pickPlaceWdg->pickPlaceStatusTextEdit->append("SerialNo:");
		m_pickPlaceWdg->pickPlaceStatusTextEdit->append(m_serialWdg->serialNoComoBox->currentText());

		if(client_serialOpen.call(serialOpen_srv))
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Serial Port opened!!!");
		}
		else
		{
		    m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Unable to open port!!!");
		}	
	}
		
	void PickPlacePanel::canRecognizedModelListInit()
	{
<<<<<<< HEAD
		//**********************************启动界面更新可识别模型列表*******************************************
=======
		//**********************************�������¿�ʶ��ģ���б�*******************************************
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				
		ork_interface::objSearch objSearch_srv;
		
		if(m_orkWdg->toSelectDeleteComoBox->count())
		{
<<<<<<< HEAD
			//清除下拉选择框前，先执行disconnect，否则会由于当前索引的改变，触发currentIndexChanged，从而导致崩溃
=======
			//�������ѡ���ǰ����ִ��disconnect����������ڵ�ǰ�����ĸı䣬����currentIndexChanged���Ӷ����±���
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			disconnect(m_orkWdg->toSelectDeleteComoBox, SIGNAL(currentIndexChanged(int)), this, SLOT(slotToSelectDeleteComoBox(int)));
			m_orkWdg->toSelectDeleteComoBox->clear();
			connect(m_orkWdg->toSelectDeleteComoBox, SIGNAL(currentIndexChanged(int)), this, SLOT(slotToSelectDeleteComoBox(int)));
			
			m_orkWdg->canRecognModelComboBox->clear();
		}
			
		if(client_objSearch.call(objSearch_srv))
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("get the Name of the object sucessful!!!");

		    //pickPlaceStatusTextEdit->append(QString::fromStdString(objSearch_srv.response.objName[1]));
					
<<<<<<< HEAD
			// 获取已训练物体个数
=======
			// ��ȡ��ѵ���������
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			int num_trained = objSearch_srv.response.objNum;
					
			if(!num_trained)
				return;
			delete[] m_trainingMdoel;
			m_trainingMdoel = new TRAINED_MODEL[num_trained];
						
			for(int i = 0;i <num_trained;i++)
			{
				m_trainingMdoel[i].name = objSearch_srv.response.objName[i];
				m_trainingMdoel[i].id = objSearch_srv.response.objId[i];
				m_trainingMdoel[i].description = objSearch_srv.response.objDescription[i];
				
				m_orkWdg->canRecognModelComboBox->addItem((QString)("Name:") + QString::fromStdString(m_trainingMdoel[i].name)+ \
				                                (QString)(";ID:") + QString::fromStdString(m_trainingMdoel[i].id));
                m_orkWdg->toSelectDeleteComoBox->addItem(QString::fromStdString(m_trainingMdoel[i].name));												
			}
			
			int index_current = m_orkWdg->toSelectDeleteComoBox->currentIndex();
			m_orkWdg->newAddModelDescripLineEdit->setText(QString::fromStdString(m_trainingMdoel[index_current].description));
		}
		else
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Get the detectable model failed,please check whether object_search is running!!!");
		}
		
		if(0 == m_orkWdg->addOrDeleteComoBox->currentIndex())
		{
			m_orkWdg->newAddModelDescripLineEdit->setText("");
		}
	}

	void PickPlacePanel::slotGripperParmSetComboBox(int index)
	{
        switch(index)
        {
<<<<<<< HEAD
            case 0:  //设置夹爪开口
            {
                m_gripperWdg->openMaxLabel->setText("最大开口:");
                m_gripperWdg->openMinLabel->setText("最小开口:");
=======
            case 0:  //���ü�צ����
            {
                m_gripperWdg->openMaxLabel->setText("��󿪿�:");
                m_gripperWdg->openMinLabel->setText("��С����:");
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685

				m_gripperWdg->openMinLabel->setVisible(true);
                m_gripperWdg->openMinLineEdit->setVisible(true);
				
                m_gripperWdg->openMaxLineEdit->setValidator(new QIntValidator(0, 1000, this));
				m_gripperWdg->openMinLineEdit->setValidator(new QIntValidator(0, 1000, this));
<<<<<<< HEAD
				m_gripperWdg->openMaxLineEdit->setPlaceholderText(tr("输入0-1000的整数"));
		        m_gripperWdg->openMaxLineEdit->setToolTip(tr("输入0-1000的整数，数值越大开口越大，\r\n且该值需要大于【开口最小值】"));
=======
				m_gripperWdg->openMaxLineEdit->setPlaceholderText(tr("����0-1000������"));
		        m_gripperWdg->openMaxLineEdit->setToolTip(tr("����0-1000����������ֵԽ�󿪿�Խ��\r\n�Ҹ�ֵ��Ҫ���ڡ�������Сֵ��"));
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				m_gripperWdg->openMaxLineEdit->setText(QString::number(openSizeMax));
				m_gripperWdg->openMinLineEdit->setText(QString::number(openSizeMin));
				
			    break;
            }
<<<<<<< HEAD
			case 1:  //设置夹爪速度
            {
				m_gripperWdg->openMaxLabel->setText("大小:");
				m_gripperWdg->openMaxLineEdit->setValidator(new QIntValidator(1, 1000, this));
				m_gripperWdg->openMaxLineEdit->setPlaceholderText("输入0-1000的整数");
		        m_gripperWdg->openMaxLineEdit->setToolTip("输入1-1000的整数，数值越大速度越快");
=======
			case 1:  //���ü�צ�ٶ�
            {
				m_gripperWdg->openMaxLabel->setText("��С:");
				m_gripperWdg->openMaxLineEdit->setValidator(new QIntValidator(1, 1000, this));
				m_gripperWdg->openMaxLineEdit->setPlaceholderText("����0-1000������");
		        m_gripperWdg->openMaxLineEdit->setToolTip("����1-1000����������ֵԽ���ٶ�Խ��");
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				m_gripperWdg->openMaxLineEdit->setText(QString::number(gripperSpeed));

				m_gripperWdg->openMinLabel->setVisible(false);
                m_gripperWdg->openMinLineEdit->setVisible(false);
				
				break;
			}
<<<<<<< HEAD
			case 2:  //设置夹爪力矩
			{
				m_gripperWdg->openMaxLabel->setText("大小:");
				m_gripperWdg->openMaxLineEdit->setValidator(new QIntValidator(50, 1000, this));
				m_gripperWdg->openMaxLineEdit->setPlaceholderText("输入50-1000的整数");
		        m_gripperWdg->openMaxLineEdit->setToolTip("输入50-1000的整数，数值越大力矩越大");
=======
			case 2:  //���ü�צ����
			{
				m_gripperWdg->openMaxLabel->setText("��С:");
				m_gripperWdg->openMaxLineEdit->setValidator(new QIntValidator(50, 1000, this));
				m_gripperWdg->openMaxLineEdit->setPlaceholderText("����50-1000������");
		        m_gripperWdg->openMaxLineEdit->setToolTip("����50-1000����������ֵԽ������Խ��");
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				m_gripperWdg->openMaxLineEdit->setText(QString::number(gripperForce));

				m_gripperWdg->openMinLabel->setVisible(false);
                m_gripperWdg->openMinLineEdit->setVisible(false);
				
				break;
			}
			default:
				break;
        }
	}
	void PickPlacePanel::slotGripperSetBtn()
	{
	    int index = m_gripperWdg->gripperSetComoBox->currentIndex();
	    switch(index)
	    {
<<<<<<< HEAD
	    	case 0:  //设置夹爪开口
=======
	    	case 0:  //���ü�צ����
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
	    	{
                hsr_gripper_driver::open_size_srv gripperOpenSize_srv;

		        gripperOpenSize_srv.request.max = m_gripperWdg->openMaxLineEdit->text().toInt();
		        gripperOpenSize_srv.request.min = m_gripperWdg->openMinLineEdit->text().toInt();

				if(client_gripperOpenSize.call(gripperOpenSize_srv))
				{
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append(tr("Set the open size of the Gripper sucessful!!!"));
					openSizeMax = m_gripperWdg->openMaxLineEdit->text().toInt();
					openSizeMin = m_gripperWdg->openMinLineEdit->text().toInt();
				}
				else
				{
				    m_pickPlaceWdg->pickPlaceStatusTextEdit->append(tr("Set the open size of the Gripper failed!!!"));
				}	
				
			    break;
	    	}
<<<<<<< HEAD
			case 1:  //设置夹爪速度
				gripperSpeed = m_gripperWdg->openMaxLineEdit->text().toInt();
				
				break;
			case 2:  //设置夹爪力矩
=======
			case 1:  //���ü�צ�ٶ�
				gripperSpeed = m_gripperWdg->openMaxLineEdit->text().toInt();
				
				break;
			case 2:  //���ü�צ����
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
                gripperForce = m_gripperWdg->openMaxLineEdit->text().toInt();
				
				break;
			default:
				break;
	    }
	}
	void PickPlacePanel::slotGripperActRunBtn()
	{
	    //
	    switch(m_gripperWdg->gripperActComoBox->currentIndex())
	    {
<<<<<<< HEAD
	    	case 0:     //夹爪打开
=======
	    	case 0:     //��צ��
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
	    	{
                hsr_gripper_driver::open_srv gripperOpen_srv;
		        gripperOpen_srv.request.speed = gripperSpeed;
				if(client_gripperOpen.call(gripperOpen_srv))
				{
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append(tr("Open the Gripper sucessful!!!"));
				}
				else
				{
				    m_pickPlaceWdg->pickPlaceStatusTextEdit->append(tr("Open the Gripper failed!!!"));
				}	
				
				break;
	    	}
<<<<<<< HEAD
			case 1:    //夹爪关闭
=======
			case 1:    //��צ�ر�
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			{
                hsr_gripper_driver::close_srv gripperClose_srv;
				
		        gripperClose_srv.request.speed = gripperSpeed;
				gripperClose_srv.request.force = gripperForce;
				
				if(client_gripperClose.call(gripperClose_srv))
				{
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append(tr("Close the Gripper sucessful!!!"));

				}
				else
				{
				    m_pickPlaceWdg->pickPlaceStatusTextEdit->append(tr("Close the Gripper failed!!!"));
				}	
				break;
			}
<<<<<<< HEAD
			case 2:    //夹爪急停
=======
			case 2:    //��צ��ͣ
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			{
				hsr_gripper_driver::stop_srv gripperStop_srv;
				
				if(client_gripperStop.call(gripperStop_srv))
				{
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append(tr("Stop the Gripper sucessful!!!"));

				}
				else
				{
				    m_pickPlaceWdg->pickPlaceStatusTextEdit->append(tr("Stop the Gripper failed!!!"));
				}	

				break;
			}
			default:

				break;
	    }
	}
	
	void PickPlacePanel::slotAddOrDeleteComboBox(int index)
	{
		//
		switch(index)
	    {
<<<<<<< HEAD
	    	case 0:     //切换到 新增模型
	    	{	
				m_orkWdg->newAddOrDeleteOkBtn->setText("确认添加");
=======
	    	case 0:     //�л��� ����ģ��
	    	{	
				m_orkWdg->newAddOrDeleteOkBtn->setText("ȷ�����");
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				
				//m_orkWdg->toSelectDeleteComoBox->removeItem(0);
				
				m_orkWdg->newAddModelNameLineEdit->setVisible(true);
				m_orkWdg->toSelectDeleteComoBox->setVisible(false);
				
				m_orkWdg->newAddModelNameLineEdit->setText("");
				m_orkWdg->newAddModelDescripLineEdit->setText("");
				
				int count_toSelectDeleteComoBox = m_orkWdg->toSelectDeleteComoBox->count();
				
				m_pickPlaceWdg->pickPlaceStatusTextEdit->append("count_toSelectDeleteComoBox:");
				m_pickPlaceWdg->pickPlaceStatusTextEdit->append(QString::number(count_toSelectDeleteComoBox, 10));
				
				break;
	    	}
<<<<<<< HEAD
			case 1:    //切换到 删除模型
			{
				m_orkWdg->newAddOrDeleteOkBtn->setText("确认删除");
=======
			case 1:    //�л��� ɾ��ģ��
			{
				m_orkWdg->newAddOrDeleteOkBtn->setText("ȷ��ɾ��");
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				
				m_orkWdg->newAddModelNameLineEdit->setVisible(false);
				m_orkWdg->toSelectDeleteComoBox->setVisible(true);
				
				m_orkWdg->newAddModelDescripLineEdit->setText("");
				
				int index = m_orkWdg->toSelectDeleteComoBox->currentIndex();
<<<<<<< HEAD
				/* 当toSelectDeleteComoBox为空时，index = -1*/
=======
				/* ��toSelectDeleteComoBoxΪ��ʱ��index = -1*/
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				if(index >= 0)
				{
		            m_orkWdg->newAddModelDescripLineEdit->setText(QString::fromStdString(m_trainingMdoel[index].description));
				}
				
				break;			
			}
			default:
				break;
	    }
		
	}
	
	void PickPlacePanel::slotToSelectDeleteComoBox(int index)
	{
		m_orkWdg->newAddModelDescripLineEdit->setText("");
		m_orkWdg->newAddModelDescripLineEdit->setText(QString::fromStdString(m_trainingMdoel[index].description));
	}
	
	void PickPlacePanel::slotNewAddOrDeleteOkBtn()
	{
		//
		switch(m_orkWdg->addOrDeleteComoBox->currentIndex())
		{
<<<<<<< HEAD
			case 0:  //确认添加
=======
			case 0:  //ȷ�����
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			{
				ork_interface::objAdd objAdd_srv;
				objAdd_srv.request.name = m_orkWdg->newAddModelNameLineEdit->text().toStdString();
				objAdd_srv.request.description = m_orkWdg->newAddModelDescripLineEdit->text().toStdString();
						
				if(client_objAdd.call(objAdd_srv))
				{
					newAddModel_ID = QString::fromStdString(objAdd_srv.response.id);
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append("new add a object sucessful!!!");
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append("*************************************");
<<<<<<< HEAD
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append("the new add object ID is：");
=======
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append("the new add object ID is��");
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append(QString::fromStdString(objAdd_srv.response.id));
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append("*************************************");
					
					canRecognizedModelListInit();
				}
				else
				{
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append("new add a object failed!!!");
				}
				
				break;
			}
<<<<<<< HEAD
			case 1:  //确认删除
=======
			case 1:  //ȷ��ɾ��
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			{
                ork_interface::objDelete objDelete_srv;
				
				int index_delete = m_orkWdg->toSelectDeleteComoBox->currentIndex();
				
		        objDelete_srv.request.id = m_trainingMdoel[index_delete].id;
				m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Delete a object ID:");
				m_pickPlaceWdg->pickPlaceStatusTextEdit->append(QString::fromStdString(objDelete_srv.request.id));
				
				if(client_objDelete.call(objDelete_srv))
				{
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Delete a object sucessful!!!");
					
<<<<<<< HEAD
					//删除某个下拉选择框前，先执行disconnect，否则会由于当前索引的改变，触发currentIndexChanged，从而导致崩溃
=======
					//ɾ��ĳ������ѡ���ǰ����ִ��disconnect����������ڵ�ǰ�����ĸı䣬����currentIndexChanged���Ӷ����±���
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				    disconnect(m_orkWdg->toSelectDeleteComoBox, SIGNAL(currentIndexChanged(int)), this, SLOT(slotToSelectDeleteComoBox(int)));
				    m_orkWdg->toSelectDeleteComoBox->removeItem(index_delete);
				    connect(m_orkWdg->toSelectDeleteComoBox, SIGNAL(currentIndexChanged(int)), this, SLOT(slotToSelectDeleteComoBox(int)));
					
					canRecognizedModelListInit();
				}
				else
				{
				    m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Delete a object failed!!!");
				}
						
				break;
			}
			default:
				break;
		}	
	}

	void PickPlacePanel::slotModelPathOpenBtn()
	{
	    //
	    QString modelFilePath=QFileDialog::getOpenFileName(this,"open modelFile dialog","/","Model files(*.stl)");

        m_orkWdg->modelPathLineEdit->setText(modelFilePath);

		m_pickPlaceWdg->pickPlaceStatusTextEdit->append("The Model Path is:");
        m_pickPlaceWdg->pickPlaceStatusTextEdit->append(modelFilePath);
				
	}
	
	void PickPlacePanel::slotAddMeshOkBtn()
	{
		//
		ork_interface::meshAdd meshAdd_srv;
		meshAdd_srv.request.id = newAddModel_ID.toStdString();
		meshAdd_srv.request.path = m_orkWdg->modelPathLineEdit->text().toStdString();
			
		if(client_meshAdd.call(meshAdd_srv))
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("new add a mesh sucessful!!!");
		}
		else
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("new add a mesh failed!!!");
		}
	}
	
	void PickPlacePanel::executeCMD(const char *cmd, char *result)
	{
		//
		char buf_ps[1024];
        char ps[1024]={0};
        FILE *ptr;
        strcpy(ps, cmd);
        if((ptr=popen(ps, "r"))!=NULL)
        {
            while(fgets(buf_ps, 1024, ptr)!=NULL)
            {
<<<<<<< HEAD
                // 可以通过这行来获取shell命令行中的每一行的输出
=======
                // ����ͨ����������ȡshell�������е�ÿһ�е����
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
                // printf("%s", buf_ps);
                strcat(result, buf_ps);
                if(strlen(result)>1024)
                break;
            }
        pclose(ptr);
        ptr = NULL;
        }
        else
        {
            printf("popen %s error\n", ps);
        }
	}
	
	void PickPlacePanel::slotToTrainBtn()
	{
		//
        ork_interface::training_srv training_srv;
		
<<<<<<< HEAD
		//等价于执行 rosrun object_recognition_core training -c /home/eima/Work/catkin_ws/src/linemod/conf/training.ork
		const char *cmd = "rospack find object_recognition_linemod";
        char path_result[1024]={0};
	
        executeCMD(cmd,path_result);              /* 执行命令，输出结果 */
		path_result[strlen(path_result)-1] = 0;   /* 删除掉末尾的\n */
		std::string s_path_result = path_result;  /* 将char[]转为std::string */
=======
		//�ȼ���ִ�� rosrun object_recognition_core training -c /home/eima/Work/catkin_ws/src/linemod/conf/training.ork
		const char *cmd = "rospack find object_recognition_linemod";
        char path_result[1024]={0};
	
        executeCMD(cmd,path_result);              /* ִ����������� */
		path_result[strlen(path_result)-1] = 0;   /* ɾ����ĩβ��\n */
		std::string s_path_result = path_result;  /* ��char[]תΪstd::string */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
		
		training_srv.request.cmd = "\'c\'"; 
		training_srv.request.path = s_path_result + "/conf/training.ork";
			
		if(client_training.call(training_srv))
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Training sucessful!!!");
		}
		else
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Training failed!!!");
		}		

	}

	void PickPlacePanel::slotToRecognBtn()
	{	
		ork_interface::detection_srv detection_srv;
		
<<<<<<< HEAD
		//等价于执行 rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
		const char *cmd = "rospack find object_recognition_linemod";
        char path_result[1024]={0};
	
        executeCMD(cmd,path_result);              /* 执行命令，输出结果 */
		path_result[strlen(path_result)-1] = 0;   /* 删除掉末尾的\n */
		std::string s_path_result = path_result;  /* 将char[]转为std::string */
=======
		//�ȼ���ִ�� rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork
		const char *cmd = "rospack find object_recognition_linemod";
        char path_result[1024]={0};
	
        executeCMD(cmd,path_result);              /* ִ����������� */
		path_result[strlen(path_result)-1] = 0;   /* ɾ����ĩβ��\n */
		std::string s_path_result = path_result;  /* ��char[]תΪstd::string */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
		
		detection_srv.request.cmd = "\'c\'"; 
		detection_srv.request.path = s_path_result + "/conf/detection.ros.ork";
        path_detect_config = detection_srv.request.path;
        m_pickPlaceWdg->pickPlaceStatusTextEdit->append("******num_detectedObj****");
        m_pickPlaceWdg->pickPlaceStatusTextEdit->append(QString::number(num_detectedObj, 10));
        detection_srv.request.isDetected = false;
	
		if(client_detection.call(detection_srv))
		{
                        removeObjectBox();
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Detection start sucessful!!!");
			sub_poseFromCamera = n_pickPlace.subscribe("/recognized_object_array", 1000, &PickPlacePanel::poseFromCamera_callback, this);
		}
		else
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Detection start failed!!!");
		}
		
	}
	
	void PickPlacePanel::poseFromCamera_callback(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
	{
		num_detectedObj = 0;
		num_detectedObj = msg->objects.size();
        ROS_ERROR("num_detectedObj:%d",num_detectedObj);
		
		if(!num_detectedObj)
			return;
        
        ork_interface::detection_srv detection_srv;
        detection_srv.request.cmd = "\'c\'"; 
		detection_srv.request.path = path_detect_config;
        detection_srv.request.isDetected = true;
        
<<<<<<< HEAD
        /* 如果检测到物体，则停止ORK检测服务 */
=======
        /* �����⵽���壬��ֹͣORK������ */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
        if(client_detection.call(detection_srv))
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Detection stop sucessful!!!");
		}
		else
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Detection stop failed!!!");
		}
		
<<<<<<< HEAD
        /* 创建新的动态数组前，先delete */
=======
        /* �����µĶ�̬����ǰ����delete */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
        //delete[] detectPoseFromCamera;
        //delete[] base_detectPoseFromCamera;
        //delete[] m_poseFromCamera;

		detectPoseFromCamera = new geometry_msgs::PoseStamped[num_detectedObj];
		base_detectPoseFromCamera = new geometry_msgs::PoseStamped[num_detectedObj];
		m_poseFromCamera = new T_POSE_FORM_CAMERA[num_detectedObj];
		
		tf::TransformListener listener;
		geometry_msgs::PoseStamped base_temp;
		
		for(int i = 0;i< num_detectedObj;i++)
		{
			detectPoseFromCamera[i].header.seq = 1;
			detectPoseFromCamera[i].header.frame_id = "kinect2_rgb_optical_frame";
			detection_ID[i] = msg->objects[i].type.key;
			detectPoseFromCamera[i].pose = msg->objects[i].pose.pose.pose;
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("detection_ID:"+QString::fromStdString(detection_ID[i]));
            ROS_ERROR("detectPoseFromCamera[i].pose.X:%f",detectPoseFromCamera[i].pose.position.x);
			
			 while(1)
            {
                static int count =0;
				try
				{
					listener.transformPose("base_link",detectPoseFromCamera[i],base_detectPoseFromCamera[i]);
				}
				catch(tf::TransformException &ex)
				{
		            ROS_ERROR("%s",ex.what());
		            ros::Duration(1.0).sleep();
		            continue;
				}

                if((base_detectPoseFromCamera[i].pose.position.x !=0.0 )&& (base_detectPoseFromCamera[i].pose.position.y !=0.0))
                     break;
<<<<<<< HEAD
                /* 循环100次，若还未读到坐标则跳出 */
=======
                /* ѭ��100�Σ���δ�������������� */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
                count++;
                if(count >100)
                {
                     ROS_ERROR("form Camera frame to Base frame failed !!!");
                     break;
                }
             }
		}

       static int count_update =0;
       ROS_ERROR("**************************************");
       ROS_ERROR("the count_update is %d",count_update);
       ROS_ERROR("**************************************");
       
       if(num_detectedObj != pro_num_detectedObj )
             count_update =0; 

       //if(!count_update)
       //{
		
        disconnect(m_orkWdg->recognizedPoseComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(slotToSelectDetectComoBox(int)));
		m_orkWdg->recognizedPoseComboBox->clear();
        
<<<<<<< HEAD
        ROS_DEBUG("**************************************");
        ROS_DEBUG("the count of objects is %d",num_detectedObj);
        ROS_DEBUG("**************************************");
		for(int i=0;i<num_detectedObj;i++)
		{ 
            if(base_detectPoseFromCamera[i].pose.position.z > 0.4)
               continue;
			m_orkWdg->recognizedPoseComboBox->addItem("识别目标"+QString::number(i+1, 10));
=======
        ROS_ERROR("**************************************");
        ROS_ERROR("the count of objects is %d",num_detectedObj);
        ROS_ERROR("**************************************");
	for(int i=0;i<num_detectedObj;i++)
	{ 
              //if(base_detectPoseFromCamera[i].pose.position.z > 30)
              //continue;
              m_orkWdg->recognizedPoseComboBox->addItem("ʶ��Ŀ��"+QString::number(i+1, 10));
              // detectionModel_ID = QString::fromStdString(detection_ID[i]);
              // m_orkWdg->recognizedPoseComboBox->addItem(detectionModel_ID);
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			
              m_poseFromCamera[i].X = base_detectPoseFromCamera[i].pose.position.x;
              m_poseFromCamera[i].Y = base_detectPoseFromCamera[i].pose.position.y;
	      m_poseFromCamera[i].Z = base_detectPoseFromCamera[i].pose.position.z;
		
              float q0 = m_poseFromCamera[i].quaterW = base_detectPoseFromCamera[i].pose.orientation.w;
	      float q1 = m_poseFromCamera[i].quaterX = base_detectPoseFromCamera[i].pose.orientation.x;
	      float q2 = m_poseFromCamera[i].quaterY = base_detectPoseFromCamera[i].pose.orientation.y;
	      float q3 = m_poseFromCamera[i].quaterZ = base_detectPoseFromCamera[i].pose.orientation.z;
		
	      m_poseFromCamera[i].eulerR = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
	      m_poseFromCamera[i].eulerP = asin(2*(q0*q2-q1*q3));
	      m_poseFromCamera[i].eulerY = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

<<<<<<< HEAD
            ROS_INFO("识别目标:%d,其X坐标为%f",i,m_poseFromCamera[i].X);
				
		}
        
=======
              ROS_ERROR("ʶ��Ŀ��:%d,��X����Ϊ%f",i,m_poseFromCamera[i].X);
              box_pose.orientation.w = 1.0;
              box_pose.position.x =  m_poseFromCamera[i].X;
              box_pose.position.y =  m_poseFromCamera[i].Y;
              box_pose.position.z =  m_poseFromCamera[i].Z;
	      addObjectBoxdefines(detection_ID[i],i,box_pose);		
	}
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
        connect(m_orkWdg->recognizedPoseComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(slotToSelectDetectComoBox(int)));
        
        if((m_poseFromCamera[0].X == 0.0) && (m_poseFromCamera[0].Y == 0.0) && (m_poseFromCamera[0].Z==0.0))
     		return;

		m_orkWdg->recognizedPoseComboBox->setCurrentIndex(0);

		m_orkWdg->ORKposeXlineEdit->setText(QString::number(m_poseFromCamera[0].X, 'f', 3));   
		m_orkWdg->ORKposeYlineEdit->setText(QString::number(m_poseFromCamera[0].Y, 'f', 3));
		m_orkWdg->ORKposeZlineEdit->setText(QString::number(m_poseFromCamera[0].Z, 'f', 3));
		
		m_orkWdg->ORKquaterXlineEdit->setText(QString::number(m_poseFromCamera[0].quaterX, 'f', 3));
		m_orkWdg->ORKquaterYlineEdit->setText(QString::number(m_poseFromCamera[0].quaterY, 'f', 3));
		m_orkWdg->ORKquaterZlineEdit->setText(QString::number(m_poseFromCamera[0].quaterZ, 'f', 3));
		m_orkWdg->ORKquaterWlineEdit->setText(QString::number(m_poseFromCamera[0].quaterW, 'f', 3));
		
		m_orkWdg->ORKeulerRlineEdit->setText(QString::number(m_poseFromCamera[0].eulerR*180/PI, 'f', 3));
		m_orkWdg->ORKeulerPlineEdit->setText(QString::number(m_poseFromCamera[0].eulerP*180/PI, 'f', 3));
		m_orkWdg->ORKeulerYlineEdit->setText(QString::number(m_poseFromCamera[0].eulerY*180/PI, 'f', 3));
        ROS_ERROR("X:%f",m_poseFromCamera[0].X);
      //}
      count_update++;
      pro_num_detectedObj = num_detectedObj;
      addObjectBox();
	}
  
       void PickPlacePanel::addObjectBoxdefines(std::string detection_id,int num,geometry_msgs::Pose box_pose)
       {
            os.clear();
            collision_object.primitives.clear();
            collision_object.primitive_poses.clear();
            collision_object.header.frame_id = "base_link";          
            if(detection_id == COKE_ID)
            {  
                os<<"coke"<<num;
                collision_object.id = os.str();
                object_ids.push_back(collision_object.id);
                primitive.type = primitive.CYLINDER;
                primitive.dimensions.resize(3);
                primitive.dimensions[0] = 0.11;
                primitive.dimensions[1] = 0.031;
                collision_object.primitives.push_back(primitive);
                collision_object.primitive_poses.push_back(box_pose);
                collision_object.operation = collision_object.ADD;
                
                collision_objects.push_back(collision_object);  
           }
           else if(detection_id == DIAMOND_ID) 
           {
               os<<"box"<<num;
               collision_object.id = os.str();
               object_ids.push_back(collision_object.id);
               primitive.type = primitive.BOX;
               primitive.dimensions.resize(3);
               primitive.dimensions[0] = 0.055;
               primitive.dimensions[1] = 0.055;
               primitive.dimensions[2] = 0.11;
               collision_object.primitives.push_back(primitive);
               collision_object.primitive_poses.push_back(box_pose);
               collision_object.operation = collision_object.ADD;
                
               collision_objects.push_back(collision_object);  
           }            
       }
    
       void PickPlacePanel::addObjectBox()
       {
              ROS_INFO("Add an object into the world");
              planning_scene_interface.addCollisionObjects(collision_objects);

              /* Sleep so we have time to see the object in RViz */
              sleep(0.8);
              collision_objects.clear();
       }
 
       void PickPlacePanel::removeObjectBox()
       {
            ROS_INFO("Remove the object from the world");
            
            object_ids.push_back(collision_object.id);
            planning_scene_interface.removeCollisionObjects(object_ids);
            /* Sleep to give Rviz time to show the object is no longer there. */
            
            sleep(1.0);
            //object_ids.clear();
       }


       void PickPlacePanel::voicePickServer()
       {                     
            //node Handle
            ROS_ERROR("Voice service is opend!");
            voice_pick_coke = n_pickPlace.advertiseService("VoicePickCoke", &PickPlacePanel::voicePickCoke_callback, this);
            voice_pick_diamond = n_pickPlace.advertiseService("VoicePickDiamond", &PickPlacePanel::voicePickDiamond_callback, this);
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Voice service is opend!");
            //ros::spin();
       }

       bool PickPlacePanel::voicePickCoke_callback(rviz_pickplace_commander::voicePickCoke::Request  &req,
                                                   rviz_pickplace_commander::voicePickCoke::Response &res)
       { 
           for(int i=0;i<num_detectedObj;i++)
	   { 
                if(detection_ID[i] != COKE_ID)
				{
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append(QString::fromStdString(detection_ID[i]) +" was not coke_id:"+QString::fromStdString(COKE_ID));	
                    continue;
				}
                //ROS_ERROR("ʶ��Ŀ��:%d,��X����Ϊ%f",i,m_poseFromCamera[i].X);
	    std::vector<std::string> object_idre;
		object_idre.push_back(object_ids[i]);
        planning_scene_interface.removeCollisionObjects(object_idre);
        m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Ready to pick...");	
		hsr_pick::pickPlace pickPlace_srv;
		
		geometry_msgs::PoseStamped pickPose,placePose;
	
		pickPose.pose.position.x = base_detectPoseFromCamera[i].pose.position.x;
		pickPose.pose.position.y = base_detectPoseFromCamera[i].pose.position.y;
		pickPose.pose.position.z = base_detectPoseFromCamera[i].pose.position.z;
		pickPose.pose.orientation.x = base_detectPoseFromCamera[i].pose.orientation.x;
		pickPose.pose.orientation.y = base_detectPoseFromCamera[i].pose.orientation.y;
		pickPose.pose.orientation.z = base_detectPoseFromCamera[i].pose.orientation.z;
		pickPose.pose.orientation.w = base_detectPoseFromCamera[i].pose.orientation.w;

		// pickPose.pose.position.x = m_pickPlaceWdg->pickXlineEdit->text().toDouble();
		// pickPose.pose.position.y = m_pickPlaceWdg->pickYlineEdit->text().toDouble();
		// pickPose.pose.position.z = m_pickPlaceWdg->pickZlineEdit->text().toDouble();
		// pickPose.pose.orientation.x = m_pickPlaceWdg->pickQuaterXlineEdit->text().toDouble();
		// pickPose.pose.orientation.y = m_pickPlaceWdg->pickQuaterYlineEdit->text().toDouble();
		// pickPose.pose.orientation.z = m_pickPlaceWdg->pickQuaterZlineEdit->text().toDouble();
		// pickPose.pose.orientation.w = m_pickPlaceWdg->pickQuaterWlineEdit->text().toDouble();
		
		placePose.pose.position.x = m_pickPlaceWdg->placeXlineEdit->text().toDouble();
		placePose.pose.position.y = m_pickPlaceWdg->placeYlineEdit->text().toDouble();
		placePose.pose.position.z = m_pickPlaceWdg->placeZlineEdit->text().toDouble();
		placePose.pose.orientation.x = m_pickPlaceWdg->placeQuaterXlineEdit->text().toDouble();
		placePose.pose.orientation.y = m_pickPlaceWdg->placeQuaterYlineEdit->text().toDouble();
		placePose.pose.orientation.z = m_pickPlaceWdg->placeQuaterZlineEdit->text().toDouble();
		placePose.pose.orientation.w = m_pickPlaceWdg->placeQuaterWlineEdit->text().toDouble();	

		pickPlace_srv.request.pickPos = pickPose;
		pickPlace_srv.request.placePos = placePose;
		
		if(client_pickPlace.call(pickPlace_srv))
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Join the pickPlace_srv sucessful!!!");
			ROS_INFO("Join the pickPlace_srv sucessful!!!");
		}
		else
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Join the pickPlace_srv failed!!!");
			ROS_ERROR("Join the pickPlace_srv failed!!!");
		}
		return true;	 
	    }
		    m_pickPlaceWdg->pickPlaceStatusTextEdit->append("no detected data");	
			ROS_ERROR("no detected data");
            return false;
       }

       bool PickPlacePanel::voicePickDiamond_callback(rviz_pickplace_commander::voicePickDiamond::Request  &req,
                                                      rviz_pickplace_commander::voicePickDiamond::Response &res)
       { 
           for(int i=0;i<num_detectedObj;i++)
	   { 
                if(detection_ID[i] != DIAMOND_ID)
                {
					//ROS_ERROR("detection_ID was not diamand_id 'f45677f2dac7ae441e6fb2901d00286a'");
					//m_pickPlaceWdg->pickPlaceStatusTextEdit->append("detection_ID was not diamand_id 'f45677f2dac7ae441e6fb2901d00286a'");
					m_pickPlaceWdg->pickPlaceStatusTextEdit->append(QString::fromStdString(detection_ID[i]) +" was not coke_id:"+QString::fromStdString(DIAMOND_ID));		
                    continue;
				}

                //ROS_ERROR("ʶ��Ŀ��:%d,��X����Ϊ%f",i,m_poseFromCamera[i].X);
		std::vector<std::string> object_idre;
		object_idre.push_back(object_ids[i]);
        planning_scene_interface.removeCollisionObjects(object_idre);

        m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Ready to pick...");	
		hsr_pick::pickPlace pickPlace_srv;
		
		geometry_msgs::PoseStamped pickPose,placePose;
	
		pickPose.pose.position.x = base_detectPoseFromCamera[i].pose.position.x;
		pickPose.pose.position.y = base_detectPoseFromCamera[i].pose.position.y;
		pickPose.pose.position.z = base_detectPoseFromCamera[i].pose.position.z;
		pickPose.pose.orientation.x = base_detectPoseFromCamera[i].pose.orientation.x;
		pickPose.pose.orientation.y = base_detectPoseFromCamera[i].pose.orientation.y;
		pickPose.pose.orientation.z = base_detectPoseFromCamera[i].pose.orientation.z;
		pickPose.pose.orientation.w = base_detectPoseFromCamera[i].pose.orientation.w;
		
		placePose.pose.position.x = m_pickPlaceWdg->placeXlineEdit->text().toDouble();
		placePose.pose.position.y = m_pickPlaceWdg->placeYlineEdit->text().toDouble();
		placePose.pose.position.z = m_pickPlaceWdg->placeZlineEdit->text().toDouble();
		placePose.pose.orientation.x = m_pickPlaceWdg->placeQuaterXlineEdit->text().toDouble();
		placePose.pose.orientation.y = m_pickPlaceWdg->placeQuaterYlineEdit->text().toDouble();
		placePose.pose.orientation.z = m_pickPlaceWdg->placeQuaterZlineEdit->text().toDouble();
		placePose.pose.orientation.w = m_pickPlaceWdg->placeQuaterWlineEdit->text().toDouble();	

		pickPlace_srv.request.pickPos = pickPose;
		pickPlace_srv.request.placePos = placePose;
		
		if(client_pickPlace.call(pickPlace_srv))
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Join the pickPlace_srv sucessful!!!");
			ROS_INFO("Join the pickPlace_srv sucessful!!!");
		}
		else
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Join the pickPlace_srv failed!!!");
			ROS_ERROR("Join the pickPlace_srv failed!!!");
		}	 
	    return true;
	    }
		ROS_ERROR("no detected data");
		m_pickPlaceWdg->pickPlaceStatusTextEdit->append("no detected data");
        return false;
       }

	void PickPlacePanel::slotToSelectDetectComoBox(int index)
	{
<<<<<<< HEAD
		m_orkWdg->ORKposeXlineEdit->setText(QString::number(m_poseFromCamera[index].X, 'f', 3));    /* X*1000 将单位由m转成mm */
=======
		m_orkWdg->ORKposeXlineEdit->setText(QString::number(m_poseFromCamera[index].X, 'f', 3));    /* X*1000 ����λ��mת��mm */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
		m_orkWdg->ORKposeYlineEdit->setText(QString::number(m_poseFromCamera[index].Y, 'f', 3));
		m_orkWdg->ORKposeZlineEdit->setText(QString::number(m_poseFromCamera[index].Z, 'f', 3));
		
		m_orkWdg->ORKquaterXlineEdit->setText(QString::number(m_poseFromCamera[index].quaterX, 'f', 3));
		m_orkWdg->ORKquaterYlineEdit->setText(QString::number(m_poseFromCamera[index].quaterY, 'f', 3));
		m_orkWdg->ORKquaterZlineEdit->setText(QString::number(m_poseFromCamera[index].quaterZ, 'f', 3));
		m_orkWdg->ORKquaterWlineEdit->setText(QString::number(m_poseFromCamera[index].quaterW, 'f', 3));
		
		m_orkWdg->ORKeulerRlineEdit->setText(QString::number(m_poseFromCamera[index].eulerR*180/PI, 'f', 3));
		m_orkWdg->ORKeulerPlineEdit->setText(QString::number(m_poseFromCamera[index].eulerP*180/PI, 'f', 3));
		m_orkWdg->ORKeulerYlineEdit->setText(QString::number(m_poseFromCamera[index].eulerY*180/PI, 'f', 3));
	}
	
	void PickPlacePanel::slotGetPickPoseFromOrk()
	{
		//
		m_pickPlaceWdg->pickXlineEdit->setText(m_orkWdg->ORKposeXlineEdit->text());
		m_pickPlaceWdg->pickYlineEdit->setText(m_orkWdg->ORKposeYlineEdit->text());
		m_pickPlaceWdg->pickZlineEdit->setText(m_orkWdg->ORKposeZlineEdit->text());
		m_pickPlaceWdg->pickEulerRlineEdit->setText(m_orkWdg->ORKeulerRlineEdit->text());
		m_pickPlaceWdg->pickEulerPlineEdit->setText(m_orkWdg->ORKeulerPlineEdit->text());
		m_pickPlaceWdg->pickEulerYlineEdit->setText(m_orkWdg->ORKeulerYlineEdit->text());
			
		m_pickPlaceWdg->pickQuaterXlineEdit->setText(m_orkWdg->ORKquaterXlineEdit->text());
		m_pickPlaceWdg->pickQuaterYlineEdit->setText(m_orkWdg->ORKquaterYlineEdit->text());
		m_pickPlaceWdg->pickQuaterZlineEdit->setText(m_orkWdg->ORKquaterZlineEdit->text());
		m_pickPlaceWdg->pickQuaterWlineEdit->setText(m_orkWdg->ORKquaterWlineEdit->text());
		
	}
	
    void PickPlacePanel::slotGetPlacePoseOkBtn()
	{
		//
		//sub_poseFrom3d = n_pickPlace.subscribe("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", 1000, &PickPlacePanel::poseFrom3d_callback, this);
    
        ork_interface::detection_srv detection_srv;
        detection_srv.request.cmd = "\'c\'"; 
		detection_srv.request.path = path_detect_config;
        detection_srv.request.isDetected = true;
        
        if(client_detection.call(detection_srv))
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Detection stop sucessful!!!");
		}
		else
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Detection stop failed!!!");
		}
    
	}
	void PickPlacePanel::poseFrom3d_callback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& msg)
	{
        detectPoseFrom3d = new geometry_msgs::PoseStamped[1];
		base_detectPoseFrom3d = new geometry_msgs::PoseStamped[1];
		//m_poseFrom3d = new T_POSE_FORM_CAMERA[1];
		
		tf::TransformListener listener;
		geometry_msgs::PoseStamped base_temp;
		
	    detectPoseFrom3d[0].header.seq = 1;
		detectPoseFrom3d[0].header.frame_id = "world";
			
		detectPoseFrom3d[0].pose = msg->pose;
        ROS_ERROR("detectPoseFrom3d[0].pose.X:%f",detectPoseFrom3d[0].pose.position.x);
			
	    while(1)
        {
        	static int count =0;
		    try
		    {
				listener.transformPose("base_link",detectPoseFrom3d[0],base_detectPoseFrom3d[0]);
			}
			catch(tf::TransformException &ex)
			{
		         ROS_ERROR("%s",ex.what());
		         ros::Duration(1.0).sleep();
                 ROS_ERROR("form world frame to Base frame failed !!!");
		    }

            if((base_detectPoseFrom3d[0].pose.position.x !=0.0 )&& (base_detectPoseFrom3d[0].pose.position.y !=0.0))
                 break;
<<<<<<< HEAD
                /* 循环100次，若还未读到坐标则跳出 */
=======
                /* ѭ��100�Σ���δ�������������� */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
            count++;
            if(count >100)
            {
               ROS_ERROR("form world frame to Base frame failed !!!");
               break;
            }
        }
		
		float X = base_detectPoseFrom3d[0].pose.position.x;
		float Y = base_detectPoseFrom3d[0].pose.position.y;
		float Z = base_detectPoseFrom3d[0].pose.position.z;
		
		float q0 = base_detectPoseFrom3d[0].pose.orientation.w;
		float q1 = base_detectPoseFrom3d[0].pose.orientation.x;
		float q2 = base_detectPoseFrom3d[0].pose.orientation.y;
		float q3 = base_detectPoseFrom3d[0].pose.orientation.z;
		
		float A = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
		float B = asin(2*(q0*q2-q1*q3));
		float C = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

		
		if(m_pickPlaceWdg->getPlacePoseMeans->currentIndex()==0)
		{
<<<<<<< HEAD
		    m_pickPlaceWdg->placeXlineEdit->setText(QString::number(X, 'f', 3));    /* X*1000 将单位由m转成mm */
=======
		    m_pickPlaceWdg->placeXlineEdit->setText(QString::number(X, 'f', 3));    /* X*1000 ����λ��mת��mm */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
		    m_pickPlaceWdg->placeYlineEdit->setText(QString::number(Y, 'f', 3));
		    m_pickPlaceWdg->placeZlineEdit->setText(QString::number(Z, 'f', 3));
		
		    m_pickPlaceWdg->placeQuaterXlineEdit->setText(QString::number(q1, 'f', 3));
		    m_pickPlaceWdg->placeQuaterYlineEdit->setText(QString::number(q2, 'f', 3));
		    m_pickPlaceWdg->placeQuaterZlineEdit->setText(QString::number(q3, 'f', 3));
		    m_pickPlaceWdg->placeQuaterWlineEdit->setText(QString::number(q0, 'f', 3));
		
		    m_pickPlaceWdg->placeEulerAlineEdit->setText(QString::number(A*180/PI, 'f', 3));
		    m_pickPlaceWdg->placeEulerBlineEdit->setText(QString::number(B*180/PI, 'f', 3));
		    m_pickPlaceWdg->placeEulerClineEdit->setText(QString::number(C*180/PI, 'f', 3));
		}
		
		if(m_pickPlaceWdg->getPickPoseMeans->currentIndex()==1)
		{
<<<<<<< HEAD
		    m_pickPlaceWdg->pickXlineEdit->setText(QString::number(X, 'f', 3));    /* X*1000 将单位由m转成mm */
=======
		    m_pickPlaceWdg->pickXlineEdit->setText(QString::number(X, 'f', 3));    /* X*1000 ����λ��mת��mm */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
		    m_pickPlaceWdg->pickYlineEdit->setText(QString::number(Y, 'f', 3));
		    m_pickPlaceWdg->pickZlineEdit->setText(QString::number(Z, 'f', 3));
		
		    m_pickPlaceWdg->pickQuaterXlineEdit->setText(QString::number(q1, 'f', 3));
		    m_pickPlaceWdg->pickQuaterYlineEdit->setText(QString::number(q2, 'f', 3));
		    m_pickPlaceWdg->pickQuaterZlineEdit->setText(QString::number(q3, 'f', 3));
		    m_pickPlaceWdg->pickQuaterWlineEdit->setText(QString::number(q0, 'f', 3));
		
		    m_pickPlaceWdg->pickEulerRlineEdit->setText(QString::number(A*180/PI, 'f', 3));
		    m_pickPlaceWdg->pickEulerPlineEdit->setText(QString::number(B*180/PI, 'f', 3));
		    m_pickPlaceWdg->pickEulerYlineEdit->setText(QString::number(C*180/PI, 'f', 3));
		}

        
	}
	
	void PickPlacePanel::slotGetPickPoseMeansComoBox(int index)
	{
		switch(index)
		{
<<<<<<< HEAD
			case 0:  //自动识别
			{
                //定时器未启动时，开启定时器
				if(getPickPoseFromORK_Timer->isActive() == false)
				{
				    getPickPoseFromORK_Timer->start(100);                      /* 启动定时器，从ORK识别模块获取抓取物位姿 */
=======
			case 0:  //�Զ�ʶ��
			{
                //��ʱ��δ��ʱ������ʱ��
				if(getPickPoseFromORK_Timer->isActive() == false)
				{
				    getPickPoseFromORK_Timer->start(100);                      /* ���ʱ������ORKʶ��ģ���ȡץȡ��λ�� */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				}
				
				m_pickPlaceWdg->pickXlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickYlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickZlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickEulerRlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickEulerPlineEdit->setReadOnly(true);
			    m_pickPlaceWdg->pickEulerYlineEdit->setReadOnly(true);
				
				m_pickPlaceWdg->pickQuaterXlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickQuaterYlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickQuaterZlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickQuaterWlineEdit->setReadOnly(true);
				
				break;
			}
<<<<<<< HEAD
			case 1:  //空间读取
			{
				getPickPoseFromORK_Timer->stop();                              /* 关闭定时器，不从ORK识别模块获取抓取物位姿 */
=======
			case 1:  //�ռ��ȡ
			{
				getPickPoseFromORK_Timer->stop();                              /* �رն�ʱ��������ORKʶ��ģ���ȡץȡ��λ�� */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				
				sub_poseFrom3d = n_pickPlace.subscribe("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", 1000,\
                                                       &PickPlacePanel::poseFrom3d_callback, this);
				
				m_pickPlaceWdg->pickXlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickYlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickZlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickEulerRlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickEulerPlineEdit->setReadOnly(true);
			    m_pickPlaceWdg->pickEulerYlineEdit->setReadOnly(true);
				
				m_pickPlaceWdg->pickQuaterXlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickQuaterYlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickQuaterZlineEdit->setReadOnly(true);
				m_pickPlaceWdg->pickQuaterWlineEdit->setReadOnly(true);
				
				break;
			}
<<<<<<< HEAD
		    case 2:  //手动设置
			{
				getPickPoseFromORK_Timer->stop();                              /* 关闭定时器，不从ORK识别模块获取抓取物位姿 */
=======
		    case 2:  //�ֶ�����
			{
				getPickPoseFromORK_Timer->stop();                              /* �رն�ʱ��������ORKʶ��ģ���ȡץȡ��λ�� */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
				
				m_pickPlaceWdg->pickXlineEdit->setReadOnly(false);
				m_pickPlaceWdg->pickYlineEdit->setReadOnly(false);
				m_pickPlaceWdg->pickZlineEdit->setReadOnly(false);
				m_pickPlaceWdg->pickEulerRlineEdit->setReadOnly(false);
				m_pickPlaceWdg->pickEulerPlineEdit->setReadOnly(false);
			    m_pickPlaceWdg->pickEulerYlineEdit->setReadOnly(false);
				
				m_pickPlaceWdg->pickQuaterXlineEdit->setReadOnly(false);
				m_pickPlaceWdg->pickQuaterYlineEdit->setReadOnly(false);
				m_pickPlaceWdg->pickQuaterZlineEdit->setReadOnly(false);
				m_pickPlaceWdg->pickQuaterWlineEdit->setReadOnly(false);
				
			    break;
			}
		    default:
			    break;
		}
	}
	
	void PickPlacePanel::slotGetPlacePoseMeansComoBox(int index)
	{
		//
		switch(index)
		{
<<<<<<< HEAD
			case 0:  //空间读取
=======
			case 0:  //�ռ��ȡ
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			{
				sub_poseFrom3d = n_pickPlace.subscribe("/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback", 1000,\
                                                       &PickPlacePanel::poseFrom3d_callback, this);
													   
			    m_pickPlaceWdg->placeXlineEdit->setReadOnly(true);
				m_pickPlaceWdg->placeYlineEdit->setReadOnly(true);
			    m_pickPlaceWdg->placeZlineEdit->setReadOnly(true);
				m_pickPlaceWdg->placeEulerAlineEdit->setReadOnly(true);
				m_pickPlaceWdg->placeEulerBlineEdit->setReadOnly(true);
				m_pickPlaceWdg->placeEulerClineEdit->setReadOnly(true);
				
				m_pickPlaceWdg->placeQuaterXlineEdit->setReadOnly(true);
				m_pickPlaceWdg->placeQuaterYlineEdit->setReadOnly(true);
				m_pickPlaceWdg->placeQuaterZlineEdit->setReadOnly(true);
				m_pickPlaceWdg->placeQuaterWlineEdit->setReadOnly(true);
				
				break;
			}
<<<<<<< HEAD
		    case 1:  //手动设置
=======
		    case 1:  //�ֶ�����
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
			{
				m_pickPlaceWdg->placeXlineEdit->setReadOnly(false);
		        m_pickPlaceWdg->placeYlineEdit->setReadOnly(false);
		        m_pickPlaceWdg->placeZlineEdit->setReadOnly(false);
		
		        m_pickPlaceWdg->placeQuaterXlineEdit->setReadOnly(false);
		        m_pickPlaceWdg->placeQuaterYlineEdit->setReadOnly(false);
		        m_pickPlaceWdg->placeQuaterZlineEdit->setReadOnly(false);
		        m_pickPlaceWdg->placeQuaterWlineEdit->setReadOnly(false);
		
		        m_pickPlaceWdg->placeEulerAlineEdit->setReadOnly(false);
		        m_pickPlaceWdg->placeEulerBlineEdit->setReadOnly(false);
		        m_pickPlaceWdg->placeEulerClineEdit->setReadOnly(false);
			    break;
			}
		    default:
			    break;
		}
	}
	
	void PickPlacePanel::slotRunPickPlaceBtn()
	{
		//client_pickPlace		
	        m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Ready to pick...");	
		hsr_pick::pickPlace pickPlace_srv;
		
		geometry_msgs::PoseStamped pickPose,placePose;
	
		pickPose.pose.position.x = m_pickPlaceWdg->pickXlineEdit->text().toDouble();
		pickPose.pose.position.y = m_pickPlaceWdg->pickYlineEdit->text().toDouble();
		pickPose.pose.position.z = m_pickPlaceWdg->pickZlineEdit->text().toDouble();
		pickPose.pose.orientation.x = m_pickPlaceWdg->pickQuaterXlineEdit->text().toDouble();
		pickPose.pose.orientation.y = m_pickPlaceWdg->pickQuaterYlineEdit->text().toDouble();
		pickPose.pose.orientation.z = m_pickPlaceWdg->pickQuaterZlineEdit->text().toDouble();
		pickPose.pose.orientation.w = m_pickPlaceWdg->pickQuaterWlineEdit->text().toDouble();
		
		placePose.pose.position.x = m_pickPlaceWdg->placeXlineEdit->text().toDouble();
		placePose.pose.position.y = m_pickPlaceWdg->placeYlineEdit->text().toDouble();
		placePose.pose.position.z = m_pickPlaceWdg->placeZlineEdit->text().toDouble();
		placePose.pose.orientation.x = m_pickPlaceWdg->placeQuaterXlineEdit->text().toDouble();
		placePose.pose.orientation.y = m_pickPlaceWdg->placeQuaterYlineEdit->text().toDouble();
		placePose.pose.orientation.z = m_pickPlaceWdg->placeQuaterZlineEdit->text().toDouble();
		placePose.pose.orientation.w = m_pickPlaceWdg->placeQuaterWlineEdit->text().toDouble();	

		pickPlace_srv.request.pickPos = pickPose;
		pickPlace_srv.request.placePos = placePose;
		
		if(client_pickPlace.call(pickPlace_srv))
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Join the pickPlace_srv sucessful!!!");
			ROS_INFO("Join the pickPlace_srv sucessful!!!");
		}
		else
		{
			m_pickPlaceWdg->pickPlaceStatusTextEdit->append("Join the pickPlace_srv failed!!!");
			ROS_ERROR("Join the pickPlace_srv failed!!!");
		}		
	}

        void PickPlacePanel::slotVoicePickPlaceBtn()
        {
              voicePickServer(); 
        }
}

<<<<<<< HEAD
/* 声明此类是一个rviz的插件 */
=======
/* ����������һ��rviz�Ĳ�� */
>>>>>>> 1d1cd814de1cc6fa51e2c11b0a872fcb7a1d9685
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_pickplace_commander::PickPlacePanel,rviz::Panel )

/* end control_pad.cpp*/

