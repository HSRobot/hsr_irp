#include "control_pad.h"

namespace rviz_control_commander
{
    /* ���캯������ʼ������ */
    ControlPanel::ControlPanel(QWidget* parent) : rviz::Panel(parent)
    {
        /* ������ */
        alarmGroupBox = new QGroupBox("Alarm");		/*����GroupBox */
        alarmShowLabel = new QLabel("No alarm error");
        clearAlarmBtn = new QPushButton("Clear");
        okAlarmBtn = new QPushButton("Ok");
		
        alarmLayout = new QGridLayout;	  /* ���������񲼾� */
        alarmLayout->addWidget(alarmShowLabel,0,0,2,5);
        alarmLayout->addWidget(clearAlarmBtn,0,5,1,1);
        alarmLayout->addWidget(okAlarmBtn,1,5,1,1);

        alarmGroupBox->setLayout(alarmLayout);    /* ���������� */

        /* �ֶ�/�Զ�ģʽ������ */
        manualOrAutoGroupBox = new QGroupBox("Manual/Auto");
        manualRadioBtn = new QRadioButton("Manual");
		manualRadioBtn->setChecked(true);    /* Ĭ��ѡ���ֶ�ģʽ */
        autoRadioBtn = new QRadioButton("Auto");

        manuaOrAutoLayout=new QHBoxLayout;
	    manuaOrAutoLayout->addWidget(manualRadioBtn);
	    manuaOrAutoLayout->addWidget(autoRadioBtn);

	    manualOrAutoGroupBox->setLayout(manuaOrAutoLayout);    /* �ֶ�/�Զ�ģʽѡ��ģ�鲼�� */
	
        /* ����ϵ������ */
        coordSelectGroupBox = new QGroupBox("Joint/Cartesian coord");
        jointRadioBtn = new QRadioButton("Joint");
		jointRadioBtn->setChecked(true);    /* Ĭ��ѡ��ؽ�����ϵ */
        cartesianRadioBtn = new QRadioButton("Cartesian");

        coordSelectLayout=new QHBoxLayout;
	    coordSelectLayout->addWidget(jointRadioBtn);
	    coordSelectLayout->addWidget(cartesianRadioBtn);

	    coordSelectGroupBox->setLayout(coordSelectLayout);    /* ����ϵ����ģ�鲼�� */
		

		/* ���������� */
        ratioGroupBox = new QGroupBox("Speed Ratio");
        ratioSlider = new QSlider(this);
        ratioSpinBox = new QSpinBox(this);

        ratioLayout=new QHBoxLayout;
	    ratioLayout->addWidget(ratioSlider);
	    ratioLayout->addWidget(ratioSpinBox);

	    ratioGroupBox->setLayout(ratioLayout);    /* ���ʵ���ģ�鲼�� */

		/*��������΢����*/
	    ratioSpinBox->setMinimum(1);  // ��Сֵ
	    ratioSpinBox->setMaximum(100);  // ���ֵ
	    ratioSpinBox->setSingleStep(1);  // ����
	    ratioSpinBox->setValue(20);

	    /*�������û�����*/
	    ratioSlider->setOrientation(Qt::Horizontal);  // ˮƽ����
	    ratioSlider->setMinimum(1);  // ��Сֵ
	    ratioSlider->setMaximum(100);  // ���ֵ
	    ratioSlider->setSingleStep(1);  // ����
	    ratioSlider->setValue(20);

	    /* ���������ź� ��Ӧ  */
	    connect(ratioSpinBox, SIGNAL(valueChanged(int)), ratioSlider, SLOT(setValue(int)));
	    connect(ratioSlider, SIGNAL(valueChanged(int)), ratioSpinBox, SLOT(setValue(int)));
		
		/* ѡ�йؽ�/�ѿ�������ϵ ��Ӧ */
		connect(jointRadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotJointCoordRadioBtn()));           /* ѡ�йؽ�����ϵ ��Ӧ */
		connect(cartesianRadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotCartesianCoordRadioBtn()));   /* ѡ�еѿ�������ϵ ��Ӧ */
		
        /* *******************�ֶ�ģʽ������ *************************************/
        manualModeGroupBox = new QGroupBox("Manual Mode");
	
        //*******************************************************
        QString jointOrPosNum[6];
		
		for(int i=0;i<6;++i)
        {
            labelJointOrPosName[i] = new QLabel(this);
		    jointOrPosNum[i]="Joint "+ QString::number(i+1, 10) + ":"; /* ��ʾJoint 1:,Joint 2:,...,Joint 6: */
		    labelJointOrPosName[i]->setText(jointOrPosNum[i]);
			
			labelJointOrPosVal[i] = new QLabel(this);
			labelJointOrPosVal[i]->setAlignment(Qt::AlignHCenter);
			
            editJointOrPosVal[i] = new QLineEdit(this);

			oppositeRotateJntBtn[i] = new QPushButton("-");    /* �ؽ������˶� */
			positiveRotateJntBtn[i] = new QPushButton("+");    /* �ؽ������˶� */
        }
		
		//editJointOrPosVal[0]->setPlaceholderText("Enter the angle value of -155 to 155");
		editJointOrPosVal[0]->setPlaceholderText("-155 to 155");
		editJointOrPosVal[1]->setPlaceholderText("-88 to 78");
		editJointOrPosVal[2]->setPlaceholderText("+141.5 to 198.5");
		editJointOrPosVal[3]->setPlaceholderText("-90 to 90");
		editJointOrPosVal[4]->setPlaceholderText("-186.5 to 6.5");
		editJointOrPosVal[5]->setPlaceholderText("-360 to 360");
		
        move2targetBtn = new QPushButton("Mov2target");      /* �˶����� ��ť */
		goHomeBtn = new QPushButton("Go Home");              /* �˶����� ��ť */
		
		/* �ֶ�ģʽ���� */
		manualModeMainLayout = new QGridLayout;

		for(int i=0;i<6;++i)
		{
		    manualModeMainLayout->addWidget(labelJointOrPosName[i],i,0,1,1);
			manualModeMainLayout->addWidget(labelJointOrPosVal[i],i,1,1,2);
			manualModeMainLayout->addWidget(editJointOrPosVal[i],i,3,1,2);	
			manualModeMainLayout->addWidget(oppositeRotateJntBtn[i],i,6,1,1);
			manualModeMainLayout->addWidget(positiveRotateJntBtn[i],i,7,1,1);
		}
        
		manualModeMainLayout->addWidget(move2targetBtn,6,6,1,1);
		manualModeMainLayout->addWidget(goHomeBtn,6,7,1,1);
		
	    manualModeGroupBox->setLayout(manualModeMainLayout);    /* �ֶ�ģʽѡ��ģ�鲼�� */

        //********************************************************************************/

        /* �Զ�ģʽ������ */
		autoModeGroupBox = new QGroupBox("Auto Mode");		/* �Զ�ģʽGroupBox */
		loadBtn = new QPushButton("Load");	    /* �����ļ�Button; */
		runProgBtn = new QPushButton("Run");	/* �����ļ�Button; */
		pauseBtn = new QPushButton("Pause");	/* �����ļ�Button; */
		unloadBtn = new QPushButton("Unload"); 	/* ж���ļ�Button; */
		
		autoModeLayout = new QHBoxLayout;
		autoModeLayout->addWidget(loadBtn);
		autoModeLayout->addWidget(runProgBtn);
		autoModeLayout->addWidget(pauseBtn);
		autoModeLayout->addWidget(unloadBtn);

		autoModeGroupBox->setLayout(autoModeLayout);    /* �Զ�ģʽ������ */

        /*ʹ��״̬ */
        isEnableGroupBox = new QGroupBox("Is it Enable?");		 /* ʹ��״̬GroupBox */
		isEnabelBtn = new QPushButton("UnEnable");
		action_enable = new QAction(this);
		action_enable->setShortcut(QKeySequence(tr("Ctrl+E")));  /* ����ʹ�ܰ�ť��ݼ� */
		isEnabelBtn->addAction(action_enable);                   /* ����ݼ���ӵ���ť�� */
		
		isEnableLayout = new QHBoxLayout;
		isEnableLayout->addWidget(isEnabelBtn);
		
		isEnableGroupBox->setLayout(isEnableLayout);	/* ʹ��״̬����*/
		
        /* ���岼�� */
        controlMainLayout = new QGridLayout;
		controlMainLayout->addWidget(alarmGroupBox,0,0,1,2);
		controlMainLayout->addWidget(ratioGroupBox,0,2,1,2);
        controlMainLayout->addWidget(manualOrAutoGroupBox,1,0,1,2);
		controlMainLayout->addWidget(coordSelectGroupBox,1,2,1,2);
        controlMainLayout->addWidget(manualModeGroupBox,2,0,6,4);
        controlMainLayout->addWidget(autoModeGroupBox,8,0,1,3);
		controlMainLayout->addWidget(isEnableGroupBox,8,3,1,1);
		
        setLayout(controlMainLayout);

		/* ROS��� */
        client_enable = n_control.serviceClient<hsr_driver::Enable>("Enable");
		client_getEnable = n_control.serviceClient<hsr_driver::GetEnable>("GetEnable");
	
		sub_jointVal = n_control.subscribe("/joint_states", 1000, &ControlPanel::callback, this);
        sub_poseVal = n_control.subscribe("/robot_pose", 1000, &ControlPanel::pose_callback, this);
		
		//ros::spin();
		/* ����������Ϣ�����ùؽڽ� */
		pub_jointVal[0] = n_control.advertise<std_msgs::Float64>("/joint1_position_controller/command", 1000);
		pub_jointVal[1] = n_control.advertise<std_msgs::Float64>("/joint2_position_controller/command", 1000);		
		pub_jointVal[2] = n_control.advertise<std_msgs::Float64>("/joint3_position_controller/command", 1000);
		pub_jointVal[3] = n_control.advertise<std_msgs::Float64>("/joint4_position_controller/command", 1000);
		pub_jointVal[4] = n_control.advertise<std_msgs::Float64>("/joint5_position_controller/command", 1000);
		pub_jointVal[5] = n_control.advertise<std_msgs::Float64>("/joint6_position_controller/command", 1000);

		pubJoint = n_control.advertise<hsr_msg::Joints>("target_joint", 1000);
		
		/* ����������Ϣ�����õѿ���λ������ */
		pub_poseVal = n_control.advertise<geometry_msgs::Pose>("/target_pose", 1000);
		
		/* ���ð���ʹ�� ��Ӧ */
		connect(isEnabelBtn,SIGNAL(clicked()),this,SLOT(slotIsEnableBtnClicked()));
		
		/* ����ʹ�ܿ�ݼ� ��Ӧ */
		connect(action_enable, SIGNAL(triggered()), this, SLOT(slotShortCutEnableBtnClicked()));
		
		/* ���ؽ����˶���ָ���ؽڽǻ�ѿ������� ��Ӧ*/
		connect(move2targetBtn, SIGNAL(clicked()), this,SLOT(slotMov2TargetBtn()));
				
		/* �ص�Home��λ ��Ӧ*/
		connect(goHomeBtn, SIGNAL(clicked()), this,SLOT(slotGoHomeBtn()));
		
		showJointTimer = new QTimer(this);
		showJointTimer->start(40); //200����ʵ��һ�β�,Ĭ�Ͽ�����ʾ�ؽڽǶ�ʱ��
		connect(showJointTimer,SIGNAL(timeout()),this,SLOT(slotShowJoint()));
		
		showPosTimer = new QTimer(this);
		connect(showPosTimer,SIGNAL(timeout()),this,SLOT(slotShowPos()));
		
    }
	
	/* ѡ��ؽ�����ϵ �ۺ��� */
	void ControlPanel::slotJointCoordRadioBtn()
	{
		QString jointOrPosNum[6];
		for(int i=0;i<6;++i)
        {
		    jointOrPosNum[i]="Joint "+ QString::number(i+1, 10) + ":"; 
		    labelJointOrPosName[i]->setText(jointOrPosNum[i]);      /* ��ʾX:,Joint 2:,...,Joint 6: */
        }
		
		showPosTimer->stop();    /* �ر���ʾλ�˵Ķ�ʱ�� */
		
		/* �����ʾ�ؽڶ�ʱ��δ���������俪�� */
		if(!(showJointTimer->isActive()))
		{
			showJointTimer->start(40);
		}
	
	}
	
	/* ѡ��ѿ�������ϵ �ۺ��� */
	void ControlPanel::slotCartesianCoordRadioBtn()
	{
		labelJointOrPosName[0]->setText("X :");
		labelJointOrPosName[1]->setText("Y :");
		labelJointOrPosName[2]->setText("Z :");
		labelJointOrPosName[3]->setText("A :");
		labelJointOrPosName[4]->setText("B :");
		labelJointOrPosName[5]->setText("C :");
		
	    showJointTimer->stop();    /* �ر���ʾλ�˵Ķ�ʱ�� */
		
		/* �����ʾ�ؽڶ�ʱ��δ���������俪�� */
		if(!(showPosTimer->isActive()))
		{
			showPosTimer->start(40);
		}
		
	}
	
	/* ʹ�ܿ��� �ۺ��� */
    void ControlPanel::slotIsEnableBtnClicked()
    {
        if(client_getEnable.call(srv_getEnable))  /* �����ǰʹ�ܿ�����ر�ʹ�� */
        {
		    srv_enable.request.enable = false;
            client_enable.call(srv_enable);
			
			if(client_getEnable.call(srv_getEnable) == false)   /* �����ǰʹ�ܹرգ�����ʾEnable OFF */
			{
		        isEnabelBtn->setText("Enable OFF");
			}
        }
	    else{                                            /* �����ǰʹ�ܹأ����ʹ�� */
		    srv_enable.request.enable = true;
		    client_enable.call(srv_enable);
			
			if(client_getEnable.call(srv_getEnable))
			{
		        isEnabelBtn->setText("Enable ON");       /* �����ǰʹ�ܹرգ�����ʾEnable ON */
			}
	    }
		
	}
	
	/* ʹ�ܿ�ݼ� �ۺ��� */
	void ControlPanel::slotShortCutEnableBtnClicked()
    {
		alarmShowLabel->setText("Ctrl+E create finish!!!");
	}
	
	/* ��ȡ�ؽڽ�callback���� */
	void ControlPanel::callback(const sensor_msgs::JointState::ConstPtr& msg)
	{		
		for(int i=0;i<6;i++)
		{
			jointValue_Str[i] = QString::number((msg->position[i])*180/PI, 'f', 3);  /* ��ȡ���ؽڽ���Ϣ������floatתΪQString�ͣ�ͬʱ����3λС����ʾ */
		}
	}
	
	/* ��ȡ�ѿ�������callback���� 
	* ��Ԫ��תZYX��ŷ�����㷨��
	* ��pos�ж�ȡq0,q1,q2,q3����Ӧornt->w,ornt->x,ornt->y,ornt->z.
	*   A=alpha=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
	*   B=beta=asin(2*(q0*q2-q1*q3));
    *   C=gamma=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
	**/
	void ControlPanel::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{	
		float q0,q1,q2,q3;
		float A,B,C,X,Y,Z;
		
		q0 = msg->pose.orientation.w;
		q1 = msg->pose.orientation.x;
		q2 = msg->pose.orientation.y;
		q3 = msg->pose.orientation.z;
	
	    	A = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
		B = asin(2*(q0*q2-q1*q3));
		C = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
		
		X = msg->pose.position.x;
		Y = msg->pose.position.y;
		Z = msg->pose.position.z;
		
		posValue_Str[0] = QString::number(X*1000, 'f', 3);    /* X*1000 ����λ��mת��mm */
		posValue_Str[1] = QString::number(Y*1000, 'f', 3);
		posValue_Str[2] = QString::number(Z*1000, 'f', 3);
		posValue_Str[3] = QString::number(A*180/PI, 'f', 3);
		posValue_Str[4] = QString::number(B*180/PI, 'f', 3);
		posValue_Str[5] = QString::number(C*180/PI, 'f', 3);
		
	}
	
	/* �˶���Ŀ��� �ۺ��� */
	void ControlPanel::slotMov2TargetBtn()
	{
		if(jointRadioBtn->isChecked()) /* ���ѡ�йؽ�����ϵ */
		{
		std_msgs::Float64 pub_msg;
		hsr_msg::Joints msg;

		msg.joint1 = ((editJointOrPosVal[0]->text())).toFloat()*PI/180;
		msg.joint2 = ((editJointOrPosVal[1]->text())).toFloat()*PI/180;	
		msg.joint3 = ((editJointOrPosVal[2]->text())).toFloat()*PI/180;	
		msg.joint4 = ((editJointOrPosVal[3]->text())).toFloat()*PI/180;
		msg.joint5 = ((editJointOrPosVal[4]->text())).toFloat()*PI/180;	
		msg.joint6 = ((editJointOrPosVal[5]->text())).toFloat()*PI/180;
		
		pubJoint.publish(msg);
		/*
		    for(int i=0;i<6;i++)
		    {
			    /* ��ȡ�ؽڽǱ༭������ �����䣨QString���ͣ�ת��Ϊfloat���ҽ��Ƕ���ת���ɻ����� */
	           /* pub_msg.data = ((editJointOrPosVal[i]->text())).toFloat()*PI/180;
	            pub_jointVal[i].publish(pub_msg);
		    }*/
		}
		else{
			//
			geometry_msgs::Pose pub_poseMsg;
			
			pub_poseMsg.position.x = ((editJointOrPosVal[0]->text())).toFloat()/1000;
			pub_poseMsg.position.y = ((editJointOrPosVal[1]->text())).toFloat()/1000;
			pub_poseMsg.position.z = ((editJointOrPosVal[2]->text())).toFloat()/1000;
			
			float alpha,beta,gamma;
			alpha = ((editJointOrPosVal[3]->text())).toFloat()*PI/180;
			beta = ((editJointOrPosVal[4]->text())).toFloat()*PI/180;
			gamma = ((editJointOrPosVal[5]->text())).toFloat()*PI/180;
			
			/* ����Ϊ��ZYX��ŷ����ת������Ԫ�� */
			pub_poseMsg.orientation.w = cos(alpha/2)*cos(beta/2)*cos(gamma/2)+sin(alpha/2)*sin(beta/2)*sin(gamma/2);
			pub_poseMsg.orientation.x = sin(alpha/2)*cos(beta/2)*cos(gamma/2)-cos(alpha/2)*sin(beta/2)*sin(gamma/2);
			pub_poseMsg.orientation.y = cos(alpha/2)*sin(beta/2)*cos(gamma/2)+sin(alpha/2)*cos(beta/2)*sin(gamma/2);
			pub_poseMsg.orientation.z = cos(alpha/2)*cos(beta/2)*sin(gamma/2)-sin(alpha/2)*sin(beta/2)*cos(gamma/2);
			
			pub_poseVal.publish(pub_poseMsg);
		}
	}
	
	/* �˶���Home�� �ۺ��� */
	void ControlPanel::slotGoHomeBtn()
	{
		std_msgs::Float64 pub_msg;
		
		for(int i=0;i<6;i++)
		{
	        //pub_msg.data = 0.0;  /* ����Home����ؽڽ�Ϊ0 */
	        //pub_jointVal[i].publish(pub_msg);
			
			hsr_msg::Joints msg;
			
			msg.joint1 = 0.0;
			msg.joint2 = 0.0;
			msg.joint3 = 0.0;
			msg.joint4 = 0.0;
			msg.joint5 = 0.0;
			msg.joint6 = 0.0;
			
			pubJoint.publish(msg);
			
			editJointOrPosVal[i]->setText("0.0");
		}
	}
	
	/* ��ʾ�ؽ� �ۺ��� */
	void ControlPanel::slotShowJoint()
	{
		for(int i=0;i<6;i++)
		{
			labelJointOrPosVal[i]->setText(jointValue_Str[i]);
		}
		//alarmShowLabel->setText("jointValue_Str[0]="+jointValue_Str[0]);
	}
	
	/* ��ʾλ�� �ۺ��� */
	void ControlPanel::slotShowPos()
	{
		for(int i=0;i<6;i++)
		{
			labelJointOrPosVal[i]->setText(posValue_Str[i]);
		}
		
		alarmShowLabel->setText("posValue_Str[0]="+posValue_Str[0]);
	}
}

/* ����������һ��rviz�Ĳ�� */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_control_commander::ControlPanel,rviz::Panel )

/* end control_pad.cpp*/

