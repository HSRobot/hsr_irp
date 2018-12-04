#include "control_pad.h"

namespace rviz_control_commander
{
    /* 构造函数，初始化变量 */
    ControlPanel::ControlPanel(QWidget* parent) : rviz::Panel(parent)
    {
        /* 报警栏 */
        alarmGroupBox = new QGroupBox("Alarm");		/*报警GroupBox */
        alarmShowLabel = new QLabel("No alarm error");
        clearAlarmBtn = new QPushButton("Clear");
        okAlarmBtn = new QPushButton("Ok");
		
        alarmLayout = new QGridLayout;	  /* 报警栏网格布局 */
        alarmLayout->addWidget(alarmShowLabel,0,0,2,5);
        alarmLayout->addWidget(clearAlarmBtn,0,5,1,1);
        alarmLayout->addWidget(okAlarmBtn,1,5,1,1);

        alarmGroupBox->setLayout(alarmLayout);    /* 报警栏布局 */

        /* 手动/自动模式设置栏 */
        manualOrAutoGroupBox = new QGroupBox("Manual/Auto");
        manualRadioBtn = new QRadioButton("Manual");
		manualRadioBtn->setChecked(true);    /* 默认选择手动模式 */
        autoRadioBtn = new QRadioButton("Auto");

        manuaOrAutoLayout=new QHBoxLayout;
	    manuaOrAutoLayout->addWidget(manualRadioBtn);
	    manuaOrAutoLayout->addWidget(autoRadioBtn);

	    manualOrAutoGroupBox->setLayout(manuaOrAutoLayout);    /* 手动/自动模式选择模块布局 */
	
        /* 坐标系设置栏 */
        coordSelectGroupBox = new QGroupBox("Joint/Cartesian coord");
        jointRadioBtn = new QRadioButton("Joint");
		jointRadioBtn->setChecked(true);    /* 默认选择关节坐标系 */
        cartesianRadioBtn = new QRadioButton("Cartesian");

        coordSelectLayout=new QHBoxLayout;
	    coordSelectLayout->addWidget(jointRadioBtn);
	    coordSelectLayout->addWidget(cartesianRadioBtn);

	    coordSelectGroupBox->setLayout(coordSelectLayout);    /* 坐标系设置模块布局 */
		

		/* 倍率设置栏 */
        ratioGroupBox = new QGroupBox("Speed Ratio");
        ratioSlider = new QSlider(this);
        ratioSpinBox = new QSpinBox(this);

        ratioLayout=new QHBoxLayout;
	    ratioLayout->addWidget(ratioSlider);
	    ratioLayout->addWidget(ratioSpinBox);

	    ratioGroupBox->setLayout(ratioLayout);    /* 倍率调节模块布局 */

		/*倍率设置微调框*/
	    ratioSpinBox->setMinimum(1);  // 最小值
	    ratioSpinBox->setMaximum(100);  // 最大值
	    ratioSpinBox->setSingleStep(1);  // 步长
	    ratioSpinBox->setValue(20);

	    /*倍率设置滑动条*/
	    ratioSlider->setOrientation(Qt::Horizontal);  // 水平方向
	    ratioSlider->setMinimum(1);  // 最小值
	    ratioSlider->setMaximum(100);  // 最大值
	    ratioSlider->setSingleStep(1);  // 步长
	    ratioSlider->setValue(20);

	    /* 倍率设置信号 响应  */
	    connect(ratioSpinBox, SIGNAL(valueChanged(int)), ratioSlider, SLOT(setValue(int)));
	    connect(ratioSlider, SIGNAL(valueChanged(int)), ratioSpinBox, SLOT(setValue(int)));
		
		/* 选中关节/笛卡尔坐标系 响应 */
		connect(jointRadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotJointCoordRadioBtn()));           /* 选中关节坐标系 响应 */
		connect(cartesianRadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotCartesianCoordRadioBtn()));   /* 选中笛卡尔坐标系 响应 */
		
        /* *******************手动模式控制栏 *************************************/
        manualModeGroupBox = new QGroupBox("Manual Mode");
	
        //*******************************************************
        QString jointOrPosNum[6];
		
		for(int i=0;i<6;++i)
        {
            labelJointOrPosName[i] = new QLabel(this);
		    jointOrPosNum[i]="Joint "+ QString::number(i+1, 10) + ":"; /* 显示Joint 1:,Joint 2:,...,Joint 6: */
		    labelJointOrPosName[i]->setText(jointOrPosNum[i]);
			
			labelJointOrPosVal[i] = new QLabel(this);
			labelJointOrPosVal[i]->setAlignment(Qt::AlignHCenter);
			
            editJointOrPosVal[i] = new QLineEdit(this);

			oppositeRotateJntBtn[i] = new QPushButton("-");    /* 关节逆向运动 */
			positiveRotateJntBtn[i] = new QPushButton("+");    /* 关节正向运动 */
        }
		
		//editJointOrPosVal[0]->setPlaceholderText("Enter the angle value of -155 to 155");
		editJointOrPosVal[0]->setPlaceholderText("-155 to 155");
		editJointOrPosVal[1]->setPlaceholderText("-88 to 78");
		editJointOrPosVal[2]->setPlaceholderText("+141.5 to 198.5");
		editJointOrPosVal[3]->setPlaceholderText("-90 to 90");
		editJointOrPosVal[4]->setPlaceholderText("-186.5 to 6.5");
		editJointOrPosVal[5]->setPlaceholderText("-360 to 360");
		
        move2targetBtn = new QPushButton("Mov2target");      /* 运动到点 按钮 */
		goHomeBtn = new QPushButton("Go Home");              /* 运动到点 按钮 */
		
		/* 手动模式布局 */
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
		
	    manualModeGroupBox->setLayout(manualModeMainLayout);    /* 手动模式选择模块布局 */

        //********************************************************************************/

        /* 自动模式控制栏 */
		autoModeGroupBox = new QGroupBox("Auto Mode");		/* 自动模式GroupBox */
		loadBtn = new QPushButton("Load");	    /* 加载文件Button; */
		runProgBtn = new QPushButton("Run");	/* 运行文件Button; */
		pauseBtn = new QPushButton("Pause");	/* 运行文件Button; */
		unloadBtn = new QPushButton("Unload"); 	/* 卸载文件Button; */
		
		autoModeLayout = new QHBoxLayout;
		autoModeLayout->addWidget(loadBtn);
		autoModeLayout->addWidget(runProgBtn);
		autoModeLayout->addWidget(pauseBtn);
		autoModeLayout->addWidget(unloadBtn);

		autoModeGroupBox->setLayout(autoModeLayout);    /* 自动模式栏布局 */

        /*使能状态 */
        isEnableGroupBox = new QGroupBox("Is it Enable?");		 /* 使能状态GroupBox */
		isEnabelBtn = new QPushButton("UnEnable");
		action_enable = new QAction(this);
		action_enable->setShortcut(QKeySequence(tr("Ctrl+E")));  /* 设置使能按钮快捷键 */
		isEnabelBtn->addAction(action_enable);                   /* 将快捷键添加到按钮上 */
		
		isEnableLayout = new QHBoxLayout;
		isEnableLayout->addWidget(isEnabelBtn);
		
		isEnableGroupBox->setLayout(isEnableLayout);	/* 使能状态布局*/
		
        /* 整体布局 */
        controlMainLayout = new QGridLayout;
		controlMainLayout->addWidget(alarmGroupBox,0,0,1,2);
		controlMainLayout->addWidget(ratioGroupBox,0,2,1,2);
        controlMainLayout->addWidget(manualOrAutoGroupBox,1,0,1,2);
		controlMainLayout->addWidget(coordSelectGroupBox,1,2,1,2);
        controlMainLayout->addWidget(manualModeGroupBox,2,0,6,4);
        controlMainLayout->addWidget(autoModeGroupBox,8,0,1,3);
		controlMainLayout->addWidget(isEnableGroupBox,8,3,1,1);
		
        setLayout(controlMainLayout);

		/* ROS相关 */
        client_enable = n_control.serviceClient<hsr_driver::Enable>("Enable");
		client_getEnable = n_control.serviceClient<hsr_driver::GetEnable>("GetEnable");
	
		sub_jointVal = n_control.subscribe("/joint_states", 1000, &ControlPanel::callback, this);
        sub_poseVal = n_control.subscribe("/robot_pose", 1000, &ControlPanel::pose_callback, this);
		
		//ros::spin();
		/* 发布话题消息，设置关节角 */
		pub_jointVal[0] = n_control.advertise<std_msgs::Float64>("/joint1_position_controller/command", 1000);
		pub_jointVal[1] = n_control.advertise<std_msgs::Float64>("/joint2_position_controller/command", 1000);		
		pub_jointVal[2] = n_control.advertise<std_msgs::Float64>("/joint3_position_controller/command", 1000);
		pub_jointVal[3] = n_control.advertise<std_msgs::Float64>("/joint4_position_controller/command", 1000);
		pub_jointVal[4] = n_control.advertise<std_msgs::Float64>("/joint5_position_controller/command", 1000);
		pub_jointVal[5] = n_control.advertise<std_msgs::Float64>("/joint6_position_controller/command", 1000);

		pubJoint = n_control.advertise<hsr_msg::Joints>("target_joint", 1000);
		
		/* 发布话题消息，设置笛卡尔位置坐标 */
		pub_poseVal = n_control.advertise<geometry_msgs::Pose>("/target_pose", 1000);
		
		/* 设置按键使能 响应 */
		connect(isEnabelBtn,SIGNAL(clicked()),this,SLOT(slotIsEnableBtnClicked()));
		
		/* 设置使能快捷键 响应 */
		connect(action_enable, SIGNAL(triggered()), this, SLOT(slotShortCutEnableBtnClicked()));
		
		/* 各关节轴运动到指定关节角或笛卡尔坐标 响应*/
		connect(move2targetBtn, SIGNAL(clicked()), this,SLOT(slotMov2TargetBtn()));
				
		/* 回到Home点位 响应*/
		connect(goHomeBtn, SIGNAL(clicked()), this,SLOT(slotGoHomeBtn()));
		
		showJointTimer = new QTimer(this);
		showJointTimer->start(40); //200毫秒实现一次槽,默认开启显示关节角定时器
		connect(showJointTimer,SIGNAL(timeout()),this,SLOT(slotShowJoint()));
		
		showPosTimer = new QTimer(this);
		connect(showPosTimer,SIGNAL(timeout()),this,SLOT(slotShowPos()));
		
    }
	
	/* 选择关节坐标系 槽函数 */
	void ControlPanel::slotJointCoordRadioBtn()
	{
		QString jointOrPosNum[6];
		for(int i=0;i<6;++i)
        {
		    jointOrPosNum[i]="Joint "+ QString::number(i+1, 10) + ":"; 
		    labelJointOrPosName[i]->setText(jointOrPosNum[i]);      /* 显示X:,Joint 2:,...,Joint 6: */
        }
		
		showPosTimer->stop();    /* 关闭显示位姿的定时器 */
		
		/* 如果显示关节定时器未开启，则将其开启 */
		if(!(showJointTimer->isActive()))
		{
			showJointTimer->start(40);
		}
	
	}
	
	/* 选择笛卡尔坐标系 槽函数 */
	void ControlPanel::slotCartesianCoordRadioBtn()
	{
		labelJointOrPosName[0]->setText("X :");
		labelJointOrPosName[1]->setText("Y :");
		labelJointOrPosName[2]->setText("Z :");
		labelJointOrPosName[3]->setText("A :");
		labelJointOrPosName[4]->setText("B :");
		labelJointOrPosName[5]->setText("C :");
		
	    showJointTimer->stop();    /* 关闭显示位姿的定时器 */
		
		/* 如果显示关节定时器未开启，则将其开启 */
		if(!(showPosTimer->isActive()))
		{
			showPosTimer->start(40);
		}
		
	}
	
	/* 使能开关 槽函数 */
    void ControlPanel::slotIsEnableBtnClicked()
    {
        if(client_getEnable.call(srv_getEnable))  /* 如果当前使能开，则关闭使能 */
        {
		    srv_enable.request.enable = false;
            client_enable.call(srv_enable);
			
			if(client_getEnable.call(srv_getEnable) == false)   /* 如果当前使能关闭，则显示Enable OFF */
			{
		        isEnabelBtn->setText("Enable OFF");
			}
        }
	    else{                                            /* 如果当前使能关，则打开使能 */
		    srv_enable.request.enable = true;
		    client_enable.call(srv_enable);
			
			if(client_getEnable.call(srv_getEnable))
			{
		        isEnabelBtn->setText("Enable ON");       /* 如果当前使能关闭，则显示Enable ON */
			}
	    }
		
	}
	
	/* 使能快捷键 槽函数 */
	void ControlPanel::slotShortCutEnableBtnClicked()
    {
		alarmShowLabel->setText("Ctrl+E create finish!!!");
	}
	
	/* 读取关节角callback函数 */
	void ControlPanel::callback(const sensor_msgs::JointState::ConstPtr& msg)
	{		
		for(int i=0;i<6;i++)
		{
			jointValue_Str[i] = QString::number((msg->position[i])*180/PI, 'f', 3);  /* 获取各关节角信息，并将float转为QString型，同时保留3位小数显示 */
		}
	}
	
	/* 读取笛卡尔坐标callback函数 
	* 四元素转ZYX型欧拉角算法：
	* 从pos中读取q0,q1,q2,q3，对应ornt->w,ornt->x,ornt->y,ornt->z.
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
		
		posValue_Str[0] = QString::number(X*1000, 'f', 3);    /* X*1000 将单位有m转成mm */
		posValue_Str[1] = QString::number(Y*1000, 'f', 3);
		posValue_Str[2] = QString::number(Z*1000, 'f', 3);
		posValue_Str[3] = QString::number(A*180/PI, 'f', 3);
		posValue_Str[4] = QString::number(B*180/PI, 'f', 3);
		posValue_Str[5] = QString::number(C*180/PI, 'f', 3);
		
	}
	
	/* 运动到目标点 槽函数 */
	void ControlPanel::slotMov2TargetBtn()
	{
		if(jointRadioBtn->isChecked()) /* 如果选中关节坐标系 */
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
			    /* 读取关节角编辑框数据 并将其（QString类型）转化为float，且将角度制转换成弧度制 */
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
			
			/* 以下为将ZYX型欧拉角转换成四元素 */
			pub_poseMsg.orientation.w = cos(alpha/2)*cos(beta/2)*cos(gamma/2)+sin(alpha/2)*sin(beta/2)*sin(gamma/2);
			pub_poseMsg.orientation.x = sin(alpha/2)*cos(beta/2)*cos(gamma/2)-cos(alpha/2)*sin(beta/2)*sin(gamma/2);
			pub_poseMsg.orientation.y = cos(alpha/2)*sin(beta/2)*cos(gamma/2)+sin(alpha/2)*cos(beta/2)*sin(gamma/2);
			pub_poseMsg.orientation.z = cos(alpha/2)*cos(beta/2)*sin(gamma/2)-sin(alpha/2)*sin(beta/2)*cos(gamma/2);
			
			pub_poseVal.publish(pub_poseMsg);
		}
	}
	
	/* 运动到Home点 槽函数 */
	void ControlPanel::slotGoHomeBtn()
	{
		std_msgs::Float64 pub_msg;
		
		for(int i=0;i<6;i++)
		{
	        //pub_msg.data = 0.0;  /* 设置Home点各关节角为0 */
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
	
	/* 显示关节 槽函数 */
	void ControlPanel::slotShowJoint()
	{
		for(int i=0;i<6;i++)
		{
			labelJointOrPosVal[i]->setText(jointValue_Str[i]);
		}
		//alarmShowLabel->setText("jointValue_Str[0]="+jointValue_Str[0]);
	}
	
	/* 显示位姿 槽函数 */
	void ControlPanel::slotShowPos()
	{
		for(int i=0;i<6;i++)
		{
			labelJointOrPosVal[i]->setText(posValue_Str[i]);
		}
		
		alarmShowLabel->setText("posValue_Str[0]="+posValue_Str[0]);
	}
}

/* 声明此类是一个rviz的插件 */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_control_commander::ControlPanel,rviz::Panel )

/* end control_pad.cpp*/

