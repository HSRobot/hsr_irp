#include "config_pad.h"

namespace rviz_configuration_commander
{
    /* 构造函数，初始化变量 */
    ConfigPanel::ConfigPanel(QWidget* parent) : rviz::Panel(parent)
    {
        /* 系统类型设置栏 */
        sysTypeGroupBox = new QGroupBox("System Type");
        type2RadioBtn = new QRadioButton("II");
        type3RadioBtn = new QRadioButton("III");
	    type3RadioBtn->setChecked(true);  /* 设置默认选中type III */

        typeLocalRadioBtn = new QRadioButton("Local");

        sysTypeLayout=new QHBoxLayout;
	    sysTypeLayout->addWidget(type2RadioBtn);
	    sysTypeLayout->addWidget(type3RadioBtn);
        sysTypeLayout->addWidget(typeLocalRadioBtn);

	    sysTypeGroupBox->setLayout(sysTypeLayout);

        /* 机器人类型设置栏 */
        rbtTypeGroupBox = new QGroupBox("Robot Type");
        br606RadioBtn = new QRadioButton("BR606");
        br606RadioBtn->setChecked(true);  /* 设置默认选中type III */

        br609RadioBtn = new QRadioButton("BR609");
        hsr605RadioBtn = new QRadioButton("HSR605");
        hsr608RadioBtn = new QRadioButton("HSR608");
        hsr612RadioBtn = new QRadioButton("HSR612");
        scaraRadioBtn = new QRadioButton("Scara");

        rbtTypeLayout=new QGridLayout;
	    rbtTypeLayout->addWidget(br606RadioBtn,0,0);
	    rbtTypeLayout->addWidget(br609RadioBtn,0,1);
        rbtTypeLayout->addWidget(hsr605RadioBtn,0,2);
	    rbtTypeLayout->addWidget(hsr608RadioBtn,1,0);
	    rbtTypeLayout->addWidget(hsr612RadioBtn,1,1);
        rbtTypeLayout->addWidget(scaraRadioBtn,1,2);

	    rbtTypeGroupBox->setLayout(rbtTypeLayout);

        /* 连接设置栏 */
        netConnectGroupBox = new QGroupBox("Net Connection");
        netAddrLabel = new QLabel("Addr:");
        netAddrLabel->setAlignment(Qt::AlignCenter);
	    netPortLabel = new QLabel("Port:");
        netPortLabel->setAlignment(Qt::AlignCenter);

	    m_netAddrLineEdit = new QLineEdit;
        m_netAddrLineEdit->setText(tr("10.10.56.214")); /* 设置Addr默认值 */
	    m_netPortLineEdit = new QLineEdit;
        m_netPortLineEdit->setText(tr("23234"));        /* 设置Port默认值 */

	    connectBtn = new QPushButton("Connect");
	    cancelBtn = new QPushButton("Disconnect");

	    connectGridLayout= new QGridLayout;
	    connectGridLayout->addWidget(netAddrLabel,0,0);
	    connectGridLayout->addWidget(m_netAddrLineEdit,0,1,1,4);
	    connectGridLayout->addWidget(netPortLabel,1,0);
	    connectGridLayout->addWidget(m_netPortLineEdit,1,1,1,4);
	    connectGridLayout->addWidget(connectBtn,2,1);
	    connectGridLayout->addWidget(cancelBtn,2,3);

        netConnectGroupBox->setLayout(connectGridLayout);

        /* 配置状态 */
        configStateLabel = new QLabel("connection success!");
        /* 整体布局 */
        configMainLayout = new QVBoxLayout;
        configMainLayout->addWidget(sysTypeGroupBox);
        configMainLayout->addWidget(rbtTypeGroupBox);
        configMainLayout->addWidget(netConnectGroupBox);
        configMainLayout->addWidget(configStateLabel);
        setLayout(configMainLayout);
		
		/* 初始化连接参数 结构体 */
		st_ConnetCfgParm = (struct ConnectConfigParm){"III","BR606",m_netAddrLineEdit->text(),m_netPortLineEdit->text()};	
		proc_connect = new QProcess;       /* 网络连接启动进程 */
		
		/* ROS相关 */
		
		connect(type2RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotType2RadioBtnClicked()));  /* 选中II型系统 槽函数 */
		connect(type3RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotType3RadioBtnClicked()));  /* 选中III型系统 槽函数 */
		connect(typeLocalRadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotTypeLocalRadioBtnClicked()));  /* 选中Local型系统 槽函数 */
		
		connect(br606RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotBr606RadioBtnClicked()));   /* 选中br606型机器人 槽函数 */
		connect(br609RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotBr609RadioBtnClicked()));   /* 选中br609型机器人 槽函数 */
		connect(hsr605RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotHsr605RadioBtn()));  /* 选中hsr605型机器人 槽函数 */
		connect(hsr608RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotHsr608RadioBtn()));  /* 选中hsr608型机器人 槽函数 */
		connect(hsr612RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotHsr612RadioBtn()));  /* 选中hsr612型机器人 槽函数 */
		connect(scaraRadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotScaraRadioBtn()));   /* 选中sacara型机器人 槽函数 */
		
		connect(connectBtn,SIGNAL(clicked(bool)),this,SLOT(slotConnectBtnClicked())); /* 网络连接 槽函数 */
    }
	/******************系统类型槽函数***********************/
	void ConfigPanel::slotType2RadioBtnClicked()
	{	
		st_ConnetCfgParm.sysTypeName = "II";
	}
	void ConfigPanel::slotType3RadioBtnClicked()
	{	
		st_ConnetCfgParm.sysTypeName = "III";
	}
	void ConfigPanel::slotTypeLocalRadioBtnClicked()
	{	
		st_ConnetCfgParm.sysTypeName = "Local";
	}	
    /***************机器人类型槽函数************************/
	void ConfigPanel::slotBr606RadioBtnClicked()
	{
		//设置机器人类型名
		st_ConnetCfgParm.rbtTypeName = "BR606";
		
		//读取模型文件
		QFile file("/home/eima/Work/catkin_ws/src/HS_Robot/trunk/hsr_br606/urdf/hsr_br606.urdf");
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
             return;
        QTextStream in(&file);
		
		QString modelXMLtextSteam = in.readAll();      /* 读取文件，存进QString字符串 */
		
		/* 设置参数，载入对应模型 */
		n_config.setParam("/robot_description", modelXMLtextSteam.toStdString());			
	}
	
	void ConfigPanel::slotBr609RadioBtnClicked()
	{
		st_ConnetCfgParm.rbtTypeName = "BR609";
		
		//读取模型文件
		QFile file("/home/eima/Work/catkin_ws/src/HS_Robot/trunk/hsr_br606/urdf/hsr_br609.urdf");
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
             return;
        QTextStream in(&file);
		
		QString modelXMLtextSteam = in.readAll();      /* 读取文件，存进QString字符串 */
		
		/* 设置参数，载入对应模型 */
		n_config.setParam("/robot_description", modelXMLtextSteam.toStdString());
	}
	
	void ConfigPanel::slotHsr605RadioBtn()
	{
		st_ConnetCfgParm.rbtTypeName = "HSR605";
	}
	
	void ConfigPanel::slotHsr608RadioBtn()
	{
		st_ConnetCfgParm.rbtTypeName = "HSR608";
	}
	
	void ConfigPanel::slotHsr612RadioBtn()
	{
		st_ConnetCfgParm.rbtTypeName = "HSR612";
	}
	
	void ConfigPanel::slotScaraRadioBtn()
	{
		st_ConnetCfgParm.rbtTypeName = "SCARA";
	}
	
	/* 连接 槽函数 */
	void ConfigPanel::slotConnectBtnClicked()
	{
        QString cmd_connect = "rosrun hsr_driver hsr_driver" + (QString)(" ") + st_ConnetCfgParm.sysTypeName \
		                        + (QString)(" ") + st_ConnetCfgParm.rbtTypeName + (QString)(" ")  \
                                        + st_ConnetCfgParm.ipAddr + (QString)(" ") + st_ConnetCfgParm.ipPort;
        proc_connect->start(cmd_connect);   /* 开启进程，执行命令行命令 */
		//proc_connect->start("rosrun hsr_driver hsr_driver 1 1 1 1");						

        configStateLabel->setText(cmd_connect);  /* 测试 输出命令行 */
	}
		
}

/* 声明此类是一个rviz的插件 */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_configuration_commander::ConfigPanel,rviz::Panel )

/* end config_pad.cpp*/
