#include "config_pad.h"

namespace rviz_configuration_commander
{
    /* ���캯������ʼ������ */
    ConfigPanel::ConfigPanel(QWidget* parent) : rviz::Panel(parent)
    {
        /* ϵͳ���������� */
        sysTypeGroupBox = new QGroupBox("System Type");
        type2RadioBtn = new QRadioButton("II");
        type3RadioBtn = new QRadioButton("III");
	    type3RadioBtn->setChecked(true);  /* ����Ĭ��ѡ��type III */

        typeLocalRadioBtn = new QRadioButton("Local");

        sysTypeLayout=new QHBoxLayout;
	    sysTypeLayout->addWidget(type2RadioBtn);
	    sysTypeLayout->addWidget(type3RadioBtn);
        sysTypeLayout->addWidget(typeLocalRadioBtn);

	    sysTypeGroupBox->setLayout(sysTypeLayout);

        /* ���������������� */
        rbtTypeGroupBox = new QGroupBox("Robot Type");
        br606RadioBtn = new QRadioButton("BR606");
        br606RadioBtn->setChecked(true);  /* ����Ĭ��ѡ��type III */

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

        /* ���������� */
        netConnectGroupBox = new QGroupBox("Net Connection");
        netAddrLabel = new QLabel("Addr:");
        netAddrLabel->setAlignment(Qt::AlignCenter);
	    netPortLabel = new QLabel("Port:");
        netPortLabel->setAlignment(Qt::AlignCenter);

	    m_netAddrLineEdit = new QLineEdit;
        m_netAddrLineEdit->setText(tr("10.10.56.214")); /* ����AddrĬ��ֵ */
	    m_netPortLineEdit = new QLineEdit;
        m_netPortLineEdit->setText(tr("23234"));        /* ����PortĬ��ֵ */

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

        /* ����״̬ */
        configStateLabel = new QLabel("connection success!");
        /* ���岼�� */
        configMainLayout = new QVBoxLayout;
        configMainLayout->addWidget(sysTypeGroupBox);
        configMainLayout->addWidget(rbtTypeGroupBox);
        configMainLayout->addWidget(netConnectGroupBox);
        configMainLayout->addWidget(configStateLabel);
        setLayout(configMainLayout);
		
		/* ��ʼ�����Ӳ��� �ṹ�� */
		st_ConnetCfgParm = (struct ConnectConfigParm){"III","BR606",m_netAddrLineEdit->text(),m_netPortLineEdit->text()};	
		proc_connect = new QProcess;       /* ���������������� */
		
		/* ROS��� */
		
		connect(type2RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotType2RadioBtnClicked()));  /* ѡ��II��ϵͳ �ۺ��� */
		connect(type3RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotType3RadioBtnClicked()));  /* ѡ��III��ϵͳ �ۺ��� */
		connect(typeLocalRadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotTypeLocalRadioBtnClicked()));  /* ѡ��Local��ϵͳ �ۺ��� */
		
		connect(br606RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotBr606RadioBtnClicked()));   /* ѡ��br606�ͻ����� �ۺ��� */
		connect(br609RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotBr609RadioBtnClicked()));   /* ѡ��br609�ͻ����� �ۺ��� */
		connect(hsr605RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotHsr605RadioBtn()));  /* ѡ��hsr605�ͻ����� �ۺ��� */
		connect(hsr608RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotHsr608RadioBtn()));  /* ѡ��hsr608�ͻ����� �ۺ��� */
		connect(hsr612RadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotHsr612RadioBtn()));  /* ѡ��hsr612�ͻ����� �ۺ��� */
		connect(scaraRadioBtn,SIGNAL(clicked(bool)),this,SLOT(slotScaraRadioBtn()));   /* ѡ��sacara�ͻ����� �ۺ��� */
		
		connect(connectBtn,SIGNAL(clicked(bool)),this,SLOT(slotConnectBtnClicked())); /* �������� �ۺ��� */
    }
	/******************ϵͳ���Ͳۺ���***********************/
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
    /***************���������Ͳۺ���************************/
	void ConfigPanel::slotBr606RadioBtnClicked()
	{
		//���û�����������
		st_ConnetCfgParm.rbtTypeName = "BR606";
		
		//��ȡģ���ļ�
		QFile file("/home/eima/Work/catkin_ws/src/HS_Robot/trunk/hsr_br606/urdf/hsr_br606.urdf");
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
             return;
        QTextStream in(&file);
		
		QString modelXMLtextSteam = in.readAll();      /* ��ȡ�ļ������QString�ַ��� */
		
		/* ���ò����������Ӧģ�� */
		n_config.setParam("/robot_description", modelXMLtextSteam.toStdString());			
	}
	
	void ConfigPanel::slotBr609RadioBtnClicked()
	{
		st_ConnetCfgParm.rbtTypeName = "BR609";
		
		//��ȡģ���ļ�
		QFile file("/home/eima/Work/catkin_ws/src/HS_Robot/trunk/hsr_br606/urdf/hsr_br609.urdf");
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
             return;
        QTextStream in(&file);
		
		QString modelXMLtextSteam = in.readAll();      /* ��ȡ�ļ������QString�ַ��� */
		
		/* ���ò����������Ӧģ�� */
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
	
	/* ���� �ۺ��� */
	void ConfigPanel::slotConnectBtnClicked()
	{
        QString cmd_connect = "rosrun hsr_driver hsr_driver" + (QString)(" ") + st_ConnetCfgParm.sysTypeName \
		                        + (QString)(" ") + st_ConnetCfgParm.rbtTypeName + (QString)(" ")  \
                                        + st_ConnetCfgParm.ipAddr + (QString)(" ") + st_ConnetCfgParm.ipPort;
        proc_connect->start(cmd_connect);   /* �������̣�ִ������������ */
		//proc_connect->start("rosrun hsr_driver hsr_driver 1 1 1 1");						

        configStateLabel->setText(cmd_connect);  /* ���� ��������� */
	}
		
}

/* ����������һ��rviz�Ĳ�� */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_configuration_commander::ConfigPanel,rviz::Panel )

/* end config_pad.cpp*/
