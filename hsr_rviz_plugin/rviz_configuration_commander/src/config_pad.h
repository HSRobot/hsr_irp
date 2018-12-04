#ifndef CONFIG_PAD_H
#define CONFIG_PAD_H

/* ros头文件 */
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   /* plugin基类的头文件 */

/* qt头文件 */
#include <QFile>
#include <QTextStream>

#include <QString>

#include <QRadioButton>
#include <QGroupBox>

#include <QLineEdit>
#include <QLabel>
#include <QPushButton>

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

#include <QProcess>


//class QLineEdit;

namespace rviz_configuration_commander
{
	/* 连接配置参数结构体 */
    struct ConnectConfigParm{
    QString sysTypeName;     /* 系统类型名 */
	QString rbtTypeName;     /* 机器人类型名 */
    QString ipAddr;          /* IP地址 */  
	QString ipPort;          /* IP端口 */  
    };
	
    /* 所有的plugin都必须是rviz::Panel的子类 */
class ConfigPanel: public rviz::Panel
{
    /* 后面需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏 */
    Q_OBJECT

public:
    /* 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可 */
    ConfigPanel( QWidget* parent = 0 );
 
    /**重载rviz::Panel基类中的函数，用于保存、加载配置文件中的数据，在我们这个plugin
    * 中，数据就是topic的名称
    */
    //virtual void load( const rviz::Config& config );
    //virtual void save( rviz::Config config ) const;
 
    /* 公共槽. */
public Q_SLOTS:
  /* 当用户输入topic的命名并按下回车后，回调用此槽来创建一个相应名称的topic publisher */
    //void setTopic( const QString& topic );
 
  // 内部槽.
protected Q_SLOTS:
/*
    void sendVel();                 // 发布当前的速度值
    void update_Linear_Velocity();  // 根据用户的输入更新线速度值
    void update_Angular_Velocity(); // 根据用户的输入更新角速度值
    void updateTopic();             // 根据用户的输入更新topic name
*/
    void slotType2RadioBtnClicked();  /* 选择II型系统槽函数 */
	void slotType3RadioBtnClicked();  /* 选择III型系统 */
	void slotTypeLocalRadioBtnClicked();  /* 选择Loacal型系统 */
	
	void slotBr606RadioBtnClicked();    /* 选择BR606机器人 */
	void slotBr609RadioBtnClicked();    /* 选择BR609机器人 */
	void slotHsr605RadioBtn();    /* 选择HSR605机器人 */
	void slotHsr608RadioBtn();    /* 选择HSR606机器人 */
	void slotHsr612RadioBtn();    /* 选择HSR612机器人 */
	void slotScaraRadioBtn();    /* 选择Scara机器人 */
	
    void slotConnectBtnClicked();   /* connect槽函数 */
  // 内部变量.
protected:
    QGroupBox *sysTypeGroupBox;        /* 系统类型GroupBox */
    QRadioButton *type2RadioBtn;       /* II型系统 */
    QRadioButton *type3RadioBtn;       /* III型系统 */
    QRadioButton *typeLocalRadioBtn;   /* 本地系统 */
    QHBoxLayout *sysTypeLayout;        /* 系统类型布局 */

    QGroupBox *rbtTypeGroupBox;        /* 机器人类型GroupBox */
    QRadioButton *br606RadioBtn;       /* br606 */
    QRadioButton *br609RadioBtn;       /* br609 */
    QRadioButton *hsr605RadioBtn;      /* hsr605 */
    QRadioButton *hsr608RadioBtn;      /* hsr608 */
    QRadioButton *hsr612RadioBtn;      /* hsr612 */
    QRadioButton *scaraRadioBtn;       /* scara */
    QGridLayout *rbtTypeLayout;        /* 机器人类型布局 */

    QGroupBox *netConnectGroupBox;        /* 网络连接GroupBox */
    QLabel *netAddrLabel,*netPortLabel;   /* 网络地址和端口Label; */
    QLineEdit *m_netAddrLineEdit,*m_netPortLineEdit;  /* 网络地址和端口LineEdit; */
    QPushButton *connectBtn,*cancelBtn;  /* 网络连接和取消Button; */
    QGridLayout *connectGridLayout;        /* 网络连接布局 */
    
    QLabel *configStateLabel;       /* 配置状态 */

    QVBoxLayout *configMainLayout;  /* 配置面板主布局 */
	
	QProcess *proc_connect;   /* 连接进程 */
	
    struct ConnectConfigParm st_ConnetCfgParm;   /* 连接配置参数(结构体) */
	
	
	/* ROS相关 */
	ros::NodeHandle n_config;           /* 节点句柄 */

};
 
} // end namespace rviz_configuration_commander

#endif
