#ifndef CONTROL_PAD_H
#define CONTROL_PAD_H

/* ros头文件 */
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>                         /* plugin基类的头文件 */
#include <hsr_driver/Enable.h>                  /* Enable服务 */
#include <hsr_driver/GetEnable.h>               /* getEnable服务 */

#include <sensor_msgs/JointState.h>             /* 读取关节状态头文件 */
#include <std_msgs/Float64.h>                   /* 设置关节信息头文件 */
#include <geometry_msgs/PoseStamped.h>          /* 读取笛卡尔姿态头文件 */
#include <hsr_msg/Joints.h>

/* qt头文件*/
#include <QTimer>

#include <QString>

#include <QRadioButton>
#include <QGroupBox>

#include<QSpinBox>
#include<QSlider>

#include <QLineEdit>
#include <QLabel>
#include <QPushButton>

#include <QShortcut>
#include <QAction>

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

/* 数学库 */
#include <cmath>

#define PI 3.1415926


//class QLineEdit;

namespace rviz_control_commander
{
    /* 所有的plugin都必须是rviz::Panel的子类 */
class ControlPanel: public rviz::Panel
{
    /* 后面需要用到Qt的信号和槽，都是QObject的子类，所以需要声明Q_OBJECT宏 */
    Q_OBJECT

public:
    /* 构造函数，在类中会用到QWidget的实例来实现GUI界面，这里先初始化为0即可 */
    ControlPanel( QWidget* parent = 0 );
	
	/**
	*函数作用：读取关节角回调函数
	*参    数：关节状态结构体
	*返 回 值：None
	*/
	void callback(const sensor_msgs::JointState::ConstPtr& msg);
	
	/**
	*函数作用：读取笛卡尔姿态回调函数
	*参    数：笛卡尔四元素姿态状态结构体、位置状态结构体
	*返 回 值：None
	*/
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
 
    /* 公共槽 */
public Q_SLOTS:
  /* 当用户输入topic的命名并按下回车后，回调用此槽来创建一个相应名称的topic publisher */
    //void setTopic( const QString& topic );
 
  // 内部槽.
protected Q_SLOTS:
 
  // 内部变量.
protected:
    /* Qt相关 */
	QGroupBox *alarmGroupBox;		/*报警GroupBox */
	QLabel *alarmShowLabel;
	QPushButton *clearAlarmBtn;
	QPushButton *okAlarmBtn;
	QGridLayout *alarmLayout;	  /* 报警栏网格布局 */

    QGroupBox *manualOrAutoGroupBox;        /* 手动或自动GroupBox */
    QRadioButton *manualRadioBtn;           /* 手动 */
    QRadioButton *autoRadioBtn;             /* 自动 */
	QHBoxLayout *manuaOrAutoLayout;	        /* 手动或自动模式 */
	
	QGroupBox *coordSelectGroupBox;        /* 坐标系选择GroupBox */
    QRadioButton *jointRadioBtn;           /* 关节坐标系 */
    QRadioButton *cartesianRadioBtn;       /* 笛卡尔坐标系 */
	QHBoxLayout *coordSelectLayout;	       /* 坐标系选择栏布局 */

	QGroupBox *ratioGroupBox;        /* 倍率GroupBox */
	QSpinBox *ratioSpinBox;          /*倍率SpinBox */
	QSlider *ratioSlider;            /*倍率拖动条*/
	QHBoxLayout *ratioLayout;	     /* 倍率设置栏布局 */

    QGroupBox *manualModeGroupBox;   /* 手动模式GroupBox */
	QLabel *labelJointOrPosName[6];               /* 轴号 */
	QLabel *labelJointOrPosVal[6];               /* 关节角显示区 */
	QString jointValue_Str[6];                   /* 关节值 */
	QString posValue_Str[6];                     /* 位姿值 */
	QLineEdit *editJointOrPosVal[6];             /* 关节角编辑框 */
	
	QPushButton *positiveRotateJntBtn[6];    /* 轴正向运动按钮 */
	QPushButton *oppositeRotateJntBtn[6];	 /* 轴反向运动按钮 */
	QPushButton *move2targetBtn;             /* 运动到点 按钮 */
    QPushButton *goHomeBtn;                  /* 运动到home点 按钮 */
	
	QGridLayout *manualModeMainLayout;	  /* 手动模式网格布局 */

    QGroupBox *autoModeGroupBox;        /* 自动模式GroupBox */
    QPushButton *loadBtn;               /* 加载文件Button */
	QPushButton *runProgBtn;            /* 运行文件Button */
	QPushButton *pauseBtn;              /* 运行文件Button;*/
	QPushButton *unloadBtn;             /* 卸载文件Button*/
    QHBoxLayout *autoModeLayout;        /* 自动模式布局 */

	QGroupBox *isEnableGroupBox;        /* 使能开关GroupBox */
	QPushButton *isEnabelBtn;           /* 使能开关按钮 */
	QAction *action_enable;             /* 使能快捷键事件 */
	QHBoxLayout *isEnableLayout;        /* 使能状态布局 */

    QGridLayout *controlMainLayout;     /* 控制面板主布局 */
	
	QTimer *showJointTimer;            /* 显示关节信息定时器 */
	QTimer *showPosTimer;              /* 显示笛卡尔位姿信息定时器 */

	/* ROS相关 */
	ros::NodeHandle n_control;           /* 节点句柄 */
	ros::ServiceClient client_enable;    /* 使能服务客户端 */
	ros::ServiceClient client_getEnable; /* getEnable服务客户端 */	
	
	hsr_driver::Enable srv_enable;        /* 定义使能server */
    hsr_driver::GetEnable srv_getEnable;  /* 定义获取使能状态server */
	
	ros::Subscriber sub_jointVal;         /* 订阅轴状态话题，读取关节角信息 */
	ros::Publisher pub_jointVal[6];       /* 发布写入关节角信息 */
	ros::Publisher pubJoint;
	
	ros::Subscriber sub_poseVal;          /* 订阅位姿话题，读取位姿信息 */
    ros::Publisher pub_poseVal;           /* 发布写入位姿信息 */

private Q_SLOTS:
	void slotIsEnableBtnClicked();        /* 使能开关槽函数 */
	void slotShortCutEnableBtnClicked();  /* 使能快捷键 槽函数 */
	
	void slotMov2TargetBtn();             /* 运动到目标点按钮 槽函数 */
    void slotGoHomeBtn();                 /* 回到Home点位 槽函数 */
	
	void slotJointCoordRadioBtn();        /* 选择关节坐标系槽函数 */
	void slotCartesianCoordRadioBtn();    /* 选择笛卡尔坐标系槽函数 */
	
	void slotShowJoint();                 /* 显示关节 槽函数 */
	void slotShowPos();                   /* 显示位姿 槽函数 */
	
};
 
} // end namespace rviz_control_commander

#endif
