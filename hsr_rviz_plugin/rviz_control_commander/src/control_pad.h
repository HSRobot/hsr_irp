#ifndef CONTROL_PAD_H
#define CONTROL_PAD_H

/* rosͷ�ļ� */
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>                         /* plugin�����ͷ�ļ� */
#include <hsr_driver/Enable.h>                  /* Enable���� */
#include <hsr_driver/GetEnable.h>               /* getEnable���� */

#include <sensor_msgs/JointState.h>             /* ��ȡ�ؽ�״̬ͷ�ļ� */
#include <std_msgs/Float64.h>                   /* ���ùؽ���Ϣͷ�ļ� */
#include <geometry_msgs/PoseStamped.h>          /* ��ȡ�ѿ�����̬ͷ�ļ� */
#include <hsr_msg/Joints.h>

/* qtͷ�ļ�*/
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

/* ��ѧ�� */
#include <cmath>

#define PI 3.1415926


//class QLineEdit;

namespace rviz_control_commander
{
    /* ���е�plugin��������rviz::Panel������ */
class ControlPanel: public rviz::Panel
{
    /* ������Ҫ�õ�Qt���źźͲۣ�����QObject�����࣬������Ҫ����Q_OBJECT�� */
    Q_OBJECT

public:
    /* ���캯���������л��õ�QWidget��ʵ����ʵ��GUI���棬�����ȳ�ʼ��Ϊ0���� */
    ControlPanel( QWidget* parent = 0 );
	
	/**
	*�������ã���ȡ�ؽڽǻص�����
	*��    �����ؽ�״̬�ṹ��
	*�� �� ֵ��None
	*/
	void callback(const sensor_msgs::JointState::ConstPtr& msg);
	
	/**
	*�������ã���ȡ�ѿ�����̬�ص�����
	*��    �����ѿ�����Ԫ����̬״̬�ṹ�塢λ��״̬�ṹ��
	*�� �� ֵ��None
	*/
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
 
    /* ������ */
public Q_SLOTS:
  /* ���û�����topic�����������»س��󣬻ص��ô˲�������һ����Ӧ���Ƶ�topic publisher */
    //void setTopic( const QString& topic );
 
  // �ڲ���.
protected Q_SLOTS:
 
  // �ڲ�����.
protected:
    /* Qt��� */
	QGroupBox *alarmGroupBox;		/*����GroupBox */
	QLabel *alarmShowLabel;
	QPushButton *clearAlarmBtn;
	QPushButton *okAlarmBtn;
	QGridLayout *alarmLayout;	  /* ���������񲼾� */

    QGroupBox *manualOrAutoGroupBox;        /* �ֶ����Զ�GroupBox */
    QRadioButton *manualRadioBtn;           /* �ֶ� */
    QRadioButton *autoRadioBtn;             /* �Զ� */
	QHBoxLayout *manuaOrAutoLayout;	        /* �ֶ����Զ�ģʽ */
	
	QGroupBox *coordSelectGroupBox;        /* ����ϵѡ��GroupBox */
    QRadioButton *jointRadioBtn;           /* �ؽ�����ϵ */
    QRadioButton *cartesianRadioBtn;       /* �ѿ�������ϵ */
	QHBoxLayout *coordSelectLayout;	       /* ����ϵѡ�������� */

	QGroupBox *ratioGroupBox;        /* ����GroupBox */
	QSpinBox *ratioSpinBox;          /*����SpinBox */
	QSlider *ratioSlider;            /*�����϶���*/
	QHBoxLayout *ratioLayout;	     /* �������������� */

    QGroupBox *manualModeGroupBox;   /* �ֶ�ģʽGroupBox */
	QLabel *labelJointOrPosName[6];               /* ��� */
	QLabel *labelJointOrPosVal[6];               /* �ؽڽ���ʾ�� */
	QString jointValue_Str[6];                   /* �ؽ�ֵ */
	QString posValue_Str[6];                     /* λ��ֵ */
	QLineEdit *editJointOrPosVal[6];             /* �ؽڽǱ༭�� */
	
	QPushButton *positiveRotateJntBtn[6];    /* �������˶���ť */
	QPushButton *oppositeRotateJntBtn[6];	 /* �ᷴ���˶���ť */
	QPushButton *move2targetBtn;             /* �˶����� ��ť */
    QPushButton *goHomeBtn;                  /* �˶���home�� ��ť */
	
	QGridLayout *manualModeMainLayout;	  /* �ֶ�ģʽ���񲼾� */

    QGroupBox *autoModeGroupBox;        /* �Զ�ģʽGroupBox */
    QPushButton *loadBtn;               /* �����ļ�Button */
	QPushButton *runProgBtn;            /* �����ļ�Button */
	QPushButton *pauseBtn;              /* �����ļ�Button;*/
	QPushButton *unloadBtn;             /* ж���ļ�Button*/
    QHBoxLayout *autoModeLayout;        /* �Զ�ģʽ���� */

	QGroupBox *isEnableGroupBox;        /* ʹ�ܿ���GroupBox */
	QPushButton *isEnabelBtn;           /* ʹ�ܿ��ذ�ť */
	QAction *action_enable;             /* ʹ�ܿ�ݼ��¼� */
	QHBoxLayout *isEnableLayout;        /* ʹ��״̬���� */

    QGridLayout *controlMainLayout;     /* ������������� */
	
	QTimer *showJointTimer;            /* ��ʾ�ؽ���Ϣ��ʱ�� */
	QTimer *showPosTimer;              /* ��ʾ�ѿ���λ����Ϣ��ʱ�� */

	/* ROS��� */
	ros::NodeHandle n_control;           /* �ڵ��� */
	ros::ServiceClient client_enable;    /* ʹ�ܷ���ͻ��� */
	ros::ServiceClient client_getEnable; /* getEnable����ͻ��� */	
	
	hsr_driver::Enable srv_enable;        /* ����ʹ��server */
    hsr_driver::GetEnable srv_getEnable;  /* �����ȡʹ��״̬server */
	
	ros::Subscriber sub_jointVal;         /* ������״̬���⣬��ȡ�ؽڽ���Ϣ */
	ros::Publisher pub_jointVal[6];       /* ����д��ؽڽ���Ϣ */
	ros::Publisher pubJoint;
	
	ros::Subscriber sub_poseVal;          /* ����λ�˻��⣬��ȡλ����Ϣ */
    ros::Publisher pub_poseVal;           /* ����д��λ����Ϣ */

private Q_SLOTS:
	void slotIsEnableBtnClicked();        /* ʹ�ܿ��زۺ��� */
	void slotShortCutEnableBtnClicked();  /* ʹ�ܿ�ݼ� �ۺ��� */
	
	void slotMov2TargetBtn();             /* �˶���Ŀ��㰴ť �ۺ��� */
    void slotGoHomeBtn();                 /* �ص�Home��λ �ۺ��� */
	
	void slotJointCoordRadioBtn();        /* ѡ��ؽ�����ϵ�ۺ��� */
	void slotCartesianCoordRadioBtn();    /* ѡ��ѿ�������ϵ�ۺ��� */
	
	void slotShowJoint();                 /* ��ʾ�ؽ� �ۺ��� */
	void slotShowPos();                   /* ��ʾλ�� �ۺ��� */
	
};
 
} // end namespace rviz_control_commander

#endif
