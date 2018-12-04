#ifndef CONFIG_PAD_H
#define CONFIG_PAD_H

/* rosͷ�ļ� */
#include <ros/ros.h>
#include <ros/console.h>
#include <rviz/panel.h>   /* plugin�����ͷ�ļ� */

/* qtͷ�ļ� */
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
	/* �������ò����ṹ�� */
    struct ConnectConfigParm{
    QString sysTypeName;     /* ϵͳ������ */
	QString rbtTypeName;     /* ������������ */
    QString ipAddr;          /* IP��ַ */  
	QString ipPort;          /* IP�˿� */  
    };
	
    /* ���е�plugin��������rviz::Panel������ */
class ConfigPanel: public rviz::Panel
{
    /* ������Ҫ�õ�Qt���źźͲۣ�����QObject�����࣬������Ҫ����Q_OBJECT�� */
    Q_OBJECT

public:
    /* ���캯���������л��õ�QWidget��ʵ����ʵ��GUI���棬�����ȳ�ʼ��Ϊ0���� */
    ConfigPanel( QWidget* parent = 0 );
 
    /**����rviz::Panel�����еĺ��������ڱ��桢���������ļ��е����ݣ����������plugin
    * �У����ݾ���topic������
    */
    //virtual void load( const rviz::Config& config );
    //virtual void save( rviz::Config config ) const;
 
    /* ������. */
public Q_SLOTS:
  /* ���û�����topic�����������»س��󣬻ص��ô˲�������һ����Ӧ���Ƶ�topic publisher */
    //void setTopic( const QString& topic );
 
  // �ڲ���.
protected Q_SLOTS:
/*
    void sendVel();                 // ������ǰ���ٶ�ֵ
    void update_Linear_Velocity();  // �����û�������������ٶ�ֵ
    void update_Angular_Velocity(); // �����û���������½��ٶ�ֵ
    void updateTopic();             // �����û����������topic name
*/
    void slotType2RadioBtnClicked();  /* ѡ��II��ϵͳ�ۺ��� */
	void slotType3RadioBtnClicked();  /* ѡ��III��ϵͳ */
	void slotTypeLocalRadioBtnClicked();  /* ѡ��Loacal��ϵͳ */
	
	void slotBr606RadioBtnClicked();    /* ѡ��BR606������ */
	void slotBr609RadioBtnClicked();    /* ѡ��BR609������ */
	void slotHsr605RadioBtn();    /* ѡ��HSR605������ */
	void slotHsr608RadioBtn();    /* ѡ��HSR606������ */
	void slotHsr612RadioBtn();    /* ѡ��HSR612������ */
	void slotScaraRadioBtn();    /* ѡ��Scara������ */
	
    void slotConnectBtnClicked();   /* connect�ۺ��� */
  // �ڲ�����.
protected:
    QGroupBox *sysTypeGroupBox;        /* ϵͳ����GroupBox */
    QRadioButton *type2RadioBtn;       /* II��ϵͳ */
    QRadioButton *type3RadioBtn;       /* III��ϵͳ */
    QRadioButton *typeLocalRadioBtn;   /* ����ϵͳ */
    QHBoxLayout *sysTypeLayout;        /* ϵͳ���Ͳ��� */

    QGroupBox *rbtTypeGroupBox;        /* ����������GroupBox */
    QRadioButton *br606RadioBtn;       /* br606 */
    QRadioButton *br609RadioBtn;       /* br609 */
    QRadioButton *hsr605RadioBtn;      /* hsr605 */
    QRadioButton *hsr608RadioBtn;      /* hsr608 */
    QRadioButton *hsr612RadioBtn;      /* hsr612 */
    QRadioButton *scaraRadioBtn;       /* scara */
    QGridLayout *rbtTypeLayout;        /* ���������Ͳ��� */

    QGroupBox *netConnectGroupBox;        /* ��������GroupBox */
    QLabel *netAddrLabel,*netPortLabel;   /* �����ַ�Ͷ˿�Label; */
    QLineEdit *m_netAddrLineEdit,*m_netPortLineEdit;  /* �����ַ�Ͷ˿�LineEdit; */
    QPushButton *connectBtn,*cancelBtn;  /* �������Ӻ�ȡ��Button; */
    QGridLayout *connectGridLayout;        /* �������Ӳ��� */
    
    QLabel *configStateLabel;       /* ����״̬ */

    QVBoxLayout *configMainLayout;  /* ������������� */
	
	QProcess *proc_connect;   /* ���ӽ��� */
	
    struct ConnectConfigParm st_ConnetCfgParm;   /* �������ò���(�ṹ��) */
	
	
	/* ROS��� */
	ros::NodeHandle n_config;           /* �ڵ��� */

};
 
} // end namespace rviz_configuration_commander

#endif
