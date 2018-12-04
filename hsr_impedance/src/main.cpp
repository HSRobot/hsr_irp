#include "Impedance.h"
#include <iostream>
#include "geometry_msgs/Wrench.h"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>	

#define DEBUG

Impedance CartesianImpedance;		// 算法实例
std::vector<double> force;		// 保存传感器数据
std::vector<double> lastJoint(6);	// 保存上一次的关节角度
std::vector<double> startPos(6);	// 保存起始关节角数据

std::string end_link;
std::string group;

bool startPosOK;
bool isSim = true;

robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* joint_model_group;

#define F_SCALE 0.2
#define T_SCALE 5
#define Z_ERR	14.5 * F_SCALE

void foruceCallback(const geometry_msgs::Wrench::ConstPtr& msg){

	force[0] = msg->force.x * 0.2;
	force[1] = -msg->force.y * 0.2;
	force[2] = msg->force.z * 0.2;
	force[3] = -msg->torque.x * 4;
	force[4] = msg->torque.y * 4;
	force[5] = msg->torque.z * 4;

}

void startPosCallback(const sensor_msgs::JointState::ConstPtr& msg){

	for(int i = 0; i < 6; i++)
		startPos[i] = msg->position[i];

	startPosOK = true;

}


/***********************算法初始化**********************************/
void initImped(){

	double Mass[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };		
	double Damping[6] = { 5.0, 5.0, 5.0, 5.0, 5.0, 5.0 };	
	double Stiffness[6] = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };
	force = std::vector<double>(6);
	force[2] = -15;
	CartesianImpedance = Impedance();
	CartesianImpedance.setStiffness(Stiffness);
	CartesianImpedance.setDamping(Damping);
}

/***********************四元数转RPY**********************************/
// x y z w A B C
void QtoE(double q1, double q2, double q3, double q0, double& A, double &B, double &C){
	A = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
	B = asin(2*(q0*q2-q1*q3));
	C = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}


/***********************初始化运动学**********************************/
void initKinematic(){
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model_loader.getModel();
	kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	joint_model_group = kinematic_model->getJointModelGroup(group);	
}

/***********************通过算法计算出实际关节角**********************************/
void doit(const std::vector<double> &in, std::vector<double> &out){

	double A_offset = 0, B_offset = 0, C_offset = 0, X_offset = 0,  Y_offset = 0,  Z_offset = 0;
	std::vector<double> Xa(6);
	int ERROR_ImpAcc , ERROR_ImpVel;
	//force[0] = 0; force[1] = 1; force[10] = 0; force[3] = 0; force[4] = 0; force[5] = 0;
	
	// 计算偏移值
	force[2] += Z_ERR;
	CartesianImpedance.reImpedance(force, ERROR_ImpAcc, ERROR_ImpVel, Xa);

	X_offset = Xa[0];Y_offset = Xa[1];//Z_offset = Xa[2];
	//A_offset = Xa[3];B_offset = Xa[4];C_offset = Xa[5];


	// 获取当前的末端姿态
	kinematic_state->setJointGroupPositions(joint_model_group, in);
	const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(end_link);
	geometry_msgs::Pose pose;
	tf::poseEigenToMsg(end_effector_state, pose);
#ifdef DEBUG
	std::cout << "orientation.x = " << pose.orientation.x << std::endl;
	std::cout << "orientation.y = " << pose.orientation.y << std::endl;
	std::cout << "orientation.z = " << pose.orientation.z << std::endl;
	std::cout << "orientation.w = " << pose.orientation.w << std::endl;
	std::cout << "position.x = " << pose.position.x << std::endl;
	std::cout << "position.y = " << pose.position.y << std::endl;
	std::cout << "position.z = " << pose.position.z << std::endl;

	std::cout << "X_offset = " << X_offset << std::endl;
	std::cout << "Y_offset = " << Y_offset << std::endl;
	std::cout << "Z_offset = " << Z_offset << std::endl;
	std::cout << "A_offset = " << A_offset << std::endl;
	std::cout << "B_offset " << B_offset << std::endl;
	std::cout << "C_offset = " << C_offset << std::endl;
#endif
	// 四元数转RPY
	double A, B, C;
	QtoE(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, A, B, C);
#ifdef DEBUG
	std::cout << "A= " << A << " B= " << B << " C= " << C << std::endl;
#endif

	A += A_offset; B += B_offset; B += B_offset;
	pose.position.x += X_offset; pose.position.y += Y_offset; pose.position.z += Z_offset;

	// 偏移后RPY转四元数
	tf::Quaternion q;
	q.setEulerZYX(C, B, A);
	tf::Vector3 axis = q.getAxis();
#ifdef DEBUG
	std::cout << "axis.x = " << axis.getX() << std::endl;
	std::cout << "axis.y = " << axis.getY() << std::endl;
	std::cout << "axis.z = " << axis.getZ() << std::endl;
	std::cout << "axis.w = " << q.getW() << std::endl;
#endif
	pose.orientation.x = axis.getX();
	pose.orientation.y = axis.getY();
	pose.orientation.z = axis.getZ();
	pose.orientation.w = q.getW();

	// 四元数转关节角
	if(!kinematic_state->setFromIK(joint_model_group, pose, end_link, 10, 0.1)){
		std::cout << "运动学逆解失败" << std::endl;
		out = lastJoint;
		return;
	}

	// 返回计算后的关节角	
	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	
	// 当不处于虚拟环境时 发送的是关节角偏移值
	if(!isSim)
		for(int i = 0; i < 6; i ++)
			joint_values[i] -= in[i];

	out = joint_values;
	lastJoint = joint_values;

}

int main(int argc, char **argv){

	ros::init(argc, argv, "hsr_impedance");
	ros::NodeHandle n;

	Impedance CartesianImpedance;
	ros::AsyncSpinner spinner(2);
  	spinner.start();

	ros::Publisher joint_state_pub;

	startPosOK = false;
	isSim = false;

	if(argc == 2)
		if(strcmp(argv[1], "true") == 0){
			// 当处于虚拟环境时  直接使用指定初始点即可
			isSim = true;
			startPosOK = true;
			ROS_INFO("虚拟环境");	
		}else if(strcmp(argv[1], "false") != 0)
			ROS_ERROR("未知参数:Sim will set be false!!! \n");

	// 获取参数
	n.param<std::string>("impedance_end_link", end_link, "ee_link");
	n.param<std::string>("impedance_group", group, "manipulator");

	// 订阅传感器数据
	ros::Subscriber pose_sub = n.subscribe("daq_data", 1000, foruceCallback);

	// 订阅机械臂起始关节角
	ros::Subscriber start_pos_sub = n.subscribe("start_pos", 1000, startPosCallback);

	// 发布关节角数据
	if(isSim)
		joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	else
		joint_state_pub = n.advertise<sensor_msgs::JointState>("impedance_err", 1000);

	// 初始化算法
	initImped();
	
	// 初始化运动学
	initKinematic();
	
	// 测试数据
	std::vector<double> out(6);
        startPos[0] = 2.1411815164231166e-05; startPos[2] = 1.5707963331877837;     startPos[4] = -1.5707989151749278;
        startPos[1] = -1.5707750110586352;    startPos[3] = -1.570785397386886;     startPos[5] = -2.216798209886406e-06;

	sensor_msgs::JointState joint_state;
	ros::Rate loop_rate(25);

	// 获取关节名
	const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
	//std::cout << "joint size" << joint_names.size() << std::endl;
	for(int i = 0; i < joint_names.size(); i++)
		std::cout << "joint" << i << " = " << joint_names[i] << std::endl;	
	while(ros::ok()){
		if(startPosOK)
			doit(startPos, out);	

		joint_state.header.stamp = ros::Time::now();
		joint_state.name.resize(6);
		joint_state.position = out;
		
		for(int i = 0; i < 6; i++)
			joint_state.name[i] =  joint_names[i];
		
		if(startPosOK)
			joint_state_pub.publish(joint_state);

		loop_rate.sleep();
	}

	// 进入消息循环
	//ros::spin();
}
