#include <iostream>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>	

using namespace std;

#define DEBUG



robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;
robot_state::JointModelGroup* joint_model_group;

/***********************四元数转RPY**********************************/
// x y z w A B C
void QtoE(double q1, double q2, double q3, double q0, double& A, double &B, double &C){
	A = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
	B = asin(2*(q0*q2-q1*q3));
	C = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
}

// 获取偏移值
void getOffset(){

}

void doit(const std::vector<double> &in, std::vector<double> &out){

	double A_offset = 0.2, B_offset = 0, C_offset = 0, X_offset = 0,  Y_offset = 0,  Z_offset = 0;
	kinematic_state->setJointGroupPositions(joint_model_group, in);
	
	// 获取当前的末端姿态
	const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link6");
	geometry_msgs::Pose pose;
	tf::poseEigenToMsg(end_effector_state, pose);
#ifdef DEBUG
	std::cout << "orientation.x = " << pose.orientation.x << std::endl;
	std::cout << "orientation.y = " << pose.orientation.y << std::endl;
	std::cout << "orientation.z = " << pose.orientation.z << std::endl;
	std::cout << "orientation.w = " << pose.orientation.w << std::endl;
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
	q.setEulerZYX((tfScalar)C, (tfScalar)B, (tfScalar)A);
	tf::Vector3 axis = q.getAxis();
#ifdef DEBUG
	cout << "axis.x = " << axis.getX() << std::endl;
	cout << "axis.y = " << axis.getY() << std::endl;
	cout << "axis.z = " << axis.getZ() << std::endl;
	cout << "axis.w = " << q.getW() << std::endl;
#endif
	pose.orientation.x = axis.getX();
	pose.orientation.y = axis.getY();
	pose.orientation.z = axis.getZ();
	pose.orientation.w = q.getW();

	// 四元数转关节角
	if(kinematic_state->setFromIK(joint_model_group, pose))
		cout << "四元数转关节角成功" << endl;
	std::vector<double> joint_values;
	kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	for(int i = 0; i < 6; ++i)
	{
	  ROS_INFO("Joint %d: %f", i, joint_values[i]);
	}
}

void init(){
	// 读取URDF配置
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model_loader.getModel();
	kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();
	joint_model_group = kinematic_model->getJointModelGroup("arm");	
}

int main(int argc, char **argv){

	ros::init (argc, argv, "right_arm_kinematics");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	vector<double> in(6), out(6);
	in[0] = 0;in[2] = 0;in[4] = 0;
	in[1] = 0;in[3] = 0;in[5] = 0;

	init();
	doit(in, out);

	ros::shutdown();
  	return 0;

}
