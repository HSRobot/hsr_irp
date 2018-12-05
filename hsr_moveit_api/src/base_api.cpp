#include <moveit/move_group_interface/move_group.h>
#include <hsr_msgs/Joints.h>

#define DEBUG
move_group_interface::MoveGroup *group;

/* --------------直角空间运动接口回调函数-----------------*/
void poseCallback(const geometry_msgs::Pose::ConstPtr& msg){
#ifdef DEBUG
	std::cout << "x: " << msg->orientation.x << std::endl;
	std::cout << "y: " << msg->orientation.y << std::endl;
	std::cout << "z: " << msg->orientation.z << std::endl;
	std::cout << "w: " << msg->orientation.w << std::endl;

	std::cout << "x: " << msg->position.x << std::endl;
	std::cout << "y: " << msg->position.y << std::endl;
	std::cout << "z: " << msg->position.z << std::endl;
#endif
	group->setPoseTarget(*msg);
	group->move();
}

/* --------------关节空间运动接口回调函数-----------------*/
void jointCallback(const hsr_msgs::Joints::ConstPtr& msg){

#ifdef DEBUG
	std::cout << "joint1: " << msg->joint1 << std::endl;
	std::cout << "joint2: " << msg->joint2 << std::endl;
	std::cout << "joint3: " << msg->joint3 << std::endl;
	std::cout << "joint4: " << msg->joint4 << std::endl;
	std::cout << "joint5: " << msg->joint5 << std::endl;
	std::cout << "joint6: " << msg->joint6 << std::endl;
#endif 
	std::vector<double> joint_pos(6);

	joint_pos[0] = msg->joint1;
	joint_pos[1] = msg->joint2;
	joint_pos[2] = msg->joint3;
	joint_pos[3] = msg->joint4;
	joint_pos[4] = msg->joint5;
	joint_pos[5] = msg->joint6;	

	group->setJointValueTarget(joint_pos);
	group->move();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "hsr_moveit_api");
	ros::NodeHandle n;

	std::string groupName;

	// 启动消息循环
	ros::AsyncSpinner spinner(1);
  	spinner.start();

	// 发布 robot_state的接口
	ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("robot_pose", 1000);
	
	// 订阅 target_pose话题
	ros::Subscriber pose_sub = n.subscribe("target_pose", 1000, poseCallback);

	// 订阅 target_joints话题
	ros::Subscriber joint_sub = n.subscribe("target_joint", 1000, jointCallback);

	// 获取轴组名称
	n.param<std::string>("move_group_name", groupName, "arm");

	// 初始化movit的接口
	group = new move_group_interface::MoveGroup(groupName);

	// 发布频率
	ros::Rate loop_rate(20);
	
	std::vector<double> rpy;

	while(ros::ok()){

		// 保存Pose信息的变量
		geometry_msgs::PoseStamped now_pose;	

		// 获取当前笛卡尔坐标
		now_pose = group->getCurrentPose();

		// 获取当前RPY
		rpy = group->getCurrentRPY();

		// 发布当前笛卡尔坐标
		pose_pub.publish(now_pose);

		// 休眠
		loop_rate.sleep();
		
		//std::cout << "rpy = " << rpy[0] << " || " << rpy[1] << " || " << rpy[2] << std::endl; 
	
	}
}
