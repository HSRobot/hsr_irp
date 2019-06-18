#include <moveit/move_group_interface/move_group.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)

{

  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);

  // 创建一个异步的自旋线程（spinning thread）

  ros::AsyncSpinner spinner(1);

  spinner.start();

 // move_group_interface::MoveGroup group("arm");//ros_indigo
  moveit::planning_interface::MoveGroupInterface group("arm");


  geometry_msgs::PoseStamped now_pose;
  now_pose = group.getCurrentPose();
/*
  std::cout << "x: " << now_pose.pose.orientation.x << std::endl;
  std::cout << "y: " << now_pose.pose.orientation.y << std::endl;
  std::cout << "z: " << now_pose.pose.orientation.z << std::endl;
  std::cout << "w: " << now_pose.pose.orientation.w << std::endl;

  std::cout << "x: " << now_pose.pose.position.x << std::endl;
  std::cout << "y: " << now_pose.pose.position.y << std::endl;
  std::cout << "z: " << now_pose.pose.position.z << std::endl;
*/

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = 0.630567;
  target_pose1.position.y = -0.0222025;
  target_pose1.position.z = 1.04143;
  target_pose1.orientation.x = 7.32729e-06;
  target_pose1.orientation.y = 0.000133285;
  target_pose1.orientation.z = 1;
  target_pose1.orientation.w = 4.22941e-05;

/*
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.x = 0;
  start_pose2.orientation.y = 1;
  start_pose2.orientation.z = 0.0;
  start_pose2.orientation.w = 0.0;
  start_pose2.position.x = -0.971;
  start_pose2.position.y = -1.623;
  start_pose2.position.z = 0.313;
  start_state.setFromIK(joint_model_group, start_pose2);
*/
/*
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link6";
  ocm.header.frame_id = "base_link";
  ocm.orientation.x = 7.32729e-06;
  ocm.orientation.y = 0.000133285;
  ocm.orientation.z = 1;
  ocm.orientation.w = 4.22941e-05;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  group.setPathConstraints(test_constraints);
*/
	group.setPoseTarget(target_pose1);

	/* Sleep to give Rviz time to visualize the plan. */
	//sleep(10.0);

 	group.move();
/*
	  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  visual_tools.deleteAllMarkers();

	  visual_tools.deleteAllMarkers();
	  visual_tools.publishAxisLabeled(start_pose2, "start");
	  visual_tools.publishAxisLabeled(target_pose1, "goal");
	  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
	  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	  visual_tools.trigger();
	  visual_tools.prompt("next step");
*/	
	group.clearPathConstraints();

  ros::waitForShutdown();

}
