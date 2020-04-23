rosparam set controller_joint_names "[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6,R_joint_1, R_joint_2, R_joint_3, R_joint_4, R_joint_5, R_joint_6]"

rosparam set controller_joint_names "[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]"
rosparam set controller_joint_names "[R_joint_1, R_joint_2, R_joint_3, R_joint_4, R_joint_5, R_joint_6]"
rosrun industrial_robot_client joint_trajectory_action


