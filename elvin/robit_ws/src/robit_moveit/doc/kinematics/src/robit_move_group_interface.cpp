#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "robit_move_group_interface");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	static const std::string PLANNING_GROUP = "arm";
	const std::string eelink = "finger";


	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);


	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup *joint_model_group = 
		move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();


	ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

	moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

	std::vector<double> joint_group_positions;
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	for (std::size_t i = 0; i < joint_names.size(); ++i) {
		ROS_INFO("Joint %-25s: %f", joint_names[i].c_str(), joint_group_positions[i]);
	}

	geometry_msgs::PoseStamped msg = move_group.getCurrentPose();
	double home_x = msg.pose.position.x;
	double home_y = msg.pose.position.y;
	double home_z = msg.pose.position.z;



	while (true) {
		double x, y, z;

		std::cin >> x >> y >> z;

		if (x == -1) {
			break;
		}

		geometry_msgs::Pose target_pose;
		target_pose.orientation.w = 1.0;
		target_pose.position.x = home_x + x;
		target_pose.position.y = home_y + y;
		target_pose.position.z = home_z + z;


		move_group.setJointValueTarget(target_pose);

		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		move_group.setPlanningTime(20.0);

		bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

		move_group.move();

		std::vector<double> toPush;

		// current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
		joint_group_positions = move_group.getCurrentJointValues();
		for (std::size_t i = 0; i < joint_names.size(); ++i) {
			if (i < 4) {
				ROS_INFO("Joint %-25s: %f %f", joint_names[i].c_str(), joint_group_positions[i], joint_group_positions[i]*180/3.1416*750/90);
				toPush.push_back(joint_group_positions[i]*180.0 / 3.1416 * 750.0 / 90.0);
			} else {
				ROS_INFO("Joint %-25s: %f %f", joint_names[i].c_str(), joint_group_positions[i], joint_group_positions[i]*180/3.1416*850/90);
				toPush.push_back(joint_group_positions[i]*180.0 / 3.1416 * 850.0 / 90.0);
			}

		}

		toPush[0] += 1350;
		toPush[2] = 650 + (toPush[1] + toPush[2]);
		toPush[1] = 2100 - toPush[1];
		toPush[3] += 1400;
		toPush[4] = 1450 - toPush[4];

		for (std::size_t i = 0; i < toPush.size(); ++i) {
			ROS_INFO("%f", toPush[i]);
		}
		
	}







	// geometry_msgs::Pose target_pose1;
	// target_pose1.orientation.w = 1.0;
	// target_pose1.position.x = home_x + 0.07;
	// target_pose1.position.y = home_y + 0.07;
	// target_pose1.position.z = home_z + 0.07;


	// move_group.setPoseTarget(target_pose1);
	// move_group.setPositionTarget(target_pose1.position.x, target_pose1.position.y, target_pose1.position.z, eelink);
	// move_group.setJointValueTarget(target_pose1);

	// moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	// move_group.setPlanningTime(20.0);

	// move_group.setEndEffectorLink(eelink);

	// ROS_INFO("%s", success ? "SUCCESS" : "FAILED");

	// bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	// ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	// move_group.move();

	// std::vector<double> toPush;

	// current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

	// joint_group_positions = move_group.getCurrentJointValues();
	// for (std::size_t i = 0; i < joint_names.size(); ++i) {
	// 	if (i < 4) {
	// 		ROS_INFO("Joint %-25s: %f %f", joint_names[i].c_str(), joint_group_positions[i], joint_group_positions[i]*180/3.1416*750/90);
	// 		toPush.push_back(joint_group_positions[i]*180.0 / 3.1416 * 750.0 / 90.0);
	// 	} else {
	// 		ROS_INFO("Joint %-25s: %f %f", joint_names[i].c_str(), joint_group_positions[i], joint_group_positions[i]*180/3.1416*850/90);
	// 		toPush.push_back(joint_group_positions[i]*180.0 / 3.1416 * 850.0 / 90.0);
	// 	}

	// }

	// toPush[0] += 1350;
	// toPush[2] = 650 + (toPush[1] + toPush[2]);
	// toPush[1] = 2100 - toPush[1];
	// toPush[3] += 1400;
	// toPush[4] = 1450 - toPush[4];

	// for (std::size_t i = 0; i < toPush.size(); ++i) {
	// 	ROS_INFO("%f", toPush[i]);
	// }

	ros::shutdown();
	return 0;
}
