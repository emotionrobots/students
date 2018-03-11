#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "robit_move_joint");
	ros::NodeHandle n;
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
	tf::TransformBroadcaster broadcaster;
	ros::Rate loop_rate(30);

	const double degree = M_PI/180;
	double rot4 = 90;

	geometry_msgs::TransformStamped odom_trans;
	sensor_msgs::JointState joint_state;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	joint_state.name.resize(6);
	joint_state.position.resize(6);
	joint_state.name[0] = "shoulder_rotate";
	joint_state.name[1] = "left_bracket_to_upper_arm";
	joint_state.name[2] = "elbow_joint";
	joint_state.name[3] = "forearm_rotate";
	joint_state.name[4] = "wrist_joint";
	joint_state.name[5] = "finger_rotate";

	while (ros::ok()) {
		joint_state.header.stamp = ros::Time::now();
		joint_state.position[0] = rot4 * degree;
		joint_state.position[1] = 0;
		joint_state.position[2] = 0;
		joint_state.position[3] = 0;
		joint_state.position[4] = 0;
		joint_state.position[5] = 0;

		odom_trans.header.stamp = ros::Time::now();
		odom_trans.transform.translation.x = 0;
		odom_trans.transform.translation.y = 0;
		odom_trans.transform.translation.z = 0;
		odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

		joint_pub.publish(joint_state);
		broadcaster.sendTransform(odom_trans);

		rot4 += 1;
		if (rot4 > 360) rot4 -= 360;

		loop_rate.sleep();
	}
	return 0;
}
