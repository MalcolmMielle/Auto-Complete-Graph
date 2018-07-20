#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom, tf::TransformBroadcaster& odom_broadcaster){

	//since all odometry is 6DOF we'll need a quaternion created from yaw
//	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom->pose.pose.orientation.);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = ros::Time::now();
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = odom->pose.pose.position.x;
	odom_trans.transform.translation.y = odom->pose.pose.position.y;
	odom_trans.transform.translation.z = odom->pose.pose.position.z;
	odom_trans.transform.rotation.x = odom->pose.pose.orientation.x;
	odom_trans.transform.rotation.y = odom->pose.pose.orientation.y;
	odom_trans.transform.rotation.z = odom->pose.pose.orientation.z;
	odom_trans.transform.rotation.w = odom->pose.pose.orientation.w;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

//	//next, we'll publish the odometry message over ROS
//	nav_msgs::Odometry odom;
//	odom.header.stamp = current_time;
//	odom.header.frame_id = "odom";
//
//	//set the position
//	odom.pose.pose.position.x = x;
//	odom.pose.pose.position.y = y;
//	odom.pose.pose.position.z = 0.0;
//	odom.pose.pose.orientation = odom_quat;
//
//	//set the velocity
//	odom.child_frame_id = "base_link";
//	odom.twist.twist.linear.x = vx;
//	odom.twist.twist.linear.y = vy;
//	odom.twist.twist.angular.z = vth;

}

int main(int argc, char** argv){

	ros::init(argc, argv, "odometry_publisher_to_tf");

	tf::TransformBroadcaster odom_broadcaster;

	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/robot3/control/odom", 1, boost::bind(&odomCallback, _1, odom_broadcaster) );
//	nh.subscribe<std_msgs::Bool>("/publish_occ_acg", 1, boost::bind(&latchOccGrid, _1, &oacg));



	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

//	ros::Rate r(1.0);
	while(n.ok()){

		ros::spinOnce();               // check for incoming messages
		current_time = ros::Time::now();

		last_time = current_time;
//		r.sleep();
	}
}
