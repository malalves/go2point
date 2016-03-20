#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

void callback ( const nav_msgs::OdometryConstPtr &msg){
	double yaw = tf::getYaw(msg->pose.pose.orientation);
	
	yaw = (yaw*180/M_PI);
	ROS_INFO ("yaw: %lf", yaw);
}

int main(int argc, char **argv){
	ros::init(argc,argv,"qYaw");
	ros::NodeHandle node;
	ros::Subscriber odometrySub = node.subscribe("/vrep/vehicle/odometry",1,callback);
	ros::spin();
	return 0;
}
