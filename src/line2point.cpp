#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

ros::Publisher twistPub;
double x,y,theta,dist;
geometry_msgs::Twist tws;

double normAng (double ang){
	if(ang>M_PI)ang=ang-M_PI;
	if(ang<=-M_PI)ang=ang+M_PI;
	return ang;
}

void callback ( const nav_msgs::OdometryConstPtr &msg){
	double yaw = tf::getYaw(msg->pose.pose.orientation);
	
	theta = std::atan2((y-msg->pose.pose.position.y),(x-msg->pose.pose.position.x));
	dist = sqrt(pow((x-msg->pose.pose.position.x),2)+pow((y-msg->pose.pose.position.y),2));

	ROS_INFO("%lf",normAng(theta-yaw));

	if(dist<0.1){
		tws.linear.x = 0;
		tws.angular.z = 0;
		ROS_INFO("I did it");
	}	
	else if((normAng(theta-yaw) >=0.01 && normAng(theta-yaw)>0)||(-1*normAng(theta-yaw) >=0.01 && normAng(theta-yaw)<0)){
		tws.linear.x = 0;
		tws.angular.z = normAng(theta-yaw);
	}
	else{
		tws.linear.x = dist;
		tws.angular.z = normAng(theta-yaw);
	}
	
	twistPub.publish(tws);
	ROS_INFO("published");
}

int main(int argc, char **argv){
	x = 4.0;
	y = 1.5;

	ros::init(argc,argv,"l2p");

	ros::NodeHandle node;

	ros::Subscriber odometrySub = node.subscribe("/vrep/vehicle/odometry",1,callback);
	twistPub = node.advertise<geometry_msgs::Twist>("/twist",1);

	ros::spin();
	return 0;
}
