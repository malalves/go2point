#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

ros::Publisher twistPub;
double x,y,theta,dist;
geometry_msgs::Twist tws;

double toDeg ( double ang){
	return (ang*180/M_PI);
}

double normAng (double ang){
	while(ang/M_PI > 1){
		ang = ang - 2*M_PI;
	}
	while(ang/M_PI <= -1){
		ang = ang + 2*M_PI;
	}
	return ang;
}

void callback ( const nav_msgs::OdometryConstPtr &msg){
	double yaw = tf::getYaw(msg->pose.pose.orientation);
	
	theta = std::atan2((y-msg->pose.pose.position.y),(x-msg->pose.pose.position.x));
	dist = sqrt(pow((x-msg->pose.pose.position.x),2)+pow((y-msg->pose.pose.position.y),2));

	ROS_INFO("%lf  %lf",toDeg(normAng(theta-yaw)),toDeg((theta-yaw)));

	if(dist<0.1){
		tws.linear.x = 0;
		tws.angular.z = 0;
		ROS_INFO("I did it");
	}	
	else{	
		if(dist>0.9){
			tws.linear.x = 0.9;
			tws.angular.z = normAng(theta-yaw);
		}
		else{
			tws.linear.x = dist;
			tws.angular.z = normAng(theta-yaw);
		}	
	}
	
	twistPub.publish(tws);
	ROS_INFO("published");
}

int main(int argc, char **argv){
	x = 4.0;
	y = 1.5;

	ros::init(argc,argv,"c2p");

	ros::NodeHandle node;

	ros::Subscriber odometrySub = node.subscribe("/vrep/vehicle/odometry",1,callback);
	twistPub = node.advertise<geometry_msgs::Twist>("/twist",1);

	ros::spin();
	return 0;
}
