#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#define LENGTH 0.26	//meters
#define RADIUS 0.036125 //meters

using namespace ros;

class snoopyTeleop
{
public:
	Publisher motor_left_pub, motor_right_pub;
	snoopyTeleop();
	~snoopyTeleop();
private:
	NodeHandle n;
	Subscriber cartesian_sub;
	void cartesian_cb(const geometry_msgs::Twist::ConstPtr& msg);
	std::string left_topic,right_topic;
};

snoopyTeleop::snoopyTeleop()
{
	ROS_INFO("Creating an object of class snoopyTeleop");
	n.getParam("left_motor/name",left_topic);
	n.getParam("right_motor/name",right_topic);
	std::string slash = "/";
	std::string cmd_vel = "cmd_vel";
	left_topic += slash + cmd_vel;
	right_topic += slash + cmd_vel;	
	motor_left_pub = n.advertise<std_msgs::Float32>(left_topic.data(),100);
	motor_right_pub = n.advertise<std_msgs::Float32>(right_topic.data(),100);
	cartesian_sub = n.subscribe<geometry_msgs::Twist>("snoopy/cmd_vel",100,&snoopyTeleop::cartesian_cb,this);
}

snoopyTeleop::~snoopyTeleop()
{
	ROS_INFO("Deleting an object of class snoopyTeleop");
}

void snoopyTeleop::cartesian_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
	float linear_vel = msg->linear.x;
	float angular_vel = msg->angular.z;
	//Calculating left and right motor velocities using kinematics
	float left_cmd = (2 * linear_vel + angular_vel * LENGTH)/(2 * RADIUS);
	float right_cmd = (2 * linear_vel - angular_vel * LENGTH)/(2 * RADIUS);
	std_msgs::Float32 left_msg,right_msg;
	left_msg.data = -left_cmd;
	right_msg.data = right_cmd;
	motor_left_pub.publish(left_msg);
	motor_right_pub.publish(right_msg);
}


int main(int argc, char* argv[])
{
	init(argc,argv,"snoopy_teleop");
	snoopyTeleop snoopy;
	Rate loop_rate(10);
	while(ok())
	{
		spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
