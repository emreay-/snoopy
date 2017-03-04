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

class snoopyKinematics
{
public:
	Publisher motor_left_pub, motor_right_pub;
	float left_cmd, right_cmd;
	snoopyKinematics();
	~snoopyKinematics();
private:
	NodeHandle n;
	Subscriber cartesian_sub;
	void cartesian_cb(const geometry_msgs::Twist::ConstPtr& msg);
};

snoopyKinematics::snoopyKinematics()
{
	ROS_INFO("Creating an object of class snoopyKinematics");
	motor_left_pub = n.advertise<std_msgs::Float32>("setpoint_left",100);
	motor_right_pub = n.advertise<std_msgs::Float32>("setpoint_right",100);
	cartesian_sub = n.subscribe<geometry_msgs::Twist>("snoopy/cmd_vel",100,&snoopyKinematics::cartesian_cb,this);
	left_cmd = 0.0;
	right_cmd = 0.0;
}

snoopyKinematics::~snoopyKinematics()
{
	ROS_INFO("Deleting an object of class snoopyKinematics");
}

void snoopyKinematics::cartesian_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
	float linear_vel = msg->linear.x;
	float angular_vel = msg->angular.z;
	//Calculating left and right motor velocities using kinematics
	left_cmd = (2 * linear_vel + angular_vel * LENGTH)/(2 * RADIUS);
	right_cmd = (2 * linear_vel - angular_vel * LENGTH)/(2 * RADIUS);
	std_msgs::Float32 left_msg,right_msg;
	left_msg.data = left_cmd;
	right_msg.data = -right_cmd;
	motor_left_pub.publish(left_msg);
	motor_right_pub.publish(right_msg);
}


int main(int argc, char* argv[])
{
	init(argc,argv,"snoopy_kinematics");
	snoopyKinematics snoopy;
    Rate loop_rate(20);
	while(ok())
	{
		spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
