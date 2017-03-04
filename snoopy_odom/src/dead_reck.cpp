#include <ros/ros.h>
#include <math.h>
#include "phidgets/motor_encoder.h"
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include "snoopy_odom/update_odom.h"
#define LENGTH 0.26	//meters
#define RADIUS 0.036125 //meters

using namespace ros;
double wleft, wright;
double theta, theta_deg;
double x;
double y;


void left_enc_cb(const phidgets::motor_encoder::ConstPtr& msg)
{
	wleft = (msg->count_change)*125*2*M_PI/900;
}

void right_enc_cb(const phidgets::motor_encoder::ConstPtr& msg)
{
	wright = -(msg->count_change)*125*2*M_PI/900;
}

bool odom_update(snoopy_odom::update_odom::Request &req,
                 snoopy_odom::update_odom::Response &res)
{
    if(req.cmd.data == "ALL")
    {
        x = req.pose.x;
        y = req.pose.y;
        theta = req.pose.theta;
        return true;
    }
    else if(req.cmd.data == "THETA")
    {
        theta = req.pose.theta;
        return true;
    }
    else return false;

}

int main(int argc, char* argv[])
{
	init(argc,argv,"dead_reck");
    NodeHandle n;
    NodeHandle n_priv("~");
    //read from server
    n_priv.param("init_pose_x",x,0.0);
    n_priv.param("init_pose_y",y,0.0);
    n_priv.param("init_pose_th",theta_deg,0.0);
    theta = theta_deg * M_PI/180;

	Subscriber left_enc, right_enc;
	Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",10);
	Publisher delta_pub = n.advertise<geometry_msgs::Vector3>("odom_delta",10);
	left_enc = n.subscribe<phidgets::motor_encoder>("left_motor/encoder",1,left_enc_cb);
	right_enc = n.subscribe<phidgets::motor_encoder>("right_motor/encoder",1,right_enc_cb);
    ServiceServer update_srv = n.advertiseService("update_odom",odom_update);
	tf::TransformBroadcaster odom_broadcaster;

    Rate loop_rate(120);
	Time current_time, prev_time;
	current_time = Time::now();
	prev_time = Time::now();

	while(ok())
	{
		spinOnce();
		current_time = Time::now();
        double w_robot = (wright - wleft) * RADIUS/LENGTH;     //turning right is positive
        double v_robot = (wright + wleft) * RADIUS/2;

		double dt = (current_time - prev_time).toSec();
		double dx = cos(theta)*v_robot*dt;
		double dy = sin(theta)*v_robot*dt;
		double dth = w_robot*dt;

		x += dx;
		y += dy;
		theta += dth;

        //______________________To be used in Particle Filter________________________
        double delta_angle_right = wright/125;
        double delta_angle_left = wleft/125;
        double delta_angle_robot = (delta_angle_right - delta_angle_left) * RADIUS/LENGTH;
        double delta_lin_robot = (delta_angle_right + delta_angle_left) * RADIUS/2;

        geometry_msgs::Vector3 delta_vec;
        delta_vec.x = delta_lin_robot;
        delta_vec.y = delta_angle_robot;
        delta_vec.z = 0.0;
        delta_pub.publish(delta_vec);
        //____________________________________________________________________________

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		odom_broadcaster.sendTransform(odom_trans);

		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = v_robot;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = w_robot;

		odom_pub.publish(odom);
		prev_time = current_time;
		loop_rate.sleep();
	}

	return 0;
}
