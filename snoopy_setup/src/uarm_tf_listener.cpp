#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>

using namespace ros;
Publisher pub;
void uarmCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	tf::TransformListener listener;
    ROS_WARN("This is a callback");
    listener.waitForTransform("/uarm_link","/odom",Time(0),Duration(10,0));
    ROS_WARN("This is a post transform callback");
    geometry_msgs::PointStamped object_pose;
    geometry_msgs::PointStamped odom_obj_pose;
    object_pose.header.frame_id = "/uarm_link";
    object_pose.header.stamp = Time();
    //UARM AXES ARE RETARDED X AND Y HAS TO CHANGE
    object_pose.point.x = msg->point.y;
    object_pose.point.y = msg->point.x;
    object_pose.point.z = msg->point.z;

    try
    {
      ROS_INFO("IM TRANSFORMING A POint");
      listener.transformPoint("/odom", object_pose, odom_obj_pose);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Exception received asdgasdfasdfas: %s",ex.what());
    }
    pub.publish(odom_obj_pose);


}

int main(int argc, char* argv[])
{
	init(argc,argv,"uarm_tf_listener");
	NodeHandle n;
	Rate r(10);
	pub = n.advertise<geometry_msgs::PointStamped>("/to_arm_frame",10);
	Subscriber target_sub = n.subscribe<geometry_msgs::PointStamped>("/tfed",10,uarmCallback);
	while(ok())
	{
		spinOnce();
		r.sleep();
	}



}
