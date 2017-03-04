#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include "phidgets/motor_encoder.h"
#include <signal.h>
#define SATUR 30

using namespace ros;

void shutDownCallback(int sig)
{
    ROS_INFO("PID NODE CATCH SHUT DOWN");
    NodeHandle n;
    NodeHandle nPrivate("~");
    std::string control_topic;
    nPrivate.getParam("control_topic",control_topic);
    Publisher control_pub = n.advertise<std_msgs::Float32>(control_topic,1);
    std_msgs::Float32 control_msg;
    control_msg.data = 0.0;

    for(int i = 0; i < 40; i++)
    {
        control_pub.publish(control_msg);
        Duration(0.1).sleep();
    }

    shutdown();
}

Time setPointMsgTime;
float desired_w, est_w;

void setPointCallback(const std_msgs::Float32::ConstPtr& msg)
{
	desired_w = (msg->data);
	setPointMsgTime = Time::now();
}

void encoderCallback(const phidgets::motor_encoder::ConstPtr& msg)
{
    est_w = (msg->count_change)*125*2*M_PI/900;
}

int main(int argc, char* argv[])
{
    init(argc,argv,"controller");
    NodeHandle n;
    NodeHandle nPrivate("~");	//private node handle to get the relative parameters of the node    
    signal(SIGINT, shutDownCallback);
    std::string setpoint_topic = "setpoint_top";
    std::string enc_topic = "enc_top";
    std::string control_topic = "control_top";
    std::string err_topic = "err_top";
    std::string est_topic = "est_top";
    nPrivate.getParam("setpoint_topic",setpoint_topic);
    nPrivate.getParam("encoder_topic",enc_topic);
    nPrivate.getParam("control_topic",control_topic);
    nPrivate.getParam("error_topic",err_topic);
    nPrivate.getParam("est_topic",est_topic);
    Subscriber setpoint_sub = n.subscribe<std_msgs::Float32>(setpoint_topic,10,setPointCallback);
    Subscriber enc_sub  = n.subscribe<phidgets::motor_encoder>(enc_topic,10,encoderCallback);
    Publisher control_pub = n.advertise<std_msgs::Float32>(control_topic,1);
    Publisher err_pub = n.advertise<std_msgs::Float32>(err_topic.data(),10);
    Publisher est_pub = n.advertise<std_msgs::Float32>(est_topic.data(),10);

    double Kp = 0.0;
    double Ki = 0.0;
    double Kd = 0.0;
    double limit = SATUR;
    nPrivate.getParam("Kp",Kp);
    nPrivate.getParam("Ki",Ki);
    nPrivate.getParam("Kd",Kd);
    Rate loop_rate(20);
    control_toolbox::Pid pid;
    pid.initPid(Kp,Ki,Kd,limit,-limit);
    Time current_time, prev_time;
    current_time = Time::now();
    prev_time = Time::now();

    std_msgs::Float32 control_msg;
    control_msg.data = 0.0;

	while(ok())
	{
		spinOnce();        
		current_time = Time::now();

        if((current_time - setPointMsgTime).toSec() < 0.2)
		{
            double err = desired_w - est_w;
            double effort = pid.computeCommand(err,current_time - prev_time);
            control_msg.data += effort;
            //publishing error and estimated angular velocity
            std_msgs::Float32 error,est;
            error.data = err;
            err_pub.publish(error);
            est.data = est_w;
            est_pub.publish(est);
		}
            else control_msg.data = 0.0;

        control_pub.publish(control_msg);
        prev_time = current_time;
        loop_rate.sleep();

	}

	return 0;
}
