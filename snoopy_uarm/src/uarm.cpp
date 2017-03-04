#include <ros/ros.h>
#include <ros/duration.h>
#include <iostream>
#include <math.h>

#include <geometry_msgs/Point.h>

#include "uarm/AttachDetach.h"
#include "uarm/MoveTo.h"
#include "uarm/Pump.h"

#include "snoopy_uarm/ObjectPosition.h"

using namespace ros;
using namespace std;

uarm::MoveTo srv;
uarm::Pump pump_srv;

ServiceClient move_cli;
ServiceClient pump_cli;

float goal_x;
float goal_y;
float goal_z;
float arm_x;
float arm_y;
float arm_z;

bool pump(bool on_off);

/*
void goalCallback(const geometry_msgs::Point::ConstPtr& msg)
{
	goal_x = msg->x;
	goal_y = msg->y;
	goal_z = msg->z;
}
*/

void arm_do()
{
	srv.request.position.x = goal_x;
	srv.request.position.y = goal_y;
	srv.request.position.z = goal_z;
	srv.request.eef_orientation = 1.57;
	srv.request.move_mode = 0;
	srv.request.movement_duration = Duration(3, 0);
	srv.request.ignore_orientation = false;
	srv.request.interpolation_type = 1;
	srv.request.check_limits = false;

	if(move_cli.call(srv))
	{
		ROS_INFO("ARM_DID");
	}
	else{ROS_INFO("ARM_DIDN'T");}
}

void arm_do_steps()
{
	srv.request.position.x = goal_x;
    srv.request.position.y = goal_y;
	srv.request.position.z = srv.response.position.z;
	srv.request.eef_orientation = 1.57;
	srv.request.move_mode = 0;
	srv.request.movement_duration = Duration(3, 0);
	srv.request.ignore_orientation = false;
	srv.request.interpolation_type = 1;
	srv.request.check_limits = false;

	if(move_cli.call(srv))
	{
		ROS_INFO("ARM_DID");
	}
	else{ROS_INFO("ARM_DIDN'T");}
	
    srv.request.position.x = goal_x;
	srv.request.position.y = goal_y;
    srv.request.position.z = goal_z;
	srv.request.eef_orientation = 1.57;
	srv.request.move_mode = 0;
	srv.request.movement_duration = Duration(3, 0);
	srv.request.ignore_orientation = false;
	srv.request.interpolation_type = 1;
	srv.request.check_limits = false;

	if(move_cli.call(srv))
	{
		ROS_INFO("ARM_DID");
	}
	else{ROS_INFO("ARM_DIDN'T");}
    /*
	srv.request.position.x = srv.response.position.x;
	srv.request.position.y = srv.response.position.y;
	srv.request.position.z = goal_z;
	srv.request.eef_orientation = 1.57;
	srv.request.move_mode = 0;
	srv.request.movement_duration = Duration(3, 0);
	srv.request.ignore_orientation = false;
	srv.request.interpolation_type = 1;
	srv.request.check_limits = false;

	if(move_cli.call(srv))
	{
		ROS_INFO("ARM_DID");
	}
	else{ROS_INFO("ARM_DIDN'T");}
    */
}

bool check_move()
{
	arm_x = srv.response.position.x;
	arm_y = srv.response.position.y;
	arm_z = srv.response.position.z;
	
	cout << arm_x << endl;
	cout << arm_y << endl;
	cout << arm_z << endl;

	float dx = abs(arm_x - goal_x);
	float dy = abs(arm_y - goal_y);
	float dz = abs(arm_z - goal_z);

	if(dx > 0.5 || dy > 0.5 || dz > 0.5){return false;}
	else{return true;}
}

void arm_redo()
{
	arm_x = srv.response.position.x;
	arm_y = srv.response.position.y;
	arm_z = srv.response.position.z;

	float dx = goal_x - arm_x;
	float dy = goal_y - arm_y;
	float dz = goal_z - arm_z;

	goal_x += dx;
	goal_y += dy;
	goal_z += dz;

	arm_do();
	ROS_INFO("ARM_REDO");
}

void reset()
{
	srv.request.position.x = 10.0;
    srv.request.position.y = 0.0;
	srv.request.position.z = 12.0;
	srv.request.eef_orientation = 1.57;
	srv.request.move_mode = 0;
	srv.request.movement_duration = Duration(3, 0);
	srv.request.ignore_orientation = false;
	srv.request.interpolation_type = 1;
	srv.request.check_limits = false;
	
	if(move_cli.call(srv))
	{
		ROS_INFO("ARM_RESET");
	}
	else{ROS_INFO("ARM_NOT_RESET");}
}

bool arm_init(snoopy_uarm::ObjectPosition::Request &req, snoopy_uarm::ObjectPosition::Response &res)
{
	goal_x = req.x;
	goal_y = req.y;
	goal_z = req.z;

	pump(true);
    arm_do_steps();
	if(check_move()){ROS_INFO("UARM IS WITHIN THRESHOLD");}
	else
	{
		//while(!check_move())
		//{
			arm_redo();
		//}
	}
	/*
	if(!check_move())
	{
		arm_do();
		while(!check_move() && count < 7)
		{
			reset();
			arm_do();
			count++;
		}
	}
	*/
	reset();
	res.success = true;
}

bool pump(bool on_off)
{
	pump_srv.request.pump_status = on_off;

	if(pump_cli.call(pump_srv))
	{
		ROS_INFO("PUMP ON|OFF");
	}
	else{ROS_INFO("PUMP FAILED");}
}

int main(int argc, char** argv)
{
	init(argc, argv, "uarm_server");
	NodeHandle n;

	move_cli = n.serviceClient<uarm::MoveTo>("/uarm/move_to");
	pump_cli = n.serviceClient<uarm::Pump>("/uarm/pump");

	//Subscriber goal_sub = n.subscribe("goal", 1000, goalCallback);
	ServiceServer arm_serv = n.advertiseService("uarm_do", arm_init);

	//move_cli = n.serviceClient<uarm::MoveTo>("/uarm/move_to");
	//pump_cli = n.serviceClient<uarm::Pump>("/uarm/pump");

	spin();
	
	return 0;
}
