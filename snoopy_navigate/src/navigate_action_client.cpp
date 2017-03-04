#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> //the pose of the robot
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <algorithm>
#include <utility>
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <snoopy_navigate/navigateAction.h>

using namespace std;
using namespace ros;
using namespace actionlib;

class navigate_client
{
public:
    NodeHandle n;
    SimpleActionClient<snoopy_navigate::navigateAction> action;
    nav_msgs::Path path;
    navigate_client();
    ~navigate_client();
    bool result;
    snoopy_navigate::navigateGoal goal;
};

navigate_client::navigate_client():
    action("navigate",true)
{
    ROS_INFO("_navigate_client_: Calling the constructor of class NAVIGATE");
}

navigate_client::~navigate_client()
{
    ROS_INFO("_navigate_client_: Deleting an object of the class navigate_client");
}


int main(int argc, char** argv)
{
    init(argc, argv, "navigate_tester");
    navigate_client nav;

    geometry_msgs::PoseStamped target1,target2;
    //,target3,target4;
    target1.pose.position.x = 0.0;
    target1.pose.position.y = -4.0;

    target2.pose.position.x = 0.0;
    target2.pose.position.y = 0.0;

//    target3.pose.position.x = 1.0;
//    target3.pose.position.y = 1.0;

//    target4.pose.position.x = 0.0;
//    target4.pose.position.y = 1.0;

    nav.path.poses.push_back(target1);
    nav.path.poses.push_back(target2);
    //nav.path.poses.push_back(target3);
    //nav.path.poses.push_back(target4);
    nav.goal.path = nav.path;

    nav.action.waitForServer();
    ROS_INFO("_navigate_client_: Action server started, sending goal");
    nav.action.sendGoal(nav.goal);

    nav.action.waitForResult();
    snoopy_navigate::navigateResult res = *(nav.action.getResult());

    ROS_INFO("_navigate_client_: RESULT x,y = %f %f", res.stop_point.x, res.stop_point.y);
//    if()
//    {
//        ROS_INFO("Action finished.");
//    }
//    else
//    {
//        ROS_INFO("Action did not finish before reaching the final goal.");
//    }


    return 0;
}

