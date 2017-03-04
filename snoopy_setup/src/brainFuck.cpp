#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>
//_________std_msgs___________
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
//_________geometry_msgs___________
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
//_________nav_msgs___________
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
//__________vis_msgs___________
#include <visualization_msgs/Marker.h>
//_________c++___________
#include <cmath>
#include <algorithm>
#include <utility>
#include <iostream>
//_________service/action___________
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "snoopy_navigate/navigateAction.h"
#include "snoopy_pathfind/pathfind.h"
#include "snoopy_pathfind/pathfindRequest.h"
#include "snoopy_pathfind/pathfindResponse.h"
#include "snoopy_map/ogm_get.h"
#include "snoopy_map/ogm_update.h"
#include "snoopy_localize/get_pose.h"
#include "snoopy_setup/explore.h"

using namespace std;
using namespace ros;
using namespace actionlib;

class brain
{
public:
    brain();
    ~brain();
    NodeHandle n, n_priv;
    Publisher vis_pub;
    //ogm_client vars
    ServiceClient ogm_getter_client;
    nav_msgs::OccupancyGrid ogm;
    int cell_occ_value;
    //astar client vars
    ServiceClient astar_client;
    double _astar_pose_x,_astar_pose_y,_astar_goal_x,_astar_goal_y;
    geometry_msgs::Pose2D astar_pose;
    geometry_msgs::Pose2D astar_goal;
    nav_msgs::Path the_path;
    //navigate client vars
    SimpleActionClient<snoopy_navigate::navigateAction> action;
    snoopy_navigate::navigateGoal goal;
    //pose client vars
    ServiceClient pose_client;        
    bool called;
    Time callTime;
    //collision var
    bool collision;
    int collision_count;
private:
    Subscriber move_base_sub, collision_sub;
    void move_base_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void collision_cb(const std_msgs::Bool::ConstPtr& msg);

};

brain::brain():
    n_priv("~"),
    action("navigate",true)
{
    ROS_INFO("_brain_: I AM AT CONSTRUCTOR...");
    //read parameters from server
    //    n_priv.getParam("astar_pose_x", _astar_pose_x);
    //    n_priv.getParam("astar_pose_y", _astar_pose_y);
    //    n_priv.getParam("astar_goal_x", _astar_goal_x);
    //    n_priv.getParam("astar_goal_y", _astar_goal_y);

    //assign values
    //    astar_pose.x = _astar_pose_x;
    //    astar_pose.y = _astar_pose_y;
    //    astar_goal.x = _astar_goal_x;
    //    astar_goal.y = _astar_goal_y;

    //ogm client
    ogm_getter_client = n.serviceClient<snoopy_map::ogm_get>("ogm_get");
    //astar client
    astar_client = n.serviceClient<snoopy_pathfind::pathfind>("astar_srv");
    //pose client
    pose_client = n.serviceClient<snoopy_localize::get_pose>("get_pose");
    //viz pub
    vis_pub = n.advertise<visualization_msgs::Marker>("snoopy/path",5);
    //rviz move base sub
    move_base_sub = n.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",1,&brain::move_base_cb,this);
    //collision sub
    collision_sub = n.subscribe<std_msgs::Bool>("/snoopy/collision", 20, &brain::collision_cb,this);

    called = false;
    callTime = Time::now();
    collision_count = 0;
    collision = false;
}

brain::~brain()
{
    ROS_INFO("_brain_: DESTRUCTOR");
}

void brain::move_base_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    astar_goal.x = msg->pose.position.x;
    astar_goal.y = msg->pose.position.y;
    called = true;
}

void brain::collision_cb(const std_msgs::Bool::ConstPtr& msg)
{
    if(!msg->data) collision = msg->data;
    else
    {
        collision_count++;
        if(collision_count > 10)
        {
            collision = msg->data;
            collision_count = 0;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "brain");
    brain the_brain;
    //ogm_get service object
    snoopy_map::ogm_get ogm_srv;
    //astar service object
    snoopy_pathfind::pathfind astar_srv;
    //pose service object
    snoopy_localize::get_pose pose_srv;
    geometry_msgs::Pose2D init;

    //_____________________ VISUALIZE PATH VARIABLES ____________________
    visualization_msgs::Marker pr_marker;
    pr_marker.header.frame_id = "/odom";
    pr_marker.header.stamp = ros::Time();
    pr_marker.ns = "odom";
    pr_marker.type = visualization_msgs::Marker::LINE_STRIP;
    pr_marker.action = visualization_msgs::Marker::ADD;
    pr_marker.scale.x = 0.02;
    pr_marker.color.a = 1.0;
    pr_marker.color.r = (0.0/255.0);
    pr_marker.color.g = (255.0/255.0);
    pr_marker.color.b = (0.0/255.0);
    tf::Quaternion quat; quat.setRPY(0.0,0.0,0.0);
    tf::quaternionTFToMsg(quat, pr_marker.pose.orientation);
    //___________________________________________________________________

    //wait until everything launches
    sleep(10);

    while(ok())
    {
        spinOnce();
        if(the_brain.called)
        {
            //call pose srv to get current pose
            if(the_brain.pose_client.call(pose_srv))
            {
                ROS_INFO("_brain_: POSE SRV CALLED");
                the_brain.astar_pose = pose_srv.response.pose;
                init = the_brain.astar_pose;
            }
            else
            {
                ROS_ERROR("_brain_: POSE SRV FAILED");
            }

            //call the ogm_get to receive grid map
            if (the_brain.ogm_getter_client.call(ogm_srv))
            {
                ROS_INFO("_brain_: OGM GETTER CALLED");
                //set the map
                the_brain.ogm = ogm_srv.response.map;
                the_brain.cell_occ_value = ogm_srv.response.cell_occ_value.data;
            }
            else
            {
                ROS_ERROR("_brain_: OGM GETTER FAILED");
                return 1;
            }

            //assign astar srv request msgs
            astar_srv.request.pose = the_brain.astar_pose;
            astar_srv.request.goal = the_brain.astar_goal;
            astar_srv.request.map = the_brain.ogm;
            astar_srv.request.cell_occ_value.data = the_brain.cell_occ_value;

            //call the astar
            if(the_brain.astar_client.call(astar_srv))
            {
                //assign the path
                the_brain.the_path = astar_srv.response.path;
                //assign the path points to the marker
                for(size_t i = 0; i < astar_srv.response.path.poses.size(); i++)
                {
                    //ROS_INFO("Path %d, x: %f y: %f",int(i),srv.response.path.poses[i].pose.position.x,srv.response.path.poses[i].pose.position.y);
                    geometry_msgs::Point p;
                    p.x = astar_srv.response.path.poses[i].pose.position.x;
                    p.y = astar_srv.response.path.poses[i].pose.position.y;
                    p.z = 0;
                    pr_marker.points.push_back(p);
                }
                the_brain.vis_pub.publish(pr_marker);
            }

            else
            {
                ROS_ERROR("_brain_: ASTAR SRV FAILED");
                return 1;
            }

            the_brain.goal.path = the_brain.the_path;
            //wait navigation server
            the_brain.action.waitForServer();
//            sleep(5);
            ROS_INFO("_brain_: Action server started, sending goal");
            the_brain.action.sendGoal(the_brain.goal);            
            while(the_brain.action.getState() != the_brain.action.getState().SUCCEEDED ||
                  the_brain.action.getState() != the_brain.action.getState().PREEMPTED)
            {
                spinOnce();
                if(the_brain.collision)
                {
                    ROS_INFO("_brain_: COLLISION DETECTED, CANCELLING GOALS");
                    Time t = Time::now();
                    the_brain.action.cancelAllGoals();
                    ROS_INFO("_brain_: PREEMPT TIME %f", (Time::now()-t).toSec());
                }
//                sleep(0.01);
            }
            //            the_brain.action.waitForResult();
            snoopy_navigate::navigateResult res = *(the_brain.action.getResult());
            the_brain.called = false;
        }//end if called
        the_brain.vis_pub.publish(pr_marker);

    }//end while ok




    /*   //_________GO BACK___________-

    if(the_brain.pose_client.call(pose_srv))
    {
        ROS_INFO("_brain_: POSE SRV CALLED");
        the_brain.astar_pose = pose_srv.response.pose;
    }
    else
    {
        ROS_ERROR("_brain_: POSE SRV FAILED");
    }

    //assign astar srv request msgs
    astar_srv.request.pose = the_brain.astar_pose;
    astar_srv.request.goal = init;
    astar_srv.request.map = the_brain.ogm;
    //clear path
    pr_marker.points.clear();
    //call the astar
    if(the_brain.astar_client.call(astar_srv))
    {
        //assign the path
        the_brain.the_path = astar_srv.response.path;
        //assign the path points to the marker
        for(size_t i = 0; i < astar_srv.response.path.poses.size(); i++)
        {
            //ROS_INFO("Path %d, x: %f y: %f",int(i),srv.response.path.poses[i].pose.position.x,srv.response.path.poses[i].pose.position.y);
            geometry_msgs::Point p;
            p.x = astar_srv.response.path.poses[i].pose.position.x;
            p.y = astar_srv.response.path.poses[i].pose.position.y;
            p.z = 0;
            pr_marker.points.push_back(p);
        }
        the_brain.vis_pub.publish(pr_marker);
    }

    else
    {
        ROS_ERROR("_brain_: ASTAR SRV FAILED");
            return 1;
    }

    the_brain.goal.path = the_brain.the_path;
    //wait navigation server
    the_brain.action.waitForServer();
    ROS_INFO("_brain_: Action server started, sending goal");
    the_brain.action.sendGoal(the_brain.goal);
    the_brain.action.waitForResult();
    res = *(the_brain.action.getResult());
*/

//    while(ok())
//    {
//        the_brain.vis_pub.publish(pr_marker);
//    }

    return 0;
}




//#include <ros/ros.h>
//#include <ros/time.h>
//#include <tf/tf.h>
////_________std_msgs___________
//#include <std_msgs/Header.h>
////_________geometry_msgs___________
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Pose2D.h>
//#include <geometry_msgs/Point.h>
//#include <geometry_msgs/PoseStamped.h>
////_________nav_msgs___________
//#include <nav_msgs/Path.h>
//#include <nav_msgs/OccupancyGrid.h>
////__________vis_msgs___________
//#include <visualization_msgs/Marker.h>
////_________c++___________
//#include <cmath>
//#include <algorithm>
//#include <utility>
//#include <iostream>
////_________service/action___________
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>
//#include "snoopy_navigate/navigateAction.h"
//#include "snoopy_pathfind/pathfind.h"
//#include "snoopy_pathfind/pathfindRequest.h"
//#include "snoopy_pathfind/pathfindResponse.h"
//#include "snoopy_map/ogm_get.h"
//#include "snoopy_map/ogm_update.h"
//#include "snoopy_localize/get_pose.h"

//using namespace std;
//using namespace ros;
//using namespace actionlib;

//class brain
//{
//public:
//    brain();
//    ~brain();
//    NodeHandle n, n_priv;
//    Publisher vis_pub;
//    //ogm_client vars
//    ServiceClient ogm_getter_client;
//    nav_msgs::OccupancyGrid ogm;
//    //astar client vars
//    ServiceClient astar_client;
//    double _astar_pose_x,_astar_pose_y,_astar_goal_x,_astar_goal_y;
//    geometry_msgs::Pose2D astar_pose;
//    geometry_msgs::Pose2D astar_goal;
//    nav_msgs::Path the_path;
//    //navigate client vars
//    SimpleActionClient<snoopy_navigate::navigateAction> action;
//    snoopy_navigate::navigateGoal goal;
//    //pose client vars
//    ServiceClient pose_client;


//};

//brain::brain():
//    n_priv("~"),
//    action("navigate",true)
//{
//    ROS_INFO("_brain_: I AM AT CONSTRUCTOR...");
//    //read parameters from server
////    n_priv.getParam("astar_pose_x", _astar_pose_x);
////    n_priv.getParam("astar_pose_y", _astar_pose_y);
//    n_priv.getParam("astar_goal_x", _astar_goal_x);
//    n_priv.getParam("astar_goal_y", _astar_goal_y);

//    //assign values
////    astar_pose.x = _astar_pose_x;
////    astar_pose.y = _astar_pose_y;
//    astar_goal.x = _astar_goal_x;
//    astar_goal.y = _astar_goal_y;

//    //ogm client
//    ogm_getter_client = n.serviceClient<snoopy_map::ogm_get>("ogm_get");
//    //astar client
//    astar_client = n.serviceClient<snoopy_pathfind::pathfind>("astar_srv");
//    //pose client
//    pose_client = n.serviceClient<snoopy_localize::get_pose>("get_pose");
//    //viz pub
//    vis_pub = n.advertise<visualization_msgs::Marker>("snoopy/path",5);

//}

//brain::~brain()
//{
//    ROS_INFO("_brain_: DESTRUCTOR");
//}

//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "brain");
//    brain the_brain;

//    sleep(10);

//    geometry_msgs::PoseStamped p;
//    p.pose.position.x = 0.75;
//    p.pose.position.y = 0.75;
//    the_brain.the_path.poses.push_back(p);

//    p.pose.position.x = 1.5;
//    p.pose.position.y = 0.75;
//    the_brain.the_path.poses.push_back(p);

//    the_brain.goal.path = the_brain.the_path;
//    //wait navigation server
//    the_brain.action.waitForServer();
//    sleep(3);
//    ROS_INFO("_brain_: Action server started, sending goal");
//    the_brain.action.sendGoal(the_brain.goal);
//    the_brain.action.waitForResult();
//    snoopy_navigate::navigateResult res = *(the_brain.action.getResult());

//    return 0;
//}










