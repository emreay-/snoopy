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
#include <snoopy_localize/fuse.h>
#include "snoopy_vision/identify.h"

using namespace std;
using namespace ros;
using namespace actionlib;

enum STATES
{
    ST_INIT,
    ST_EXPLORE,
    ST_STOP_COLLISION,
    ST_OBJ_IDENTIFY,
    ST_STOP_OBJECT,
    ST_ARM_PICK
};

class brain
{
public:
    brain();
    ~brain();
    NodeHandle n, n_priv;
    Publisher vis_pub,pub_speak;

    STATES state;
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

    //collision vars
    bool collision;
    int collision_count;

    //object detect vars
    bool object_detected;
    bool same_object;
    std_msgs::Bool pickup;
    std_msgs::String object_type;
    geometry_msgs::PointStamped obj_pose, obj_pose_prev;
    Subscriber obj_sub;
    ServiceClient obj_client;
    snoopy_vision::identify identify_;
    Time obj_detection_time;

    //map update vars
    ServiceClient ogm_update_client;
    snoopy_map::ogm_update ogm_update_;

    //explore client vars
    ServiceClient explore_client;
    snoopy_setup::explore explore_;

    //fuse client vars
    ServiceClient fuse_client;
    snoopy_localize::fuse fuse;

    //other vars
    int _slp;
    std_msgs::String start;

private:
    Subscriber collision_sub;
    void collision_cb(const std_msgs::Bool::ConstPtr& msg);
    void object_cb(const std_msgs::Bool::ConstPtr& msg);

};

brain::brain():
    n_priv("~"),
    action("navigate",true)
{
    ROS_INFO("_brain_: I AM AT CONSTRUCTOR...");

    n_priv.param("init_sleep",_slp,10);

    //ogm client
    ogm_getter_client = n.serviceClient<snoopy_map::ogm_get>("ogm_get");
    //astar client
    astar_client = n.serviceClient<snoopy_pathfind::pathfind>("astar_srv");
    //pose client
    pose_client = n.serviceClient<snoopy_localize::get_pose>("get_pose");
    //explore client
    explore_client = n.serviceClient<snoopy_setup::explore>("explore");
    //viz pub
    vis_pub = n.advertise<visualization_msgs::Marker>("snoopy/path",5);
    //collision sub
    collision_sub = n.subscribe<std_msgs::Bool>("/snoopy/collision", 20, &brain::collision_cb,this);
    //object sub
    obj_sub = n.subscribe<std_msgs::Bool>("/snoopy/obj_found", 5, &brain::object_cb, this);
    //object client
    obj_client = n.serviceClient<snoopy_vision::identify>("identify");
    //fuse_client
    fuse_client = n.serviceClient<snoopy_localize::fuse>("fuse");
    //map update client
    ogm_update_client = n.serviceClient<snoopy_map::ogm_update>("ogm_update");
    //espeak publisher
    pub_speak = n.advertise<std_msgs::String>("/espeak/string",5);

    start.data = "start";
    collision_count = 0;
    collision = false;
    object_detected = false;
    same_object = false;
    obj_detection_time = Time::now();

    obj_pose_prev.point.x = -1.0;
    obj_pose_prev.point.y = -1.0;

    state = ST_INIT;
}

brain::~brain()
{
    ROS_INFO("_brain_: DESTRUCTOR");
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

void brain::object_cb(const std_msgs::Bool::ConstPtr& msg)
{
    object_detected = msg->data;
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


    while(ok())
    {
        spinOnce();

        switch(the_brain.state)
        {
    //____________________________ST_INIT________________________________________
            case ST_INIT:
                ROS_WARN("_brain_: INIT STATE");
                //wait until everything launches
                for(int i = the_brain._slp; i > 0; i--)
                {
                    ROS_INFO("_brain_: WAITING FOR %d SECS",i);
                    sleep(1);
                }
                the_brain.pub_speak.publish(the_brain.start);
                the_brain.state = ST_EXPLORE;
                break;
    //____________________________ST_EXPLORE________________________________________
            case ST_EXPLORE:
                ROS_WARN("_brain_: EXPLORE STATE");
                //call the explore and continue if get some goal
                if(the_brain.explore_client.call(the_brain.explore_))
                {
            //____________________________EXPLORE GOAL RECEIVED________________________________________
                    ROS_INFO("_brain_: EXPLORE GOAL RECEIVED");
                    the_brain.astar_goal = the_brain.explore_.response.goal;

                    //call pose srv to get current pose
                    if(the_brain.pose_client.call(pose_srv))
                    {
            //____________________________CURRENT POSE RECEIVED________________________________________
                        ROS_INFO("_brain_: POSE SRV CALLED");
                        the_brain.astar_pose = pose_srv.response.pose;
                        init = the_brain.astar_pose;

                        //call the ogm_get to receive grid map
                        if (the_brain.ogm_getter_client.call(ogm_srv))
                        {
            //________________________________GRID MAP RECEIVED________________________________________
                            ROS_INFO("_brain_: OGM GETTER CALLED");
                            //set the map
                            the_brain.ogm = ogm_srv.response.map;
                            the_brain.cell_occ_value = ogm_srv.response.cell_occ_value.data;

                            //assign astar srv request msgs
                            astar_srv.request.pose = the_brain.astar_pose;
                            astar_srv.request.goal = the_brain.astar_goal;
                            astar_srv.request.map = the_brain.ogm;
                            astar_srv.request.cell_occ_value.data = the_brain.cell_occ_value;

                            //call the astar
                            if(the_brain.astar_client.call(astar_srv))
                            {
            //_________________________________A* PATH RECEIVED________________________________________
                                //assign the path
                                the_brain.the_path = astar_srv.response.path;
                                //assign the path points to the marker
                                pr_marker.points.clear();
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

                                //set the goal path
                                the_brain.goal.path = the_brain.the_path;
                                //wait navigation server
                                the_brain.action.waitForServer();
                                ROS_INFO("_brain_: Action server started, sending goal");
                                the_brain.action.sendGoal(the_brain.goal);
            //________________________________________NAVIGATING____________________________________________
                                bool succeeded = false, preempted = false;

                                while(!succeeded || !preempted)
                                {
                                    spinOnce();
                                    if(the_brain.collision)
                                    {
                                        ROS_INFO("_brain_: COLLISION DETECTED, CANCELLING GOALS");
                                        preempted = true;
                                        the_brain.state = ST_STOP_COLLISION;
                                        break;
                                    }
//                                    bool sceneTime = (Time::now()-the_brain.obj_detection_time).toSec() > 6;

                                    if(the_brain.object_detected && !the_brain.same_object)
                                    {
                                        ROS_INFO("_brain_: OBJECT DETECTED, CANCELLING GOALS");
                                        preempted = true;
                                        the_brain.state = ST_STOP_OBJECT;
                                        break;
                                    }
        //                            ROS_INFO("_brain_: ACTION STATE %d",the_brain.action.getState().state_);
                                    if(the_brain.action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                                    {
                                        succeeded = true;
                                        break;
                                    }

                                }
                                break;
                                //ROS_INFO("_brain_: PATH DONE");
                                //if return to start point, say "I am done"
            //________________________________________PATH DONE____________________________________________
        //                        snoopy_navigate::navigateResult res = *(the_brain.action.getResult());
                            }// IF CALL ASTAR

                            else
                            {
                                ROS_ERROR("_brain_: ASTAR SRV FAILED");
                                break;
                            }


                        }// IF CALL OGM GET
                        else
                        {
                            ROS_ERROR("_brain_: OGM GETTER FAILED");
                            break;
                        }

                    }//IF CALL POSE GET
                    else
                    {
                        ROS_ERROR("_brain_: POSE SRV FAILED");
                        break;
                    }

                }//IF CALL EXPLORE
                else
                {
                    ROS_ERROR("_brain_: EXPLORE GOAL CANT RECEIVED");
                    break;
        //            return 1;
                }
    //____________________________ST_STOP COLLISION___________________________________
            case ST_STOP_COLLISION:
                ROS_WARN("_brain_: STOP_COLLISION STATE");
                the_brain.action.cancelAllGoals();
                ROS_INFO("_brain_: WAITING PF CONVERGENCE.");
                //sleep(5);
                the_brain.fuse.request.cmd.data = "ALL";
                if(the_brain.fuse_client.call(the_brain.fuse))
                {
                    ROS_INFO("_brain_: CALLED FOR FUSION SUCCEDED.");
                    the_brain.state = ST_EXPLORE;
                }
                else
                {
                    ROS_INFO("_brain_: CALLED FOR FUSION FAILED.");
                }
                break;

    //____________________________ST_STOP_OBJECT____________________________________
            case ST_STOP_OBJECT:
                ROS_WARN("_brain_: STOP_OBJECT STATE");
                the_brain.action.cancelAllGoals();
                ROS_INFO("_brain_: WAITING FOR OBJECT DETECTION.");
                //sleep(5);

                if(the_brain.obj_client.call(the_brain.identify_))
                {
                    the_brain.obj_detection_time = Time::now();
                    ROS_INFO("_brain_: CALL FOR OBJECT IDENTIFICATION SUCCEEDED.");
                    the_brain.pickup      = the_brain.identify_.response.pickup;
                    the_brain.object_type = the_brain.identify_.response.object;
                    the_brain.obj_pose    = the_brain.identify_.response.position;

                    if( abs(the_brain.obj_pose.point.x - the_brain.obj_pose_prev.point.x) <= 0.1 &&
                        abs(the_brain.obj_pose.point.y - the_brain.obj_pose_prev.point.y) <= 0.1)
                    {
                        the_brain.same_object = true;
                    }

                    else
                    {
                        the_brain.obj_pose_prev = the_brain.obj_pose;
                        the_brain.same_object = false;
                    }

                    //Espeak publish
                    ROS_ERROR("_brain_:OBJECT IDENTIFIED %s",the_brain.object_type.data.c_str());
//                    the_brain.pub_speak.publish(the_brain.object_type);

                    //call server for update ogm
                    the_brain.ogm_update_.request.update_cell.x = the_brain.obj_pose.point.x ;
                    the_brain.ogm_update_.request.update_cell.y = the_brain.obj_pose.point.y ;
                    the_brain.ogm_update_.request.cmd.data = "FILL";

                    if(the_brain.ogm_update_client.call(the_brain.ogm_update_)) { ROS_INFO("_brain_: ogm_update CALL SUCCESSFUL."); }
                    else { ROS_INFO("_brain_: ogm_update CALL FAILED."); }

                    the_brain.state = ST_EXPLORE;
//                    if(the_brain.pickup.data)
//                    {
//                        the_brain.state = ST_ARM_PICK;
//                    }
//                    else the_brain.state = ST_EXPLORE;

                }
                else
                {
                    ROS_INFO("_brain_: CALL FOR OBJECT IDENTIFICATION FAILED.");
                    the_brain.state = ST_EXPLORE;
                }
                break;

        }//END SWITCH CASE



    }//END WHILE OK

    return 0;
}

