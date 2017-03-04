#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include "snoopy_pathfind/pathfind.h"
#include "snoopy_pathfind/pathfindRequest.h"
#include "snoopy_pathfind/pathfindResponse.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

using namespace ros;

nav_msgs::OccupancyGrid grid;
geometry_msgs::Pose2D goal;
geometry_msgs::Pose2D pose;
int _cell_occ_value;
bool called = false;


void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    grid.header = msg->header;
    grid.info = msg->info;
    grid.data = msg->data;
//    ROS_INFO("GRID CALLBACK");
}

void move_base_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    called = true;
}

int main(int argc, char** argv)
{
    init(argc, argv, "astar_client");
    NodeHandle n("~"), nh;

    double pose_x;
    double pose_y;
//    double goal_x;
//    double goal_y;
    std::string _astar_service;

    n.getParam("pose_x", pose_x);
    n.getParam("pose_y", pose_y);
//    n.getParam("goal_x", goal_x);
//    n.getParam("goal_y", goal_y);
    n.getParam("astar_service",_astar_service);
    n.param("cell_occ_value",_cell_occ_value,100);
    std::string _map_topic;
    n.param<std::string>("grid_map_topic",_map_topic,"gridMap");
    Subscriber sub = nh.subscribe<nav_msgs::OccupancyGrid>(_map_topic, 10, gridCallback);
    ServiceClient client = nh.serviceClient<snoopy_pathfind::pathfind>(_astar_service);
    Subscriber move_base_sub = nh.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal",1,move_base_cb);
    Rate r(10);

    pose.x = pose_x;
    pose.y = pose_y;

    //_____________________ VISUALIZE PATH VARIABLES ____________________
    Publisher vis_pub = nh.advertise<visualization_msgs::Marker>("snoopy/path",5);
    visualization_msgs::Marker pr_marker;
    pr_marker.header.frame_id = "/odom";
    pr_marker.header.stamp = ros::Time();
    pr_marker.ns = "odom";
    pr_marker.type = visualization_msgs::Marker::LINE_STRIP;
    pr_marker.action = visualization_msgs::Marker::ADD;
    pr_marker.scale.x = 0.005;
    pr_marker.color.a = 1.0;
    pr_marker.color.r = (255.0/255.0);
    pr_marker.color.g = (0.0/255.0);
    pr_marker.color.b = (0.0/255.0);
    tf::Quaternion quat; quat.setRPY(0.0,0.0,0.0);
    tf::quaternionTFToMsg(quat, pr_marker.pose.orientation);
    //___________________________________________________________________

    sleep(2);

    while(ok())
    {
        spinOnce();
        if(called)
        {
            snoopy_pathfind::pathfind srv;
            srv.request.pose = pose;
            srv.request.goal = goal;
            srv.request.map = grid;
            srv.request.cell_occ_value.data = _cell_occ_value;

            if(client.call(srv))
            {
                pr_marker.points.clear();
                for(size_t i = 0; i < srv.response.path.poses.size(); i++)
                {
                    ROS_INFO("Path %d, x: %f y: %f",int(i),srv.response.path.poses[i].pose.position.x,srv.response.path.poses[i].pose.position.y);
                    geometry_msgs::Point p;
                    p.x = srv.response.path.poses[i].pose.position.x;
                    p.y = srv.response.path.poses[i].pose.position.y;
                    p.z = 0;
                    pr_marker.points.push_back(p);
                }
            }

            else
            {
                ROS_ERROR("F A I L...");
//                    return 1;
            }

            called = false;
            vis_pub.publish(pr_marker);
            r.sleep();
        }
    }

	return 0;
}
