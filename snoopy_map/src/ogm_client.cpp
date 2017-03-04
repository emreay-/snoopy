//
//  ogm_client.cpp
//
//
//  Created by Emre Ay on 17/11/16.
//
//

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <vector>
#include <fstream>
#include <limits>
#include <math.h>
#include <algorithm>    // std::min_element, std::max_element
#include <iostream>
#include "snoopy_map/ogm_get.h"
#include "snoopy_map/ogm_update.h"


using namespace ros;
using namespace std;

class ogm_client
{
public:
    ogm_client();
    ~ogm_client();
    NodeHandle n, n_priv;
    double _upd_cell_x, _upd_cell_y;
    string _upd_cmd;
    ServiceClient update_client, getter_client;
};

ogm_client::ogm_client()
    : n_priv("~")
{
    ROS_INFO("_ogm_client_: I AM AT CONSTRUCTOR...");

    //collect parameter values from the server
    n_priv.getParam("upd_cell_x",_upd_cell_x);
    n_priv.getParam("upd_cell_y",_upd_cell_y);
    //n_priv.getParam("upd_cmd",_upd_cmd);

    update_client = n.serviceClient<snoopy_map::ogm_update>("ogm_update");
    getter_client = n.serviceClient<snoopy_map::ogm_get>("ogm_get");
}

ogm_client::~ogm_client()
{
    ROS_INFO("_ogm_client_: Destructor");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ogm_client");
    ogm_client ogmClient;
    snoopy_map::ogm_update srv;
    Rate r(2);

    sleep(3);

//    srv.request.update_cell.x = ogmClient._upd_cell_x;
//    srv.request.update_cell.y = ogmClient._upd_cell_y;
//    string cmd = "FILL";
//    srv.request.cmd.data = cmd.c_str();

//    if (ogmClient.update_client.call(srv))
//    {
//        ROS_INFO("_ogm_client_: UPDATE CALLED FOR %f,%f CMD: %s",srv.request.update_cell.x,srv.request.update_cell.y,srv.request.cmd.data.c_str());
//    }
//    else
//    {
//        ROS_ERROR("_ogm_client_: UPDATE CALL FAILED");
//        return 1;
//    }

//    spin();


    while(ok())
    {
        srv.request.update_cell.x = ogmClient._upd_cell_x;
        srv.request.update_cell.y = ogmClient._upd_cell_y;
        string cmd = "FILL";
        srv.request.cmd.data = cmd.c_str();

        if (ogmClient.update_client.call(srv))
        {
            ROS_INFO("_ogm_client_: UPDATE CALLED FOR %f,%f CMD: %s",srv.request.update_cell.x,srv.request.update_cell.y,srv.request.cmd.data.c_str());
        }
        else
        {
            ROS_ERROR("_ogm_client_: UPDATE CALL FAILED");
            return 1;
        }

        sleep(1);

        cmd = "CLEAR";
        srv.request.cmd.data = cmd.c_str();

        if (ogmClient.update_client.call(srv))
        {
            ROS_INFO("_ogm_client_: UPDATE CALLED FOR %f,%f CMD: %s",srv.request.update_cell.x,srv.request.update_cell.y,srv.request.cmd.data.c_str());
        }
        else
        {
            ROS_ERROR("_ogm_client_: UPDATE CALL FAILED");
            return 1;
        }

        r.sleep();

    }

    return 0;
}
