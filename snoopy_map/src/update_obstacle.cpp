//
//  update_obstacle.cpp
//
//
//  Created by Emre Ay on 26/11/16.
//
//

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <fstream>
#include <limits>
#include <math.h>
#include <iostream>
#include "snoopy_map/ogm_get.h"
#include "snoopy_map/ogm_update.h"
#include "snoopy_map/ogm_get_cell.h"
#include "snoopy_map/ogm_info.h"
#include "snoopy_localize/get_pose.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>


using namespace ros;
using namespace std;
typedef vector< vector<int> > map_type;
typedef pair<int,int> coord;

vector<double> ranges;
double map_cell_size, map_dilation, angle_min, angle_inc;
int map_x_lim, map_y_lim, cell_occ_value;
bool isLaserMsgReceived = false;
bool isPoseMsgReceived = false;
ServiceClient ogm_get_cell_client;
geometry_msgs::Pose2D pose;
double robot_v, robot_w;

void pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    pose.x = msg->x;
    pose.y = msg->y;
    pose.theta = msg->theta;
    isPoseMsgReceived = true;
}

void twist_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    robot_v = msg->linear.x;
    robot_w = msg->angular.z;
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{    
    angle_min = msg->angle_min;
    angle_inc = msg->angle_increment;

    for(size_t i = 0; i < msg->ranges.size(); i++)
    {
        if(msg->ranges[i] != std::numeric_limits<double>::infinity() && !isnan(msg->ranges[i]))
            ranges[i] = msg->ranges[i];
    }
    isLaserMsgReceived = true;
}

vector<coord> bresenham(coord c1, coord c2)
{
    int x1 = c1.first,
        y1 = c1.second,
        x2 = c2.first,
        y2 = c2.second;

    vector<coord> result;

    // _______________ Bresenham's line algorithm ___________________
    // Code Source = http://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B

    bool steep = (abs(y2 - y1) > abs(x2 - x1));

    if(steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    double dx = x2 - x1;
    double dy = abs(y2 - y1);

    double error = dx / 2.0f;
    int ystep = (y1 < y2) ? 1 : -1;
    int y = y1;

    int maxX = x2;

    for(int x = x1; x <= maxX; x++)
    {
        if(steep) result.push_back(make_pair(y,x));
        else      result.push_back(make_pair(x,y));

        error -= dy;
        if(error < 0)
        {
            y += ystep;
            error += dx;
        }
    }

    return result;
}

map_type mapParser(nav_msgs::OccupancyGrid &msg)
{
    map_type map;
    //based on ogm_server.cpp:
    int map_x = msg.info.width;
    int map_y = msg.info.height;
    map_x_lim = map_x;
    map_y_lim = map_y;
    map_cell_size = msg.info.resolution;
    //resize the map
    map.resize(map_x);
    for (size_t i = 0; i < map_x; i++) { map[i].resize(map_y); }
    for(size_t y = 0; y < map_y; y++)
    {
        for(size_t x = 0; x < map_x; x++)
        {
            //parsing the ROS map into our map
            map[x][y] = msg.data[map_x*y + x];
        }
     }
    //print map
//    for(size_t x = 0; x < map_x; x++)
//    {
//        std::string line;
//        for(size_t y = 0; y < map_y; y++)
//        {
//            stringstream ss;
//            if(map[x][y] == 100) ss << "1";
//            else ss << map[x][y];

//            line.append(ss.str());
//        }
//        ROS_INFO("%s",line.data());
//    }

    return map;
}

int cellValueAt(int x, int y)
{
    int cell_value = 0;
    snoopy_map::ogm_get_cell ogm_get_cell_;
    ogm_get_cell_.request.cell.x = x;
    ogm_get_cell_.request.cell.y = y;
    if(ogm_get_cell_client.call(ogm_get_cell_)) cell_value = ogm_get_cell_.response.value.data;
    return cell_value;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "update_obstacle");
    NodeHandle n, n_priv("~");
    //get parameter
    string _scan_topic;
    n_priv.param<string>("scan_topic",_scan_topic,"/scan");
    double _check_radius;
    n_priv.param("check_radius",_check_radius,0.3);
//    int _cell_occ_value;
//    n_priv.param("cell_occ_value",_cell_occ_value,100);
    double _range_min;
    n_priv.param("range_min",_range_min,0.2);
    //subscriber for lidar data
    Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>(_scan_topic,5,scan_callback);
    Subscriber pose_sub = n.subscribe<geometry_msgs::Pose2D>("/snoopy/pose_fusion",5,pose_callback);
    Subscriber twist_sub = n.subscribe<geometry_msgs::Twist>("/snoopy/cmd_vel",5,twist_callback);
    //clients for get map, update map and get pose
    ogm_get_cell_client = n.serviceClient<snoopy_map::ogm_get_cell>("ogm_get_cell");
    ServiceClient ogm_info_client = n.serviceClient<snoopy_map::ogm_info>("ogm_info");
    ServiceClient ogm_update_client = n.serviceClient<snoopy_map::ogm_update>("ogm_update");
    ServiceClient ogm_get_client = n.serviceClient<snoopy_map::ogm_get>("ogm_get");
    //serive objects
    snoopy_map::ogm_update ogm_update_;
    snoopy_map::ogm_info ogm_info_;
    snoopy_map::ogm_get ogm_get_;
    //map container
    //map_type grid;
    int map_x, map_y;
    //listener
    tf::TransformListener listener;
    //resize ranges
    ranges.resize(360,0);

    Rate loop(10);
    sleep(5);

    //get the map info
    if(ogm_info_client.call(ogm_info_))
    {
        cell_occ_value = ogm_info_.response.cell_occ_value.data;
        map_x_lim = ogm_info_.response.x_size.data;
        map_y_lim = ogm_info_.response.y_size.data;
        map_cell_size = ogm_info_.response.cell_size.data;
        map_dilation = ogm_info_.response.dilate_radius.data;
    }
    else { ROS_INFO("_update_obstacles: ogm_get CALL FAILED."); }

    while(ok())
    {
        robot_v = 0.0;
        robot_w = 0.0;

        spinOnce();
        map_type map;
        if(ogm_get_client.call(ogm_get_)){map = mapParser(ogm_get_.response.map);}
        else { ROS_INFO("_update_obstacles: ogm_get CALL FAILED."); }


//        if(isLaserMsgReceived && isPoseMsgReceived) //check if cb data received
        if(isLaserMsgReceived && robot_v == 0 && robot_w == 0) //check if cb data received
//        if(isLaserMsgReceived) //check if cb data received
        {
            //check for obstacles in radius
            for(size_t i = 0; i < ranges.size(); i++)
            {
                double RANGE = ranges[i];

                //for the rays that hit something within the check radius
                if(_range_min <= RANGE && RANGE <= _check_radius)
                {
                    ROS_INFO("Angle in radius: %d",int(i));
                    listener.waitForTransform("/odom","/laser_frame",Time(0),Duration(0.01));
                    geometry_msgs::PointStamped range_pt;
                    range_pt.header.frame_id = "/laser_frame";
                    range_pt.header.stamp = Time();

                    double angle = angle_min + i*angle_inc;
                    range_pt.point.x = RANGE*cos(angle);
                    range_pt.point.y = RANGE*sin(angle);
                    range_pt.point.z = 0.0;
                    try
                    {
                        geometry_msgs::PointStamped converted_pt;
                        listener.transformPoint("/odom",range_pt,converted_pt);
                        map_x = round(converted_pt.point.x/map_cell_size);
                        map_y = round(converted_pt.point.y/map_cell_size);

                    }
                    catch(tf::TransformException& ex)
                    {
                        ROS_ERROR("Exception received: %s",ex.what());
                    }

                    //check if the point lies within the map
                    if( (0 <= map_x) && (map_x < map_x_lim) && (0 <= map_y) && (map_y < map_y_lim) )
                    {                        
                        //______________ UPDATE RAY HIT POINT _______________
//                        int cell_value = cellValueAt(map_x,map_y);
//                        if(cell_value != cell_occ_value)
                        if(map[map_x][map_y] != cell_occ_value)
                        {
                            ROS_INFO("VOTE UP");
                            ogm_update_.request.update_cell.x = map_x*map_cell_size;
                            ogm_update_.request.update_cell.y = map_y*map_cell_size;
                            ogm_update_.request.cmd.data = "VOTE_UP";
                            //call server for update ogm
                            if(ogm_update_client.call(ogm_update_)) { ROS_INFO("_update_obstacles: ogm_update CALL SUCCESSFUL."); }
                            else { ROS_INFO("_update_obstacles: ogm_update CALL FAILED."); }
                        }

//                        //______________ UPDATE POINTS BEFORE RAY HIT POINT _______________
//                        coord robot = make_pair(round(pose.x/map_cell_size),round(pose.y/map_cell_size));
//                        coord ray_hit = make_pair(map_x,map_y);
//                        double theta = atan2(ray_hit.second - robot.second, ray_hit.first - robot.first);
//                        int dilated_cells = ceil(map_dilation/map_cell_size);
//                        coord ray_hit_dilated = make_pair(ray_hit.first - ceil(cos(theta)*dilated_cells),ray_hit.second - ceil(sin(theta)*dilated_cells));
//                        vector<coord> ray_line = bresenham(robot,ray_hit_dilated);

//                        for(size_t j = 0; j < ray_line.size(); j++)
//                        {
//                            coord cell = ray_line[j];
//                            int value = cellValueAt(cell.first,cell.second);

//                            if(value != 0)
//                            {
//                                ogm_update_.request.update_cell.x = cell.first*map_cell_size;
//                                ogm_update_.request.update_cell.y = cell.second*map_cell_size;
//                                ogm_update_.request.cmd.data = "VOTE_DOWN";
//                                //call server for update ogm
//                                if(ogm_update_client.call(ogm_update_)){} //ROS_INFO("_update_obstacles: ogm_update CALL SUCCESSFUL."); }
//                                else { ROS_INFO("_update_obstacles: ogm_update CALL FAILED."); }
//                            }
//                        }
                    }
                }//end if ray within check radius
                /*
                //for the rays that do not hit anything within the check radius
                else
                {
                    listener.waitForTransform("/odom","/laser_frame",Time(0),Duration(0.2));
                    geometry_msgs::PointStamped range_pt;
                    range_pt.header.frame_id = "/laser_frame";
                    range_pt.header.stamp = Time();

                    double angle = angle_min + i*angle_inc;
                    range_pt.point.x = _check_radius*cos(angle);
                    range_pt.point.y = _check_radius*sin(angle);
                    range_pt.point.z = 0.0;
                    try
                    {
                        geometry_msgs::PointStamped converted_pt;
                        listener.transformPoint("/odom",range_pt,converted_pt);
                        map_x = round(converted_pt.point.x/map_cell_size);
                        map_y = round(converted_pt.point.y/map_cell_size);

                    }
                    catch(tf::TransformException& ex)
                    {
                        ROS_ERROR("Exception received: %s",ex.what());
                    }

                    //check if the point lies within the map
                    if( (0 <= map_x) && (map_x < map_x_lim) && (0 <= map_y) && (map_y < map_y_lim) )
                    {
                        if(grid[map_x][map_y] != 0)
                        {
                            ogm_update_.request.update_cell.x = map_x*map_cell_size;
                            ogm_update_.request.update_cell.y = map_y*map_cell_size;
                            ogm_update_.request.cmd.data = "VOTE_DOWN";
                            //call server for update ogm
                            if(ogm_update_client.call(ogm_update_)) { ROS_INFO("_update_obstacles: ogm_update CALL SUCCESSFUL."); }
                            else { ROS_INFO("_update_obstacles: ogm_update CALL FAILED."); }
                        }
                    }
                }
            */
            }//end for

        }//end of if laser msg received
        loop.sleep();
    }//end of while ok

    return 0;
}

