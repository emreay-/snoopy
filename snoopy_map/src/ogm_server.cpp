//
//  ogm_server.cpp
//
//
//  Created by Emre Ay on 17/11/16.
//
//

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int16.h>
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
#include "snoopy_map/ogm_info.h"
#include "snoopy_map/ogm_get_cell.h"


using namespace ros;
using namespace std;

class occupancyGrid
{
public:
    occupancyGrid(double x_s, double y_s, double cell_s, std::vector<double>& segmentX, std::vector<double>& segmentY);
    ~occupancyGrid();
    NodeHandle n, n_priv;
    double x_size, y_size, cell_size; //[meters]
    int grid_x_size; //rows of the grid
    int grid_y_size; //cols of the grid
    vector< vector<int> > Grid;
    nav_msgs::OccupancyGrid convertToROS();
    string _frame_id;
    int _header_seq;
    double _map_yaw;
    geometry_msgs::Pose map_origin;
    int _cell_occ_value;
    double _dilate_radius;
private:
    ServiceServer update_server, getter_server, cell_getter_server, info_server;
    bool updater(snoopy_map::ogm_update::Request &req, snoopy_map::ogm_update::Response &res);
    bool getter(snoopy_map::ogm_get::Request &req, snoopy_map::ogm_get::Response &res);
    bool info(snoopy_map::ogm_info::Request &req, snoopy_map::ogm_info::Response &res);
    bool cell_getter(snoopy_map::ogm_get_cell::Request &req, snoopy_map::ogm_get_cell::Response &res);
};

occupancyGrid::occupancyGrid(double x_s, double y_s, double cell_s, std::vector<double>& segmentX, std::vector<double>& segmentY)
    : n_priv("~")
{
    ROS_INFO("_ogm_server_: I AM AT CONSTRUCTOR...");
    x_size = x_s;
    y_size = y_s;
    cell_size = cell_s;

    grid_x_size = ceil(x_size/cell_size)+1; //cells
    x_size = grid_x_size*(cell_size);  //readjust the x_size of the map
    grid_y_size = ceil(y_size/cell_size)+1; //cells
    y_size = grid_y_size*(cell_size);  //readjust the width of the map

    ROS_INFO("_ogm_server_: NUMBER OF X in GRID:   %d", grid_x_size);
    ROS_INFO("_ogm_server_: NUMBER OF Y in GRID:   %d", grid_y_size);
    ROS_INFO("_ogm_server_: ADJUSTED MAP X    %f", x_size);
    ROS_INFO("_ogm_server_: ADJUSTED MAP Y    %f", y_size);
    //Resize the grid
    Grid.resize(grid_x_size);
    for (int i = 0; i < grid_x_size; i++) { Grid[i].resize(grid_y_size); }
    //Initialize the grid with zeros
    for (size_t i = 0; i < grid_x_size; i++)
    {
        for (size_t j = 0; j < grid_y_size; j++) { Grid[i][j] = 0;} //ROS_INFO("INIT GRID AT %d %d",i,j);}
    }

    map_origin.position.z = 0;
    //collect parameter values from the server
    n_priv.getParam("frame_id",_frame_id);
    n_priv.getParam("header_seq",_header_seq);
    n_priv.getParam("map_origin_x",map_origin.position.x);
    n_priv.getParam("map_origin_y",map_origin.position.y);
    n_priv.getParam("map_yaw",_map_yaw);
    n_priv.getParam("cell_occ_value",_cell_occ_value);
    n_priv.getParam("dilate_radius",_dilate_radius);

    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(_map_yaw), map_origin.orientation);

    int numberOfSegments = int(segmentX.size())/2;
    int dilate_dist = floor((100*_dilate_radius)/(100*cell_size));
//    ROS_INFO("_ogm_server_: Dilate Radius %f Cell Size %f Dilate Dist %f=%d",_dilate_radius,cell_size,_dilate_radius/cell_size,dilate_dist);

    for (int i = 0; i < numberOfSegments; i++)  //for each segment
    {
        //ROS_INFO("NUMBER OF SEGMENT %d IN TOTAL %d SEGMENTS",i+1,numberOfSegments);
        double x1 = segmentX[2*i]/cell_size;
        double x2 = segmentX[2*i+1]/cell_size;
        double y1 = segmentY[2*i]/cell_size;
        double y2 = segmentY[2*i+1]/cell_size;
        //ROS_INFO("x1 = %f y1 = %f x2 = %f y2 = %f", segmentX[i], segmentY[i], segmentX[i+1], segmentY[i+1]);

        // _______________ Bresenham's line algorithm ___________________
        // Code Source = http://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B

        bool steep = (fabs(y2 - y1) > fabs(x2 - x1));

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
        double dy = fabs(y2 - y1);

        double error = dx / 2.0f;
        int ystep = (y1 < y2) ? 1 : -1;
        int y = (int)y1;

        int maxX = (int)x2;

        for(int x=(int)x1; x<=maxX; x++)
        {
//            ROS_INFO("_ogm_server_: Dilate Distance [cells] %d",dilate_dist);
            int count = 0;
            if(steep)
            {
                Grid[y][x] = _cell_occ_value;
                count--;
                //ROS_INFO("STEEP Occupy cell at x,y: %d %d",y,x);                
                //Dilate the occupied cells -- Converted for STEEP case
                for (int x_dilate = y - dilate_dist; x_dilate <= y + dilate_dist; x_dilate++)
                {
                    for (int y_dilate = x - dilate_dist; y_dilate <= x + dilate_dist; y_dilate++)
                    {                                                
                        count++;
                        if( (0 <= x_dilate) && (x_dilate < grid_x_size) && (0 <= y_dilate) && (y_dilate < grid_y_size))
                            Grid[x_dilate][y_dilate] = _cell_occ_value;                        
                    }
                }
            }
            else
            {
                Grid[x][y] = _cell_occ_value;
                //ROS_INFO("Occupy cell at x,y: %d %d",x,y);
                count--;
                //Dilate the occupied cells
                for (int x_dilate = x - dilate_dist; x_dilate <= x + dilate_dist; x_dilate++)
                {
                    for (int y_dilate = y - dilate_dist; y_dilate <= y + dilate_dist; y_dilate++)
                    {
                        count++;
                        if( (0 <= x_dilate) && (x_dilate < grid_x_size) && (0 <= y_dilate) && (y_dilate < grid_y_size))
                            Grid[x_dilate][y_dilate] = _cell_occ_value;
                    }
                }
            }

//            ROS_INFO("_ogm_server_: Dilation count %d at cell %d",count,x);
            error -= dy;
            if(error < 0)
            {
                y += ystep;
                error += dx;
            }
        }
        // _______________ End Bresenham's line algorithm ___________________

    }//end for every segment

    update_server = n.advertiseService("ogm_update",&occupancyGrid::updater,this);
    getter_server = n.advertiseService("ogm_get",&occupancyGrid::getter,this);
    cell_getter_server = n.advertiseService("ogm_get_cell",&occupancyGrid::cell_getter,this);
    info_server = n.advertiseService("ogm_info",&occupancyGrid::info,this);
}

occupancyGrid::~occupancyGrid()
{
    ROS_INFO("_ogm_server_: Destructor");
}

nav_msgs::OccupancyGrid occupancyGrid::convertToROS()
{
    //ROS_INFO("_ogm_server_: I AM AT CONVERTION...");
    nav_msgs::OccupancyGrid result;
    result.header.seq = _header_seq;
    result.header.stamp = Time();
    result.header.frame_id = _frame_id;
    result.info.map_load_time = Time();
    result.info.resolution = cell_size; //meters/cell
    result.info.width = grid_x_size; //cells
    result.info.height = grid_y_size; //cells
    result.info.origin = map_origin;

    for(size_t i = 0; i < grid_y_size; i++)
    {
        for(size_t j = 0; j < grid_x_size; j++)
        {
            result.data.push_back(Grid[j][i]);
        }
    }

//    for (int x = 0; x < grid_x_size; x++)
//    {
//        for (int y = 0; y < grid_y_size; y++)
//        {
//            result.data.push_back(Grid[y][x]);
//        }
//    }

    return result;
}

bool occupancyGrid::getter(snoopy_map::ogm_get::Request &req, snoopy_map::ogm_get::Response &res)
{
    ROS_INFO("_ogm_server_: CALL FOR GETTER SERVER");
    res.map = convertToROS();
    res.cell_occ_value.data = _cell_occ_value;
    return true;
}

bool occupancyGrid::updater(snoopy_map::ogm_update::Request &req, snoopy_map::ogm_update::Response &res)
{
    ROS_INFO("_ogm_server_: CALL FOR UPDATER SERVER");
    int update_cell_x = round(req.update_cell.x/cell_size);
    int update_cell_y = round(req.update_cell.y/cell_size);    

    //check if the cell to be updated is within range
    if( (0 <= update_cell_x) && (update_cell_x < grid_x_size) &&
        (0 <= update_cell_y) && (update_cell_y < grid_y_size))
    {
        //check the update command
        if(req.cmd.data == "FILL")
        {
            Grid[update_cell_x][update_cell_y] = _cell_occ_value;
        }
        else if(req.cmd.data == "CLEAR")
        {
            Grid[update_cell_x][update_cell_y] = 0;
        }
        else if(req.cmd.data == "VOTE_UP")
        {
            Grid[update_cell_x][update_cell_y] += 10;
            if(Grid[update_cell_x][update_cell_y] >= _cell_occ_value/2)
                Grid[update_cell_x][update_cell_y] = _cell_occ_value;
        }
        else if(req.cmd.data == "VOTE_DOWN")
        {
            Grid[update_cell_x][update_cell_y] -= 10;
            if(Grid[update_cell_x][update_cell_y] <= _cell_occ_value/2)
                Grid[update_cell_x][update_cell_y] = 0;
        }
        else { ROS_INFO("_ogm_server_: UNDEFINED OGM COMMAND"); return false; }

        return true;
    }
    else
    {
        ROS_INFO("_ogm_server_: REQUESTED CELL IS NOT WITHIN THE RANGE");
        return false;
    }

}

bool occupancyGrid::info(snoopy_map::ogm_info::Request &req, snoopy_map::ogm_info::Response &res)
{
    ROS_INFO("_ogm_server_: CALL FOR INFO SERVER");
    res.cell_occ_value.data = _cell_occ_value;
    res.cell_size.data = cell_size;
    res.dilate_radius.data = _dilate_radius;
    res.x_size.data = grid_x_size;
    res.y_size.data = grid_y_size;
    return true;
}

bool occupancyGrid::cell_getter(snoopy_map::ogm_get_cell::Request &req, snoopy_map::ogm_get_cell::Response &res)
{
    ROS_INFO("_ogm_server_: CALL FOR CELL GETTER SERVER");
    int x = req.cell.x,
        y = req.cell.y;

    if(0 <= x && x < grid_x_size && 0 <= y && y <= grid_y_size)
    {
        res.value.data = Grid[x][y];
        return true;
    }
    else
    {
        ROS_ERROR("_ogm_server_: CELL GETTER: REQUESTED CELL NOT WITHIN THE MAP");
        return false;
    }
}

void readMapFile(string file_name, vector<double>& segmentX, vector<double>& segmentY)
{
    ROS_INFO("_ogm_server_: I AM AT MAP READING...");
    ifstream map_fs; map_fs.open(file_name.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("_ogm_server_: Could not read maze map from "<<file_name<<". Please double check that the file exists. Aborting.");
    }
    string line;

    while (getline(map_fs, line))
    {        
        if (line[0] == '#') {
            // comment -> skip
            continue;
        }

        double max_num = std::numeric_limits<double>::max();
        double x1 = max_num,
               x2 = max_num,
               y1 = max_num,
               y2 = max_num;

        std::istringstream line_stream(line);
        line_stream >> x1 >> y1 >> x2 >> y2;

        //ROS_INFO("x1 y1 x2 y2 :  %f %f %f %f  ", x1,y1,x2,y2);

        if ((x1 == max_num) || (x2 == max_num) || (y1 == max_num) || (y2 == max_num))
        {
            ROS_WARN("_ogm_server_: Format not matching or segment coordinates are not found. Skipping line: %s",line.c_str());
        // angle and distance
        }
        else if ((x1 != max_num) && (x2 != max_num) && (y1 != max_num) && (y2 != max_num))
        {
            //ROS_INFO("I ENTERED PUSH BACK");
            segmentX.push_back(x1);
            segmentX.push_back(x2);
            segmentY.push_back(y1);
            segmentY.push_back(y2);
        }
    }//end while

}

void writeGridToFile(occupancyGrid gridMap, string output_file)
{
    ofstream file(output_file.data());
      if (file.is_open())
      {
          for (size_t i = 0; i < gridMap.grid_x_size; i++)
          {
              for (size_t j = 0; j < gridMap.grid_y_size; j++)
              {
                  file << gridMap.Grid[i][j];
                  //ROS_INFO("Write to the file");
              }
              file << "\n";
          }

        file.close();
      }
      else ROS_INFO("Cant open file.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "occupany_grid_node");
    ros::NodeHandle n("~"), nh;
    ros::Rate r(5);
    string _map_topic;
    n.getParam("grid_map_topic",_map_topic);
    Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>(_map_topic,10);

    //Read from map file and extract the segment end point coordinates
    string _map_file, _output_file;
    n.param<string>("map_file", _map_file, "maze_map.txt");
    n.param<string>("output_file",_output_file,"out_map.txt");
    vector<double> segmentX, segmentY;
    readMapFile(_map_file, segmentX, segmentY);
    //Get maximums of x and y
    double max_x = 0, max_y = 0 , elem_x = 0, elem_y = 0;
    for(size_t i = 0; i < segmentX.size(); i++)
    {
        elem_x = segmentX[i];
        if(elem_x > max_x) max_x = elem_x;
        elem_y = segmentY[i];
        if(elem_y > max_y) max_y = elem_y;
    }

//    ROS_INFO("_ogm_server_: MAX ELEMENT OF SEG X:    %f", max_x);
//    ROS_INFO("_ogm_server_: MAX ELEMENT OF SEG Y:    %f", max_y);

    double x_size = max_x;
    double y_size = max_y;

//    ROS_INFO("_ogm_server_: X SIZE %f",x_size);
//    ROS_INFO("_ogm_server_: Y SIZE %f",y_size);

    //Get cell size parameter from the server
    double _cell_size; //in meters
    n.getParam("cell_size",_cell_size);
//    ROS_INFO("_ogm_server_: CELL SIZE:   %f", _cell_size);

    /*
    for(size_t i = 0; i < segmentX.size(); i++)
    {
        ROS_INFO("SEGMENT %d __ x1: %f y1: %f x2: %f y2: %f", i, segmentX[i], segmentY[i], segmentX[i+1], segmentY[i+1]);
    }
    */

    //__________________________OGM OBJECT_______________________________
    occupancyGrid gridMap(x_size, y_size, _cell_size, segmentX, segmentY);
    //Write the grid as a txt file
    writeGridToFile(gridMap,_output_file);    

    while(ok())
    {
        spinOnce();
        //convert the grid to a ros msg type
        nav_msgs::OccupancyGrid ros_grid = gridMap.convertToROS();
        grid_pub.publish(ros_grid);
        r.sleep();
    }
    return 0;

}
