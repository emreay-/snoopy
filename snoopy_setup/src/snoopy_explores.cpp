#include <time.h>
#include <ros/ros.h>
#include <fstream>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include <boost/random/mersenne_twister.hpp> //rand number generator
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>
//services
#include "snoopy_map/ogm_get.h"
#include "snoopy_setup/explore.h"

using namespace ros;
using namespace std;
typedef vector< vector<int> > map_type;


class explore
{
public:
    explore();
    ~explore();
    NodeHandle n, n_priv;
    void map_update();
    Publisher explored_pub;
    //ogm get vars
    ServiceClient ogm_client;
    snoopy_map::ogm_get ogm_get_;
    ServiceServer explore_server;
    boost::random::mt19937 num_gen; //rand number generator
    //map vars
    map_type explore_map;
    int map_x_size, map_y_size, cell_occ_value, cell_visited_value;
    double map_cell_size;
    void mapParser(nav_msgs::OccupancyGrid &msg);
    geometry_msgs::Pose2D pose, prev_pose;
    double _significant_change;
    nav_msgs::GridCells explored_cells;
    //laser vars
    double angle_min, angle_max, angle_increment;
    vector<double> ranges, avg_ranges, sorted_avg_ranges;
    //explore regions
    double _x_explore, _y_explore, _explore_boundary;
    vector<geometry_msgs::Pose2D> nodes;
    //bool flags
    bool isPoseMsgReceived;
    bool isMapMsgReceived;
    bool significantPoseChange;
    bool isLaserMsgReceived;

private:
    Subscriber pose_sub;    
    geometry_msgs::Pose2D goal;
    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& input);
    bool explore_cb(snoopy_setup::explore::Request &req, snoopy_setup::explore::Response &res);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    double averageRanges(int angle, int interval);

};

explore::explore() : n_priv("~")
{
    ROS_INFO("_EXPLORE_: Constructor");
    num_gen.seed(time(0));

    n_priv.param("x_explore", _x_explore, 1.0);
    n_priv.param("y_explore", _y_explore, 1.0);
    n_priv.param("explore_boundary", _explore_boundary, 2.0);
    n_priv.param("significant_change", _significant_change, 0.30);

    explored_pub = n.advertise<nav_msgs::GridCells>("/snoopy/explored_cells",10);
    pose_sub = n.subscribe<geometry_msgs::Pose2D>("/snoopy/pose_fusion",100,&explore::poseCallback, this);
    explore_server = n.advertiseService("explore",&explore::explore_cb,this);
    ogm_client = n.serviceClient<snoopy_map::ogm_get>("ogm_get");

    explored_cells.header.frame_id = "odom";
    explored_cells.header.stamp = Time();
    cell_visited_value = -100;
    isPoseMsgReceived = false;
    isMapMsgReceived = false;
    significantPoseChange = false;
    pose.x = 0.0;
    pose.y = 0.0;
    prev_pose.x = 0.0;
    prev_pose.x = 0.0;
}

explore::~explore()
{
    ROS_INFO("_EXPLORE_: Destructor");
}

//____________________________________________________________________________
void explore::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    angle_max = msg->angle_max;
    angle_min = msg->angle_min;
    angle_increment = msg->angle_increment;

    for(size_t i = 0; i < msg->ranges.size(); i++)
    {
        if(msg->ranges[i] != std::numeric_limits<double>::infinity() && !isnan(msg->ranges[i]))
            ranges[i] = msg->ranges[i];
    }
    //calculate average range every 10 degrees
    for(int i = 0; i < ranges.size(); i += 10)
    {
        sorted_avg_ranges[i] = averageRanges(i,15);
    }
    avg_ranges = sorted_avg_ranges;
    sort(sorted_avg_ranges.begin(),sorted_avg_ranges.end());
    isLaserMsgReceived = true;
}

double explore::averageRanges(int angle, int interval)
{
    if (angle - interval < 0) angle = ranges.size() + (angle - interval);
    else angle = angle - interval;

    double sum;
    double count = 0;
    double range_avg = 0;
    //sum around its neighbor
    for (int t = 0; t < 2*interval+1; t++)
    {
        if (ranges[angle] != std::numeric_limits<double>::infinity() && !isnan(ranges[angle]))
        {
            count++;
            sum += ranges[angle];
            if (angle == ranges.size() - 1) angle = 0;
            else angle++;
         }
    }

    if (count > 0) range_avg = sum/count;
    else range_avg = std::numeric_limits<double>::infinity();

    return range_avg;
}
//____________________________________________________________________________________
bool explore::explore_cb(snoopy_setup::explore::Request &req, snoopy_setup::explore::Response &res)
{
    spinOnce();
    if(isMapMsgReceived && isPoseMsgReceived)
    {
        ROS_INFO("_EXPLORE_: CALL TO EXPLORE, MAP AND POSE ARE RECEIVED.");
        int robot_x = round(pose.x/map_cell_size);
        int robot_y = round(pose.y/map_cell_size);
        int x_explore = round(_x_explore/map_cell_size);
        int y_explore = round(_y_explore/map_cell_size);
        int explore_boundary = round(_explore_boundary/map_cell_size);
        boost::random::uniform_int_distribution<> uniform_x(robot_x - explore_boundary,robot_x + explore_boundary);
        boost::random::uniform_int_distribution<> uniform_y(robot_y - explore_boundary,robot_y + explore_boundary);

        geometry_msgs::Pose2D temp, possible;
        bool validGoalFound = false;

        while(!validGoalFound)
        {
//            //_______________________comment from here________________________________________
//            bool ValidGoalFoundtemp = false;

//            double angle = 0;
//            int count = 0;
//            for(int i = sorted_avg_ranges.size();i > sorted_avg_ranges.size(); i--)
//            {

//                for(int j = 0; j < avg_ranges.size();j++)
//                {
//                    if(sorted_avg_ranges[i]==avg_ranges[j])
//                    {
//                        angle = j*30;
//                        break;
//                    }
//                }
//                //check if the place has largest laser scan has been visited or not
//                boost::random::uniform_real_distribution<> uniform_x(0,pose.x + cos(-pose.theta+angle) * min(sorted_avg_ranges[i], _explore_boundary));
//                boost::random::uniform_real_distribution<> uniform_y(0,pose.y + sin(-pose.theta+angle) * min(sorted_avg_ranges[i], _explore_boundary));

//                possible.x = uniform_x(num_gen);
//                possible.y = uniform_y(num_gen);

//                int goal_x = round(possible.x/map_cell_size);
//                int goal_y = round(possible.y/map_cell_size);

//                // assign next goal as the first found qualified candidate
//                if(explore_map[goal_x][goal_y] != cell_occ_value &&
//                      explore_map[goal_x][goal_y] != cell_visited_value &&
//                        !ValidGoalFoundtemp)
//                {
//                    ROS_INFO("__EXPLORE_: A GOAL HAS BEEN FOUND: [%f, %f]", temp.x, temp.y);
//                    temp = possible;
//                    ValidGoalFoundtemp = true;
//                }
//                //if the cell is not occupied and not visited
//                if(explore_map[goal_x][goal_y] != cell_occ_value &&
//                   explore_map[goal_x][goal_y] != cell_visited_value)
//                {
//                   count ++;
//                }
//                //if all the cell has been visited, go to the last node, and pop the last node
//                else
//                {
//                    temp.x = nodes.back().x;
//                    temp.y = nodes.back().y;
//                    nodes.pop_back();
//                    ROS_INFO("_EXPLORE_: GO BACK TO LAST NODE x,y: %f,%f",temp.x,temp.y);
//                    ValidGoalFoundtemp = true;
//                }

//            }
//            if(count > 1)
//            {
//                bool add_node = false;
//                //calculate the distance from all the nodes
//                for(int j=0; j<nodes.size(); j++)
//                {
//                    double distance = sqrt(pow((nodes[j].x - temp.x),2) + pow((nodes[j].y - temp.y),2));
//                    if(distance < 0.4)
//                    {
//                        add_node = false;
//                        break;
//                    }
//                    else
//                    {
//                        add_node = true;
//                    }
//                }
//                if(add_node)
//                {
//                    geometry_msgs::Pose2D node;
//                    node.x = pose.x;
//                    node.y = pose.y;
//                    nodes.push_back(node);
//                    ROS_INFO("__EXPLORE_: A NODE HAS BEEN FOUND: [%f, %f]", node.x, node.y);
//                }

//            }
//            validGoalFound = ValidGoalFoundtemp;

//            //_____________________comment end here__________________________________________
                int goal_x = uniform_x(num_gen),
                    goal_y = uniform_y(num_gen);

    //            ROS_INFO("_EXPLORE_: RAND x,y: %d,%d",goal_x,goal_y);

                if(fabs(goal_x - robot_x) >= _x_explore && fabs(goal_y - robot_y) >= _y_explore)
                {
                    if(0 <= goal_x && goal_x < map_x_size && 0 <= goal_y && goal_y < map_y_size)
                    {
                        if(explore_map[goal_x][goal_y] != cell_occ_value &&
                           explore_map[goal_x][goal_y] != cell_visited_value)
                        {
                            temp.x = goal_x * map_cell_size;
                            temp.y = goal_y * map_cell_size;
                            ROS_INFO("_EXPLORE_: PICKED GOAL x,y: %f,%f",temp.x,temp.y);
                            validGoalFound = true;
                        }
                    }
                }

        }
        res.goal = temp;
        ROS_INFO("_EXPLORE_: CALL RETURNED SUCCESSFULLY.");
        return true;
    }
    else
    {
        ROS_ERROR("_EXPLORE_: MAP AND/OR POSE INFO NOT YET RETRIEVED.");
        return false;
    }
}

void explore::mapParser(nav_msgs::OccupancyGrid &msg)
{
    int map_x = msg.info.width;
    int map_y = msg.info.height;
    map_x_size = map_x;
    map_y_size = map_y;
    map_cell_size = msg.info.resolution;
    //resize the map
    explore_map.resize(map_x);
    for (size_t i = 0; i < map_x; i++) { explore_map[i].resize(map_y); }
    for(size_t y = 0; y < map_y; y++)
    {
        for(size_t x = 0; x < map_x; x++)
        {
            //parsing the ROS map into our map
            explore_map[x][y] = msg.data[map_x*y + x];
        }
     }
    //print map
//    for(size_t x = 0; x < map_x; x++)
//    {
//        std::string line;
//        for(size_t y = 0; y < map_y; y++)
//        {
//            stringstream ss;
//            if(explore_map[x][y] == 100) ss << "1";
//            else ss << explore_map[x][y];

//            line.append(ss.str());
//        }
//        ROS_INFO("%s",line.data());
//    }

}

void explore::map_update()
{
    if(significantPoseChange)
    {
        int robot_x = round(pose.x/map_cell_size);
        int robot_y = round(pose.y/map_cell_size);
        int sign = (_significant_change/(map_cell_size));

        for(int visit_x = robot_x - sign; visit_x < robot_x + sign; visit_x++)
        {
            for(int visit_y = robot_y - sign; visit_y < robot_y + sign; visit_y++)
            {
                if(0 <= visit_x && visit_x < map_x_size && 0 <= visit_y && visit_y < map_y_size
                   && explore_map[visit_x][visit_y] != cell_occ_value)
                {
                    explore_map[visit_x][visit_y] = cell_visited_value;
                    geometry_msgs::Point p;
                    p.x = visit_x*map_cell_size;
                    p.y = visit_y*map_cell_size;
                    p.z = 0.0;
                    explored_cells.cells.push_back(p);
                }
            }
        }
    }

}

void explore::poseCallback(const geometry_msgs::Pose2D::ConstPtr& input)
{
    pose.x = input->x;
    pose.y = input->y;
    pose.theta = input->theta;
    isPoseMsgReceived = true;

    if(fabs(pose.x - prev_pose.x) > _significant_change ||
       fabs(pose.y - prev_pose.y) > _significant_change)
    {
        prev_pose = pose;
        significantPoseChange = true;
    }
    else significantPoseChange = false;

}

int main(int argc, char** argv)
{
    init(argc, argv, "snoopy_explores");
    explore exp;

    //call the service and get the map from server
    nav_msgs::OccupancyGrid m;

    while(!exp.isMapMsgReceived)
    {
        if(exp.ogm_client.call(exp.ogm_get_))
        {
            m = exp.ogm_get_.response.map;
            exp.mapParser(m);
            exp.cell_occ_value = exp.ogm_get_.response.cell_occ_value.data;

            exp.explored_cells.cell_height = exp.map_cell_size;
            exp.explored_cells.cell_width = exp.map_cell_size;

            exp.isMapMsgReceived = true;
        }
        else
        {
            ROS_ERROR("_EXPLORE_: OGM GET CALL FAILED");
        }
    }

    while(ok())
    {
        spinOnce();
        exp.map_update();
        exp.explored_pub.publish(exp.explored_cells);
    }

	return 0;
}
