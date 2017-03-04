#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <utility>
#include <string>
#include <queue>
#include <map>
#include <algorithm>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <snoopy_pathfind/pathfind.h>
#include <snoopy_pathfind/pathfindRequest.h>
#include <snoopy_pathfind/pathfindResponse.h>
#include <string>
#include <istream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace ros;
using namespace std;
using namespace cv;
bool verbose = false;
int PATH_COUNT;

typedef pair<int,int> coord;
typedef pair<int,coord> Node; //type to use in priority queue. First element location, second is priority parameter
typedef priority_queue< Node, vector<Node>, std::greater<Node> > nodeQueue; //priority queueu, sorted from lowest to highest

vector<coord> DIRECTIONS(8);
//map< coord, char > ACTIONS;
vector<coord> line;

class pathfind {

public:
    pathfind();
    ~pathfind();
    NodeHandle n, nh;
    Subscriber move_base_sub;

    //columns are x and rows are y coordinates of the map

private:
    ServiceServer server;
    //functions
    bool astar(snoopy_pathfind::pathfind::Request &req, snoopy_pathfind::pathfind::Response &res);
    string _astar_service;
    void mapParser(nav_msgs::OccupancyGrid &msg);
    vector<coord> smooth(vector<coord> path);
    bool alex_star(coord one, coord two);
    bool alex_star2(coord one, coord two);
    vector<coord> bresenham(coord c1, coord c2);
    bool checkBresenham(vector<coord> line);
    vector<coord> getNeighbours(coord loc);
    int manhattanDistance(coord nd1, coord nd2);
    int diagonalDistance(coord nd1, coord nd2);
    //variables
    vector< vector<int> > map;
    coord goal;
    coord pose;
    vector<coord> path;
    int map_x;
    int map_y;
    int cell_occ_value;
    double map_cell_size;    
    int emptyNeighbours(coord loc);
    int movementCost(coord x);
    void costMapInit();
	bool first_run;
    //costmap vars
    Mat cost_map;
    int _costmap_kernel_size;
    double _costmap_sigma;
    int _cost_gain;
};

pathfind::pathfind() : n("~")
{
    ROS_INFO("_ASTAR_: CONSTRUCTOR OF PATHFIND");
    n.param<string>("astar_service",_astar_service,"astar_srv");
    n.param("costmap_sigma",_costmap_sigma,5.0);
    n.param("costmap_kernel_size",_costmap_kernel_size,5);
    n.param("cost_gain",_cost_gain,10);
    server = nh.advertiseService(_astar_service,&pathfind::astar,this);
}


pathfind::~pathfind()
{
    ROS_INFO("_ASTAR_:DECONSTRUCTOR OF PATHFIND");
}

vector<coord> pathfind::bresenham(coord c1, coord c2)
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
        if(steep)
        {
            result.push_back(make_pair(y,x));
            for (int x_dilate = y - 1; x_dilate <= y + 1; x_dilate++)
            {
                for (int y_dilate = x - 1; y_dilate <= x + 1; y_dilate++)
                {
                    result.push_back(make_pair(x_dilate,y_dilate));
                }
            }
        }
        else
        {
            result.push_back(make_pair(x,y));
            for (int x_dilate = x - 1; x_dilate <= x + 1; x_dilate++)
            {
                for (int y_dilate = y - 1; y_dilate <= y + 1; y_dilate++)
                {
                    result.push_back(make_pair(x_dilate,y_dilate));
                }
            }
        }

        error -= dy;
        if(error < 0)
        {
            y += ystep;
            error += dx;
        }
    }

    return result;
}


bool pathfind::checkBresenham(vector<coord> line)
{
    if(line.empty())    return true;
    else
    {
        bool isOccupied = false;
        for(size_t i = 0; i < line.size(); i++)
        {
            coord check = line[i];
            if( (0 <= check.first) && (check.first < map_x) && (0 <= check.second) && (check.second < map_y))
                isOccupied = isOccupied || (map[check.first][check.second] == cell_occ_value);
        }
        return !isOccupied;
    }
}

vector<coord> pathfind::smooth(vector<coord> path)
{
    vector<coord> smoothed = path;

    for(size_t i = 1; i < smoothed.size()-1; i++)
    {
        vector<coord> connection = bresenham(smoothed[i-1],smoothed[i+1]);
        if(checkBresenham(connection))
        {
            smoothed.erase(smoothed.begin()+i);
            i--;
        }

    }
    return smoothed;
}

int pathfind::movementCost(coord x)
{
    int cost;
    cost = (int)cost_map.at<unsigned char>(x.first,x.second);
    return cost/_cost_gain;
}

void pathfind::costMapInit()
{
    ROS_INFO("_ASTAR_: COSTMAP FNC");
    Mat init_map(map.size(), map[0].size(), CV_8UC1);
    cost_map = Mat(map.size(), map[0].size(), CV_8UC1);
    for(size_t i = 0; i < map.size(); i++)
    {
        for(size_t j = 0; j < map[i].size(); j++)
        {
            init_map.at<unsigned char>(i,j) = map[i][j];
        }
    }

    Size kernel(_costmap_kernel_size,_costmap_kernel_size);
    GaussianBlur(init_map,cost_map,kernel,_costmap_sigma,_costmap_sigma);
    //print map

    for(size_t x = 0; x < map_x; x++)
    {
        std::string line;
        for(size_t y = 0; y < map_y; y++)
        {
            stringstream ss;
            ss << (int)cost_map.at<unsigned char>(x,y);
            line.append(ss.str());
        }
//        ROS_INFO("%s",line.data());
    }
//    namedWindow("Cost Map");
//    imshow("Cost Map", cost_map);
//    waitKey(0);


}

bool pathfind::astar(snoopy_pathfind::pathfind::Request &req, snoopy_pathfind::pathfind::Response &res)
{
    ROS_INFO("_ASTAR_: CALL TO ASTAR");
    //get the occ grid and parse it to our local 2d vector
    nav_msgs::OccupancyGrid grid = req.map;
    mapParser(grid);
    costMapInit();
    cell_occ_value = req.cell_occ_value.data;
    //get robot and goal pose, find their corresponding cell coordinates in grid
    double xc, yc, xg, yg;
    xc = req.pose.x;
    yc = req.pose.y;
    xg = req.goal.x;
    yg = req.goal.y;
    coord start, goal;

    start.second = round(yc/map_cell_size);
    start.first = round(xc/map_cell_size);
    goal.second = round(yg/map_cell_size);
    goal.first = round(xg/map_cell_size);

    ROS_INFO("_ASTAR_ : GOAL %d %d", goal.first,goal.second);
    ROS_INFO("_ASTAR_ : POSE %d %d", start.first,start.second);

    ////////// Path Search: A* Algorithm //////////

    nodeQueue checkList;
    std::map< coord , int > gcost;
    std::map< coord, coord > parent;

    //int movementCost = 5;
    Node startNode;
    startNode.first = 0;
    startNode.second = start;
    checkList.push(startNode);
    parent[start] = start;
    gcost[start] = 0;

    bool noPath = false;
    coord prev = start;

    while (!checkList.empty())
    {
        coord current = checkList.top().second;
        checkList.pop();


        if (current == goal)
        {
            break;
        }
        else
        {
            vector<coord> neighbourVec = getNeighbours(current);

            if(!neighbourVec.empty())
            {
                for(size_t i = 0; i < neighbourVec.size(); i++)
                {
                    coord neighbour = neighbourVec[i];
                    int next_gcost = movementCost(neighbour) + gcost[current];
                    //ROS_INFO("movement cost %d",movementCost(neighbour)*10 );
                    if (gcost.count(neighbour) == 0 || next_gcost < gcost[neighbour])
                    {
                        gcost[neighbour] = next_gcost;
//                         int hcost = manhattanDistance(neighbour, goal)*10;
                        //Diagonal Distance since diagonal movements are allowed
                        int hcost = diagonalDistance(neighbour,goal)*1;
                        int fcost = next_gcost + hcost;
                        Node n;
                        n.first = fcost;
                        n.second = neighbour;
                        checkList.push(n);
                        parent[neighbour] = current;

                    }
                }

            }

        }


    }//end of while checklist

    ////////// Parsing the path //////////

    if (checkList.empty())
    {
        noPath = true;
        ROS_INFO("_ASTAR_ : THERE IS NO PATH TO THE GOAL");
        return false;
    }

    if (!noPath)    //there is a path!
    {
        ROS_INFO("_ASTAR_ : PATH IS FOUND");
        nav_msgs::Path path;
        vector<coord> temp_path;
        //we will track the path back starting from the goal
        coord current = goal;
        while(current != start)
        {
            coord prev = parent[current];
            temp_path.push_back(prev);
            current = prev;
        }

        reverse(temp_path.begin(), temp_path.end());
        temp_path = smooth(temp_path);
//        temp_path = smooth(temp_path);

        //pushing the path to the message
        PATH_COUNT++;
        path.header.seq = PATH_COUNT;
        path.header.stamp = Time::now();
        for(size_t i = 0; i < temp_path.size(); i++)
        {
            geometry_msgs::PoseStamped p;
            //  -------------------------------------------- //
            p.pose.position.x = (temp_path[i].first * map_cell_size) + map_cell_size/2;
            p.pose.position.y = temp_path[i].second * map_cell_size + map_cell_size/2;
            //  -------------------------------------------- //
            path.poses.push_back(p);
//            ROS_INFO("_astar_: path %d = %d,%d",(int) i,temp_path[i].first,temp_path[i].second);
        }
        res.path = path;

        return true;

    }//end if !noPath

}//end of astar()

void pathfind::mapParser(nav_msgs::OccupancyGrid &msg)
{
    //y coordinate of world map is x of node map
    map_x = msg.info.width;
    // x coordinate of world map is y of node map
    map_y = msg.info.height;
    //ROS_INFO("MAP ROWS:  %d  MAP COLS: %d",this->map_rows, this->map_cols);
    map_cell_size = msg.info.resolution;
    //ROS_INFO("CELL SIZE %f",map_cell_size);
    //resize the map
    map.resize(map_x);
    for (size_t i = 0; i < map_x; i++) { map[i].resize(map_y); }

    //converting back from ROS type to OUR TYPE, look for occupancyGrid.cpp
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
//            ss << map[x][y];
//            line.append(ss.str());
//        }
//        ROS_INFO("%s",line.data());
//    }

}


int pathfind::emptyNeighbours(coord loc)
{
    int result = 0;

    coord up(-1,0); //up
    coord left_up(-1,-1); //left-up diagonal
    coord left(0,-1); //left
    coord down(1,0); //down
    coord right_down(1,1); //right-down diagonal
    coord left_down(1,-1); //left-down diagonal
    coord right(0,1); //right
    coord right_up(-1,1); //right-up diagonal

    DIRECTIONS[0] = up;
    DIRECTIONS[1] = right;
    DIRECTIONS[2] = down;
    DIRECTIONS[3] = left;
    DIRECTIONS[4] = right_up;
    DIRECTIONS[5] = right_down;
    DIRECTIONS[6] = left_down;
    DIRECTIONS[7] = left_up;

    int x = loc.first;
    int y = loc.second;

    for(size_t i = 0; i < DIRECTIONS.size(); i++)
    {
        coord neighDir = DIRECTIONS[i];
        int dirx = neighDir.first;
        int diry = neighDir.second;
        int x_neigh = x+dirx;
        int y_neigh = y+diry;

        if( (0 <= x_neigh) && (x_neigh < map_x) && (0 <= y_neigh) && (y_neigh < map_y))
        {
            if(map[x_neigh][y_neigh] != cell_occ_value) result++;
        }
    }
    return result;
}

//Gets the locations of the neighbours of the node at the given locations if they are free or goal
vector< coord > pathfind::getNeighbours(coord loc)
{
    coord up(-1,0); //up
    coord left_up(-1,-1); //left-up diagonal
    coord left(0,-1); //left
    coord down(1,0); //down
    coord right_down(1,1); //right-down diagonal
    coord left_down(1,-1); //left-down diagonal
    coord right(0,1); //right
    coord right_up(-1,1); //right-up diagonal

    DIRECTIONS[0] = up;
    DIRECTIONS[1] = right;
    DIRECTIONS[2] = down;
    DIRECTIONS[3] = left;
    DIRECTIONS[4] = right_up;
    DIRECTIONS[5] = right_down;
    DIRECTIONS[6] = left_down;
    DIRECTIONS[7] = left_up;

    int x = loc.first;
    int y = loc.second;
    vector< coord > neighbours;


    for(size_t i = 0; i < DIRECTIONS.size(); i++)
    {
        coord neighDir = DIRECTIONS[i];
        int dirx = neighDir.first;
        int diry = neighDir.second;
        int x_neigh = x+dirx;
        int y_neigh = y+diry;


        if( (0 <= x_neigh) && (x_neigh < map_x) && (0 <= y_neigh) && (y_neigh < map_y))
        {
            if(map[x_neigh][y_neigh] != cell_occ_value)
            {
                if(i < 4) //if main direction
                {
                    neighbours.push_back(make_pair(x_neigh,y_neigh));
                }
                else //if diagonal
                {
                    coord n = make_pair(dirx,diry);

                    if( n == right_up &&
                        map[x+up.first][y+up.second] != cell_occ_value &&
                        map[x+right.first][y+right.second] != cell_occ_value)
                    {neighbours.push_back(make_pair(x_neigh,y_neigh)); }

                    else if( n == right_down &&
                        map[x+down.first][y+down.second] != cell_occ_value &&
                        map[x+right.first][y+right.second] != cell_occ_value)
                    {neighbours.push_back(make_pair(x_neigh,y_neigh)); }

                    else if( n == left_up &&
                        map[x+up.first][y+up.second] != cell_occ_value &&
                        map[x+left.first][y+left.second] != cell_occ_value)
                    {neighbours.push_back(make_pair(x_neigh,y_neigh)); }

                    else if( n == left_down &&
                        map[x+down.first][y+down.second] != cell_occ_value &&
                        map[x+left.first][y+left.second] != cell_occ_value)
                    {neighbours.push_back(make_pair(x_neigh,y_neigh)); }

                }

            }
        }
    }
//    ROS_INFO("map_parser end");
    return neighbours;
}

int pathfind::manhattanDistance(coord nd1, coord nd2)
{
    return abs(nd1.first - nd2.first) + abs(nd1.second - nd2.second);
}

int pathfind::diagonalDistance(coord nd1, coord nd2)
{
    int dx = abs(nd1.first - nd2.first);
    int dy= abs(nd1.second - nd2.second);
    return (dx + dy - min(dx,dy));
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "astar_node");

    PATH_COUNT = 0;

    pathfind pf;

    spin();

    return 0;
}//end of main
