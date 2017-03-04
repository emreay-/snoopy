//
//  particleFilter4.cpp
//
//
//  Created by Emre Ay on 29/11/16.
//
//

#include <time.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <math.h>
#include <boost/random/mersenne_twister.hpp> //rand number generator
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/normal_distribution.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <limits>
#include <queue>
#include <std_msgs/ColorRGBA.h>

using namespace ros;
using namespace std;


class particleFilter
{
public:
    particleFilter();
    ~particleFilter();
    NodeHandle nh, n;
    int numberOfParticles;
    double _process_sigma_lin, _process_sigma_ang, _meas_sigma;
    double v_robot, w_robot, dt;
    double lin_del, angle_del;
    double _map_x_max, _map_y_max;
    double angle_min, angle_max;
    double _init_pose_x, _init_pose_y,_init_pose_th, _init_sigma_x, _init_sigma_y, _init_sigma_th;
    bool _init_uniform;
    vector<double> ranges;
    vector<double> simScan(geometry_msgs::Pose2D particle);
    vector<double> weights;
    int _laser_angle_inc;
    vector<double> segmentX, segmentY;
    string _grid_topic, _pf_output_topic,_map_file, _scan_topic;    
    double _angle_offset;
    Publisher pose_pub;
    boost::random::mt19937 num_gen; //rand number generator
    void initParticlesUniformly();
    void initParticles();
    void predict();
    void update();
    void resample();
    void giveAnEstimate();
    void resetAcc();   
    Subscriber delta_sub, grid_sub, laser_sub;
    vector< vector<int> > map;
    int map_rows, map_cols;
    double map_cell_size;
    vector<geometry_msgs::Pose2D> particleSet;
    void delta_callback(const geometry_msgs::Vector3::ConstPtr& msg);
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    geometry_msgs::Pose2D pose_est;
};

particleFilter::particleFilter() : nh("~")
{
    ROS_INFO("_pf_: Constructor of class particleFilter");
    num_gen.seed(time(0));
    nh.param<string>("map_file", _map_file, "maze_map.txt");

    ifstream map_fs; map_fs.open(_map_file.c_str());
    if (!map_fs.is_open()){
        ROS_ERROR_STREAM("_pf_: Could not read maze map from "<<_map_file<<". Please double check that the file exists. Aborting.");
    }
    string line;

    while (getline(map_fs, line))
    {
        if (line[0] == '#') {
            // comment -> skip
            continue;
        }

        double max_num = std::numeric_limits<double>::max();
        double x1= max_num,
               x2= max_num,
               y1= max_num,
               y2= max_num;

        std::istringstream line_stream(line);
        line_stream >> x1 >> y1 >> x2 >> y2;

        if ((x1 == max_num) || (x2 == max_num) || (y1 == max_num) || (y2 == max_num))
        {
            ROS_WARN("_pf_: Segment error. Skipping line: %s",line.c_str());
        // angle and distance
        }
        else if ((x1 != max_num) && (x2 != max_num) && (y1 != max_num) && (y2 != max_num))
        {
            segmentX.push_back(x1);
            segmentX.push_back(x2);
            segmentY.push_back(y1);
            segmentY.push_back(y2);
        }
    }//end while

    double max_x = 0, max_y = 0 , elem_x = 0, elem_y = 0;
    for(size_t i = 0; i < segmentX.size(); i++)
    {
        elem_x = segmentX[i];
        if(elem_x > max_x) max_x = elem_x;
        elem_y = segmentY[i];
        if(elem_y > max_y) max_y = elem_y;
    }

    _map_x_max = max_x; //[meters]
    _map_y_max = max_y; //[meters]

    nh.getParam("number_of_particles",numberOfParticles);
    nh.getParam("process_sigma_lin",_process_sigma_lin);
    nh.getParam("process_sigma_ang",_process_sigma_ang);
    nh.getParam("measurement_sigma",_meas_sigma);    
    nh.getParam("pf_output_topic",_pf_output_topic);    
    nh.param("laser_angle_inc",_laser_angle_inc,8);
    nh.param("init_uniform",_init_uniform,false);
    nh.param("angle_offset",_angle_offset,5.0);
    nh.param<string>("scan_topic",_scan_topic,"/scan");
    ROS_INFO("_pf_: Scan topic: %s", _scan_topic.c_str());

    particleSet.resize(numberOfParticles);
    delta_sub = nh.subscribe<geometry_msgs::Vector3>("/odom_delta",1000,&particleFilter::delta_callback,this);
    laser_sub = nh.subscribe<sensor_msgs::LaserScan>(_scan_topic,10,&particleFilter::laser_callback,this);
    pose_pub = nh.advertise<geometry_msgs::Pose2D>(_pf_output_topic,5,this);
}

particleFilter::~particleFilter()
{
    ROS_INFO("_pf_: Deleting an object of class particleFilter");
}

void particleFilter::delta_callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    lin_del += msg->x;
    angle_del += msg->y;
}

void particleFilter::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    ranges.clear();
    for(size_t i = 0; i < msg->ranges.size(); i+=_laser_angle_inc)
    {
        ranges.push_back(msg->ranges[i]);
    }
}


void particleFilter::initParticlesUniformly()
{
    ROS_INFO("_pf_: initParticlesUniformly FUNC.");
    boost::random::uniform_real_distribution<> uniform_row(0,_map_x_max);
    boost::random::uniform_real_distribution<> uniform_col(0,_map_y_max);
    boost::random::uniform_int_distribution<> uniform_angle(0,360);
    for (size_t i = 0; i < numberOfParticles; i++)
    {
        double x = uniform_row(num_gen);
        double y = uniform_col(num_gen);
        double th = uniform_angle(num_gen);

        particleSet[i].x = x;
        particleSet[i].y = y;
        particleSet[i].theta = th * M_PI/180;
    }
}

void particleFilter::initParticles()
{
    nh.getParam("init_pose_x",_init_pose_x);
    nh.getParam("init_pose_y",_init_pose_y);
    nh.getParam("init_pose_th",_init_pose_th);
    nh.getParam("init_sigma_x",_init_sigma_x);
    nh.getParam("init_sigma_y",_init_sigma_y);
    nh.getParam("init_sigma_th",_init_sigma_th);
    boost::random::normal_distribution<> gauss_x(_init_pose_x,_init_sigma_x);
    boost::random::normal_distribution<> gauss_y(_init_pose_y,_init_sigma_y);
    boost::random::normal_distribution<> gauss_th(_init_pose_th,_init_sigma_th);

    for (size_t i = 0; i < numberOfParticles; i++)
    {
        double x = gauss_x(num_gen);
        double y = gauss_y(num_gen);
        double th = gauss_th(num_gen);

        particleSet[i].x = x;
        particleSet[i].y = y;
        particleSet[i].theta = th * M_PI/180;
    }
}

void particleFilter::predict()
{
    ROS_INFO("_pf_: PARTICLE PREDICT");
    boost::random::normal_distribution<> gauss_lin(0.0,_process_sigma_lin);
    boost::random::normal_distribution<> gauss_ang(0.0,_process_sigma_ang);
    for (size_t i = 0; i < numberOfParticles; i++)
    {
        particleSet[i].x += (lin_del * cos(particleSet[i].theta) + gauss_lin(num_gen));
        particleSet[i].y += (lin_del * sin(particleSet[i].theta) + gauss_lin(num_gen));
        particleSet[i].theta += (angle_del + gauss_ang(num_gen));
    }
}

void particleFilter::update()
{
    ROS_INFO("_pf_: PARTICLE UPDATE");

    weights.clear();
    double norm_factor = 0;
    for (int par = 0; par < numberOfParticles; par++)
    {
        double weight_p = 0;
        if( 0 <= particleSet[par].x && particleSet[par].x <= _map_x_max &&
            0 <= particleSet[par].y && particleSet[par].y <= _map_y_max )
        {
            vector<double> sim_scan = simScan(particleSet[par]);
            //        ROS_INFO("_pf_: PARTICLE X: %f", particleSet[p].x);
            //        ROS_INFO("_pf_: RANGES SIZE %d", int(ranges.size()));
            for(int i = 0; i < int(ranges.size()); i++)
            {
                double particle_theta = particleSet[par].theta * 180/M_PI;
                int angle_offset = floor(particle_theta/_laser_angle_inc);
                double measured_range = ranges[i],
                        simulated_range = sim_scan[(i + angle_offset) % int(ranges.size())];
                //                   simulated_range = sim_scan[i];
                if(measured_range == std::numeric_limits<float>::infinity() || isnan(measured_range)) measured_range = 100;
                if(simulated_range == std::numeric_limits<float>::infinity()) simulated_range = 100;

                double diff = simulated_range - measured_range;

                //            double temp;
                //            if(diff != 0) temp = 1/diff;
                //            else temp = 100000;
                double temp = (exp(-pow(diff,2)/(2*pow(_meas_sigma,2))))/(sqrt(2*M_PI)*_meas_sigma);
                //            ROS_INFO("_pf_: gaussian %f", temp);
                //sum likelihoods of all rays
                weight_p += temp;
            }
        }
        //weight of the particle is the mean of likelihoods of all rays
        weight_p = weight_p/int(ranges.size());
        weights.push_back(weight_p);
        //sum all the weights of all the particles for normalization factor
        norm_factor += weight_p;
//        ROS_INFO("_pf_: particle %d has weight %f", par, weight_p);
    }
    //normalize the weights
    for(size_t i = 0; i < weights.size(); i++)
    {
        weights[i] = weights[i]/norm_factor;
    }

}

//Axis Aligned Bounding Box Collision Algorithm
bool doBoundingBoxesCollide(pair<double,double> segA1, pair<double,double> segA2, pair<double,double> segB1, pair<double,double> segB2)
{
    //bounding box coordinates association
    double Ax = min(segA1.first,segA2.first),
            AX = max(segA1.first,segA2.first),
            Ay = min(segA1.second,segA2.second),
            AY = max(segA1.second,segA2.second),
            Bx = min(segB1.first,segB2.first),
            BX = max(segB1.first,segB2.first),
            By = min(segB1.second,segB2.second),
            BY = max(segB1.second,segB2.second);

    return !( (AX < Bx) || (BX < Ax) || (AY < By) || (BY < Ay));
}

double crossProduct(double x1, double y1, double x2, double y2){ return x1*y2 - x2*y1; }

bool crossCheck(pair<double,double> segA1, pair<double,double> segA2, pair<double,double> segB1, pair<double,double> segB2)
{
    //move the first segment to the origin
    double tempA1x = 0.0,
           tempA1y = 0.0,
           tempA2x = segA2.first - segA1.first,
           tempA2y = segA2.second - segA1.second;
    //move the second segment same as much as first segment
    double tempB1x = segB1.first - segA1.first,
           tempB1y = segB1.second - segA1.second,
           tempB2x = segB2.first - segA1.first,
           tempB2y = segB2.second - segA1.second;
    //crossproduct with tempB1
    double cp1 = crossProduct(tempA2x, tempA2y, tempB1x, tempB1y);
    //crossproduct with tempB2
    double cp2 = crossProduct(tempA2x, tempA2y, tempB2x, tempB2y);

    //if one of the two endpoints are on the line or the endpoints are at different sides of the line
    //these two lines are intersecting
    bool pointOnLine1 = (abs(cp1) < 0.00001),
         pointOnLine2 = (abs(cp2) < 0.00001),
         pointOnRightSide1 = (cp1 < 0),
         pointOnRightSide2 = (cp2 < 0);

    return pointOnLine1 || pointOnLine2 || (pointOnRightSide1 ^ pointOnRightSide2);
}

bool doLinesIntersect(pair<double,double> segA1, pair<double,double> segA2, pair<double,double> segB1, pair<double,double> segB2)
{
    return doBoundingBoxesCollide(segA1, segA2, segB1, segB2) &&
           crossCheck(segA1, segA2, segB1, segB2) &&
           crossCheck(segB1, segB2, segA1, segA2);

//    return crossCheck(segA1, segA2, segB1, segB2) &&
//           crossCheck(segB1, segB2, segA1, segA2);
}

vector<double> particleFilter::simScan(geometry_msgs::Pose2D particle)
{
    vector<double> result;
    const double RAY_MAX = 6.0;

    double xp, yp;
    xp = particle.x;
    yp = particle.y;
    pair<double,double> ray_seg1;
    ray_seg1 = make_pair(xp,yp);

    int segments = int(segmentX.size()/2);
    for(int angle = 0; angle < 360; angle+=_laser_angle_inc)
    {
        double angle_ray = angle*M_PI/180;
        pair<double,double> ray_seg2;
        ray_seg2 = make_pair( (xp + RAY_MAX*cos(angle_ray)) , (yp + RAY_MAX*sin(angle_ray)) );

        std::priority_queue<double, std::vector<double>, std::greater<double> > dist_q;
        for(int i = 0; i < segments; i++)
        {
            pair<double,double> line_seg1, line_seg2;
            line_seg1 = make_pair(segmentX[2*i],segmentY[2*i]);
            line_seg2 = make_pair(segmentX[2*i+1],segmentY[2*i+1]);            

            //check if the ray and wall segment intersect
            if(doLinesIntersect(ray_seg1, ray_seg2, line_seg1, line_seg2))
            {
                double x1 = line_seg1.first,
                        y1 = line_seg1.second,
                        x2 = line_seg2.first,
                        y2 = line_seg2.second,
                        x3 = ray_seg1.first,
                        y3 = ray_seg1.second,
                        x4 = ray_seg2.first,
                        y4 = ray_seg2.second,
                        a = crossProduct(x1,y1,x2,y2),
                        b = crossProduct(x3,y3,x4,y4),
                        c = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4),
                        x_intersect = (a * (x3-x4) - b * (x1-x2)) / c,
                        y_intersect = (a * (y3-y4) - b * (y1-y2)) / c,
                        d_ray = sqrt(pow((x_intersect-x3),2)+pow((y_intersect-y3),2));

                dist_q.push(d_ray);

            }

        }//end for

        if(dist_q.size() != 0) //if the queue is not empty
            result.push_back(dist_q.top());  //push the closest segment's range
        else result.push_back(std::numeric_limits<float>::infinity());
    }//end for

    return result;
}


void particleFilter::resample()
{
    ROS_INFO("_pf_: PARTICLE RESAMPLE");
    vector<geometry_msgs::Pose2D> newSet;
    vector<double> CDF(numberOfParticles,0); //cumulative distribution function of particles
    //create CDF
    for(int par = 0; par < numberOfParticles; par++)
    {
       for(int i = 0; i <= par; i++)
       {
           CDF[par] += weights[i];
       }
    }
    //do the systematic resampling
    boost::random::uniform_real_distribution<> uni_rand(0,(1/double(numberOfParticles)));
    double comp = uni_rand(num_gen);
    double sys_inc = (1/double(numberOfParticles));
//    ROS_INFO("_pf_: COMP: %f", comp);
    for(int par = 0; par < numberOfParticles; par++)
    {
//        double sys_comp = comp + double(par/numberOfParticles);
        int j = 0;
        while(CDF[j] < comp) j++;
        newSet.push_back(particleSet[j]);
        comp += sys_inc;
    }
    particleSet.clear();
    particleSet = newSet;
}

void particleFilter::giveAnEstimate()
{
//    geometry_msgs::Pose2D pose_est;
    double x = 0.0,y = 0.0,th = 0.0;
    for(int i = 0; i < numberOfParticles; i++)
    {
        x += particleSet[i].x;
        y += particleSet[i].y;
        th += particleSet[i].theta;
    }
    pose_est.x = x/numberOfParticles;
    pose_est.y = y/numberOfParticles;
    pose_est.theta = fmod((th/numberOfParticles)+2*M_PI,2*M_PI) + (_angle_offset * M_PI/180);
//    pose_est.theta = fmod(((th/numberOfParticles)*180/M_PI) + 360,360);
    pose_pub.publish(pose_est);

}

void particleFilter::resetAcc()
{
    lin_del = 0.0;
    angle_del = 0.0;
}

int main(int argc, char* argv[])
{
    init(argc,argv,"particle_filter");
    //spinOnce(); //get map and delta values
    particleFilter PF;
    Rate loop_rate(10);
    //Half a second of delay! It is important to wait
    //until all the topics we subscribed starts publishing
    //we get problems otherwise...
    Duration(5).sleep();

    spinOnce();
    if(PF._init_uniform)    PF.initParticlesUniformly();
    else                    PF.initParticles();

    NodeHandle n;
    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>("/pf_viz",5);
    visualization_msgs::MarkerArray all_markers;
    //_____________________________________________________
    visualization_msgs::Marker pr_marker;
    pr_marker.header.frame_id = "/odom";
    pr_marker.header.stamp = ros::Time();
    pr_marker.ns = "particles";
    pr_marker.type = visualization_msgs::Marker::POINTS;
    pr_marker.action = visualization_msgs::Marker::ADD;
    pr_marker.scale.x = 0.02;
    pr_marker.scale.y = 0.02;
    pr_marker.color.a = 1.0;
    pr_marker.color.r = (0.0/255.0);
    pr_marker.color.g = (0.0/255.0);
    pr_marker.color.b = (255.0/255.0);
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(0.0),pr_marker.pose.orientation);
    //______________________________________________________
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.frame_id = "/odom";
    arrow_marker.header.stamp = ros::Time();
    arrow_marker.ns = "arrow";
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.scale.x = 0.5;
    arrow_marker.scale.y = 0.03;
    arrow_marker.scale.z = 0.01;
    arrow_marker.color.a = 1.0;
    arrow_marker.color.r = (150.0/255.0);
    arrow_marker.color.g = (0.0/255.0);
    arrow_marker.color.b = (255.0/255.0);
    //_____________________________________________________

//    while (n.ok())
//    {
//        spinOnce();
////        pr_marker.points.clear();
//        geometry_msgs::Pose2D particle;
//        particle.x = 1.0;
//        particle.y = 1.0;
//        particle.theta = M_PI;
//        vector<double> sim = PF.simScan(particle);
//        for(int i = 0; i < sim.size(); i++)
//        {
////            ROS_INFO("SIM SCAN %d : %f", i, sim[(i+90)%360]);
//            if(sim[i] == std::numeric_limits<float>::infinity()) sim[i] = 1;
////            ROS_INFO("ANGLE %d (RANGES[%d]),  MEASURED RANGE: %f, SIM RANGE: %f", i*PF._laser_angle_inc,i,PF.ranges[i],sim[(i + 90/PF._laser_angle_inc) % int(sim.size())]);
//            ROS_INFO("SIM RANGE %d = %f",i,sim[i]);
//            geometry_msgs::Point p;
//            p.x = sim[i]*cos(PF._laser_angle_inc*i*M_PI/180) + particle.x;
//            p.y = sim[i]*sin(PF._laser_angle_inc*i*M_PI/180) + particle.y;
//            p.z = 0;
//            pr_marker.points.push_back(p);
//        }
//        vis_pub.publish(pr_marker);

//        loop_rate.sleep();
//    }

    while (n.ok())
    {
        PF.resetAcc();
        spinOnce();

        PF.predict();
        PF.update();
        PF.resample();
        PF.giveAnEstimate();

        //__________________________VISUZALIZE____________________________
        all_markers.markers.clear();
        pr_marker.points.clear();
        for(size_t i = 0; i < PF.particleSet.size(); i++)
        {
            geometry_msgs::Point p;
            p.x = PF.particleSet[i].x;
            p.y = PF.particleSet[i].y;
            p.z = 0;
            pr_marker.points.push_back(p);
        }
        all_markers.markers.push_back(pr_marker);

        tf::quaternionTFToMsg(tf::createQuaternionFromYaw(PF.pose_est.theta), arrow_marker.pose.orientation);
        arrow_marker.pose.position.x = PF.pose_est.x;
        arrow_marker.pose.position.y = PF.pose_est.y;
        all_markers.markers.push_back(arrow_marker);
        vis_pub.publish(all_markers);
        //_________________________________________________________________

        loop_rate.sleep();
    }



    return 0;
}
