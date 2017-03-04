#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Pose2D.h> //the pose of the robot
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/LaserScan.h>
#include "snoopy_localize/get_pose.h"
#include <tf/tf.h>

using namespace ros;
using namespace std;

class avoidance
{
protected:
    NodeHandle n, n_priv;

public:
    avoidance();
    ~avoidance();

    vector<double> ranges;
    Subscriber twist_sub;
    Subscriber laser_sub;
    Publisher avoid_pub, marker_pub;
    string _avoid_topic;
    bool avoidance_check;
    bool crash;
    void predict_collision();
    visualization_msgs::MarkerArray all_markers;
//    visualization_msgs::Marker arrow_marker;
    ServiceClient pose_client;
    double angle_min, angle_max, angle_increment;
    double _delta_t, _ahead;
    double averageRanges(int angle, int interval);
    bool isLaserMsgReceived;
    void clearTwist();

private:
    geometry_msgs::Twist twist;
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);  
    double ctrl_a, ctrl_l;       
    bool isCollision(double theta, double range);

};

avoidance::avoidance() : n_priv("~")
{
    ROS_INFO("_avoid_: Calling the constructor of calss AVOIDANCE");
    n_priv.param("ahead", _ahead, 0.25);
    n_priv.param("delta_t", _delta_t, 0.5);
    n_priv.param<string>("avoid_topic", _avoid_topic, "/snoopy/collision");

    laser_sub = n.subscribe<sensor_msgs::LaserScan>("/scan_converted",10,&avoidance::laserCallback,this);
    twist_sub = n.subscribe<geometry_msgs::Twist>("/snoopy/cmd_vel",10,&avoidance::twistCallback,this);
    avoid_pub = n.advertise<std_msgs::Bool>(_avoid_topic, 5);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("/avoidance_marker",80);
    pose_client = n.serviceClient<snoopy_localize::get_pose>("get_pose");

    ranges.resize(360,0.0);
    isLaserMsgReceived = false;
    //initialize crash as false when construct class
    crash = false;
}

avoidance::~avoidance()
{
//    ROS_INFO("_avoidance_: Deleting an object of the class AVOIDANCE");
}

void avoidance::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    angle_max = msg->angle_max;
    angle_min = msg->angle_min;
    angle_increment = msg->angle_increment;

    for(size_t i = 0; i < msg->ranges.size(); i++)
    {
        if(msg->ranges[i] != std::numeric_limits<double>::infinity() && !isnan(msg->ranges[i]))
            ranges[i] = msg->ranges[i];

        // check between some degree if there is range less than threshold
        if(i < 23 || i > 337)
        {
            //20 is the size of the robot?
            if((ranges[i]) < _ahead && (ranges[i]) > 0.12)
            {
                avoidance_check = true;
            }
        }
    }
    isLaserMsgReceived = true;

//    ranges.clear();
//    for(size_t i = 0; i < msg->ranges.size(); i++)
//    {
//        ranges.push_back(msg->ranges[i]);
//        // check between +-60 degree if there is range less than threshold
//        if(i < 15 || i > 345)
//        {
//            //20 is the size of the robot?
//            if((msg->ranges[i]) < _ahead && (msg->ranges[i]) > 0.2)
//            {
//                avoidance_check = true;
//            }
//        }
//    }
}

void avoidance::twistCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    ctrl_a = msg -> angular.z;
    ctrl_l = msg -> linear.x;
}

void avoidance::predict_collision()
{
    double predict_theta, predict_range;

    if(ctrl_a != 0.0 || ctrl_l != 0.0)
    {
        predict_theta = ctrl_a * _delta_t;
        predict_range = ctrl_l * _delta_t + _ahead;
        crash = isCollision(predict_theta - 5*M_PI/180, predict_range) ||
                isCollision(predict_theta, predict_range) ||
                isCollision(predict_theta + 5*M_PI/180, predict_range);
    }
    else crash = false;

}

bool avoidance::isCollision(double theta, double range)
{
    //convert angle to [0, 2*pi)
    theta = fmod(theta + 2*M_PI, 2*M_PI);
    //index in laser scan
    int ind = floor(theta/angle_increment);
    double avg = averageRanges(ind,2);
    if(range > avg) return true;
    else return false;
}

double avoidance::averageRanges(int angle, int interval)
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

void avoidance::clearTwist()
{
    ctrl_a = 0.0;
    ctrl_l = 0.0;
}

int main(int argc, char **argv)
{
    init(argc, argv, "avoidance");
    avoidance avoid;
    Rate loop(20);
    while(ok())
    {
        avoid.clearTwist();
        spinOnce();
        snoopy_localize::get_pose pose_srv;
        geometry_msgs::Pose2D pose;
        if(avoid.pose_client.call(pose_srv))
        {
            pose = pose_srv.response.pose;
        }
//        else ROS_ERROR("_POSE SRV FAILED");

        if(avoid.isLaserMsgReceived)
        {
            avoid.all_markers.markers.clear();
            for(int i = 0; i < avoid.ranges.size(); i+=10)
            {
                visualization_msgs::Marker arrow_marker;
                arrow_marker.id = i;
                arrow_marker.header.frame_id = "/odom";
                arrow_marker.header.stamp = ros::Time();
                arrow_marker.ns = "avoid_arrows";
                arrow_marker.type = visualization_msgs::Marker::ARROW;
                arrow_marker.action = visualization_msgs::Marker::ADD;
                arrow_marker.scale.x = avoid._ahead;
                arrow_marker.scale.y = 0.005;
                arrow_marker.scale.z = 0.01;
                arrow_marker.color.a = 1.0;
                arrow_marker.pose.position.x = pose.x;
                arrow_marker.pose.position.y = pose.y;

                tf::quaternionTFToMsg(tf::createQuaternionFromYaw(-pose.theta + avoid.angle_min + i*avoid.angle_increment),arrow_marker.pose.orientation);
                double avg = avoid.ranges[i];
                //                double avg = avoid.averageRanges(i,1);
                if(0 <= avg && avg <= avoid._ahead)
                {
                    arrow_marker.color.r = (255.0/255.0);
                    arrow_marker.color.g = (0.0/255.0);
                    arrow_marker.color.b = (0.0/255.0);
                }
                else
                {
                    arrow_marker.color.r = (0.0/255.0);
                    arrow_marker.color.g = (150.0/255.0);
                    arrow_marker.color.b = (150.0/255.0);
                }
                avoid.all_markers.markers.push_back(arrow_marker);

            }
            avoid.marker_pub.publish(avoid.all_markers);

            if (avoid.avoidance_check == true)
            {
                avoid.predict_collision();
                std_msgs::Bool msg;
                msg.data = avoid.crash;
                avoid.avoid_pub.publish(msg);
            }
        }
        loop.sleep();
    }

    return 0;
}
