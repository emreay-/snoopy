#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include "snoopy_odom/update_odom.h"
#include "snoopy_localize/get_pose.h"
#include "snoopy_localize/fuse.h"

using namespace ros;
double x, y, th;

class poseFusion
{
public:
    poseFusion();
    ~ poseFusion();
    NodeHandle n, n_priv;
    Publisher pose_pub, pf_info_pub, odom_info_pub;
    geometry_msgs::Pose2D output_pose;
    geometry_msgs::Pose2D pf_pose, dead_pose;
    bool _do_fusion;
    std::string _fuse_type;
    ServiceClient update_odom_client;
    snoopy_odom::update_odom odom_srv;
    double _update_x_threshold, _update_y_threshold, _update_theta_threshold;

private:
    Subscriber pf_sub, dead_sub;
    void dead_cb(const nav_msgs::Odometry::ConstPtr& msg);
    void pf_cb(const geometry_msgs::Pose2D::ConstPtr& msg);
    ServiceServer pose_server, fuse_server;
    double _init_x, _init_y, _init_theta;
    bool pose_getter(snoopy_localize::get_pose::Request &req, snoopy_localize::get_pose::Response &res);
    bool fuse(snoopy_localize::fuse::Request &req, snoopy_localize::fuse::Response &res);
    bool requested_update_;
    bool updated_;
};

poseFusion::poseFusion(): n_priv("~")
{
    ROS_INFO("_pose_fusion_: CONSTRUCTOR");
    dead_sub = n.subscribe<nav_msgs::Odometry>("odom",10,&poseFusion::dead_cb,this);
    pf_sub = n.subscribe<geometry_msgs::Pose2D>("snoopy/particle_filter/pose_est",10,&poseFusion::pf_cb,this);
    pose_pub = n.advertise<geometry_msgs::Pose2D>("snoopy/pose_fusion",10);
    pf_info_pub = n.advertise<geometry_msgs::Pose2D>("snoopy/info/pf_pose",10);
    odom_info_pub = n.advertise<geometry_msgs::Pose2D>("snoopy/info/odom_pose",10);
    update_odom_client = n.serviceClient<snoopy_odom::update_odom>("update_odom");
    pose_server = n.advertiseService("get_pose",&poseFusion::pose_getter,this);
    fuse_server = n.advertiseService("fuse",&poseFusion::fuse,this);

    n_priv.param("do_fusion",_do_fusion,true);
    n_priv.param<std::string>("fuse_type",_fuse_type,"ALL");
    ROS_INFO("_pose_fusion_: FUSION SWITCHED ON: %d, FUSE TYPE: %s",_do_fusion,_fuse_type.c_str());
    n_priv.param("update_x_threshold",_update_x_threshold,0.05);
    n_priv.param("update_y_threshold",_update_y_threshold,0.05);
    n_priv.param("update_theta_threshold",_update_theta_threshold,5.0);
    _update_theta_threshold *= M_PI/180;
    n_priv.param("init_x",_init_x,0.0);
    n_priv.param("init_y",_init_y,0.0);
    n_priv.param("init_theta",_init_theta,0.0);
    _init_theta *= M_PI/180;

    pf_pose.x = _init_x;
    pf_pose.y = _init_y;
    pf_pose.theta = _init_theta;

    dead_pose.x = _init_x;
    dead_pose.y = _init_y;
    dead_pose.theta = _init_theta;
}

poseFusion::~poseFusion()
{
    ROS_INFO("_pose_fusion_: DESTRUCTOR");
}

void poseFusion::dead_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    dead_pose.x = msg->pose.pose.position.x;
    dead_pose.y = msg->pose.pose.position.y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation,q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll,pitch,yaw);
    dead_pose.theta = fmod(yaw+2*M_PI,2*M_PI);

    //monitoring purposes
    geometry_msgs::Pose2D info;
    info.x = dead_pose.x;
    info.y = dead_pose.y;
    info.theta = fmod(dead_pose.theta+2*M_PI,2*M_PI) * 180/M_PI;
    odom_info_pub.publish(info);

    output_pose = dead_pose;

}

void poseFusion::pf_cb(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    pf_pose.x = msg->x;
    pf_pose.y = msg->y;
    pf_pose.theta = fmod(msg->theta+2*M_PI,2*M_PI);

//    if(_do_fusion) //is fusion turned on?
//    {
//        odom_srv.request.pose = pf_pose;
//        odom_srv.request.cmd.data = "ALL";
//        ROS_INFO("_pose_fusion_: Odom being update from PF.");
//        if(update_odom_client.call(odom_srv)) ROS_INFO("_pose_fusion_: Odom update successfull.");
//        else ROS_ERROR("_pose_fusion_: Odom update failed.");

//    }

    geometry_msgs::Pose2D info;
    info.x = pf_pose.x;
    info.y = pf_pose.y;
    info.theta = pf_pose.theta * 180/M_PI;
    pf_info_pub.publish(info);
    if (requested_update_)
    {
        requested_update_ = false;
        updated_ = true;
    }
}

bool poseFusion::pose_getter(snoopy_localize::get_pose::Request &req, snoopy_localize::get_pose::Response &res)
{
    res.pose = output_pose;
    return true;
}

bool poseFusion::fuse(snoopy_localize::fuse::Request &req, snoopy_localize::fuse::Response &res)
{
    spinOnce();
    std::string cmd = req.cmd.data;
    requested_update_ = true;
    updated_ = false;
    if(cmd == "THETA")
    {
        while(!updated_)
        {
            spinOnce();
            sleep(0.1);
        }
        odom_srv.request.pose = pf_pose;
        odom_srv.request.cmd.data = "THETA";
        ROS_INFO("_pose_fusion_: Odom theta being update from PF in FUSE SRV.");
        if(update_odom_client.call(odom_srv))
        {
            ROS_INFO("_pose_fusion_: Odom update successfull in FUSE SRV.");
            return true;
        }
        else
        {
            ROS_ERROR("_pose_fusion_: Odom update failed in FUSE SRV.");
            return false;
        }
    }
    else if(cmd == "ALL")
    {
        while(!updated_)
        {
            spinOnce();
            sleep(0.1);
        }
        odom_srv.request.pose = pf_pose;
        odom_srv.request.cmd.data = "ALL";
        ROS_INFO("_pose_fusion_: Odom being update from PF in FUSE SRV.");
        if(update_odom_client.call(odom_srv))
        {
            ROS_INFO("_pose_fusion_: Odom update successfull in FUSE SRV.");
            return true;
        }
        else
        {
            ROS_ERROR("_pose_fusion_: Odom update failed in FUSE SRV.");
            return false;
        }
    }
    else return false;
}

int main(int argc, char* argv[])
{
    init(argc,argv,"pose_fusion");
    poseFusion fusion;
    Rate loop_rate(120);
    sleep(2);
    int count = 0;
    while(ok())
    {
        spinOnce();

        if(fusion._do_fusion) //is fusion turned on?
        {
            if(fusion._fuse_type == "ALL")
            {
    //            ROS_INFO("tresh x: %f   tresh y: %f  tresh th: %f",fusion._update_x_threshold,fusion._update_y_threshold,fusion._update_theta_threshold);
                if( fabs(fusion.dead_pose.x-fusion.pf_pose.x) > fusion._update_x_threshold ||
                    fabs(fusion.dead_pose.y-fusion.pf_pose.y) > fusion._update_y_threshold ||
                    fabs(fusion.dead_pose.theta -fusion.pf_pose.theta) > fusion._update_theta_threshold)
                {
                    count++;
                    ROS_INFO("_pose_fusion_: Pose update %d",count);
                    ROS_INFO("_pose_fusion_: odom_x: %f\todom_y: %f\todom_th: %f",fusion.dead_pose.x,fusion.dead_pose.y,fusion.dead_pose.theta);
                    ROS_INFO("_pose_fusion_: part_x: %f\tpart_y: %f\tpart_th: %f",fusion.pf_pose.x,fusion.pf_pose.y,fusion.pf_pose.theta);
                    fusion.odom_srv.request.pose = fusion.pf_pose;
                    fusion.odom_srv.request.cmd.data = "ALL";
    //                ROS_INFO("_pose_fusion_: Odom deviated, being update from PF.");
                    if(fusion.update_odom_client.call(fusion.odom_srv)) ROS_INFO("_pose_fusion_: Odom update successfull.");
                    else ROS_ERROR("_pose_fusion_: Odom update failed.");
                }
            }
            else if(fusion._fuse_type == "THETA")
            {
                if(fabs(fusion.dead_pose.theta-fusion.pf_pose.theta) > fusion._update_theta_threshold)
                {
                    fusion.odom_srv.request.pose = fusion.pf_pose;
                    fusion.odom_srv.request.cmd.data = "THETA";
                    ROS_INFO("_pose_fusion_: Odom deviated, being update from PF.");
                    if(fusion.update_odom_client.call(fusion.odom_srv)) ROS_INFO("_pose_fusion_: Odom update successfull.");
                    else ROS_ERROR("_pose_fusion_: Odom update failed.");
                }
            }

        }

        fusion.pose_pub.publish(fusion.output_pose);

        loop_rate.sleep();
    }

    return 0;
}
