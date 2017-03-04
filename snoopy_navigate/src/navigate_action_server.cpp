#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Pose2D.h> //the pose of the robot
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <algorithm>
#include <utility>
#include <signal.h>  //for sigint
#include <iostream>
#include <snoopy_navigate/navigateAction.h>
#include <actionlib/server/simple_action_server.h>
#include <snoopy_localize/fuse.h>

using namespace std;
using namespace ros;
using namespace actionlib;

void shutDownCallback(int sig)
{
    ROS_INFO("CATCH SHUT DOWN");
    NodeHandle n;
    Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/snoopy/cmd_vel",10);
    geometry_msgs::Twist move_msg;
    move_msg.linear.x = 0.0;
    move_msg.linear.y = 0.0;
    move_msg.linear.z = 0.0;
    move_msg.angular.x = 0.0;
    move_msg.angular.y = 0.0;
    move_msg.angular.z = 0.0;
    for(int i = 0; i < 50; i++)
    {
        twist_pub.publish(move_msg);
        Duration(0.2).sleep();
    }

    shutdown();
}


class navigate
{
protected:
    NodeHandle n,n_priv;
    SimpleActionServer<snoopy_navigate::navigateAction> action;
    string navigator;
    snoopy_navigate::navigateFeedback feedback;
    snoopy_navigate::navigateResult result;
public:

    navigate(string name);
    ~navigate();
    Publisher twist_pub;
    //for testing
    Publisher angle_pub;
    geometry_msgs::Pose2D pre_pose;
    geometry_msgs::Pose2D current_pos;
    vector<geometry_msgs::Pose2D> goal_array;

    double _angular_limit, _linear_limit;
    double angle_diff(geometry_msgs::Pose2D pose, geometry_msgs::Pose2D goal);
    double dis_diff(geometry_msgs::Pose2D pose, geometry_msgs::Pose2D goal);
    void control_angular(double diff);
    void control_linear(double diff);
    void control_lin(double lin_diff, double ang_diff);
    void control_all(double lin_diff, double ang_diff);
    string _pre_update_type;


    ServiceClient fuse_client;
    snoopy_localize::fuse fuse;
private:
    void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
    void goalCallback(const nav_msgs::Path::ConstPtr& msg);
    void executeCB(const snoopy_navigate::navigateGoalConstPtr &goal);
    double saturate(int type, double control);
    double saturate_abhishek(int type, double control);
    Subscriber pose_sub;
    string _position_topic, _goal_topic;
    double _kp_angular, _kp_linear, _kp_angular2;
    bool _pre_update;
    double _conv_time;
    bool _fuse_switch;

};

navigate::navigate(string name) :
    action(n, name, boost::bind(&navigate::executeCB, this, _1), false),
    navigator(name),
    n_priv("~")
{
    action.start();
    ROS_INFO("_navigate_: Calling the constructor of class NAVIGATE");
    signal(SIGINT, shutDownCallback);
    n_priv.getParam("position_topic", _position_topic);
    n_priv.getParam("goal_topic", _goal_topic);
    n_priv.getParam("angular_limit",_angular_limit);
    n_priv.getParam("linear_limit",_linear_limit);
    n_priv.getParam("kp_angular",_kp_angular);
    n_priv.getParam("kp_angular2",_kp_angular2);
    n_priv.getParam("kp_linear",_kp_linear);
    n_priv.param("pre_update",_pre_update,false);
    n_priv.param("fuse_switch",_fuse_switch,true);
    n_priv.param("conv_time",_conv_time,2.0);
    n_priv.param<string>("pre_update_type",_pre_update_type,"ANY");
    twist_pub = n.advertise<geometry_msgs::Twist>("/snoopy/cmd_vel",10);
    pose_sub = n.subscribe<geometry_msgs::Pose2D>(_position_topic.data(), 5, &navigate::poseCallback, this);
    angle_pub = n.advertise<std_msgs::Float32>("angle_test",10);
    fuse_client = n.serviceClient<snoopy_localize::fuse>("fuse");
}

navigate::~navigate()
{
    ROS_INFO("_navigate_: Deleting an object of the class NAVIGATE");
}


void navigate::poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    //get the target pose
    current_pos.x = msg -> x;
    current_pos.y = msg -> y;
    current_pos.theta = fmod((msg -> theta + 2*M_PI),2*M_PI);
}


double navigate::saturate(int type, double control)
{
    if(type == 0)
    {
        if(control>_angular_limit)
        {
            return _angular_limit;
        }
        else if(control<-_angular_limit)
        {
            return -_angular_limit;
        }
        else{
            return control;
        }

    }
    if(type == 1)
    {
        if(control>_linear_limit)
        {
            return _linear_limit;
        }
        else if(control<-_linear_limit)
        {
            return -_linear_limit;
        }
        else{
            return control;
        }
    }
}

double navigate::saturate_abhishek(int type, double control)
{
    if(type == 0)
    {
        if(control>_angular_limit)
        {
            return _angular_limit;
        }
        else if(control<-_angular_limit)
        {
            return -_angular_limit;
        }
        else{
            return control;
        }

    }
    if(type == 1)
    {
        if(control>_linear_limit)
        {
            return _linear_limit;
        }
        else if(control<-_linear_limit)
        {
            return -_linear_limit;
        }
        else{
            return control;
        }
    }
}

double navigate::angle_diff(geometry_msgs::Pose2D pose, geometry_msgs::Pose2D goal)
{
    double x_diff = goal.x - pose.x;
    double y_diff = goal.y - pose.y;
    double goal_theta = fmod(atan2(y_diff,x_diff),2*M_PI);
    double theta_diff = goal_theta - pose.theta;
    theta_diff = fmod(theta_diff, 2*M_PI);
//    ROS_INFO("theta_diff: %f, pose_theta: %f", theta_diff, pose.theta);
    if(theta_diff >= M_PI) theta_diff = theta_diff - 2*M_PI;
    else if(theta_diff <= -M_PI) theta_diff = theta_diff + 2*M_PI;
//    ROS_INFO("transformed theta_diff: %f", theta_diff);
    //for testing
    std_msgs::Float32 msg;
    msg.data = theta_diff * 180/M_PI;
    angle_pub.publish(msg);
    return theta_diff;
}

double navigate::dis_diff(geometry_msgs::Pose2D pose, geometry_msgs::Pose2D goal)
{
    double x_diff = pose.x - goal.x;
    double y_diff = pose.y - goal.y;
    double distance_diff =  sqrt(pow(x_diff, 2) + pow(y_diff,2));
    return distance_diff;
}


void navigate::control_lin(double lin_diff, double ang_diff)
{
    double control = lin_diff * _kp_linear;
    double control_ang = ang_diff * _kp_angular2 ;
    geometry_msgs::Twist move_msg;
    move_msg.linear.x = saturate(1,control);
    move_msg.linear.y = 0.0;
    move_msg.linear.z = 0.0;
    move_msg.angular.x = 0.0;
    move_msg.angular.y = 0.0;
    float rotate_angle = -saturate(0,control_ang);
//    ROS_INFO("rotate angle: %f", rotate_angel);
    move_msg.angular.z = rotate_angle;
    twist_pub.publish(move_msg);
}

void navigate::control_all(double lin_diff, double ang_diff)
{
    double control = lin_diff * _kp_linear;
    double control_ang = ang_diff * _kp_angular2 ;
    geometry_msgs::Twist move_msg;
    move_msg.linear.x = saturate(1,control);
    move_msg.linear.y = 0.0;
    move_msg.linear.z = 0.0;
    move_msg.angular.x = 0.0;
    move_msg.angular.y = 0.0;
    float rotate_angle = -saturate(0,control_ang);
//    ROS_INFO("rotate angle: %f", rotate_angel);
    move_msg.angular.z = rotate_angle;
    twist_pub.publish(move_msg);
}

void navigate::control_angular(double diff)
{
    double control = diff * _kp_angular;
    geometry_msgs::Twist move_msg;
    move_msg.linear.x = 0.0;
    move_msg.linear.y = 0.0;
    move_msg.linear.z = 0.0;
    move_msg.angular.x = 0.0;
    move_msg.angular.y = 0.0;
    float rotate_angle = -saturate(0,control);
//    ROS_INFO("rotate angle: %f", rotate_angel);
    move_msg.angular.z = rotate_angle;
    twist_pub.publish(move_msg);
}

void navigate::control_linear(double diff)
{
    double control = diff * _kp_linear;
    geometry_msgs::Twist move_msg;
    move_msg.linear.x = saturate(1,control);
    move_msg.linear.y = 0.0;
    move_msg.linear.z = 0.0;
    move_msg.angular.x = 0.0;
    move_msg.angular.y = 0.0;
    move_msg.angular.z = 0.0;
    twist_pub.publish(move_msg);
}

void navigate::executeCB(const snoopy_navigate::navigateGoalConstPtr &path)
{
    goal_array.clear();
    for (size_t i = 0; i < path->path.poses.size(); i++) {
        geometry_msgs::Pose2D goal;
        goal.x = path->path.poses.at(i).pose.position.x;
        goal.y = path->path.poses.at(i).pose.position.y;
        goal_array.push_back(goal);
    }
    Rate loop_rate(20);
    bool done = false;
    while (ok() && !action.isPreemptRequested() && !done)
    {        
        for(size_t i = 0; i < goal_array.size(); i++)
        {
            if(_pre_update && _fuse_switch)
            {
                if(_pre_update_type == "FIRST" && i == 0)
                {
                    ROS_INFO("_navigate_: PRE UPDATE, WAITING PF CONVERGENCE.");
                    sleep(_conv_time);
                    fuse.request.cmd.data = "ALL";
                    if(fuse_client.call(fuse)) ROS_INFO("_navigate_: CALLED FOR FUSION SUCCEDED.");
                    else                       ROS_INFO("_navigate_: CALLED FOR FUSION FAILED.");
                }
                else if(_pre_update_type == "ANY")
                {
                    ROS_INFO("_navigate_: PRE UPDATE, WAITING PF CONVERGENCE.");
                    sleep(_conv_time);
                    fuse.request.cmd.data = "ALL";
                    if(fuse_client.call(fuse)) ROS_INFO("_navigate_: CALLED FOR FUSION SUCCEDED.");
                    else                       ROS_INFO("_navigate_: CALLED FOR FUSION FAILED.");
                }
            }

            bool isUpdated = false;
            while(true)
            {
                Time t = Time::now();
                spinOnce();

                if(action.isPreemptRequested()) break;

                double linear_dist = dis_diff(current_pos,goal_array[i]);
                double angular_dist = angle_diff(current_pos,goal_array[i]);
                if(abs(linear_dist) > 0.08)
                {

                    if(abs(fmod(angular_dist,2*M_PI)) > 5*(M_PI/180))
                    {
                        control_angular(angular_dist);
                        loop_rate.sleep();
                    }
                    else
                    {
                        if(_fuse_switch)
                        {
                            if(!isUpdated)
                            {
                                ROS_INFO("_navigate_: WAITING PF CONVERGENCE.");
                                sleep(_conv_time);
                                fuse.request.cmd.data = "ALL";
                                if(fuse_client.call(fuse))
                                {
                                    ROS_INFO("_navigate_: CALLED FOR FUSION SUCCEDED.");
                                    isUpdated = true;
                                }
                                else ROS_INFO("_navigate_: CALLED FOR FUSION FAILED.");
                            }
                        }

                        control_lin(linear_dist, angular_dist);
                        loop_rate.sleep();
                    }

                }
                else break;
            }//end while true

            if(abs(dis_diff(current_pos,goal_array[i])) < 0.08)
            {
                //push back feedback to actionlib
                feedback.sequence.push_back(goal_array[i]);
                //publish feedback
                action.publishFeedback(feedback);

                result.stop_point = goal_array[i];
                if(result.stop_point.x == goal_array.back().x &&
                   result.stop_point.y == goal_array.back().y)
                {
                    ROS_INFO("%s: Succeeded", navigator.c_str());
                    //set the action state to succeeded
                    action.setSucceeded(result);
                    done = true;
                }
            }
        }

        if(action.isPreemptRequested()) break;
    }//end while

    if(action.isPreemptRequested() || !ok())
    {
        ROS_INFO("_navigate_: ACTION PREEMPTED");
        action.setPreempted();
    }

}
int main(int argc, char** argv)
{
    init(argc, argv, "navigate");
    string name = "navigate";
    navigate nav(name);
    spin();

    return 0;
}

//__________________________________________________________________________________

//double linear_dist = dis_diff(current_pos,goal_array[i]);
//double angular_dist = angle_diff(current_pos,goal_array[i]);

//ROS_INFO("_navigate_: TARGET X: %f Y: %f", goal_array[i].x, goal_array[i].y);
//while(1)
//{
//    while(abs(fmod(angular_dist,2*M_PI)) > 5*(M_PI/180))
//    {
//        ROS_INFO("_navigate_: ANGULAR CONTROL, TARGET %d, ANG DIST: %f",int(i),angular_dist);
//        control_angular(angular_dist);
//        spinOnce();
//        angular_dist = angle_diff(current_pos,goal_array[i]);
//        if(action.isPreemptRequested()) break;
//        loop_rate.sleep();
//    }

//    fuse.request.cmd.data = "ALL";
//    for(int j = 0; j < 100; j++)
//    {
//        if(fuse_client.call(fuse)) ROS_INFO("_navigate_: CALLED FOR FUSION SUCCEDED.");
//        else ROS_INFO("_navigate_: CALLED FOR FUSION FAILED.");
//        sleep(0.05);
//    }
//    //update second time
//    if(fuse_client.call(fuse)) ROS_INFO("_navigate_: CALLED FOR FUSION SUCCEDED.");
//    else ROS_INFO("_navigate_: CALLED FOR FUSION FAILED.");
//    spinOnce();
//    angular_dist = angle_diff(current_pos,goal_array[i]);

//    while(abs(linear_dist) > 0.08 && abs(fmod(angular_dist,2*M_PI)) < 5*(M_PI/180))
//    {
//        ROS_INFO("_navigate_: LINEAR CONTROL, TARGET %d, LIN DIST: %f, ANG DIST: %f",int(i),linear_dist, angular_dist);
//        //control_linear(linear_dist);
//        control_lin(linear_dist, angular_dist);
//        spinOnce();
//        linear_dist = dis_diff(current_pos,goal_array[i]);
//        angular_dist = angle_diff(current_pos,goal_array[i]);

//        if(action.isPreemptRequested()) break;
//        loop_rate.sleep();
//    }
//    if(abs(linear_dist) < 0.08)
//    {
//        //push back feedback to actionlib
//        feedback.sequence.push_back(goal_array[i]);
//        //publish feedback
//        action.publishFeedback(feedback);

//        result.stop_point = goal_array[i];
//        if(result.stop_point.x == goal_array.back().x &&
//           result.stop_point.y == goal_array.back().y)
//        {
//            ROS_INFO("%s: Succeeded", navigator.c_str());
//            //set the action state to succeeded
//            action.setSucceeded(result);
//            done = true;
//        }
//        break;
//    }

////                fuse.request.cmd.data = "ALL";
////                if(fuse_client.call(fuse)) ROS_INFO("_navigate_: CALLED FOR FUSION SUCCEDED.");
////                else ROS_INFO("_navigate_: CALLED FOR FUSION FAILED.");
//}
