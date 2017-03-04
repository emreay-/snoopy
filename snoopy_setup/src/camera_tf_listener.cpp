#include <ros/ros.h>
#include <ros/time.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>

using namespace ros;

class cameraListener
{
public:
    cameraListener();
    ~cameraListener();
    NodeHandle n;
    void transformIt();
    Publisher pub;

private:

    Subscriber sub;
    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input);
    tf::TransformListener listener;
    pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud_converted;
    std::string target_frame;
    std::string source_frame;
};

cameraListener::cameraListener()
{
    ROS_INFO("CONSTRUCTOR LISTENER CAMERA");
    sub = n.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points",5,&cameraListener::pcl_callback,this);
    pub = n.advertise< sensor_msgs::PointCloud2 >("snoopy/pcl_converted",10);
    target_frame = "base_link";
    source_frame = "camera_rgb_optical_frame";
}

cameraListener::~cameraListener()
{
    ROS_INFO("DESTRUCTOR LISTENER CAMERA");
}

void cameraListener::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //covert from ros msgs to pcl
    cloud.clear();
    pcl::fromROSMsg (*input, cloud);
}

void cameraListener::transformIt()
{
    cloud_converted.clear();

    tf::StampedTransform transform;

    try
    {
        ROS_INFO("AT LOOK UP");
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
    }
    catch(tf::LookupException ex)
    {
        ROS_ERROR("Exception received: %s",ex.what());
    }

    try
    {
        ROS_INFO("AT TRANSFORM");
        pcl_ros::transformPointCloud(cloud,cloud_converted,transform);
    }

    catch(tf::TransformException ex)
    {
        ROS_ERROR("Exception received: %s",ex.what());
    }

    cloud_converted.header.frame_id = target_frame;
    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(cloud_converted, output_cloud);

    pub.publish(output_cloud);
}

int main(int argc, char* argv[])
{
    init(argc,argv,"camera_tf_listener");
    cameraListener cam_lis;
    Rate r(20);
    Duration(5).sleep();
    while(ok())
    {
        spinOnce();
        cam_lis.transformIt();
        r.sleep();
    }
}

// -----------------------------------------------------------

//void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input)
//{
//    //covert from ros msgs to pcl
//    pcl::PointCloud<pcl::PointXYZRGB> cloud, cloud_converted;
//    pcl::fromROSMsg (*input, cloud);
//    tf::StampedTransform transform;
//    tf::TransformListener listener;
//    try
//    {
//        ROS_INFO("AT LOOK UP");
//        listener.lookupTransform("/odom", input->header.frame_id, ros::Time(0), transform);
//    }
//    catch(tf::LookupException ex)
//    {
//        ROS_ERROR("Exception received: %s",ex.what());
//    }

//    try
//    {
//        ROS_INFO("AT TRANSFORM");
//        pcl_ros::transformPointCloud(cloud,cloud_converted,transform);
//    }

//    catch(tf::TransformException ex)
//    {
//        ROS_ERROR("Exception received: %s",ex.what());
//    }

//    cloud_converted.header.frame_id = "/odom";
//    sensor_msgs::PointCloud2 output_cloud;
//    pcl::toROSMsg(cloud_converted, output_cloud);

//    pub.publish(output_cloud);

////    const std::string base_link = "base_link";

//////    listener.lookupTransform(base_link,cloud.header.frame_id,Time(0),transform);
//////    listener.waitForTransform("/base_link",cloud.header.frame_id,Time(0),Duration(10.0));
////      if(!has_transform)
////      {
////          listener.waitForTransform(base_link,input->header.frame_id,Time(0),Duration(10.0));
////          has_transform = true;
////      }
//////    listener.lookupTransform(base_link,input->header.frame_id,Time(0),transform);

////  try
////  {
////    ROS_INFO("IM TRANSFORMING PCL");
//////    pcl_ros::transformPointCloud("/base_link", cloud, cloud_converted, listener);
//////    pcl_ros::transformPointCloud(cloud, cloud_converted, transform);
//////      pcl_ros::transformPointCloud(base_link, transform, *input, cloud_converted);
////    pcl_ros::transformPointCloud(base_link,*input,cloud_converted,listener);

////  }
////  catch(tf::TransformException& ex)
////  {
////    ROS_ERROR("Exception received: %s",ex.what());
////  }
////  pub.publish(cloud_converted);
//}


//int main(int argc, char* argv[])
//{
//    init(argc,argv,"camera_tf_listener");
//    NodeHandle n;
//    Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 5,pcl_callback);
////    pub = n.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("snoopy/pcl_converted",10);
//    pub = n.advertise< sensor_msgs::PointCloud2 >("snoopy/pcl_converted",10);
//    spin();
//}

// OLD CALLBACK WHERE X->Y Y->X CONVERTED HERE

//void objDetCallback(const geometry_msgs::Point::ConstPtr& msg)
//{
//  tf::TransformListener listener;
//  ROS_WARN("This is a callback");
//  listener.waitForTransform("/camera_link","/odom",Time(0),Duration(10,0));
//  ROS_WARN("This is a post transform callback");
//  geometry_msgs::PointStamped object_pose;
//  geometry_msgs::PointStamped odom_obj_pose;
//  object_pose.header.frame_id = "/camera_link";
//  object_pose.header.stamp = Time();
//  object_pose.point.x = msg->z;
//  object_pose.point.y = -msg->x;
//  object_pose.point.z = -msg->y;

//  try
//  {
//    ROS_INFO("IM TRANSFORMING A POint");
//    listener.transformPoint("/odom", object_pose, odom_obj_pose);
//  }


//  catch(tf::TransformException& ex)
//  {
//    ROS_ERROR("Exception received asdgasdfasdfas: %s",ex.what());
//  }
//  pub.publish(odom_obj_pose);

//}

// -----------------------------------------------------------

// SECOND OLDEST CODE WHERE WE DIRECTLY TRIED TO CONVERT X->X Y->Y ETC.

//void objDetCallback(const geometry_msgs::Point::ConstPtr& msg)
//{
//  tf::TransformListener listener;
//  ROS_WARN("This is a callback");
//  listener.waitForTransform("/camera_link","/odom",Time(0),Duration(10,0));
//  ROS_WARN("This is a post transform callback");
//  geometry_msgs::PointStamped object_pose;
//  geometry_msgs::PointStamped odom_obj_pose;
//  object_pose.header.frame_id = "/camera_link";
//  object_pose.header.stamp = Time();
//  object_pose.point.x = msg->x;
//  object_pose.point.y = msg->y;
//  object_pose.point.z = msg->z;

//  try
//  {
//    ROS_INFO("IM TRANSFORMING A POint");
//    listener.transformPoint("/base_link", object_pose, odom_obj_pose);
//  }


//  catch(tf::TransformException& ex)
//  {
//    ROS_ERROR("Exception received asdgasdfasdfas: %s",ex.what());
//  }
//  pub.publish(odom_obj_pose);

//}


//int main(int argc, char* argv[])
//{
//	init(argc,argv,"camera_tf_listener");
//	NodeHandle n;
//	Rate r(10);
//    pub = n.advertise<geometry_msgs::PointStamped>("/tfed",10);
//	Subscriber camera_sub = n.subscribe<geometry_msgs::Point>("/snoopy/camera/object_pose",10,objDetCallback);
//	while(ok())
//	{
//		spinOnce();
//		r.sleep();
//	}
//}

// -----------------------------------------------------------
