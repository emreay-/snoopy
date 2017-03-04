/*
Preprocess the shizz from the RGB image and Depth Image
*/
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Point.h>
#include <string.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/visualization/pcl_visualizer.h>

//____________C++_____________//
#include <algorithm>
//________SNOOPY______________//
#include "snoopy_map/ogm_update.h"

using namespace ros;
using namespace std;
using namespace pcl;

bool verbose(true);

class objDet
{
public:
    objDet();
    ~objDet();
    Publisher pub, pub_objFound;
    Subscriber sub;

    PointCloud<PointXYZRGB> cloud;

    PointCloud<PointXYZRGB>::Ptr object_detection;

    std_msgs::String sound_out;
    std_msgs::Bool object_in_scene;

    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input);
    void cluster_scene();
    void find_objPose(const std_msgs::StringConstPtr& msg);


private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    ServiceClient map_update;
    snoopy_map::ogm_update map_srv;
    geometry_msgs::Pose2D obj_pose;
    string op_var;
};

objDet::objDet() : nh_priv("~")
{
    // sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, &preproc::pcl_callback, this);
    sub = nh.subscribe<sensor_msgs::PointCloud2>("/snoopy/pcl_converted", 10, &objDet::pcl_callback, this);
    pub = nh.advertise<PointCloud<PointXYZRGB> >("/snoopy/find_object_pcl",100);
    pub_objFound = nh.advertise <std_msgs::Bool>("/snoopy/obj_found",100);
    map_update = nh.serviceClient<snoopy_map::ogm_update>("ogm_update");

}

objDet::~objDet()
{}

void objDet::find_objPose(const std_msgs::StringConstPtr& msg)
{
    double x_obj,y_obj,z_obj;
    geometry_msgs::PointStamped odom_obj_pose, object_pose;

    int count =0;
    //Find Pose

    object_pose.header.frame_id = "/base_link";
    object_pose.header.stamp = Time();
    object_pose.point.x = x_obj/count;
    object_pose.point.y = y_obj/count;
    object_pose.point.z = 0;


    tf::TransformListener listener;
    listener.waitForTransform("/base_link","/odom",Time(0),Duration(10,0));

    try
    {
        ROS_INFO("IM TRANSFORMING A POint");
        listener.transformPoint("/odom", object_pose, odom_obj_pose);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Exception received: %s",ex.what());
    }

    if(verbose) ROS_INFO("X POSE:%f  Y POSE:%f", x_obj/count, y_obj/count);

    obj_pose.x = odom_obj_pose.point.x;
    obj_pose.y = odom_obj_pose.point.y;

    map_srv.request.update_cell = obj_pose;
    map_srv.request.cmd.data = "FILL" ;

    if(map_update.call(map_srv))
    {
        ROS_INFO("Snoopy wants to update the map");
    }


}


void objDet::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //covert from ros msgs to pcl with only depth info
    cloud.clear();
    fromROSMsg (*input, cloud);

    object_detection = PointCloud<PointXYZRGB>::Ptr (new PointCloud<PointXYZRGB>);

    //filter the NaNs
    PointCloud<PointXYZRGB>::Ptr cloudNanRemoved(new PointCloud<PointXYZRGB>);
    vector<int> indices;
    removeNaNFromPointCloud(cloud,*cloudNanRemoved, indices);

    ///////////////////////
    /*
    Passthrough Filtering (remove points more than 80cm away from camera and less than 1cm
    from the floor)
    */
    ///////////////////////
    PointCloud<PointXYZRGB>::Ptr cloud_filtered (new PointCloud<PointXYZRGB>);
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud (cloudNanRemoved);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.01f,0.5f);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.00f,0.05f);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.12f,0.12f);
    pass.filter (*cloud_filtered);


    //////////////////
    //Voxel Filtering
    //////////////////
    PointCloud<PointXYZRGB>::Ptr cloud_voxeld(new PointCloud<PointXYZRGB>);
    VoxelGrid<PointXYZRGB> vox;
    vox.setInputCloud (cloud_filtered);
    float leaf = 0.002f;
    vox.setLeafSize(leaf,leaf,leaf);
    vox.filter(*cloud_voxeld);
//    cloud_voxeld.swap(cloud_filtered);

    // Create the segmentation object
    SACSegmentation<PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.01);

    //Extract Indices
    ExtractIndices<PointXYZRGB> extract;
    int points_num= (int)cloud_voxeld->points.size();
    int i=0;
    while(cloud_voxeld->points.size() > 0.20* points_num)
    {
        i++;
        ModelCoefficients::Ptr coeff(new ModelCoefficients);
        PointIndices::Ptr inliers(new PointIndices);
        seg.setInputCloud (cloud_voxeld);
        seg.segment (*inliers, *coeff);
        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            break;
        }
//        ROS_INFO("_OBJECT AVOID VISION_: Cloud Size Pre Removal %d", (int)cloud_voxeld->size());
//        ROS_INFO("_OBJECT AVOID VISION_: Inliers Size %d at attempt number %d", (int)inliers->indices.size(), i);

        if((int) inliers->indices.size() > 9000)
        {
            extract.setInputCloud(cloud_voxeld);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*object_detection);
//            ROS_INFO("_OBJECT AVOID_: Cloud Size Post Removal %d", (int)object_detection->size());
            cloud_voxeld.swap(object_detection);
        }
        if((int) inliers->indices.size() < 9000)
        {
             break;
        }

    }

    int cloud_size = (int)cloud_voxeld->size();
//    ROS_INFO("_OBJECT AVOID_:Cloud Size %d", cloud_size);

    cloud_voxeld->header.frame_id = cloudNanRemoved->header.frame_id;
    pub.publish(cloud_voxeld);

    int upper_threshold = 9000;
    int lower_threshold = 1000;
    object_in_scene.data = false;
    if(cloud_size < upper_threshold && cloud_size > lower_threshold)
    {
        double x_avg=0;
        double y_avg=0;
        double z_avg=0;
        object_in_scene.data = true;
//        for(size_t i=0; i < object_detection->size();i++)
//        {
//            x_avg += object_detection->points[i].x;
//            y_avg += object_detection->points[i].z;
//            z_avg += object_detection->points[i].y;
//        }
//        if(z_avg < 0.5)
//        {
//            object_in_scene.data = true;
//        }
    }
    pub_objFound.publish(object_in_scene);



    // Creating the KdTree object for the search method of the extraction
//    search::KdTree<pcl::PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
//    tree->setInputCloud (cloud_voxeld);

//    vector<PointIndices> cluster_indices;
//    EuclideanClusterExtraction<PointXYZRGB> ec;
//    ec.setClusterTolerance (0.001); // 1mm
//    ec.setMinClusterSize (1000);
//    ec.setMaxClusterSize (50000);
//    ec.setSearchMethod (tree);
//    ec.setInputCloud (cloud_voxeld);
//    ec.extract (cluster_indices);



//    int j = 0;
//    for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//    {
//        PointCloud<PointXYZRGB>::Ptr cloud_cluster (new PointCloud<pcl::PointXYZRGB>);

//        for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//        {
//            cloud_cluster->points.push_back (cloud_voxeld->points[*pit]); //*
//        }

//        cloud_cluster->width = cloud_cluster->points.size ();
//        cloud_cluster->height = 1;
//        cloud_cluster->is_dense = true;

//        ROS_INFO("Cluster Number %d",j);
//        j++;
//    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_detect");
    objDet snoopy_detect;
    Duration(2).sleep();

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    while(ok())
    {
        spinOnce();
//        snoopy_detect.pub_objFound.publish(snoopy_detect.object_in_scene);
    }
}
