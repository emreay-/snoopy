/*
Preprocess the shizz from the RGB image and Depth Image
*/
#include "std_msgs/String.h"
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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>

#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>

#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/surface/mls.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

//____________C++_____________//
#include <algorithm>
//________SNOOPY______________//
#include "snoopy_map/ogm_update.h"

using namespace ros;
using namespace std;
using namespace pcl;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (true);
bool use_cloud_resolution_ (true);
bool use_hough_ (true);
bool resolved(false);
bool verbose (false);

bool found(false);
bool cube_found(false);
bool sphere_found(false);
bool tri_found(false);
bool hcube_found(false);
bool cylinder_found(false);
bool cross_found(false);
bool patric_found(false);

int found_count_= 0;
int number_shapes = 7;
//_____Cube=0, Triangle=1, HCube=2, Sphere=3, Cylinder=4 Cross=5 Patric=6_____//
std::vector<int> shape_count (number_shapes,0);

PointCloud<PointXYZRGB>::Ptr cube_model(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr hcube_model(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cylinder_model(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr triangle_model(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr sphere_model(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cross_model(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr patric_model(new PointCloud<PointXYZRGB>);

PointCloud<PointXYZRGB>::Ptr cube_key(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr hcube_key(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cylinder_key(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr triangle_key(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr sphere_key(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr cross_key(new PointCloud<PointXYZRGB>);
PointCloud<PointXYZRGB>::Ptr patric_key(new PointCloud<PointXYZRGB>);

PointCloud<Normal>::Ptr cube_normals(new PointCloud<Normal> ());
PointCloud<Normal>::Ptr hcube_normals(new PointCloud<Normal> ());
PointCloud<Normal>::Ptr cylinder_normals(new PointCloud<Normal> ());
PointCloud<Normal>::Ptr triangle_normals(new PointCloud<Normal> ());
PointCloud<Normal>::Ptr sphere_normals(new PointCloud<Normal> ());
PointCloud<Normal>::Ptr cross_normals(new PointCloud<Normal> ());
PointCloud<Normal>::Ptr patric_normals(new PointCloud<Normal> ());

PointCloud<SHOT352>::Ptr cube_descriptors(new PointCloud<SHOT352> ());
PointCloud<SHOT352>::Ptr hcube_descriptors(new PointCloud<SHOT352> ());
PointCloud<SHOT352>::Ptr cylinder_descriptors(new PointCloud<SHOT352> ());
PointCloud<SHOT352>::Ptr triangle_descriptors(new PointCloud<SHOT352> ());
PointCloud<SHOT352>::Ptr sphere_descriptors(new PointCloud<SHOT352> ());
PointCloud<SHOT352>::Ptr cross_descriptors(new PointCloud<SHOT352> ());
PointCloud<SHOT352>::Ptr patric_descriptors(new PointCloud<SHOT352> ());

PointCloud<ReferenceFrame>::Ptr cube_rf(new PointCloud<ReferenceFrame> ());
PointCloud<ReferenceFrame>::Ptr hcube_rf(new PointCloud<ReferenceFrame> ());
PointCloud<ReferenceFrame>::Ptr cylinder_rf(new PointCloud<ReferenceFrame> ());
PointCloud<ReferenceFrame>::Ptr triangle_rf(new PointCloud<ReferenceFrame> ());
PointCloud<ReferenceFrame>::Ptr sphere_rf(new PointCloud<ReferenceFrame> ());
PointCloud<ReferenceFrame>::Ptr cross_rf(new PointCloud<ReferenceFrame> ());
PointCloud<ReferenceFrame>::Ptr patric_rf(new PointCloud<ReferenceFrame> ());


class preproc
{
public:
    preproc();
    ~preproc();
    Publisher pub,pub_sound, pub_pose;
    Subscriber sub;

    PointCloud<int> KI_cube,KI_hcube, KI_triangle, KI_cylinder, KI_sphere, KI_cross, KI_patric;
    PointCloud<PointXYZRGB> cloud;

    vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    vector<Correspondences> clustered_corrs;

    PointCloud<PointXYZRGB>::Ptr scene_keypoints;
    PointCloud<Normal>::Ptr scene_normals;
    PointCloud<SHOT352>::Ptr scene_descriptors;
    PointCloud<PointXYZRGB>::Ptr plane_removed;

    std_msgs::String sound_out;


//    PointCloud<PointXYZRGB>::Ptr plane_removed;
    string shape_detect(string shape);
    void read_models();
    void proc_models();
    void preprocess_scene();
    void pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input);
    void cluster_scene();
    void find_objPose();
    double computeCloudResolution (const PointCloud<PointXYZRGB>::ConstPtr &cloud);


private:
    double model_ss_,scene_ss_,rf_rad_m,rf_rad_s,descr_rad_s,descr_rad_m,cg_size_,cg_thresh_,k_neigh,neigh_dist,rad;
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    ServiceClient map_update;
    snoopy_map::ogm_update map_srv;
    geometry_msgs::Pose2D obj_pose;
    string op_var;
};

preproc::preproc() : nh_priv("~")
{

    // sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, &preproc::pcl_callback, this);
    sub = nh.subscribe<sensor_msgs::PointCloud2>("/snoopy/pcl_converted", 10, &preproc::pcl_callback, this);

    pub = nh.advertise< PointCloud<PointXYZRGB> >("/snoopy/shape_detected",100);
    pub_pose = nh.advertise<geometry_msgs::PointStamped> ("/snoopy/object_pose",100);

    pub_sound = nh.advertise< std_msgs::String >("/snoopy/shape_string",5);
    map_update = nh.serviceClient<snoopy_map::ogm_update>("ogm_update");

    nh_priv.getParam("modelss",model_ss_);
    nh_priv.getParam("sceness",scene_ss_);
    nh_priv.getParam("rfrad_mod",rf_rad_m);
    nh_priv.getParam("rfrad_scene",rf_rad_s);
    nh_priv.getParam("descrad_mod",descr_rad_m);
    nh_priv.getParam("descrad_scene",descr_rad_s);
    nh_priv.getParam("cgsize",cg_size_);
    nh_priv.getParam("cgthresh",cg_thresh_);
    nh_priv.getParam("kneigh",k_neigh);
    nh_priv.getParam("neigh_dist",neigh_dist);
    nh_priv.getParam("debug_sph_rad",rad);
    nh_priv.getParam("operate_var",op_var);

}

preproc::~preproc()
{}

double preproc::computeCloudResolution (const PointCloud<PointXYZRGB>::ConstPtr &cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  vector<int> indices (2);
  vector<float> sqr_distances (2);
  search::KdTree<PointXYZRGB> tree;
  tree.setInputCloud (cloud);

  for (size_t i = 0; i < cloud->size (); ++i)
  {
    if (! pcl_isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

void preproc::find_objPose()
{
    double x_obj,y_obj;
    geometry_msgs::PointStamped odom_obj_pose, object_pose;

    int count =0;
    for (size_t i = 0; i < rototranslations.size (); ++i)
    {
        for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
        {
            if (cube_found) PointXYZRGB& model_point           = cube_key->at (clustered_corrs[i][j].index_query);
            else if (hcube_found) PointXYZRGB& model_point     = hcube_key->at (clustered_corrs[i][j].index_query);
            else if (cylinder_found) PointXYZRGB& model_point  = cylinder_key->at (clustered_corrs[i][j].index_query);
            else if (sphere_found) PointXYZRGB& model_point    = sphere_key->at (clustered_corrs[i][j].index_query);
            else if (tri_found) PointXYZRGB& model_point       = triangle_key->at (clustered_corrs[i][j].index_query);
            else if (cross_found) PointXYZRGB& model_point     = cross_key->at (clustered_corrs[i][j].index_query);
            else if (patric_found) PointXYZRGB& model_point    = patric_key->at (clustered_corrs[i][j].index_query);

            PointXYZRGB& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

            x_obj += scene_point.x;
            y_obj += scene_point.y;
            count++;
        }
    }

    object_pose.header.frame_id = "/base_link";
    object_pose.header.stamp = Time();
    object_pose.point.x = x_obj/count;
    object_pose.point.y = y_obj/count;
    object_pose.point.z = 0;

    tf::TransformListener listener;
    listener.waitForTransform("/base_link","/odom",Time(0),Duration(10,0));

    try
    {
//        ROS_INFO("IM TRANSFORMING A Point");
        listener.transformPoint("/base_link", object_pose, odom_obj_pose);
    }
    catch(tf::TransformException& ex)
    {
        ROS_ERROR("Exception received: %s",ex.what());
    }

    if(verbose)ROS_ERROR("_OBJ IDENT:_X POSE:%f  Y POSE:%f", x_obj/count, y_obj/count);

    obj_pose.x = odom_obj_pose.point.x;
    obj_pose.y = odom_obj_pose.point.y;
    pub_pose.publish(odom_obj_pose);
}

void preproc::read_models()
{
    string cube= "/home/ras23/catkin_ws/src/snoopy/snoopy_vision/models/cube.pcd";
    string hollow_cube= "/home/ras23/catkin_ws/src/snoopy/snoopy_vision/models/hcube.pcd";
    string triangle= "/home/ras23/catkin_ws/src/snoopy/snoopy_vision/models/triangle.pcd";
    string sphere= "/home/ras23/catkin_ws/src/snoopy/snoopy_vision/models/sphere.pcd";
    string cylinder= "/home/ras23/catkin_ws/src/snoopy/snoopy_vision/models/cylinder.pcd";
    string cross= "/home/ras23/catkin_ws/src/snoopy/snoopy_vision/models/cross.pcd";
    string patric= "/home/ras23/catkin_ws/src/snoopy/snoopy_vision/models/patric.pcd";



    if (io::loadPCDFile(cube, *cube_model) < 0)
    {
        cout << "Error loading cube model " << endl;
    }
    if (io::loadPCDFile(triangle, *triangle_model) < 0)
    {
        cout << "Error loading triangle model " << endl;
    }
    if (io::loadPCDFile(sphere, *sphere_model) < 0)
    {
        cout << "Error loading triangle model " << endl;
    }
    if (io::loadPCDFile(cylinder, *cylinder_model) < 0)
    {
        cout << "Error loading cylinder model " << endl;
    }
    if (io::loadPCDFile(hollow_cube, *hcube_model) < 0)
    {
        cout << "Error loading hollow cube model " << endl;
    }
    if (io::loadPCDFile(cross, *cross_model) < 0)
    {
        cout << "Error loading cross model " << endl;
    }
    if (io::loadPCDFile(patric, *patric_model) < 0)
    {
        cout << "Error loading patric model " << endl;
    }

}

void preproc::proc_models()
{
    //
    //  Compute Normals
    //
    NormalEstimationOMP<PointXYZRGB, Normal> norm_est;
    norm_est.setKSearch (k_neigh);
    norm_est.setInputCloud (cube_model);
    norm_est.compute (*cube_normals);

    norm_est.setInputCloud (hcube_model);
    norm_est.compute (*hcube_normals);

    norm_est.setInputCloud (cylinder_model);
    norm_est.compute (*cylinder_normals);

    norm_est.setInputCloud (triangle_model);
    norm_est.compute (*triangle_normals);

    norm_est.setInputCloud (sphere_model);
    norm_est.compute (*sphere_normals);

    norm_est.setInputCloud (cross_model);
    norm_est.compute (*cross_normals);

    norm_est.setInputCloud (patric_model);
    norm_est.compute (*patric_normals);

    //
    //  Downsample Clouds to Extract keypoints
    //
    UniformSampling<PointXYZRGB> uniform_sampling;
    uniform_sampling.setRadiusSearch (model_ss_);

    uniform_sampling.setInputCloud (cube_model);
    uniform_sampling.compute(KI_cube);
    copyPointCloud(*cube_model, KI_cube.points, *cube_key);

    uniform_sampling.setInputCloud (hcube_model);
    uniform_sampling.compute(KI_hcube);
    copyPointCloud(*hcube_model, KI_hcube.points, *hcube_key);

    uniform_sampling.setInputCloud (cylinder_model);
    uniform_sampling.compute(KI_cylinder);
    copyPointCloud(*cylinder_model, KI_cylinder.points, *cylinder_key);

    uniform_sampling.setInputCloud (triangle_model);
    uniform_sampling.compute(KI_triangle);
    copyPointCloud(*triangle_model, KI_triangle.points, *triangle_key);

    uniform_sampling.setInputCloud (sphere_model);
    uniform_sampling.compute(KI_sphere);
    copyPointCloud(*sphere_model, KI_sphere.points, *sphere_key);

    uniform_sampling.setInputCloud (cross_model);
    uniform_sampling.compute(KI_cross);
    copyPointCloud(*cross_model, KI_cross.points, *cross_key);

    uniform_sampling.setInputCloud (patric_model);
    uniform_sampling.compute(KI_patric);
    copyPointCloud(*patric_model, KI_patric.points, *patric_key);


    //
    //  Compute Descriptor for keypoints
    //
    SHOTEstimationOMP<PointXYZRGB, Normal,SHOT352> descr_est;
    descr_est.setRadiusSearch (descr_rad_m);
    descr_est.setInputCloud (cube_key);
    descr_est.setInputNormals (cube_normals);
    descr_est.setSearchSurface (cube_model);
    descr_est.compute (*cube_descriptors);

    descr_est.setInputCloud (hcube_key);
    descr_est.setInputNormals (hcube_normals);
    descr_est.setSearchSurface (hcube_model);
    descr_est.compute (*hcube_descriptors);

    descr_est.setInputCloud (cylinder_key);
    descr_est.setInputNormals (cylinder_normals);
    descr_est.setSearchSurface (cylinder_model);
    descr_est.compute (*cylinder_descriptors);

    descr_est.setInputCloud (triangle_key);
    descr_est.setInputNormals (triangle_normals);
    descr_est.setSearchSurface (triangle_model);
    descr_est.compute (*triangle_descriptors);

    descr_est.setInputCloud (sphere_key);
    descr_est.setInputNormals (sphere_normals);
    descr_est.setSearchSurface (sphere_model);
    descr_est.compute (*sphere_descriptors);

    descr_est.setInputCloud (cross_key);
    descr_est.setInputNormals (cross_normals);
    descr_est.setSearchSurface (cross_model);
    descr_est.compute (*cross_descriptors);

    descr_est.setInputCloud (patric_key);
    descr_est.setInputNormals (patric_normals);
    descr_est.setSearchSurface (patric_model);
    descr_est.compute (*patric_descriptors);

    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //

    BOARDLocalReferenceFrameEstimation<PointXYZRGB, Normal, ReferenceFrame> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_m);

    rf_est.setInputCloud (cube_key);
    rf_est.setInputNormals (cube_normals);
    rf_est.setSearchSurface (cube_model);
    rf_est.compute (*cube_rf);

    rf_est.setInputCloud (hcube_key);
    rf_est.setInputNormals (hcube_normals);
    rf_est.setSearchSurface (hcube_model);
    rf_est.compute (*hcube_rf);

    rf_est.setInputCloud (cylinder_key);
    rf_est.setInputNormals (cylinder_normals);
    rf_est.setSearchSurface (cylinder_model);
    rf_est.compute (*cylinder_rf);

    rf_est.setInputCloud (triangle_key);
    rf_est.setInputNormals (triangle_normals);
    rf_est.setSearchSurface (triangle_model);
    rf_est.compute (*triangle_rf);

    rf_est.setInputCloud (sphere_key);
    rf_est.setInputNormals (sphere_normals);
    rf_est.setSearchSurface (sphere_model);
    rf_est.compute (*sphere_rf);

    rf_est.setInputCloud (cross_key);
    rf_est.setInputNormals (cross_normals);
    rf_est.setSearchSurface (cross_model);
    rf_est.compute (*cross_rf);

    rf_est.setInputCloud (patric_key);
    rf_est.setInputNormals (patric_normals);
    rf_est.setSearchSurface (patric_model);
    rf_est.compute (*patric_rf);

}

void preproc::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //covert from ros msgs to pcl with only depth info
    cloud.clear();
    fromROSMsg (*input, cloud);
}

void preproc::preprocess_scene()
{
    plane_removed = PointCloud<PointXYZRGB>::Ptr (new PointCloud<PointXYZRGB>);
    scene_keypoints = PointCloud<PointXYZRGB>::Ptr (new PointCloud<PointXYZRGB>);
    scene_descriptors = PointCloud<SHOT352>::Ptr (new PointCloud<SHOT352>);
    scene_normals = PointCloud<Normal>::Ptr (new PointCloud<Normal>);

    found = false;
    cube_found = false;
    sphere_found = false;
    tri_found = false;
    hcube_found = false;
    cylinder_found = false;
    cross_found = false;
    patric_found = false;

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
    pass.setFilterLimits (0.002f,0.05f);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.1f,0.1f);
    pass.filter (*cloud_filtered);


    //////////////////
    //Voxel Filtering
    //////////////////
    PointCloud<PointXYZRGB>::Ptr cloud_voxeld(new PointCloud<PointXYZRGB>);
    VoxelGrid<PointXYZRGB> vox;
    vox.setInputCloud (cloud_filtered);
    float leaf = 0.005f;
    vox.setLeafSize(leaf,leaf,leaf);
    vox.filter(*cloud_voxeld);
    cloud_voxeld.swap(cloud_filtered);

    // Create the segmentation object
    SACSegmentation<PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (SACMODEL_PLANE);
    seg.setMethodType (SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.1);

    //Extract Indices
    ExtractIndices<PointXYZRGB> extract;
//    //Segment the largest plane till only 20% of the cloud is remaining
    int points_num= (int)cloud_voxeld->points.size();
    while(cloud_voxeld->points.size() > 0.20* points_num)
    {
        ModelCoefficients::Ptr coeff(new ModelCoefficients);
        PointIndices::Ptr inliers(new PointIndices);
        seg.setInputCloud (cloud_voxeld);
        seg.segment (*inliers, *coeff);
        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            break;
        }
        extract.setInputCloud(cloud_voxeld);
        extract.setIndices(inliers);
        extract.setNegative(true); //check with false also
        extract.filter(*plane_removed);
        cloud_voxeld.swap(plane_removed);
    }

//    ROS_WARN("_OBJECT AVOID VISION_: Cloud Size Post Removal %d", (int)plane_removed->size());

    // To record more models
    plane_removed->header.frame_id = cloudNanRemoved->header.frame_id;
    pub.publish(plane_removed);


    //
    //  Compute Normals
    //
    NormalEstimationOMP<PointXYZRGB, Normal> norm_est;
    norm_est.setKSearch (k_neigh);
    norm_est.setInputCloud (plane_removed);
    norm_est.compute (*scene_normals);

    //
    //  Downsample Clouds to Extract keypoints
    //
    UniformSampling<PointXYZRGB> uniform_sampling;
    uniform_sampling.setInputCloud (plane_removed);
    uniform_sampling.setRadiusSearch (scene_ss_);
    PointCloud<int> keypointIndices;
    uniform_sampling.compute(keypointIndices);
    copyPointCloud(*plane_removed, keypointIndices.points, *scene_keypoints);
    if(verbose)ROS_INFO("Scene Keypoints: %d", (int) scene_keypoints->size());

    //
    //  Compute Descriptor for keypoints
    //
    SHOTEstimationOMP<PointXYZRGB, Normal,SHOT352> descr_est;
    descr_est.setRadiusSearch (descr_rad_s);
    descr_est.setInputCloud (scene_keypoints);
    descr_est.setInputNormals (scene_normals);
    descr_est.setSearchSurface (plane_removed);
    descr_est.compute (*scene_descriptors);

    string output;
    if(!found && !sphere_found )
    {
        resolved = false;
        output = shape_detect("sphere");
    }
    if(!found && !hcube_found)
    {
        resolved = false;
        output = shape_detect("hcube");
    }
    if(!found && !cylinder_found)
    {
        resolved = false;
        output = shape_detect("cylinder");
    }
    if(!found && !cube_found)
    {
        resolved = false;
        output = shape_detect("cube");
    }
    if(!found && !tri_found)
    {
        resolved = false;
        output = shape_detect("triangle");
    }
    if(!found && !cross_found)
    {
        resolved = false;
        output = shape_detect("cross");
    }
    if(!found && !patric_found)
    {
        resolved = false;
        output = shape_detect("patric");
    }

}

string preproc::shape_detect(string shape)
{
    string notshape = "NOT "+ shape;
    //
    //  Find Model-Scene Correspondences with KdTree
    //
    CorrespondencesPtr model_scene_corrs (new Correspondences ());
    KdTreeFLANN<SHOT352> match_search;
    rototranslations.clear();
    clustered_corrs.clear();


    if(shape=="cube")
    {
        match_search.setInputCloud (cube_descriptors);
        if(verbose)ROS_INFO("Model Cube Keypoints: %d", (int) KI_cube.size());
    }
    if(shape=="hcube")
    {
        match_search.setInputCloud (hcube_descriptors);
        if(verbose)ROS_INFO("Model Hollow Cube Keypoints: %d", (int) KI_hcube.size());
    }
    if(shape=="triangle")
    {
        match_search.setInputCloud (triangle_descriptors);
        if(verbose)ROS_INFO("Model Triangle Keypoints: %d", (int) KI_triangle.size());
    }
    if(shape=="cylinder")
    {
        match_search.setInputCloud (cylinder_descriptors);
        if(verbose)ROS_INFO("Model Cylinder Keypoints: %d", (int) KI_cylinder.size());
    }
    if(shape=="sphere")
    {
        match_search.setInputCloud (sphere_descriptors);
        if(verbose)ROS_INFO("Model Sphere Keypoints: %d", (int) KI_sphere.size());
    }
    if(shape=="cross")
    {
        match_search.setInputCloud (cross_descriptors);
        if(verbose)ROS_INFO("Model Cross Keypoints: %d", (int) KI_cross.size());
    }
    if(shape=="patric")
    {
        match_search.setInputCloud (patric_descriptors);
        if(verbose)ROS_INFO("Model Patric Keypoints: %d", (int) KI_patric.size());
    }

    // For K closest neighbours
    for (size_t i = 0; i < scene_descriptors->size (); ++i)
    {
        std::vector<int> neigh_indices (k_neigh);
        std::vector<float> neigh_sqr_dists (k_neigh);
        if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
        {
            continue;
        }
        int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), k_neigh, neigh_indices, neigh_sqr_dists);
        for(int k = 0; k < found_neighs; k++)
        {
            if(neigh_sqr_dists[k] < neigh_dist) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
            {
                Correspondence corr (neigh_indices[k], static_cast<int> (i), neigh_sqr_dists[k]);
                model_scene_corrs->push_back (corr);
            }
        }
    }

    //
    //  Actual Clustering
    //
    //  Using Hough3D
    if (use_hough_)
    {
        //
        //  Compute (Keypoints) Reference Frames only for Hough
        //
        PointCloud<ReferenceFrame>::Ptr scene_rf (new PointCloud<ReferenceFrame> ());

        BOARDLocalReferenceFrameEstimation<PointXYZRGB, Normal, ReferenceFrame> rf_est;
        rf_est.setFindHoles (true);
        rf_est.setRadiusSearch (rf_rad_s);

        rf_est.setInputCloud (scene_keypoints);
        rf_est.setInputNormals (scene_normals);
        rf_est.setSearchSurface (plane_removed);
        rf_est.compute (*scene_rf);

        //  Clustering
        Hough3DGrouping<PointXYZRGB, PointXYZRGB, ReferenceFrame, ReferenceFrame> clusterer;
        clusterer.setHoughBinSize (cg_size_);
        clusterer.setHoughThreshold (cg_thresh_);
        clusterer.setUseInterpolation (true);
        clusterer.setUseDistanceWeight (false);

        if(shape=="cube")
        {
            clusterer.setInputCloud (cube_key);
            clusterer.setInputRf (cube_rf);
        }
        if(shape=="sphere")
        {
            clusterer.setInputCloud (sphere_key);
            clusterer.setInputRf (sphere_rf);
        }
        if(shape=="triangle")
        {
            clusterer.setInputCloud (triangle_key);
            clusterer.setInputRf (triangle_rf);
        }
        if(shape=="hcube")
        {
            clusterer.setInputCloud (hcube_key);
            clusterer.setInputRf (hcube_rf);
        }
        if(shape=="cylinder")
        {
            clusterer.setInputCloud (cylinder_key);
            clusterer.setInputRf (cylinder_rf);
        }
        if(shape=="cross")
        {
            clusterer.setInputCloud (cross_key);
            clusterer.setInputRf (cross_rf);
        }
        if(shape=="patric")
        {
            clusterer.setInputCloud (patric_key);
            clusterer.setInputRf (patric_rf);
        }
        clusterer.setSceneCloud (scene_keypoints);
        clusterer.setSceneRf (scene_rf);
        clusterer.setModelSceneCorrespondences (model_scene_corrs);

        //clusterer.cluster (clustered_corrs);
        clusterer.recognize (rototranslations, clustered_corrs);
    }

    if(rototranslations.size () !=0 && found==false )
    {
        if(verbose) ROS_INFO("Instances: %d",(int)rototranslations.size());
        found = true;

    }
    if(found)
    {
        pub_sound.publish(sound_out);
        found_count_++;
        if(shape=="cube")
        {
            shape_count[0] +=1;
            if(!verbose)ROS_INFO("Cubes: %d", shape_count[0]);
        }
        else if(shape=="triangle")
        {
            shape_count[1] +=1;
            if(!verbose)ROS_INFO("Triangles: %d", shape_count[1]);
        }
        else if(shape=="hcube")
        {
            shape_count[2] +=1;
            if(!verbose)ROS_INFO("Hollow Cubes: %d", shape_count[2]);
        }
        else if(shape=="sphere")
        {
            shape_count[3] +=1;
            if(!verbose)ROS_INFO("Spheres: %d", shape_count[3]);
        }
        else if(shape=="cylinder")
        {
            shape_count[4] +=1;
            if(!verbose)ROS_INFO("Cylinders: %d", shape_count[4]);
        }
        else if(shape=="cross")
        {
            shape_count[5] +=1;
            if(!verbose)ROS_INFO("Crosses: %d", shape_count[5]);
        }
        else if(shape=="patric")
        {
            shape_count[6] +=1;
            if(!verbose)ROS_INFO("Patric: %d", shape_count[6]);
        }

        if(found_count_== 2)
        {
            find_objPose();
            found_count_=0;
            int max_index = (int) distance(shape_count.begin(), max_element(shape_count.begin(), shape_count.end()));
//            if(max_index == NULL) ROS_INFO("Max Index is NULL");



            if(!verbose)ROS_INFO("MAX INDEX OF SHAPE: %d", max_index);
            switch (max_index) {
                case 0:
                    sound_out.data = "Cube";
                    cube_found=true;
                    break;
                case 1:
                    sound_out.data = "Triangle";
                    tri_found = true;
                    break;
                case 2:
                    sound_out.data = "Hollow Cube";
                    hcube_found = true;
                    break;
                case 3:
                    sound_out.data = "Sphere";
                    sphere_found = true;
                    break;
                case 4:
                    sound_out.data = "Cylinder";
                    cylinder_found = true;
                    break;
                case 5:
                    sound_out.data = "Cross";
                    cross_found = true;
                    break;
                case 6:
                    sound_out.data = "Patric";
                    patric_found = true;
                    break;
            }
            fill(shape_count.begin(),shape_count.end(),0);
        }
        return shape;
    }
    else
    {
        return notshape;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "preprocess");
    preproc snoopy_see;

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    snoopy_see.read_models();
    snoopy_see.proc_models();
    Duration(10).sleep();

    while(ok())
    {
        spinOnce();
        snoopy_see.preprocess_scene();
    }
}
