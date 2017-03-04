/*
Takes threshold values (in RGB space) for different colors from the parameter server,
detects color in the image based on these threshold values
*/

//____________________ROS__________________//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <ras_msgs/RAS_Evidence.h>
//____________________C++__________________//
#include <Eigen/Geometry>
#include <string.h>
//____________________PCL__________________//
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>

#include <snoopy_vision/identify.h>

using namespace ros;
using namespace std;
using namespace pcl;

bool verbose(false);
bool spoken (false);

float vMAX (256);
float sMAX (256);

class Object_Ident
{
public:
    Object_Ident();
    ~Object_Ident();
    //______ROS______//
    Publisher pub_speak,pub_test, evidence;
    Subscriber sub, sub_sound, sub_pose;

    std_msgs::String output_shape;
    std_msgs::Bool pickup;
    geometry_msgs::PointStamped object_pose;

    //______LOGIC____//
    string shape_check;
    string shape;

    //______HSV__h: 0:360, s:0-1, v:0-1______//
    float hMax,hMin,sMax,sMin,vMax,vMin; //Red
    float hMaxy,hMiny,sMaxy,sMiny,vMaxy,vMiny; //Yellow
    float hMaxo,hMino,sMaxo,sMino,vMaxo,vMino; //Orange
    float hMaxp,hMinp,sMaxp,sMinp,vMaxp,vMinp; //Purple
    float hMaxb, hMinb; //Blue
    float hMaxg, hMing; //Green

    //______PCL______//
    PointCloud<PointXYZHSV> cloud_hsv;

    //flag var
    bool isPoseMsgReceived;
    bool isShapeMsgReceived;

private:
    int cloud_size;
    int red_points,blue_points,green_points,yellow_points, orange_points, purple_points;

    ServiceServer ident_server;
    bool ident(snoopy_vision::identify::Request& req, snoopy_vision::identify::Response& res);

    NodeHandle nh;
    NodeHandle nh_priv;

    //________Callbacks________//
    void pcl_callbackHSV(const PointCloud<PointXYZRGB>::ConstPtr& input);
    void color_callback(const std_msgs::StringConstPtr& msg);
    void pose_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
};

Object_Ident::Object_Ident() : nh_priv("~")
{
    sub = nh.subscribe<PointCloud<PointXYZRGB> >("/snoopy/shape_detected", 1, &Object_Ident::pcl_callbackHSV, this);
    sub_pose = nh.subscribe<geometry_msgs::PointStamped>("/snoopy/object_pose",1, &Object_Ident::pose_callback, this);
    sub_sound = nh.subscribe< std_msgs::String >("/snoopy/shape_string",5,&Object_Ident::color_callback, this);

    pub_speak = nh.advertise<std_msgs::String>("/espeak/string",5);
    pub_test = nh.advertise< PointCloud<PointXYZHSV> >("/snoopy/testing_color",100);
    evidence = nh.advertise<ras_msgs::RAS_Evidence> ("/evidence",10);

    ident_server = nh.advertiseService("identify",&Object_Ident::ident,this);

    nh_priv.getParam("smin",sMin);
    nh_priv.getParam("smax",sMax);
    nh_priv.getParam("vmin",vMin);
    nh_priv.getParam("vmax",vMax);

    //Red
    nh_priv.getParam("hmin",hMin);
    nh_priv.getParam("hmax",hMax);

    //Blue
    nh_priv.getParam("hminb",hMinb);
    nh_priv.getParam("hmaxb",hMaxb);

    //Green
    nh_priv.getParam("hming",hMing);
    nh_priv.getParam("hmaxg",hMaxg);

    //Yellow
    nh_priv.getParam("hminy",hMiny);
    nh_priv.getParam("hmaxy",hMaxy);
    nh_priv.getParam("sminy",sMiny);
    nh_priv.getParam("smaxy",sMaxy);
    nh_priv.getParam("vminy",vMiny);
    nh_priv.getParam("vmaxy",vMaxy);

    //Orange
    nh_priv.getParam("hminy",hMino);
    nh_priv.getParam("hmaxy",hMaxo);
    nh_priv.getParam("sminy",sMino);
    nh_priv.getParam("smaxy",sMaxo);
    nh_priv.getParam("vminy",vMino);
    nh_priv.getParam("vmaxy",vMaxo);

    //Purple
    nh_priv.getParam("hminy",hMiny);
    nh_priv.getParam("hmaxy",hMaxy);
    nh_priv.getParam("sminy",sMiny);
    nh_priv.getParam("smaxy",sMaxy);
    nh_priv.getParam("vminy",vMiny);
    nh_priv.getParam("vmaxy",vMaxy);

    isPoseMsgReceived = false;
    isShapeMsgReceived = false;

}
Object_Ident::~Object_Ident()
{}

bool Object_Ident::ident(snoopy_vision::identify::Request& req, snoopy_vision::identify::Response& res)
{
    if(!isPoseMsgReceived || !isShapeMsgReceived) sleep(5);
    spinOnce();

    if(isPoseMsgReceived && isShapeMsgReceived)
    {
        ROS_WARN("_OBJ IDENT_: Shape Type %s", output_shape.data.c_str());
        ROS_WARN("_OBJ IDENT_: Pose  (%f,%f)", object_pose.point.x, object_pose.point.y);
        res.object = output_shape;
        res.pickup.data = true;
        res.position = object_pose;
    }
    else
    {
        ROS_INFO("_OBJ IDENT_: Pose/Shape msg not yet received");
        return false;
    }

}

void Object_Ident::pose_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    object_pose.header.frame_id = msg->header.frame_id;
    object_pose.header.stamp = Time();
    object_pose.point.x = msg->point.x;
    object_pose.point.y = msg->point.y;

    isPoseMsgReceived = true;
}

void Object_Ident::color_callback(const std_msgs::StringConstPtr& msg)
{
    string output;
    string output_evidence;
    shape = msg->data;

    isShapeMsgReceived = true;

    if(verbose)ROS_WARN("_OBJ IDENT_: Green Points/Cloud Size %f", (float) green_points/ (float)cloud_size);
    if(shape == shape_check)
    {
        spoken = false;
    }

    //______FOR EVIDENCE____________//
    if(red_points > 0.250 * cloud_size && red_points > yellow_points )
    {
        if(verbose)ROS_WARN("_OBJ IDENT_:IN RED");
        if(shape != "Cube") output_evidence = "Red Hollow Cube";
        else output_evidence = " Red " + shape;
    }
    else if(blue_points > 0.50 * cloud_size)
    {
        if(shape != "Cube") output_evidence = "Blue Triangle";
        else output_evidence = "Blue "+ shape;
    }
    else if(green_points > 0.250 * cloud_size)
    {
        if(verbose)ROS_WARN("_OBJ IDENT_:IN GREEN");
        if(shape != "Cube") output_evidence = "Green Cylinder";
        else output_evidence = "Green "+ shape;
    }
    else if(yellow_points > 0.250 * cloud_size && yellow_points>red_points )
    {
        if(shape != "Cube") output_evidence = "Yellow Sphere";
        else output_evidence = "Yellow "+ shape;
    }
    else if(orange_points > 0.250 * cloud_size && orange_points>red_points )
    {
        output = "Patric";
    }
    else if(purple_points > 0.250 * cloud_size && purple_points>red_points )
    {
        if(shape == "Cross") output_evidence = "Purple "+shape;
        else output_evidence = "Purple Star";
    }

    // ______TO CHECK CLASSIFICATION______//
    // if(red_points > 0.25 * cloud_size)
    // {
    //     output = "Snoopy sees a Red " + shape;
    // }
    //
    // else if(blue_points > 0.25 * cloud_size)
    // {
    //     output = "Snoopy sees a Blue "+ shape;
    // }
    // else if(green_points > 0.25 * cloud_size)
    // {
    //     output = "Snoopy sees a Green "+ shape;
    // }
    // else if(yellow_points > 0.25 * cloud_size && yellow_points>red_points )
    // {
    //     output = "Snoopy sees a Yellow"+ shape;
    // }
    //    output_shape.data = output;


    if (verbose) cout<<output<<endl;
    output_shape.data = output_evidence;
    pub_speak.publish(output_shape);

    //Publish only once for each positive ID
    if(shape!=shape_check && spoken ==false)
    {
        shape_check = shape;
        spoken=true;
    }

}

void Object_Ident::pcl_callbackHSV(const PointCloud<PointXYZRGB>::ConstPtr& cloud)
{
    PointCloud<PointXYZRGB> cloud_temp;
    cloud_hsv.clear();
    copyPointCloud(*cloud,cloud_temp);
    PointCloudXYZRGBtoXYZHSV(cloud_temp,cloud_hsv);

    cloud_size = (int) cloud->size();
    red_points=0;
    blue_points=0;
    green_points=0;
    yellow_points=0;
    orange_points=0;
    purple_points=0;


    for(size_t i = 0; i < cloud_hsv.size(); ++i)
    {
//        if(verbose)
//        {
//            cout<<"Hue: "<<cloud_hsv.points[i].h<<endl;
//            cout<<"Sat: "<<cloud_hsv.points[i].s<<endl;
//            cout<<"Val: "<<cloud_hsv.points[i].v<<endl;
//        }

        //______RED COLOR______//
        if( cloud_hsv.points[i].h < hMax && cloud_hsv.points[i].h > hMin &&
            cloud_hsv.points[i].s < sMax && cloud_hsv.points[i].s > sMin &&
            cloud_hsv.points[i].v < vMax && cloud_hsv.points[i].v > vMin)
        {
            red_points++;
        }

        //______BLUE COLOR_________//
        if( cloud_hsv.points[i].h < hMaxb && cloud_hsv.points[i].h > hMinb &&
            cloud_hsv.points[i].s < sMax && cloud_hsv.points[i].s > sMin &&
            cloud_hsv.points[i].v < vMax && cloud_hsv.points[i].v > vMin)
        {
            blue_points++;
        }

        //______GREEN COLOR__________//
        if( cloud_hsv.points[i].h < hMaxg && cloud_hsv.points[i].h > hMing &&
            cloud_hsv.points[i].s < sMax && cloud_hsv.points[i].s > sMin &&
            cloud_hsv.points[i].v < vMax && cloud_hsv.points[i].v > vMin)
        {
            green_points++;
        }

        //______YELLOW COLOR_________//
        if( cloud_hsv.points[i].h < hMaxy && cloud_hsv.points[i].h > hMiny &&
            cloud_hsv.points[i].s < sMaxy && cloud_hsv.points[i].s > sMiny &&
            cloud_hsv.points[i].v < vMaxy && cloud_hsv.points[i].v > vMiny)
        {
            yellow_points++;
        }
        //______ORANGE COLOR_________//
        if( cloud_hsv.points[i].h < hMaxo && cloud_hsv.points[i].h > hMino &&
            cloud_hsv.points[i].s < sMaxo && cloud_hsv.points[i].s > sMino &&
            cloud_hsv.points[i].v < vMaxo && cloud_hsv.points[i].v > vMino)
        {
            orange_points++;
        }
        //______PURPLE COLOR_________//
        if( cloud_hsv.points[i].h < hMaxp && cloud_hsv.points[i].h > hMinp &&
            cloud_hsv.points[i].s < sMaxp && cloud_hsv.points[i].s > sMinp &&
            cloud_hsv.points[i].v < vMaxp && cloud_hsv.points[i].v > vMinp)
        {
            purple_points++;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Object_Identification");
    Object_Ident colorDet;

    //RED
    colorDet.shape_check = "Nothing";
    colorDet.sMax = colorDet.sMax/sMAX;
    colorDet.sMin = colorDet.sMin/sMAX;
    colorDet.vMax = colorDet.vMax/vMAX;
    colorDet.vMin = colorDet.vMin/vMAX;

    //YELLOW
    colorDet.sMaxy = colorDet.sMaxy/sMAX;
    colorDet.sMiny = colorDet.sMiny/sMAX;
    colorDet.vMaxy = colorDet.vMaxy/vMAX;
    colorDet.vMiny = colorDet.vMiny/vMAX;

    //PURPLE
    colorDet.sMaxp = colorDet.sMaxp/sMAX;
    colorDet.sMinp = colorDet.sMinp/sMAX;
    colorDet.vMaxp = colorDet.vMaxp/vMAX;
    colorDet.vMinp = colorDet.vMinp/vMAX;

    //ORANGE
    colorDet.sMaxo = colorDet.sMaxo/sMAX;
    colorDet.sMino = colorDet.sMino/sMAX;
    colorDet.vMaxo = colorDet.vMaxo/vMAX;
    colorDet.vMino = colorDet.vMino/vMAX;

    while(ok())
    {
        spinOnce();
    }
    return 0;
}
