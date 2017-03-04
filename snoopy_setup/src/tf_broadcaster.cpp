#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"tf_publisher");
    ros::NodeHandle n("~");
    ros::Rate r(100);
    tf::TransformBroadcaster broadcaster;
    double _laser_tr_x, _laser_tr_y, _laser_tr_z, _laser_rot_x, _laser_rot_y, _laser_rot_z,
           _cam_tr_x, _cam_tr_y, _cam_tr_z, _cam_rot_x, _cam_rot_y, _cam_rot_z,
           _arm_tr_x, _arm_tr_y, _arm_tr_z, _arm_rot_x, _arm_rot_y, _arm_rot_z;

    //Reading from the parameter server
    //the parameters are set in tf_parameters.yaml file!
    n.getParam("laser_tran_x",_laser_tr_x);
    n.getParam("laser_tran_y",_laser_tr_y);
    n.getParam("laser_tran_z",_laser_tr_z);
    n.getParam("laser_rot_x",_laser_rot_x);
    n.getParam("laser_rot_y",_laser_rot_y);
    n.getParam("laser_rot_z",_laser_rot_z);

    n.getParam("camera_tran_x",_cam_tr_x);
    n.getParam("camera_tran_y",_cam_tr_y);
    n.getParam("camera_tran_z",_cam_tr_z);
    n.getParam("camera_rot_x",_cam_rot_x);
    n.getParam("camera_rot_y",_cam_rot_y);
    n.getParam("camera_rot_z",_cam_rot_z);

    n.getParam("arm_tran_x",_arm_tr_x);
    n.getParam("arm_tran_y",_arm_tr_y);
    n.getParam("arm_tran_z",_arm_tr_z);
    n.getParam("arm_rot_x",_arm_rot_x);
    n.getParam("arm_rot_y",_arm_rot_y);
    n.getParam("arm_rot_z",_arm_rot_z);

    //Conversion from deg to rad
    _laser_rot_x *= M_PI/180.0;
    _laser_rot_y *= M_PI/180.0;
    _laser_rot_z *= M_PI/180.0;
    _cam_rot_x *= M_PI/180.0;
    _cam_rot_y *= M_PI/180.0;
    _cam_rot_z *= M_PI/180.0;
    _arm_rot_x *= M_PI/180.0;
    _arm_rot_y *= M_PI/180.0;
    _arm_rot_z *= M_PI/180.0;

    //Conversion from mm to m
    _laser_tr_x /= 1000.0;
    _laser_tr_y /= 1000.0;
    _laser_tr_z /= 1000.0;
    _cam_tr_x /= 1000.0;
    _cam_tr_y /= 1000.0;
    _cam_tr_z /= 1000.0;
    _arm_tr_x /= 1000.0;
    _arm_tr_y /= 1000.0;
    _arm_tr_z /= 1000.0;

    while (n.ok())
    {
// --------------------------------------------------------------------------------------
//        OLD TF KEEP IT IN CASE OF ANY PROBLEM

//        //Transform for laser- (0 x_axis, 8cm y_axis, 11cm z_axis)
//        broadcaster.sendTransform(
//          tf::StampedTransform(
//          tf::Transform(tf::Quaternion(0,0,1,0),tf::Vector3(-0.08,0,0.11)),
//            ros::Time::now(),"base_link","laser_frame" ));

//          //Transform for camera- (0 x_axis, 14cm y_axis, 8cm z_axis)
//        broadcaster.sendTransform(
//          tf::StampedTransform(
//          tf::Transform(tf::createQuaternionFromRPY(0,M_PI/12,0) ,tf::Vector3(0.14,0,0.08)),
//          ros::Time::now(),"base_link","camera_link" ));

//        //Transform for uarm (0 x_axis, 0cm y_axis, 13,5cm z_axis)
//        broadcaster.sendTransform(
//            tf::StampedTransform(
//            tf::Transform(tf::createQuaternionFromRPY(0,0,M_PI/8) ,tf::Vector3(0,0,-0.135)),
//            ros::Time::now(),"base_link","uarm_link" ));

// --------------------------------------------------------------------------------------


//        ROS_INFO("_cam_tr_x: %f",_cam_tr_x);
        //Transform for laser
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(
                            tf::createQuaternionFromRPY(_laser_rot_x,_laser_rot_y,_laser_rot_z),
                            tf::Vector3(_laser_tr_x,_laser_tr_y,_laser_tr_z)),
                            ros::Time::now(),"base_link","laser_frame" ));

          //Transform for camera
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(
                            tf::createQuaternionFromRPY(_cam_rot_x,_cam_rot_y,_cam_rot_z) ,
                            tf::Vector3(_cam_tr_x,_cam_tr_y,_cam_tr_z)),
                            ros::Time::now(),"base_link","camera_link" ));

        //Transform for uarm
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(
                            tf::createQuaternionFromRPY(_arm_rot_x,_arm_rot_y,_arm_rot_z),
                            tf::Vector3(_arm_tr_x,_arm_tr_y,_arm_tr_z)),
                            ros::Time::now(),"base_link","uarm_link" ));

        r.sleep();
    }
}
