#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"robot_tf_publisher");
  ros::NodeHandle n_;

  ros::Rate r(10);

  tf::TransformBroadcaster broadcaster;

  while (n_.ok())
  {
    //Transform for laser- (0 x_axis, 8cm y_axis, 11cm z_axis)
    // broadcaster.sendTransform(
    //   tf::StampedTransform(
    //   tf::Transform(tf::Quaternion(0,0,1,0),tf::Vector3(-0.08,0,0.11)),
	//     ros::Time::now(),"base_link","laser_frame" ));

      //Transform for camera- (0 x_axis, 14cm y_axis, 8cm z_axis)
    broadcaster.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::createQuaternionFromRPY(0,0,0) ,tf::Vector3(0.14,0,0.08)),
      ros::Time::now(),"base_link","camera_link" ));

    r.sleep();
  }
}
