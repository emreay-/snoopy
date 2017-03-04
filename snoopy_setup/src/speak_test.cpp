#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace ros;
int main(int argc, char** argv)
{
  ros::init(argc,argv,"speak");
  ros::NodeHandle n("~");
  ros::Rate r(1);
  Publisher pub = n.advertise<std_msgs::String>("/espeak/string",5);
  while (n.ok())
  {
    std_msgs::String speak;
    n.getParam("speak",speak.data);
    pub.publish(speak);
    r.sleep();
  }
}
