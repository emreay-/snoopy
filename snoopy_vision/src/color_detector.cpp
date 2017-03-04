/*
Color detection in the HSV space with RGB image in OpenCV with simple HSV
thresholding
*/

//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <map>

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace ros;
using namespace std;

static const char WINDOW[] = "Image Processed";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

//color hsv limits is respectively lowH, upH, lowS, upS, lowV, upV
int LowerH, UpperH, LowerS, UpperS, LowerV, UpperV;
//int LowerR, UpperR, LowerG, UpperG, LowerB, UpperB;

// std::map< std::string,std::vector<int> > colorHash;
// int blue_[] = {92,107,95,256,77,256};
// int green_[] = {31,140,143,256,78,157};
// int red_[] = {0,8,182,256,99,218};
// std::vector<int> blue(blue_, blue_+6);
// std::vector<int> green(green_,green_+6);
// std::vector<int> red(red_,red_+6);

void colorDetectionCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Error Bro %s", e.what());
        return;
    }

    //Draw contours around detected color in the image
    cv::Mat img_mask,img_hsv,img_color_det, img_color_det_bin;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::cvtColor(cv_ptr->image,img_hsv,CV_BGR2HSV);

    //With HSV Representation
    cv::inRange(img_hsv,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),img_mask);

    //With RGB Representation
//    cv::inRange(cv_ptr->image,cv::Scalar(LowerB,LowerG,LowerR),cv::Scalar(UpperB,UpperG,UpperR),img_mask);

    //find the contours
    findContours(img_mask, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    //Get moments from the contours
    std::vector<cv::Moments> mmt(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        mmt[i]= cv::moments(contours[i], true);
    }

    //Get the center of mass from the contours
    std::vector<cv::Point2f> com(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        com[i]=cv::Point2f(mmt[i].m10/mmt[i].m00, mmt[i].m01/mmt[i].m00 );
    }

    //draw the contours
//    cv::Mat drawing = cv::Mat::zeros(img_hsv.size(),CV_8UC3);
    cv::Mat drawing = cv_ptr->image;
//    cv::Mat drawing = img_hsv;
    for( int i = 0; i< contours.size(); i++ )
       {
         cv::Scalar color = cv::Scalar(0,0,0);
         drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
         circle(drawing, com[i], 4, cv::Scalar(0,0,255), -1, 8, 0);
       }

    //Display the image using OpenCV
    cv::imshow(WINDOW, drawing);
    cv::waitKey(3);

    //Convert the CvImage to a ROS image message toImageMsgand publish it on the "camera/image_processed" topic.
    pub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "color_detector");

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

    cv::namedWindow("Ball");
    // std::string color2detect = "";
    // colorHash["blue"] = blue;
    // colorHash["green"] = green;
    // colorHash["red"] = red;
    // // nh_priv.getParam("color_to_detect",color2detect);
    // std::vector<int> hsv_params = colorHash[color2detect];
    // LowerH = hsv_params[0];
    // UpperH = hsv_params[1];
    // LowerS = hsv_params[2];
    // UpperS = hsv_params[3];
    // LowerV = hsv_params[4];
    // UpperV = hsv_params[5];
    cv::createTrackbar("LowerH","Ball",&LowerH,180,NULL);
    cv::createTrackbar("UpperH","Ball",&UpperH,180,NULL);
    cv::createTrackbar("LowerS","Ball",&LowerS,256,NULL);
    cv::createTrackbar("UpperS","Ball",&UpperS,256,NULL);
    cv::createTrackbar("LowerV","Ball",&LowerV,256,NULL);
    cv::createTrackbar("UpperV","Ball",&UpperV,256,NULL);
//    cv::createTrackbar("LowerB","Ball",&LowerB,256,NULL);
//    cv::createTrackbar("UpperB","Ball",&UpperB,256,NULL);
//    cv::createTrackbar("UpperG","Ball",&UpperG,256,NULL);
//    cv::createTrackbar("LowerG","Ball",&LowerG,256,NULL);
//    cv::createTrackbar("LowerR","Ball",&LowerR,256,NULL);
//    cv::createTrackbar("UpperR","Ball",&UpperR,256,NULL);

    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);

    //Subscribe to the color image topic from the camera
    //image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
    image_transport::Subscriber sub = it.subscribe("camera/rgb/image_raw", 1, colorDetectionCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);

    //Use this to trigger ROS
    pub = it.advertise("camera/color_det_rgb", 1);

    ros::spin();
}
