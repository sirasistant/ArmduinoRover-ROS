#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ArmduinoRover/circle.h>
#include <ArmduinoRover/arrayOfCircles.h>

image_transport::Subscriber sub;
image_transport::Publisher imPub;
ros::Publisher circlePub;

void onImageReceived(const sensor_msgs::ImageConstPtr& msg){
    ROS_INFO_STREAM("Getting image");
    int iLowH = 15;
    int iHighH = 45;

    int iLowS = 0;
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    cv_bridge::CvImagePtr cv_ptr;
    try  {
        cv_ptr= cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2HSV);
    inRange(cv_ptr->image, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), cv_ptr->image); //Threshold the image
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(cv_ptr->image,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
    cv::RotatedRect box;
    ArmduinoRover::arrayOfCircles circles;
    int radio;
    for(int i=0;i<contours.size();i++){
        if(contours[i].size()>300){
            ROS_INFO_STREAM("Found one");
            box=cv::fitEllipse(contours[i]);
            if(box.size.height/box.size.width>0.9 && box.size.height/box.size.width<1.1 && box.size.area()>300){
                radio=(box.size.width+box.size.height)/4;
                cv::Scalar color(255,255,255);
                cv::circle(cv_ptr->image,box.center,radio,color,4,8,0);
                ArmduinoRover::circle circle;
                circle.center.x=box.center.x;
                circle.center.y=box.center.y;
                circle.center.z=0;
                circle.color="yellow";
                circle.radius=radio;
                circle.reliability=100;
                circles.circles.push_back(circle);
            }
        }
    }
    cv_ptr->encoding=sensor_msgs::image_encodings::MONO8;
    imPub.publish(cv_ptr->toImageMsg());
    if(circles.circles.size()>0){
        circlePub.publish(circles);
    }

}

int main (int argc, char* argv[]){
    ros::init(argc,argv,"circle_detector");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Starting circle detection");
    image_transport::ImageTransport it(nh);
    imPub = it.advertise("camera_hsv",1);
    sub = it.subscribe("camera_raw",1,&onImageReceived);
    circlePub=nh.advertise<ArmduinoRover::arrayOfCircles>("circles",1000);
    ros::spin();
}
