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

int minSize;
double maxAxialDiff;

void onImageReceived(const sensor_msgs::ImageConstPtr& msg) {
	ROS_INFO_STREAM("Getting image");

	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2HSV);

	cv::Mat dstA;
	cv::Mat dstB;

	inRange(cv_ptr->image, cv::Scalar(0, 30, 30), cv::Scalar(4, 255, 255),
			dstA);
	inRange(cv_ptr->image, cv::Scalar(174, 30, 30), cv::Scalar(179, 255, 255),
			dstB);
	cv_ptr->image = dstA | dstB;
	cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(7, 7), 0, 0);
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(cv_ptr->image, contours, CV_RETR_LIST,
			CV_CHAIN_APPROX_NONE);
	cv::RotatedRect box;
	ArmduinoRover::arrayOfCircles circles;
	circles.sizeX = cv_ptr->image.cols;
	circles.sizeY = cv_ptr->image.rows;
	int radio;
	for (int i = 0; i < contours.size(); i++) {
		if (contours[i].size() > minSize) {
			ROS_INFO_STREAM("Found one");
			box = cv::fitEllipse(contours[i]);
			if (box.size.height / box.size.width > (1 - maxAxialDiff)
					&& box.size.height / box.size.width < (1 + maxAxialDiff)
					&& box.size.area() > minSize) {
				radio = (box.size.width + box.size.height) / 4;
				cv::Scalar color(255, 255, 255);
				cv::circle(cv_ptr->image, box.center, radio, color, 4, 8, 0);
				ArmduinoRover::circle circle;
				circle.center.x = box.center.x;
				circle.center.y = box.center.y;
				circle.center.z = 0;
				circle.color = "red";
				circle.radius = radio;
				circle.reliability = 100
						* (box.size.height / box.size.width - maxAxialDiff);
				circles.circles.push_back(circle);
			}
		}
	}
	cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
	imPub.publish(cv_ptr->toImageMsg());

	circlePub.publish(circles);

}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "circle_detector");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Starting circle detection");
	image_transport::ImageTransport it(nh);
	imPub = it.advertise("camera_hsv", 1);
	sub = it.subscribe("camera_raw", 1, &onImageReceived);
	circlePub = nh.advertise<ArmduinoRover::arrayOfCircles>("circles", 1000);
	ros::param::param("min_size", minSize, 300);
	ros::param::param("max_axial_diff", maxAxialDiff, 0.3);
	ros::spin();
}
