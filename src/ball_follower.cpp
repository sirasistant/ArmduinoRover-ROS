#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ArmduinoRover/circle.h>
#include <ArmduinoRover/arrayOfCircles.h>

ros::Subscriber circlesSub;
int setPointX;
int xError;
int radError;
int radiusSetpoint;

bool circlesDetected = false;

void onCirclesDetected(const ArmduinoRover::arrayOfCirclesConstPtr& msg) {
	std::vector<ArmduinoRover::circle> circles = msg->circles;
	if (circles.size() > 0) {
		circlesDetected = true;
		int imageWidth = msg->sizeX;
		int imageHeight = msg->sizeY;

		//find the most reliable circle

		int maxReliability = -1;
		int mostReliableIndex = -1;

		for (int i = 0; i < circles.size(); i++) {
			ArmduinoRover::circle circle = circles.at(i);
			if (circle.reliability > maxReliability) {
				maxReliability = circle.reliability;
				mostReliableIndex = i;
			}
		}

		ArmduinoRover::circle mostReliable = circles.at(mostReliableIndex);
		int circleX = mostReliable.center.x;
		setPointX = imageWidth / 2;
		xError = setPointX - circleX;
		int circleRad = mostReliable.radius;
		radError = radiusSetpoint - circleRad;
	} else {
		circlesDetected = false;
	}

}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "ball_follower");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Starting ball following");
	circlesSub = nh.subscribe<ArmduinoRover::arrayOfCircles>("circles", 1000,
			&onCirclesDetected);
	ros::param::param("radius_setpoint", radiusSetpoint, 100);
	double freq;
	ros::param::param("frequency", freq, 5.0);
	ros::Rate loop_rate(freq);
	while (ros::ok()) {
		ros::spinOnce();
		if(circlesDetected){
			//TODO publish errors to pid node
		}
		loop_rate.sleep();
	}
}
