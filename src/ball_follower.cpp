#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ArmduinoRover/circle.h>
#include <ArmduinoRover/arrayOfCircles.h>
#include <ArmduinoRover/controlMovement.h>
#include <ArmduinoRover/controlIdAssign.h>

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
		xError = circleX - setPointX;
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

	ros::service::waitForService("control_id_assign");
	ros::Publisher movePub = nh.advertise<ArmduinoRover::controlMovement>(
			"control_movement", 1000);
	ros::ServiceClient idClient = nh.serviceClient<
			ArmduinoRover::controlIdAssign>("control_id_assign");

	ArmduinoRover::controlIdAssign idSrv;

	while (!idClient.call(idSrv)) {
		ROS_INFO_STREAM("Waiting for control server");
		ros::Duration(0.5).sleep();
	}
	int id = idSrv.response.id;

	int priority;
	ros::param::param("priority",priority,50);
	ros::param::param("radius_setpoint", radiusSetpoint, 150);
	double freq;
	ros::param::param("frequency", freq, 15.0);
	ros::Rate loop_rate(freq);
	while (ros::ok()) {
		ros::spinOnce();
		if (circlesDetected) {
			ArmduinoRover::controlMovement moveMsg;
			float multPower = ((float) radError) / ((float) radiusSetpoint);
			if (multPower > 1)
				multPower = 1;
			if (multPower < -1)
				multPower = -1;
			int power = multPower * 128;
			float multTurn = -((float) xError) / ((float) setPointX);
			int leftPower;
			int rightPower;
			if (multTurn > 0) {
				leftPower = power * (1 - multTurn);
				rightPower = power * (1 + multTurn);
			} else {
				rightPower = power * (1 + multTurn);
				leftPower = power * (1 - multTurn);
			}
			moveMsg.id=id;
			moveMsg.left=leftPower;
			moveMsg.right=rightPower;
			moveMsg.priority=priority;
			moveMsg.twist=0;
			movePub.publish(moveMsg);
		}
		loop_rate.sleep();
	}
}
