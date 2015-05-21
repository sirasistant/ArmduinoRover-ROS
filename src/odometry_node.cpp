#include "ros/ros.h"
#include <cstdlib>
#include "ArmduinoRover/encoder_data.h"
#include "math.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class Vector2D {

public:
	Vector2D(float X = 0, float Y = 0) {
		x = X;
		y = Y;
	}
	;
	~Vector2D() {
	}
	;

	float x, y;

	Vector2D operator*(float scalar) const {
		return Vector2D(x * scalar, y * scalar);
	}

	Vector2D operator+(const Vector2D &vect) const {
		return Vector2D(x + vect.x, y + vect.y);
	}
	Vector2D operator-(const Vector2D &vect) const {
		return Vector2D(x - vect.x, y - vect.y);
	}

	void rotate(float angle) {
		float xt = (x * cosf(angle)) - (y * sinf(angle));
		float yt = (y * cosf(angle)) + (x * sinf(angle));
		x = xt;
		y = yt;
	}

	float crossproduct(const Vector2D &vect2) const {
		return (this->x * vect2.y) - (this->y * vect2.x);
	}

	float magnitude() {
		return sqrtf(x * x + y * y);
	}

	void normalise() {
		float mag = sqrtf(x * x + y * y);
		this->x = x / mag;
		this->y = y / mag;
	}

	// return dot product
	float dotproduct(const Vector2D &vect) const {
		return (x * vect.x) + (y * vect.y);
	}
};

double wheelRadius;
int notchCount;
double lastX;
double lastY;
double lastRotation;
double axisDistance;


void calcOdometry(double dist1,double dist2,double lastOrientation,double& deltaX,double& deltaY,double& deltaRot){
	if(dist1==dist2){
		deltaX = cos(lastOrientation)*dist1;
		deltaY = sin(lastOrientation)*dist2;
		deltaRot=0;
		ROS_INFO_STREAM("Same distance on both encoders \n");
		return;

	    }else{
	        if(dist2>dist1){
	            double angle=(dist2-dist1)/axisDistance;
	            double distanceFromCenter=std::min(dist1,dist2)/angle+axisDistance/2;
	            double incrementInLocalsX=+(cos(angle)*distanceFromCenter-distanceFromCenter);
	            double incrementInLocalsY=sin(angle)*distanceFromCenter;
	            double incrementInGlobalsX=cos(lastOrientation)*incrementInLocalsY+cos(lastOrientation-M_PI/2)*incrementInLocalsX;
	            double incrementInGlobalsY=sin(lastOrientation)*incrementInLocalsY+sin(lastOrientation-M_PI/2)*incrementInLocalsX;
	            deltaX=incrementInGlobalsX;
	            deltaY=incrementInGlobalsY;
	            deltaRot=angle;
	            return;
	        }else{
	            double angle=(dist1-dist2)/axisDistance;
	            double distanceFromCenter=std::min(dist1,dist2)/angle+axisDistance/2;
	            double incrementInLocalsX=-(cos(angle)*distanceFromCenter-distanceFromCenter);
	            double incrementInLocalsY=sin(angle)*distanceFromCenter;
	            double incrementInGlobalsX=cos(lastOrientation)*incrementInLocalsY+cos(lastOrientation-M_PI/2)*incrementInLocalsX;
	            double incrementInGlobalsY=sin(lastOrientation)*incrementInLocalsY+sin(lastOrientation-M_PI/2)*incrementInLocalsX;
	            deltaX=incrementInGlobalsX;
	            deltaY=incrementInGlobalsY;
	            deltaRot=-angle;
	            return;
	        }

	    }
}

double toMeters(int notchMeasured){
	double rads=2*M_PI*((double)notchMeasured)/((double)notchCount);
	return rads*wheelRadius;
}

void onEncoderReceived(const ArmduinoRover::encoder_data::ConstPtr& msg) {
	int twist = msg->twist;
	std::vector<int> encoders = msg->lectures.data;
	int count = msg->lectures.count;
	if (count != 4) {
		ROS_ERROR_STREAM("Invalid number of encoders");
	} else {
		int fl = encoders.at(1);
		int fr = encoders.at(0);
		int bl = encoders.at(2);
		int br = encoders.at(3);
		double leftEncoder = toMeters(bl);
		double rightEncoder = toMeters(fr);
		// calc increment in x , y, and rotation
		double deltaX;
		double deltaY;
		double deltaRot;
		calcOdometry(leftEncoder,rightEncoder,lastRotation,deltaX,deltaY,deltaRot);

		Vector2D delta(deltaX, deltaY);
		delta.rotate(2.0 * M_PI * twist / 360.0); //rotate by twist in radians

		//add delta
		lastRotation+=deltaRot;
		lastX+=delta.x;
		lastY+=delta.y;

		//send the tf
		tf2_ros::TransformBroadcaster tfBroadcaster;
		geometry_msgs::TransformStamped transformStamped;

		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "world";
		transformStamped.child_frame_id = "rover";
		transformStamped.transform.translation.x = lastX;
		transformStamped.transform.translation.y = lastY;
		transformStamped.transform.translation.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, lastRotation);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();

		tfBroadcaster.sendTransform(transformStamped);

	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "odometry_node");

	ros::NodeHandle n;

	ros::Subscriber encoderSub = n.subscribe("encoder_lectures", 1000,
			onEncoderReceived);

	ros::param::param("initial_x", lastX, 0.0);
	ros::param::param("initial_y", lastY, 0.0);
	ros::param::param("initial_rotation", lastRotation, 0.0);
	ros::param::param("wheel_radius", wheelRadius, 0.041);
	ros::param::param("notch_count", notchCount, 12);
	ros::param::param("axis_distance",axisDistance,0.137);
	ros::spin();
	return 0;
}
