#include "ros/ros.h"
#include <cstdlib>
#include <ArmduinoRover/controlIdAssign.h>
#include <ArmduinoRover/controlMovement.h>
#include "std_msgs/Int32.h"

int id;
int priority;
int minDistance;
int distance;
ros::ServiceClient moveClient;

void onRangeReceived(const std_msgs::Int32::ConstPtr& msg) {
    distance = msg->data;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "wall_avoider");

	ros::NodeHandle n;
	moveClient = n.serviceClient<ArmduinoRover::controlMovement>(
			"control_movement");
	ros::ServiceClient idClient =
			n.serviceClient<ArmduinoRover::controlIdAssign>(
					"control_id_assign");
	ros::Subscriber pointSub = n.subscribe("range_lectures", 1000,
			onRangeReceived);
	ArmduinoRover::controlIdAssign idSrv;

	ros::param::param("priority", priority, 50);
	ros::param::param("min_distance", minDistance, 30);

	while (!idClient.call(idSrv)) {
		ROS_INFO_STREAM("Waiting for control server");
		ros::Duration(0.5).sleep();
	}
	id = idSrv.response.id;
    double freq;
    ros::param::param("frequency",freq,15.0);
	ros::Rate loop_rate(freq);
	while(ros::ok()){
		ros::spinOnce();
		if (distance < minDistance) {
				ArmduinoRover::controlMovement mvSrv;
				float multiplicator=0.5+(minDistance-distance)/(2*minDistance);
				mvSrv.request.right = 128*multiplicator;
				mvSrv.request.left = -128*multiplicator;
				mvSrv.request.id = id;
				mvSrv.request.priority = priority;
				mvSrv.request.twist = 0;
				if (!moveClient.call(mvSrv)) {
					ROS_ERROR_STREAM("Can't contact control server anymore");
				} else {
					if (!mvSrv.response.done) {
						ROS_WARN_STREAM("Can't avoid!");
					}
				}
			}
		loop_rate.sleep();
	}
	return 0;
}
