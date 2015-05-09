#include "ros/ros.h"
#include <cstdlib>
#include <ArmduinoRover/controlIdAssign.h>
#include <ArmduinoRover/controlMovement.h>

const static double FREQ = 10.0;

int main(int argc, char **argv) {
	ros::init(argc, argv, "wander_node");

	ros::NodeHandle n;
	ros::ServiceClient moveClient = n.serviceClient<
			ArmduinoRover::controlMovement>("control_movement");
	ros::ServiceClient idClient =
			n.serviceClient<ArmduinoRover::controlIdAssign>(
					"control_id_assign");

	ArmduinoRover::controlIdAssign idSrv;

	while (!idClient.call(idSrv)) {
		ROS_INFO_STREAM("Waiting for control server");
		ros::Duration(0.5).sleep();
	}
	int id = idSrv.response.id;

	ros::Rate loop_rate(FREQ);
	double forwardSecs;
	ros::param::param("forward_secs", forwardSecs, 10.0); //name, variable and default value
	double turnSecs;
	ros::param::param("turn_secs", turnSecs, 1.5);
	int priority;
	ros::param::param("priority", priority, 5);

	int ticksPassed = 0;
	bool turning = false;
	int ticksTurning = turnSecs * FREQ;
	int ticksForward = forwardSecs * FREQ;
	while (ros::ok()) {
		ArmduinoRover::controlMovement mvSrv;
		ticksPassed++;
		if (turning) {
			mvSrv.request.left = 128;
			mvSrv.request.right = -128;
			if (ticksPassed > ticksTurning) {
				ticksPassed = 0;
				turning = false;
			}
		} else {
			mvSrv.request.left = 128;
			mvSrv.request.right = -128;
			if (ticksPassed > ticksForward) {
				ticksPassed = 0;
				turning = true;
			}
		}
		mvSrv.request.id = id;
		mvSrv.request.priority = priority;
		mvSrv.request.twist = 0;
		if (!moveClient.call(mvSrv)) {
			ROS_ERROR_STREAM("Can't contact control server anymore");
		} else {
			if (!mvSrv.response.done) {
				ROS_WARN_STREAM("Can't wander");
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
