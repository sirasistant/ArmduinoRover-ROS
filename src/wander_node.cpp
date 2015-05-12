#include "ros/ros.h"
#include <cstdlib>
#include <ArmduinoRover/controlIdAssign.h>
#include <ArmduinoRover/controlMovement.h>

const static double FREQ = 10.0;

int main(int argc, char **argv) {
	ros::init(argc, argv, "wander_node");

	ros::NodeHandle n;
	ros::service::waitForService("control_id_assign");
	ros::Publisher movePub = n.advertise<
			ArmduinoRover::controlMovement>("control_movement",1000);
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
		ArmduinoRover::controlMovement moveMsg;
		ticksPassed++;
		if (turning) {
			moveMsg.left = -128;
			moveMsg.right = 128;
			if (ticksPassed > ticksTurning) {
				ROS_INFO_STREAM("FORWARD");
				ticksPassed = 0;
				turning = false;
			}
		} else {
			moveMsg.left = 128;
			moveMsg.right = 128;
			if (ticksPassed > ticksForward) {
				ROS_INFO_STREAM("TURNING");
				ticksPassed = 0;
				turning = true;
			}
		}
		moveMsg.id = id;
		moveMsg.priority = priority;
		moveMsg.twist = 0;
		movePub.publish(moveMsg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
