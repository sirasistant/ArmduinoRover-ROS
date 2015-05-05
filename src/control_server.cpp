#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ArmduinoRover/controlArm.h"
#include "ArmduinoRover/controlMovement.h"
#include "ArmduinoRover/setArm.h"
#include "ArmduinoRover/setEngines.h"
#include "ArmduinoRover/setTwist.h"
#include "ArmduinoRover/controlIdAssign.h"

static const int VERTICAL_COUNT = 3;
static const int HORIZONTAL_COUNT = 3;
ros::ServiceClient enginesClient;
ros::ServiceClient twistClient;
ros::ServiceClient armClient;

struct movement_req {
	int left;
	int right;
	int twist;
	int priority;
	bool active;
};
struct arm_req {
	int vertical[VERTICAL_COUNT];
	int horizontal[HORIZONTAL_COUNT];
	int gripper; //TODO
	int priority;
	bool active;
};

std::vector<movement_req> moveRequests;
std::vector<arm_req> armRequests;
int idCount = 0;

bool controlMovement(ArmduinoRover::controlMovement::Request &req,
		ArmduinoRover::controlMovement::Response &res) {
	movement_req moveRequest;
	moveRequest.left = req.left;
	moveRequest.right = req.right;
	moveRequest.twist = req.twist;
	moveRequest.priority = req.priority;
	moveRequest.active = true;
	moveRequests[req.id] = moveRequest;
	res.done = true;
	return true;

}
bool controlArm(ArmduinoRover::controlArm::Request &req,
		ArmduinoRover::controlArm::Response &res) {
	arm_req armRequest;
	armRequest.active = true;
	armRequest.horizontal[0] = req.horizontal1;
	armRequest.horizontal[1] = req.horizontal2;
	armRequest.horizontal[2] = req.horizontal3;
	armRequest.vertical[0] = req.vertical1;
	armRequest.vertical[1] = req.vertical2;
	armRequest.vertical[2] = req.vertical3;
	armRequest.gripper = req.gripper;
	armRequest.priority = req.priority;
	armRequests[req.id] = armRequest; //TODO
	return true;
}

bool assignId(ArmduinoRover::controlIdAssign::Request &req,
		ArmduinoRover::controlIdAssign::Response &res) {
	res.id = idCount;
	idCount++;
	moveRequests.resize(idCount);
	armRequests.resize(idCount);
	return true;
}

int main(int argc, char **argv) {
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "control_server");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	enginesClient = n.serviceClient<ArmduinoRover::setEngines>("set_engines");
	twistClient = n.serviceClient<ArmduinoRover::setTwist>("set_twist");
	armClient = n.serviceClient<ArmduinoRover::setArm>("set_arm");
	ros::ServiceServer moveService = n.advertiseService("control_movement",
			controlMovement);
	ros::ServiceServer twistService = n.advertiseService("control_arm",
			controlArm);
	ros::ServiceServer idService = n.advertiseService("control_id_assign",
			assignId);
	ROS_INFO("Ready to control robot.");
	ros::Rate loop_rate(0.1);
	while (ros::ok()) {
		int left = 0;
		int right = 0;
		int twist = 0;
		int movePrioritySum = 0;

		for (int i = 0; i < moveRequests.size(); i++) {
			movement_req req = moveRequests.at(i);
			if (req.active) {
				ROS_INFO_STREAM("Processing movement request");
				left += req.left * req.priority;
				right += req.right * req.priority;
				twist += req.twist * req.priority;
				movePrioritySum += req.priority;
			}
		}
		if (movePrioritySum > 0) {
			left /= movePrioritySum;
			right /= movePrioritySum;
			twist /= movePrioritySum;
			ArmduinoRover::setEngines eng_srv;
			ArmduinoRover::setTwist tw_srv;
			eng_srv.request.left = left;
			eng_srv.request.right = right;
			tw_srv.request.twist = twist;
			if (enginesClient.call(eng_srv)) {
				if (twistClient.call(tw_srv)) {
					ROS_INFO_STREAM("done sending movement");
				}
			}
		}
		int vertical[VERTICAL_COUNT];
		int horizontal[HORIZONTAL_COUNT];
		int gripper;
		int armPrioritySum = 0;
		for (int i = 0; i < armRequests.size(); i++) {
			arm_req req = armRequests.at(i);
			if (req.active) {
				ROS_INFO_STREAM("Processing arm request");
				for (int i = 0; i < VERTICAL_COUNT; i++) {
					vertical[i] += req.vertical[i];
				}
				for (int i = 0; i < HORIZONTAL_COUNT; i++) {
					horizontal[i] += req.horizontal[i];
				}
				gripper = req.gripper;
				armPrioritySum += req.priority;
			}
		}
		if (armPrioritySum > 0) {
			for (int i = 0; i < VERTICAL_COUNT; i++) {
				vertical[i] /= armPrioritySum;
			}
			for (int i = 0; i < HORIZONTAL_COUNT; i++) {
				horizontal[i] /= armPrioritySum;
			}
			gripper /= armPrioritySum;
			ArmduinoRover::setArm arm_srv;
			arm_srv.request.vertical1=vertical[0];
			arm_srv.request.vertical2=vertical[1];
			arm_srv.request.vertical3=vertical[2];
			arm_srv.request.horizontal1=horizontal[0];
			arm_srv.request.horizontal2=horizontal[1];
			arm_srv.request.horizontal3=horizontal[2];
			arm_srv.request.gripper=gripper;
			if(armClient.call(arm_srv)){
				ROS_INFO_STREAM("done sending arm");
			}
		}
		moveRequests.clear();
		moveRequests.resize(idCount);
		armRequests.clear();
		armRequests.resize(idCount);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
