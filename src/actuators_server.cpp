#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ArmduinoRover/floatArr.h"
#include "ArmduinoRover/intArr.h"
#include "ArmduinoRover/cliComm.h"
#include "ArmduinoRover/setArm.h"
#include "ArmduinoRover/setTwist.h"
#include "ArmduinoRover/setEngines.h"
#include <string>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

ros::ServiceClient client;

std::vector<std::string> split(std::string str, std::string sep) {
	char* cstr = const_cast<char*>(str.c_str());
	char* current;
	std::vector<std::string> arr;
	current = strtok(cstr, sep.c_str());
	while (current != NULL) {
		arr.push_back(current);
		current = strtok(NULL, sep.c_str());
	}
	return arr;
}

bool moveEngines(ArmduinoRover::setEngines::Request &req,
		ArmduinoRover::setEngines::Response &res) {
	ArmduinoRover::cliComm cli_srv;
	std::stringstream ss;
	ss<< "0 0 0 0 0 0 0 "<<req.left<<" "<<req.right<<" "<<req.left<<" "<<req.right<<" 0 0 0 0 000000011110000\n";
	cli_srv.request.str=ss.str();
	if (client.call(cli_srv)) {
		std::string resp = cli_srv.response.str;
		res.done=true; //TODO check response
		return true;
	} else {
		return false;
	}

}
bool setTwist(ArmduinoRover::setTwist::Request &req,
		ArmduinoRover::setTwist::Response &res) {
	ArmduinoRover::cliComm cli_srv;
	int twist=req.twist;
	std::stringstream ss;
	ss<<"0 0 0 0 0 0 0 0 0 0 0 "<<twist<<" "<<twist<<" "<<twist<<" "<<twist<<" 000000000001111\n";
	cli_srv.request.str=ss.str();
	if (client.call(cli_srv)) {
		std::string resp = cli_srv.response.str;
		res.done=true; //TODO check response
		return true;
	} else {
		return false;
	}
}
bool setArm(ArmduinoRover::setArm::Request &req,
		ArmduinoRover::setArm::Response &res) {
	ArmduinoRover::cliComm cli_srv;
	int horizontal1=req.horizontal1;
	int horizontal2=req.horizontal2;
	int horizontal3=req.horizontal3;
	int gripper=req.gripper;
	int vertical1=req.vertical1;
	int vertical2=req.vertical2;
	int vertical3=req.vertical3;
	std::stringstream ss;
	ss<< ""<<horizontal1<<" "<<horizontal2<<" "<<horizontal3<<" "<<vertical1<<" "<<vertical2<<" "<<vertical3<<" "<<gripper<<" 0 0 0 0 0 0 0 0 111111100000000\n";
	cli_srv.request.str=ss.str();
	if (client.call(cli_srv)) {
		std::string resp = cli_srv.response.str;
		res.done=true; //TODO check response
		return true;
	} else {
		return false;
	}
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
	ros::init(argc, argv, "actuators_server");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	client = n.serviceClient<ArmduinoRover::cliComm>("cli_communication");

	ros::ServiceServer enginesService = n.advertiseService("set_engines", moveEngines);
	ros::ServiceServer twistService = n.advertiseService("set_twist", setTwist);
	ros::ServiceServer armService = n.advertiseService("set_arm", setArm);
	ROS_INFO("Ready to use actuators.");
	ros::spin();

	return 0;
}
