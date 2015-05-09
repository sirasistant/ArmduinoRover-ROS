#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ArmduinoRover/floatArr.h"
#include "ArmduinoRover/intArr.h"
#include "ArmduinoRover/cliComm.h"
#include "ArmduinoRover/encoder_data.h"
#include "std_msgs/Int32.h"
#include <string>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>

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
	ros::init(argc, argv, "sensors_publisher");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	ros::Publisher battery_pub = n.advertise<ArmduinoRover::floatArr>(
			"battery_lectures", 1000);
	ros::Publisher encoder_pub = n.advertise<ArmduinoRover::encoder_data>(
			"encoder_lectures", 1000);
	ros::Publisher range_pub = n.advertise<std_msgs::Int32>("range_lectures",
			1000);
	ros::Rate loop_rate(10);
	ros::ServiceClient client = n.serviceClient<ArmduinoRover::cliComm>(
			"cli_communication");
	while (ros::ok()) {
		ArmduinoRover::cliComm cli_srv;
		cli_srv.request.str = "READSENSORS\n";
		if (client.call(cli_srv)) {
			std::string resp = cli_srv.response.str;
			ROS_INFO_STREAM(resp);
			std::vector<std::string> arr;
			arr = split(resp, " ");
			ArmduinoRover::floatArr batt_msg;
			batt_msg.count = 0;
			ArmduinoRover::encoder_data encoder_msg;
			encoder_msg.lectures.count = 0;
			std::vector<bool> areEncodersBackwards;
			for (int i = 0; i < arr.size(); i++) {
				if (i < 2) {
					batt_msg.data.push_back(
							(10 * atoi(arr.at(i).c_str()) / 1024.0f));
					batt_msg.count++;
				} else {
					if (i < 6) {
						encoder_msg.lectures.data.push_back(
								atoi(arr.at(i).c_str()));
						encoder_msg.lectures.count++;
					} else {
						if (i < 7) { //TWIST
							encoder_msg.twist = atoi(arr.at(i).c_str()) - 90;
						} else {
							if (i < 8) { //LEFT
								areEncodersBackwards.push_back(
										255 > atoi(arr.at(i).c_str()));
								areEncodersBackwards.push_back(
										255 > atoi(arr.at(i).c_str()));
							} else { //RIGHT
								if (i < 9) {
									areEncodersBackwards.push_back(
											255 > atoi(arr.at(i).c_str()));
									areEncodersBackwards.push_back(
											255 > atoi(arr.at(i).c_str()));
								} else {
									if (i < 10) { //HC-SR04
										int dist = atoi(arr.at(i).c_str());
										if (dist != 51) {
											std_msgs::Int32 rangeMsg;
											rangeMsg.data = dist;
											range_pub.publish(rangeMsg);
										}
									}
								}
							}
						}
					}
				}
			}

			for (int i = 0; i < encoder_msg.lectures.count; i++) {
				if (areEncodersBackwards.at(i))
					encoder_msg.lectures.data.at(i) =
							-encoder_msg.lectures.data.at(i);
			}
			battery_pub.publish(batt_msg);
			encoder_pub.publish(encoder_msg);
		} else {
			ROS_ERROR("Failed to call cli service ");
		}
		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
