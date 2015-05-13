#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ArmduinoRover/encoder_data.h"
#include "std_msgs/Int32.h"
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include "ArmduinoRover/cliComm.h"

using namespace std;

vector<string> split(string str, string sep) {
	char* cstr = const_cast<char*>(str.c_str());
	char* current;
	vector<string> arr;
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
	ros::init(argc, argv, "navigation_publisher");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;
	ros::service::waitForService("cli_communication");
	ros::Publisher encoder_pub = n.advertise<ArmduinoRover::encoder_data>(
			"encoder_lectures", 1000);
	double freq;
	ros::param::param("frequency", freq, 5.0);
	ros::Rate loop_rate(freq);
	ros::ServiceClient client = n.serviceClient<ArmduinoRover::cliComm>(
			"cli_communication");
	while (ros::ok()) {
		ros::spinOnce();

		ArmduinoRover::cliComm cli_srv;
		cli_srv.request.str = "READNAVIGATION\n";
		if (client.call(cli_srv)) {
			string resp = cli_srv.response.str;
			ROS_INFO_STREAM(resp);
			vector<string> arr;
			arr = split(resp, " ");
			ArmduinoRover::encoder_data encodersData;
			for (int i = 2; i < arr.size(); i++) {
				if (i < 6) {
					encodersData.lectures.data.push_back(
							(atoi(arr.at(i).c_str())));
					encodersData.lectures.count++;
				} else {
					if (i < 7) {
						encodersData.twist = (atoi(arr.at(i).c_str()));
					}
				}
			}
			encoder_pub.publish(encodersData);
		} else {
			ROS_ERROR("Failed to call cli service ");
		}

		loop_rate.sleep();
	}

	return 0;
}
