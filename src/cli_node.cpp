#include "ros/ros.h"
#include <ArmduinoRover/cliComm.h>
#include<iostream>    //cout
#include<stdio.h> //printf
#include<string.h>    //strlen
#include<string>  //string
#include<sys/socket.h>    //socket
#include<arpa/inet.h> //inet_addr
#include<netdb.h> //hostent

using namespace std;

ros::ServiceClient cliBridgeClient;

bool cliComm(ArmduinoRover::cliComm::Request &req,
		ArmduinoRover::cliComm::Response &res) {
	ArmduinoRover::cliComm  bridgeSrv;
	bridgeSrv.request.str=req.str;
	if(cliBridgeClient.call(bridgeSrv)){
		res.str=bridgeSrv.response.str;
		return true;
	}
	return false;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "cli_communication_node");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;

	ros::ServiceServer cliServer = n.advertiseService("cli_communication",
			cliComm);
	cliBridgeClient=n.serviceClient<ArmduinoRover::cliComm>("cli_bridge");

	ros::spin();

	return 0;
}

