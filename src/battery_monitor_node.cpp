#include "ros/ros.h"
#include "ArmduinoRover/floatArr.h"
#include <string>
#include <vector>

using namespace std;

vector<float> thresholds;

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

void onBatteryReceived(const ArmduinoRover::floatArr::ConstPtr& msg) {
    if(msg->data.size()!=thresholds.size()){
    	ROS_ERROR_STREAM("Bad battery monitor config");
    }else{
    	for(int i=0;i<msg->data.size();i++){
    		if(msg->data.at(i)<thresholds.at(i)){
    			ROS_ERROR_STREAM("BATTERY LOW BATTERY LOW BATTERY LOW!!!!");
    		}else{
    			ROS_INFO_STREAM("Battery "<<i<<" is at "<<msg->data.at(i)<<" volts");
    		}
    	}
    }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "battery_monitor_node");

	ros::NodeHandle n;

	ros::Subscriber encoderSub = n.subscribe("battery_lectures", 1000,
			onBatteryReceived);
	string thresholdsStr;
	ros::param::param<string>("thresholds", thresholdsStr, "7.0 5.0");
	vector<string>  thresholdSplit=split(thresholdsStr," ");

	for(int i=0;i<thresholdSplit.size();i++){
		thresholds.push_back(atof(thresholdSplit.at(i).c_str()));
	}
	ros::spin();
	return 0;
}
