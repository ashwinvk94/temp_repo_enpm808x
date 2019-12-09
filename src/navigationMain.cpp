#include "../include/Navigation.hpp"

int main(int argc, char** argv){
	ros::init(argc, argv, "navigation"); //node name
	ROS_INFO_STREAM("Started navigation node");

	
	// ROSModule * nav =  new Navigation();
    Navigation nav;
	ROS_INFO_STREAM("Spinning");

	return 0;
};