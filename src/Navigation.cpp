#include "../include/Navigation.hpp"

Navigation::Navigation() {
    initializeServiceServers();
}
Navigation::~Navigation(){
}

void Navigation::initializeServiceServers() {
    server = handler.advertiseService("/knd/moveTo", &Navigation::moveToSrv, this);
    ROS_INFO_STREAM("Running Service");
    ros::spin();
}

void Navigation::setGoal(const geometry_msgs::PoseStamped& goalPose) {
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goalPose.pose.position.x;
    goal.target_pose.pose.position.y = goalPose.pose.position.y;
    goal.target_pose.pose.position.z = goalPose.pose.position.z;
    goal.target_pose.pose.orientation.x = goalPose.pose.orientation.x;
    goal.target_pose.pose.orientation.y = goalPose.pose.orientation.y;
    goal.target_pose.pose.orientation.z = goalPose.pose.orientation.z;
    goal.target_pose.pose.orientation.w = goalPose.pose.orientation.w;
}
move_base_msgs::MoveBaseGoal Navigation::getGoalPose() {
    return this->goal;

}
bool Navigation::moveToSrv(kids_next_door::moveTo::Request& req,
                           kids_next_door::moveTo::Response& resp) {
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  
  	MoveBaseClient ac("/move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO_STREAM("Waiting for the action server to come up");
    }
    // get new target position
    geometry_msgs::PoseStamped goalPose = req.goalPose;
    setGoal(goalPose);
    ROS_INFO_STREAM("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();
    std_msgs::Bool reachedTarget;
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO_STREAM("Action Completed, moved to goal position");
        reachedTarget.data = true;
    } else {
        ROS_INFO_STREAM("Failed to move to goal position");
        reachedTarget.data = false;
    }
    resp.reachedGoal = reachedTarget;
    return true;
}

// int main(int argc, char** argv){
// 	ros::init(argc, argv, "navigation"); //node name

// 	ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

// 	ROS_INFO_STREAM("Started navigation node");

	
// 	// ROSModule * nav =  new Navigation();
//     Navigation nav;
// 	ROS_INFO_STREAM("Spinning");
	
	
// 	return 0;
// };
