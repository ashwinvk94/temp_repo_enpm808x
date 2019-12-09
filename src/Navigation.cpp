/******************************************************************************
 *  MIT License
 *
 *  Copyright (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

/**
 * @file        Navigation.hpp
 * @author      Abhinav Modi
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 1, 2019
 * @brief       Function Declaration for Navigation Class
 */

#include <iostream>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "ros/ros.h"
#include "../include/ROSModule.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "kids_next_door/moveTo.h"
#include "../include/Navigation.hpp"

Navigation::Navigation() {
    initializeServiceServers();
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

