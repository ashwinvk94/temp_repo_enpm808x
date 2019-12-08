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
 * @file        TaskPlanner.cpp
 * @author      Rohan Singh
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 7, 2019
 * @brief       Function definition for class TaskPlanner
 */

#include <iterator>
#include <vector>

#include "TaskPlanner.hpp"

TaskPlanner::TaskPlanner() {
    ROS_INFO_STREAM("Constructing TaskPlanner node...");
    /* Check if everything is OK */
    if (!ros::ok()) {
        ROS_FATAL_STREAM("ROS node is not running.");
    }
    /* Create ROS node handle */
    ros::NodeHandle nh;
  
}

~TaskPlanner::TaskPlanner() {

}

void TaskPlanner::initializePublishers() {
    ROS_INFO_STREAM("Initializing Publishers...");

    /* Publisher for toy ID lookup */
    toyIDPub = nh.advertise<std_msgs::Int32>("/knd/toyID", 1, true);

    /* Publisher for desired robot pose */
    goalPosePub = nh.advertise<geometric_msgs::PoseStamped>
                    ("/knd/goalPose", 1, true);
}

void TaskPlanner::initializeSubscribers() {
    ROS_INFO_STREAM("Initializing Subscribers...");

    /* Subscriber for checking toy visibility */
    toyFoundSub = nh.subscribe("/knd/toyFoundFlag", 1,
                    &TaskPlanner::toyFoundCallback, this);

    /* Subscriber for current robot pose */
    currPoseSub = nh.subscribe("/knd/currPose", 1,
                    &TaskPlanner::currPoseCallback, this);

    /* Subscriber for current toy pose */
    toyPoseSub = nh.subscribe("/knd/toyPose", 1,
                    &TaskPlanner::toyPoseCallback, this);
}

// void TaskPlanner::explore() {

// }

void TaskPlanner::search() {

}

// bool TaskPlanner::inRangeCheck() {

// }

// void TaskPlanner::addNewTask(int taskID, std::string taskName) {

// }

// int TaskPlanner::currTask() {

// }

void TaskPlanner::taskPlanner() {
    /* Create iterator for toy IDs */
    std::vector<int>::iterator currToyID = toyIDs.begin();
    /* Set node rate */
    ros::Rate loopRate(nodeHz);

    bool reachedToyFlag = false;
    bool pickedToyFlag = false;
    bool reachedStorgeFlag = false;
    bool storedToyFlag = false;
    bool cleanupFlag = false;

    while (ros::ok()) {
        /* If cleanup of toys needs to be done */
        if (!cleanupFlag) {
            /* If toy hasn't been reached */
            if (!reachedToyFlag) {
                if (lookForToy(*currToyID)) {   // Look for toy
                    ROS_INFO_STREAM("Found for toy with ID : " << *currToyID);
                    reachedToyFlag = goToToy();
                } else {        // Search if can't see toy
                    ROS_INFO_STREAM("Looking for toy with ID : "
                                    << *currToyID);
                    search();
                }
            /* If toy has been reached, pick up toy */
            } else if (reachedToyFlag && !pickedToyFlag) {
                ROS_INFO_STREAM("Picking up toy with ID : " << *currToyID);
                if (pickUpToy()) {
                    ROS_INFO_STREAM("Picked up toy with ID : " << *currToyID);
                    pickedToyFlag = true;
                }
            /* If toy has been picked, go to storage */
            } else if (pickedToyFlag && !reachedStorgeFlag){
                ROS_INFO_STREAM("Going to storage");
                if (goToStorage()) {
                    ROS_INFO_STREAM("Reached storage");
                    reachedStorgeFlag = true;
                }
            /* If storage has been reached, keep toy back */
            } else if (reachedStorgeFlag && ! storedToyFlag) {
                ROS_INFO_STREAM("Storing toy with ID : " << *currToyID);
                if (storeToy()) {
                    ROS_INFO_STREAM("Stored toy with ID : " << *currToyID);
                    storedToyFlag = true;
                }
            /* If toy is placed in storage, move to next */
            } else if (storedToyFlag) {
                currToyID++;
                reachedToyFlag = false;
                pickedToyFlag = false;
                reachedStorgeFlag = false;
                storedToyFlag = false;
            }

            /* If all toys have been stored */
            if (currToyID == toyIDs.end()) {
                ROS_INFO_STREAM("All toys stored. Cleanup complete!!!.");
                cleanupFlag = true;
            }
        } else {
            shutdownRobot();
        }
    }
}

bool TaskPlanner::lookForToy(int toyID) {
    std_msgs::Int32 _toyID;
    _toyID.data = toyID;
    toyIDPub.publish(_toyID);
    return toyFoundFlag.data;
}

bool TaskPlanner::goToToy() {

}

bool TaskPlanner::pickUpToy() {

}

bool TaskPlanner::goToStorage() {

}

bool TaskPlanner::storeToy() {

}

void TaskPlanner::shutdownRobot() {

}

bool TaskPlanner::() {

}