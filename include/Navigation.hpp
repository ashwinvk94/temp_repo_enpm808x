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
 * @brief       Header file for Navigation Module which contains methods for
 *              localization and movement of the robot through the world.
 */

#ifndef INCLUDE_NAVIGATION_HPP_
#define INCLUDE_NAVIGATION_HPP_

#include <iostream>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#include "../include/ROSModule.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "kids_next_door/moveTo.h"


class Navigation {
 public:

  Navigation();

  /**
   * @brief Service server which gets new target pose in request
   *        and starts moving towards it. It returns true/false once
   *        the service call is completed.       
   *
   * @param req - service request object of new target Pose 
   * @param resp - service response object of completed action
   *
   * @return bool - true when the service is completed
   */

  bool moveToSrv(kids_next_door::moveTo::Request& req, 
                 kids_next_door::moveTo::Response& resp);
  /**
   * @brief Method for initializinig service servers
   *
   * @param None
   *
   * @return None
   */
  void initializeServiceServers();\

  /**
   * @brief Method to set new goal position
   *
   * @param goalPose - const reference to new goal position 
   *
   * @return None
   */
  void setGoal(const geometry_msgs::PoseStamped& goalPose);

  /**
   * @brief Method for accessing the private member goalPose
   *
   * @param None
   *
   * @return move_base_msgs::MoveBaseGoal - access to data member goalPose 
   */
  move_base_msgs::MoveBaseGoal getGoal();

    
  /**
   * @brief Default Destructor for navigation class
   */    
  ~Navigation();

 private :
  /**
   * @brief pose of the target in the robot's body frame   
   */
  move_base_msgs::MoveBaseGoal goal;

  /**
   * @brief Node handler object for navigation node   
   */
  ros::NodeHandle handler;

  /**
   * @brief Service server object for moveTo service   
   */  
  ros::ServiceServer server;

};

#endif  // INCLUDE_NAVIGATION_HPP_