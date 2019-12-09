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
#include "ROSModule.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "kids_next_door/moveTo.h"


class Navigation : public ROSModule {
 public:

  Navigation();

  /**
   * @brief Method for publishing goal positions for the motion robot base
   *        through the environment  
   *
   * @param pos The desired pose the robot has to be moved to.
   *
   * @return None
   */

  bool moveToSrv(kids_next_door::moveTo::Request& req, 
                 kids_next_door::moveTo::Response& resp);
  /**
   * @brief Method for getting current pose of the robot
   *
   * @param None
   *
   * @return Returns variable currPos
   */
  // void initializeSubscribers();

  void initializeServiceServers();
  void setGoal(const geometry_msgs::PoseStamped& goalPose);
 private :
  /**
   * @brief pose of the target in the robot's body frame   
   */
  move_base_msgs::MoveBaseGoal goal;

  /**
   * @brief pose of the target in the robot's body frame   
   */
  ros::NodeHandle handler;

  ros::ServiceServer server;

};

#endif  // INCLUDE_NAVIGATION_HPP_