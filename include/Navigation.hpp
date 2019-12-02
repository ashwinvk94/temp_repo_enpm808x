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
#include <opencv/core/core.hpp>
#include <opencv/highgui/highgui.hpp>
#include <opencv/imgproc/imgproc.hpp>
#include "ros/ros.h"

class Navigation {
 public:
  /**
   * @brief Callback method for the robot location subscriber  
   *
   * @param None
   *
   * @return None
   */
  void localizeCb();
   
  /**
   * @brief Method for publishing goal positions for the motion robot base
   *        through the environment  
   *
   * @param pos The desired pose the robot has to be moved to.
   *
   * @return None
   */
  void moveBasePub(geometry_msgs::PoseStamped pos);

  /**
   * @brief Method for getting current pose of the robot
   *
   * @param None
   *
   * @return Returns variable currPos
   */
  geometry_msgs::PoseStamped getCurrPose();
    
 private :
  /**
   * @brief current Pose of the robot in the world coordinate frame  
   */
  geometry_msgs::Pose currPos;
}

#endif  // INCLUDE_NAVIGATION_HPP_