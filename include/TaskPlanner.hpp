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
 * @file        TaskPlanner.hpp
 * @author      Abhinav Modi
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 1, 2019
 * @brief       Header file for Task planner module to make high-level decisions for 
 *              switching between navigation, manipulation and obstacle avoidance tasks
 */

#ifndef INCLUDE_TASKPLANNER_HPP_
#define INCLUDE_TASKPLANNER_HPP_

#include <iostream>
#include <vector>
#include <opencv/core/core.hpp>
#include <opencv/highgui/highgui.hpp>
#include <opencv/imgproc/imgproc.hpp>
#include "ros/ros.h"

class TaskPlanner {
 public:

  /**
   * @brief search operation for scanning the area for ArUco markers  
   *
   * @param None
   *
   * @return None
   */
  void search();

  /**
   * @brief inRangeCheck method checks if the toy object is in a reachable range
   *        for the manipulator 
   *
   * @param None
   *
   * @return bool Returns true if the toy object is in range otherwise false
   */
  bool inRangeCheck();

  /**
   * @brief Method to add a new task to the planner 
   *
   * @param taskID  Integer id for the new task
   *
   * @param taskName  Definition of the new task 
   *
   * @return None
   */
  void addNewTask(int taskID, std::string taskName);

  /**
   * @brief currTask outputs the integer index of the current task being 
   *        performed by the robot 
   *
   * @param None
   *
   * @return int Integer ID of the task
   */
  int currTask();

  /**
   * @brief Main method which switches from the current task to the next one 
   *        based on feedback from the robot.
   *
   * @param None
   *
   * @return None
   */
  void taskPlanner();

 private :
  /**
   * @brief map object which contains the list of tasks to be performed
   *        by the robot indexed by an integer key for each task.
   */
  std::map<int, std::string> taskList; 

  /**
   * @brief 2D grid map of the world for navigation purposes
   */
  cv::Mat map; 
}

#endif  // INCLUDE_TASKPLANNER_HPP_