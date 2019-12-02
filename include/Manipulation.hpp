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
 * @brief       Header file for Manipulation module. It contains functions 
 *              and data members required for the manipulation tasks like picking up
 *              and placing objects.
 */

#ifndef INCLUDE_MANIPULATION_HPP_
#define INCLUDE_MANIPULATION_HPP_

#include <iostream>
#include <vector>
#include <opencv/core/core.hpp>
#include <opencv/highgui/highgui.hpp>
#include <opencv/imgproc/imgproc.hpp>
#include "ros/ros.h"

class Manipulation {
 public:
  /**
   * @brief getToyPosition gets the position of the toy to be picked up from
   *        the ToyDetection module
   *
   * @param None
   *
   * @return None
   */
  void getToyPosition();
  
  /**
   * @brief Method for picking up the desired object at a known position  
   *
   * @param None
   *
   * @return bool - returns true if the object is successfully picked up else
   *                false
   */
  bool pickup();
  
  /**
   * @brief Method for dropping the object in hand at a desired location 
   *
   * @param None
   *
   * @return bool - returns true of the drop operation is successful else false
   */
  bool drop();

 private :
  /**
   * @brief position of the toy object undergoing a manipulation task 
   */
  geometry_msgs::Pose toyPosition;
}

#endif  // INCLUDE_MANIPULATION_HPP_