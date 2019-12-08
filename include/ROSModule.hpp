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
 * @file        ROSModule.hpp
 * @author      Rohan Singh
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 6, 2019
 * @brief       Base Header file for ROS Modules
 */

#ifndef INCLUDE_ROSMODULE_HPP_
#define INCLUDE_ROSMODULE_HPP_

#include <iostream>
#include <vector>
#include "ros/ros.h"

class ROSModule {
 public:

  /**
   * @brief Constructor for class
   *
   * @param None
   *
   * @return None
   */
  ROSModule(){};

  /**
   * @brief Destructor for class
   *
   * @param None
   *
   * @return None
   */
  virtual ~ROSModule(){};

  /**
   * @brief Function to initialize Subscribers for node
   *
   * @param None
   *
   * @return None
   */
  virtual void initializeSubscribers(){};

  /**
   * @brief Function to initialize Publishers for node
   *
   * @param None
   *
   * @return None
   */
  virtual void initializePublishers(){};

  /**
   * @brief Function to initialize Service Servers for node
   *
   * @param None
   *
   * @return None
   */
  virtual void initializeServiceServers(){};

  /**
   * @brief Function to initialize Service Clients for node
   *
   * @param None
   *
   * @return None
   */
  virtual void initializeServiceClients(){};

  /**
   * @brief Function to initialize Action Client for node
   *
   * @param None
   *
   * @return None
   */
  virtual void initializeActionClients(){};

  /**
   * @brief Function to initialize Transform Broadcasters for node
   *
   * @param None
   *
   * @return None
   */
  virtual void initializeTransformBroadcaster(){};

  /**
   * @brief Function to initialize Transform Listener for node
   *
   * @param None
   *
   * @return None
   */
  virtual void initializeTransformListener(){};
};

#endif  // INCLUDE_ROSMODULE_HPP_