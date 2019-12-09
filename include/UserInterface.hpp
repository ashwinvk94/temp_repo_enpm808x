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
 * @file        UserInterface.hpp
 * @author      Rohan Singh
 * @copyright   MIT License (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 
 * @date        Dec 9, 2019
 * @brief       Header file for UserInterface Class
 */

#ifndef INCLUDE_USERINTERFACE_HPP_
#define INCLUDE_USERINTERFACE_HPP_

#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class UserInterface {
  public:
    /**
     * @brief Constructor for class
     *
     * @param None
     *
     * @return None
     */
    UserInterface();

    /**
     * @brief Constructor for class with input/output stream
     *
     * @param inputStream Input Stream object
     * 
     * @param outputStream Output Stream object
     *
     * @return None
     */
    UserInterface(std::istream& ipStream, std::ostream& opStream);

    /**
     * @brief Destructor for class
     *
     * @param None
     *
     * @return None
     */
    ~UserInterface();

    /**
     * @brief Function to input number of toys of each type
     *
     * @param None
     *
     * @return None
     */
    std::vector<int> getIDs();

    /**
     * @brief Function to input storage location
     *
     * @param None
     *
     * @return None
     */
    geometry_msgs::PoseStamped getStorageLocation();

  private:
    /* Input stream object */
    std::istream& inputStream;

    /* Output stream object */
    std::ostream& outputStream;
};

#endif  // INCLUDE_USERINTERFACE_HPP_