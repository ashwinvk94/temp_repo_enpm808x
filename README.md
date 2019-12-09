# Kids Next Door
Pick and Place Robot simulation for toy collection in pre-schools

[![Build Status](https://travis-ci.org/ashwinvk94/temp_repo_enpm808x.svg?branch=travis)](https://travis-ci.org/ashwinvk94/temp_repo_enpm808x)
[![Coverage Status](https://coveralls.io/repos/github/rohansingh42/kids_next_door/badge.svg?branch=master)](https://coveralls.io/github/rohansingh42/kids_next_door?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview

This repository contains the simulation of a mobile manipulator robot, __Tiago++__, used for pick and place operations for tidying up toys in play areas. This system can be deployed in child-care centers and pre-schools for after-hours cleanup. The robot operates in a known map with a known covariance of where the toys could be. Each type of toy contains an ArUco marker with a unique id which are pre-known. This helps in both easy identification of toys for segregation and pose computation for moving towards a new -found toy.

## Workflow
The following steps explain the overall workflow of the pipeline:
  1. Once the robot is spawned and the ArUco blocks(toys) are generated in the environment, the robot moves to the location where the probability of finding the toys is the highest.
  2. Upon reaching the location it searches for the toy by yawing. 
  3. When a toy is detected in the frame, it's pose is computed in the world frame.
  4. This pose information is used to call a service "/knd/moveTo" which runs actionlib to go to the detected toy.
  5. After reaching the toy position, it returns to home and then goes back for the next toy.
  6. This process is repeated till all the toys are returned to home.
## Personnel 

__Rohan Singh__: I am in my third semester of M.Eng. in Robotics at University of Maryland. You can follow me on my [Linkedin](www.linkedin.com/in/rohansingh42)

__Abhinav Modi__: I am in my third semester of M.Eng. in Robotics at University of Maryland. You can follow me on my [Linkedin](https://www.linkedin.com/in/abhinavmodi16/)

__Ashwin Varghese Kuruttukulam__: I am in my third semester of M.Eng. in Robotics at University of Maryland. You can follow me on my [Linkedin](https://www.linkedin.com/in/ashwinvk94/)

## Product Backlog 
[![Packagist](https://img.shields.io/badge/AIP-Backlog-orange)](https://docs.google.com/spreadsheets/d/1EpZC6qNrfh5d6ULUvAsA_EainnZeKeKZD43DMa95FU4/edit?usp=sharing)
[![Packagist](https://img.shields.io/badge/AIP-Sprint-brightgreen)](https://docs.google.com/document/d/1S4FX_vaaVi4O-uJQCASvIETeV3dRj0eVRvi1tA0-r3A/edit?usp=sharing)

The Agile Iterative Process was followed for the development of the software. Follow this Google Sheets [link](https://docs.google.com/spreadsheets/d/1EpZC6qNrfh5d6ULUvAsA_EainnZeKeKZD43DMa95FU4/edit?usp=sharing) to view the product backlog and sprint schedules. Follow this Google Docs [link](https://docs.google.com/document/d/1S4FX_vaaVi4O-uJQCASvIETeV3dRj0eVRvi1tA0-r3A/edit?usp=sharing) to view Sprint notes.

## Building the package
To build this package follow the steps below:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/rohansingh42/kids_next_door
cd ..
<rohan add extra data instructions>
catkin_make
source devel/setup.bash
```

## Running the simulation demo
Once the build is complete successfully and you have sourced the bash file, you can run it using roslaunch:
```
roslaunch kids_next_door combined.launch
```
##  License
```
MIT License

Copyright (c) 2019 Rohan Singh, Abhinav Modi, Ashwin Kuruttukulam 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

