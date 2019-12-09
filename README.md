# Kids Next Door
Pick and Place Robot simulation for toy collection in pre-schools

[![Build Status](https://travis-ci.org/rohansingh42/kids_next_door.svg?branch=master)](https://travis-ci.org/rohansingh42/kids_next_door)
[![Coverage Status](https://coveralls.io/repos/github/rohansingh42/kids_next_door/badge.svg?branch=master)](https://coveralls.io/github/rohansingh42/kids_next_door?branch=master)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview

This repository contains the simulation of a mobile manipulator robot, __Tiago++__, used for pick and place operations for tidying up toys in play areas of children. It could be deployed in pre-school classrooms. 

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

