# GAMRCPP (Goal Assignment-based Multi-Robot Coverage Path Planning)
Implementation of **Scalable Online Coverage Path Planning for Multi-Robot Systems** [IROS 2022] in **ROS Melodic Morenia** using **C++**.

###### Instructions:

1.  Download the source code:<br/> 
    `cd ~/catkin_ws/src/`<br/> 
    `git clone https://github.com/iitkcpslab/GAMRCPP.git`
2.  Understand the directory structure:<br/> 
    1.  include: Contains the header files.<br/> 
        *Note*: By default, the source code is configured for **GAMRCPP** with the motions of *Quadcopters in 2D workspace*. However, this can be changed in *debug.h* file. You may uncomment `#define TURTLEBOT` to configure the source code for **GAMRCPP** with the motions of *Turtlebots*. Similarly, you may uncomment `#define HOR_LEN_MAX` to configure the source code for **GAMRCPP<sub>MAX</sub>** instead of **GAMRCPP**. 
