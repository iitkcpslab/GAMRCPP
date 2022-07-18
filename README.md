# GAMRCPP (Goal Assignment-based Multi-Robot Coverage Path Planning)
Implementation of **Scalable Online Coverage Path Planning for Multi-Robot Systems** [IROS 2022] in **ROS Melodic Morenia** using **C++**.

###### Instructions:

1.  Download the source code:<br/> 
    `cd ~/catkin_ws/src/`<br/> 
    `git clone https://github.com/iitkcpslab/GAMRCPP.git`
2.  Understand the directory structure:<br/> 
    1.  include: Contains the header files.<br/> 
        *Note*: By default, the source code is configured for **GAMRCPP** with the motions of a *Quadcopter in a 2D workspace*. However, this can be reconfigured in *debug.h* file. You may uncomment `#define TURTLEBOT` to configure the source code for **GAMRCPP** with the motions of a *Turtlebot*. Similarly, you may uncomment `#define HOR_LEN_MAX` to configure the source code for **GAMRCPP<sub>MAX</sub>** instead of **GAMRCPP**. 
    2.  input: *robot_ws.txt* represents the 2D workspace grid with obstacles and initial robot locations. 
         | Value | Meaning                |
         | ----- | ---------------------- |
         | 0.0   | Obstacle-occupied cell |
         | 0.5   | Obstacle-free cell                   |
         | i (&ge; 1) | Initial location of **Robot-i** |
         
        *E.g.*, the given file represents a *4 x 3* grid with 2 robots - Robot-1 and Robot-2, having initial locations (0,0) and (3,2), respectively. 
    3.  msg: Contains message files. 
    4.  output: Contains the obtained results when run with the input. The *cap_j.csv* file stores the generated paths of the robots in the *j*-th horizon. *resultPerHorizon.txt* file stores horizon-wise computation times. *fp_stat.txt* file stores the performance of algorithm, which computes feasible paths. The format of each row is as follows -<br/> 

        | Horizon ID | Iteration ID of the inner while loop | Active robot count | Killed robot count | Revived robot count | Visited goal count |
        | ----- | ----- | ----- | ----- | ----- | ----- |