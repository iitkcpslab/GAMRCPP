# GAMRCPP (Goal Assignment-based Multi-Robot Coverage Path Planning)
Implementation of **Scalable Online Coverage Path Planning for Multi-Robot Systems** [IROS 2022] in **ROS Melodic Morenia** using **C++**.

###### Instructions:

1.  Download the source code package:<br/> 
    `cd ~/catkin_ws/src/`<br/> 
    `git clone https://github.com/iitkcpslab/GAMRCPP.git`
2.  Understand the directory structure:<br/> 
    1.  include: Contains the header files.<br/> 
        By default, the package is configured for **GAMRCPP** with the motions of a *Quadcopter in a 2D workspace*. However, this can be reconfigured in *debug.h* file. You may uncomment `#define TURTLEBOT` to configure the package for **GAMRCPP** with the motions of a *Turtlebot*. Similarly, you may uncomment `#define HOR_LEN_MAX` to configure the package for **GAMRCPP<sub>MAX</sub>** instead of **GAMRCPP**. 
    2.  input: *robot_ws.txt* represents the 2D workspace grid with obstacles and initial locations of the robots. 
         | Value      | Meaning                              |
         | -----      | ------------------------------------ |
         | 0.0        | Obstacle-occupied cell               |
         | 0.5        | Obstacle-free cell                   |
         | i (&ge; 1) | Initial location of **Robot-i**      |

        *E.g.*, the given file represents a *4 x 3* grid with 2 robots - Robot-1 and Robot-2, having initial locations (0,0) and (3,2), respectively. 
    3.  msg: Contains message files. 
    4.  output: Contains the obtained results when run with the input. 
        * *cap_j.csv* file stores the collision-free paths of the robots generated in the *j*-th horizon. 
        * *resultPerHorizon.txt* file stores horizon-wise computation times and horizon lengths. 
        * *fp_stat.txt* file stores the performance of algorithm, which computes feasible paths. The format of each row is as follows -

        | Horizon ID | Iteration ID of the inner WHILE loop | Active robot count | Killed robot count | Revived robot count | Visited goal count |
        | ---------- | ------------------------------------ | ------------------ | ------------------ | ------------------- | ------------------ |
    5.  src: Contains the source files corresponding to the header files. 
        * Robot side: *robot.cpp* emulates a robot and *start_robots.cpp* starts as many robots as required in an experiment. 
        * Coverage Planner side: The rest of the source files. 
    6.  srv: Contains the service files. 
3.  Build the package:<br/> 
    `cd ~/catkin_ws ; catkin_make clean && catkin_make`
4.  Run the package in a Terminal:
    -   Tab 1:
        `rosclean purge -y && pkill roscore ; roscore`
    -   Tab 2:
        `source ~/catkin_ws/devel/setup.bash`<br/> 
        `rm ~/catkin_ws/src/GAMRCPP/output/* ; rosrun gamrcpp_pkg gamrcppMainExecutable _ws_x:=<Workspace size along the x axis> _ws_y:=<Workspace size along the y axis> _rc:=<Robot count>`
    -   Tab 3:
        `cp ~/<Workspace directory>/robot_ws.txt ~/catkin_ws/src/GAMRCPP/input/`<br/> 
        `source ~/catkin_ws/devel/setup.bash`<br/> 
        `rosrun gamrcpp_pkg start_robots <Robot count>`

##### Gazebo simulation videos with quadcopters using PX4

https://user-images.githubusercontent.com/17489756/181875418-7d6f449d-bd06-4b48-8321-0521ec531a89.mp4
https://user-images.githubusercontent.com/17489756/181875451-d613dc90-03b5-4e05-b1c5-5a790d403a5a.mp4

