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
         <table>
            <tr>
                <th>Value</th>
                <th>Meaning</th>
            </tr>
            <tr>
                <td>0.0</td>
                <td>Obstacle-occupied cell</td>
            </tr>
            <tr>
                <td>0.5</td>
                <td>Obstacle-free cell</td>
            </tr>
            <tr>
                <td>i (&ge; 1)</td>
                <td>Initial location of Robot-i</td>
            </tr>
        </table> 
