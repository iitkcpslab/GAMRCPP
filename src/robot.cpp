/*
Purpose: Robot
Last updated: 
Last updated on: 17 July 2022
Author: Ratijit Mitra
*/


#include <ros/ros.h>
#include <ros/package.h>

#include <math.h>
#include <string>
#include <stdlib.h>
#include <vector> 
#include <iostream>
#include <fstream>
#include <string.h>

#include <boost/lexical_cast.hpp>

#include "gamrcpp_pkg/basics.h"
#include "gamrcpp_pkg/debug.h"
#include "gamrcpp_pkg/PlanForHorizon.h"
#include "gamrcpp_pkg/ShareLocalInformation.h"


#define IP_DIR_NAME "/input/"						// Input directory name
#define ROB_WS_FILENAME "robot_ws.txt"				// Workspace with obstacles and robots
#define CP_SERVICE_NAME "/gap_node/share_workspace"
#define MOTION_PRIMITIVE_EXECUTAION_TIME 1			// 2 seconds


using namespace ros;


class RobotClass
{	
	public:
		NodeHandle *nh;
		int rob_id;						// Robot ID
		uint hor_id;					// Horizon ID
		ros::ServiceServer path_ss;		// Path Service Server
		double_mat ws_lidar;			// LiDAR values
		uint ws_size_x;					// Workspace size
		uint ws_size_y;
		float posX;						// Current pose
		float posY;
		float posTheta;
		double_mat lv;					// Local view of the workspace
		plan_vec_t path;				// Path


		//====================================================================================================
		void initializeRobot(NodeHandle *nh)
		{
			this->nh = nh;
			this->nh->getParam("rid", rob_id);

			string path_service_name = "robot_" + to_string(rob_id) + "/share_plan";
			path_ss = nh->advertiseService(path_service_name, &RobotClass::getPath, this);
			printf("Path Service Server = %s", path_service_name.c_str());
		}


		//====================================================================================================
		void populateLidarValues()
		{
			string rob_ws_file_path = ros::package::getPath("gamrcpp_pkg") + IP_DIR_NAME + ROB_WS_FILENAME;
			ifstream ifs;
		    ifs.open(rob_ws_file_path.c_str()); 
		  	string line;
		  	uint row = 0, col;
		  	float d;

		    while(ifs)
		    {
		        getline(ifs, line);
		        col = 0;
		        char *token = strtok(&line[0], ",");
		        double_vec ws_lidar_val;

		        while(token)
		        {
		        	d = atof(token);		// 0: Obstacle, 0.5: Free

		        	if(d >= 1)		// Robot IDs
		        	{
		        		if(int(d) == rob_id + 1)			// Robot IDs start from 0
		        		{
		        			posX = row;
							posY = col;
							posTheta = 1;		// 0, rob_id % 4, rand() % 4		//0 = E, 1 = N, 2 = W, 3 = S
		        		}
			        	
			        	// ws_lidar[row][col] = 0.5;
			        	ws_lidar_val.push_back(0.5);
		        	}
		        	else
		        		// ws_lidar[row][col] = d;
			        	ws_lidar_val.push_back(d);
		        	
		        	token = strtok(NULL, ",");
		        	col++;
		        	
	        		if(ws_size_y < col)
	        			ws_size_y = col;
		        }

		        row++;
		        ws_lidar.push_back(ws_lidar_val);
		    }

		    ws_size_x = row - 1;
		    ifs.close();

		    // for(int j = ws_size_y - 1; j >= 0; j--)
		    // {
		    // 	for(int i = 0; i < ws_size_x; i++)
		    // 	{
		    // 		printf("%.1lf\t", ws_lidar[i][j]);
		    // 	}

		    // 	printf("\n");
		    // }

		    printf("\nR_%d Initial State = (%d, %d, %d)\n==========\n", rob_id, (int)posX, (int)posY, (int)posTheta);

		    for(uint i = 0; i < ws_size_x; i++)		// Initialize: -1 = Unexplored
		    {
		    	double_vec lv_val;

				for(uint j = 0; j < ws_size_y; j++)
					// lv[i][j] = -1;
					lv_val.push_back(-1);

				lv.push_back(lv_val);
		    }
		}
		
		
		//====================================================================================================
		void updateLocalView()
		{
			int current_x = (int)posX;
	        int current_y = (int)posY;
	        int current_theta = (int)posTheta;
			printf("R_%d (%d, %d, %d)\n", rob_id, current_x, current_y, current_theta);

			int neighbourX, neighbourY;

			switch(current_theta)		// 0: East, 1: North, 2: West, 3: South
			{
				case 0:	neighbourX = current_x + 1;		// Forward cell
						neighbourY = current_y;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))		// Valid neighbour
							if(lv[neighbourX][neighbourY] == -1)		// Only update unexplored cells
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x;		// Left cell
						neighbourY = current_y + 1;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x;		// Right cell
						neighbourY = current_y - 1;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x - 1;		// Backward cell
						neighbourY = current_y;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];
					break;
				case 1:	neighbourX = current_x;		// Forward cell
						neighbourY = current_y + 1;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x - 1;		// Left cell
						neighbourY = current_y;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x + 1;		// Right cell
						neighbourY = current_y;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x;		// Backward cell
						neighbourY = current_y - 1;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];
					break;
				case 2:	neighbourX = current_x - 1;		// Forward cell
						neighbourY = current_y;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x;		// Left cell
						neighbourY = current_y - 1;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x;		// Right cell
						neighbourY = current_y + 1;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x + 1;		// Backward cell
						neighbourY = current_y;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];
					break;
				case 3:	neighbourX = current_x;		// Forward cell
						neighbourY = current_y - 1;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x + 1;		// Left cell
						neighbourY = current_y;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x - 1;		// Right cell
						neighbourY = current_y;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

						neighbourX = current_x;		// Backward cell
						neighbourY = current_y + 1;

						if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
							if(lv[neighbourX][neighbourY] == -1)
								lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];
					break;
			}

			lv[current_x][current_y] = 1.0;			// Robot's current cell is covered
		}


		void updateLocalView_longitudinal()
		{
			int current_x = (int)posX;
	        int current_y = (int)posY;
	        int current_theta = (int)posTheta;
			printf("Robot_%d (%d, %d, %d)\n", rob_id, current_x, current_y, current_theta);

			int neighbourX, neighbourY;

			neighbourX = current_x + 1;		// Right cell
			neighbourY = current_y;

			if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
				if(lv[neighbourX][neighbourY] == -1)		// Only update unexplored cells
					lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

			neighbourX = current_x;		// Top cell
			neighbourY = current_y + 1;

			if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
				if(lv[neighbourX][neighbourY] == -1)
					lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

			neighbourX = current_x - 1;		// Left cell
			neighbourY = current_y;

			if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
				if(lv[neighbourX][neighbourY] == -1)
					lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

			neighbourX = current_x;		// Bottom cell
			neighbourY = current_y - 1;

			if((0 <= neighbourX && neighbourX < ws_size_x) && (0 <= neighbourY && neighbourY < ws_size_y))
				if(lv[neighbourX][neighbourY] == -1)
					lv[neighbourX][neighbourY] = ws_lidar[neighbourX][neighbourY];

			lv[current_x][current_y] = 1.0;			// Robot's current cell is covered
		}
		
		
		//====================================================================================================
		void printWorkspace()
		{
			int i, j;
			printf("R_%d (Horizon = %d) local view...\n\n", rob_id, hor_id);

			for(j = ws_size_y - 1; j >= 0; j--)
			{
				for(i = 0; i < ws_size_x; i++)
				{
					printf("%.1lf ", lv[i][j]);
				}
				
				printf("\n");
			}
				
			printf("\n");
		}
		
		
		//====================================================================================================
		void sendLocalView()
		{
			ros::ServiceClient localview_sc = nh->serviceClient<gamrcpp_pkg::ShareLocalInformation>(CP_SERVICE_NAME);
			
			gamrcpp_pkg::ShareLocalInformation lv_srv;
			lv_srv.request.robot_id = rob_id;
			lv_srv.request.horizon = hor_id;
			lv_srv.request.x = posX;
	    	lv_srv.request.y = posY;
			lv_srv.request.theta = posTheta;
			
			for(uint i = 0; i < ws_size_x; i++)
				for(uint j = 0; j < ws_size_y; j++)
					lv_srv.request.workspace.push_back(lv[i][j]);
			
			if(localview_sc.call(lv_srv))
				printf("R_%d (Horizon = %d) sendLocalView().\n", rob_id, (int)lv_srv.response.next_horizon);
			else
				printf("R_%d (Horizon = %d) sendLocalView()!\n", rob_id, hor_id);
		}
		

		//====================================================================================================
		bool getPath(gamrcpp_pkg::PlanForHorizon::Request &req, gamrcpp_pkg::PlanForHorizon::Response &res)
		{
			res.next_horizon = hor_id;
			path.clear();
			 
			for(uint i = 0; i < req.plans.size(); i++)
			{
				motionPlan tmp_plan;
				tmp_plan.id = (int)req.plans[i].robot_id;
	  			tmp_plan.time_instance = (int)req.plans[i].horizon_step;
				tmp_plan.location_x = req.plans[i].x;
				tmp_plan.location_y = req.plans[i].y;
				tmp_plan.theta = req.plans[i].theta;
				path.push_back(tmp_plan);
			}

			#ifdef TURTLEBOT
				followPath();
			#else
				followPath_longitudinal();
			#endif

	  		return true;
		}


		//====================================================================================================
		void followPath()
		{
			size_t path_size = path.size();

			if(path_size)
			{
				float posX_next, posY_next, posTheta_next;
				int posTheta_int = (int)posTheta, posTheta_next_int;

				printf("\nFollowing path...\n");
				Time primitive_begin, primitive_end;

				for(size_t i = 1; i < path_size; i++)
				{
					primitive_begin = Time::now();
					posX_next = path[i].location_x;
					posY_next = path[i].location_y;
					posTheta_next = path[i].theta;
					posTheta_next_int = (int)posTheta_next;

					if(posTheta == posTheta_next)
						if((posX == posX_next) && (posY == posY_next))
							printf("R_%d (Horizon = %d) Waited\n", rob_id, hor_id);
						else if(((posX + 1) == posX_next) || ((posX - 1) == posX_next) || ((posY + 1) == posY_next) || ((posY - 1) == posY_next))
							printf("R_%d (Horizon = %d) Moved Forward\n", rob_id, hor_id);
						else
						{
							printf("R_%d Invalid motion (%1.0f, %1.0f, %d) (%1.0f, %1.0f, %d)! T=%zu\n", rob_id, posX, posY, posTheta_int, posX_next, posY_next, posTheta_next_int, i);
							exit(1);
						}
					else
						if(((posTheta_int + 1) % 4) == posTheta_next_int)
							printf("R_%d (Horizon = %d) Rotated Left\n", rob_id, hor_id);
						else if(((posTheta_next_int + 1) % 4) == posTheta_int)
							printf("R_%d (Horizon = %d) Rotated Right\n", rob_id, hor_id);
						else
						{
							printf("R_%d Invalid motion (%1.0f, %1.0f, %d) (%1.0f, %1.0f, %d)! T=%zu\n", rob_id, posX, posY, posTheta_int, posX_next, posY_next, posTheta_next_int, i);
							exit(1);
						}

					posX = posX_next;
					posY = posY_next;
					posTheta = posTheta_next;
					posTheta_int = (int)posTheta;
					
					updateLocalView();
					//printWorkspace();
					primitive_end = Time::now();
					fflush(stdout);
					Duration(Duration(MOTION_PRIMITIVE_EXECUTAION_TIME) - (primitive_end - primitive_begin)).sleep();
				}

				updateHorizon();
				sendLocalView();
			}
			else
			{
				printf("R_%d (Horizon = %d). Coverage completed.\n", rob_id, hor_id);
				ros::shutdown();
			}
		}


		void followPath_longitudinal()
		{
			const size_t path_size = path.size();

			if(path_size)
			{
				float posX_next, posY_next, posTheta_next;
				int posTheta_int = (int)posTheta, posTheta_next_int;

				printf("\nFollowing path...\n");
				Time primitive_begin, primitive_end;

				for(size_t i = 1; i < path_size; i++)
				{
					primitive_begin = Time::now();
					posX_next = path[i].location_x;
					posY_next = path[i].location_y;
					posTheta_next = path[i].theta;
					posTheta_next_int = (int)posTheta_next;

					if((posX == posX_next) && (posY == posY_next))
						printf("Robot_%d (Horizon = %d) Waited\n", rob_id, hor_id);
					else
						if(posX == posX_next)
							if(posY + 1 == posY_next)
								printf("Robot_%d (Horizon = %d) Moved Top\n", rob_id, hor_id);
							else
								printf("Robot_%d (Horizon = %d) Moved Bottom\n", rob_id, hor_id);
						else
							if(posX + 1 == posY_next)
								printf("Robot_%d (Horizon = %d) Moved Right\n", rob_id, hor_id);
							else
								printf("Robot_%d (Horizon = %d) Moved Left\n", rob_id, hor_id);

					posX = posX_next;
					posY = posY_next;
					posTheta = posTheta_next;

					updateLocalView_longitudinal();
					//printWorkspace();
					primitive_end = Time::now();
					fflush(stdout);
					Duration(Duration(MOTION_PRIMITIVE_EXECUTAION_TIME) - (primitive_end - primitive_begin)).sleep();
				}

				updateHorizon();
				sendLocalView();
			}
			else
			{
				printf("Robot_%d (Horizon = %d). Coverage completed.\n", rob_id, hor_id);
				ros::shutdown();
			}
		}


		//====================================================================================================
		void updateHorizon()
		{
			hor_id++;
			printf("Next Horizon = %d\n==================================================\n", hor_id);
		}
};


//====================================================================================================
int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "robotNode");		// Rename using __name:=robot_i in the rosrun command where i (>= 0) is the robot id
	NodeHandle nh("~");
	
	RobotClass *robot_obj = new RobotClass();
	robot_obj->initializeRobot(&nh);
	robot_obj->populateLidarValues();

	#ifdef TURTLEBOT
		robot_obj->updateLocalView();
	#else
		robot_obj->updateLocalView_longitudinal();
	#endif
	
	//robot_obj->printWorkspace();
	robot_obj->sendLocalView();

	ros::spin();
	ros::shutdown();
	return 0;
}