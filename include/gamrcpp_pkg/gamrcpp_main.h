/*
Purpose: 
Last updated: Variable names
Last updated on: 19 Mar 2022
Author: Ratijit Mitra
*/


#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <stack>


#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pthread.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>


#include "gamrcpp_pkg/PlanForHorizon.h"
#include "gamrcpp_pkg/ShareLocalInformation.h"


#include "gamrcpp_pkg/gamrcpp.h"

#ifdef DEBUG_RVIZ
	#include "gamrcpp_pkg/rviz.h"
#endif


using namespace std;


class GAMRCPP_MAIN
{
	public:
		void initializeGAMRCPP(ros::NodeHandle *nh);
		bool getLocalView(gamrcpp_pkg::ShareLocalInformation::Request &req, gamrcpp_pkg::ShareLocalInformation::Response &res);
		void updateGlobalView(gamrcpp_pkg::ShareLocalInformation::Request &req);
		void printWorkspace();
		void checkTotalRequests();
		int getDiscoveredCells();
		void checkCompletion(int discovered_cells);
		void sharePlan(plan_vec_t &totalPlan, int horizon_length);
		void callRobot(ros::ServiceClient sc, gamrcpp_pkg::PlanForHorizon pfh, std::string psn);
		void printHorizonInformation(uint active_robs_count, int horizon_length, double comp_time);
		void updateHorizonInformation(int horizon_length, double comp_time);
		void generateNextPlan();

		int ws_size_x;		// Workspace size along x
		int ws_size_y;		// Workspace size along y
		int rob_count;		// Number of robots

		ros::NodeHandle *nh;
		ros::ServiceServer localview_ss;		// Localview service server
		bool flag_mis_began;		// Has the mission begun?
		ros::Time mis_time_begin;	// Begining of the mission time

		int hor_id;		// Horizon ID
		int lv_count;	// Number of localviews in a horizon

		uint obs_count;		// Number of obstacles
		uint goal_count;	// Number of goals
		uint cov_count;		// Number of covered cells

		double tot_comp_time;	// Total computation time
		int tot_hor_len;		// Total horizon length

		vector<vector<double> > ws;
		robot_status_vec_t robots;

		#ifdef DEBUG_RVIZ
			cRViz *rviz_obj;
		#endif
};