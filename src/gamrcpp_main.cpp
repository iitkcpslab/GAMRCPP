/*
Purpose: 
Last updated: Variable names
Last updated on: 19 Mar 2022
Author: Ratijit Mitra
*/


#include "gamrcpp_pkg/gamrcpp_main.h"


using namespace ros;


void GAMRCPP_MAIN::initializeGAMRCPP(ros::NodeHandle *nh)
{
	this -> nh = nh;
	hor_id = 0;
	tot_hor_len = 0;
	tot_comp_time = 0;
	lv_count = 0;
	obs_count = 0;
	goal_count = 0;
	cov_count = 0;
	flag_mis_began = false;

	nh -> getParam("ws_x", ws_size_x);
	nh -> getParam("ws_y", ws_size_y);
	nh -> getParam("rc", rob_count);

	printf("Workspace Size = %d x %d\n", ws_size_x, ws_size_y);

	printf("#Robots = %d\n", rob_count);
	position btp;
	btp.x = btp.y = -1;
    
	for(int i = 0; i < rob_count; i++)
	{
		robot_status tmp_status;
		tmp_status.id = i;
  		tmp_status.status = 1;
  		tmp_status.x = -1;
  		tmp_status.y = -1;;
  		tmp_status.theta = 0;
		robots.push_back(tmp_status);
	}

	vector<vector<double> > ws_init(ws_size_x, vector<double>(ws_size_y, -1));			// Initialized to Unexplored Cells (-1)
	ws = ws_init;

	std::string gapPkgPath = ros::package::getPath("gamrcpp_pkg") + OUTPUT_DIR;
	ofstream rph_file;			//Empty resultPerHorizon.txt
	rph_file.open((gapPkgPath + RESULT_PER_HORIZON_FILE).c_str(), ios::out);
	rph_file.close();

	ofstream rs_file;			//Empty result.txt
	rs_file.open((gapPkgPath + RESULT_FILE).c_str(), ios::out);
	rs_file.close();

	#ifdef DEBUG_RVIZ
		rviz_obj = new cRViz();
		rviz_obj -> init_rviz(ws_size_x, ws_size_y, ws, nh);
	#endif

	localview_ss = nh->advertiseService("share_workspace", &GAMRCPP_MAIN::getLocalView, this);
	printf("Server = share_workspace started.\n==================================================\n");
}


bool GAMRCPP_MAIN::getLocalView(gamrcpp_pkg::ShareLocalInformation::Request &req, gamrcpp_pkg::ShareLocalInformation::Response &res)
{
	if(!flag_mis_began)
	{
		flag_mis_began = true;
		mis_time_begin = ros::Time::now();
	}

	int id = (int)req.robot_id;
	robots[id].x = (int)req.x;
	robots[id].y = (int)req.y;
	robots[id].theta = (double)req.theta;
	int horizon = (int)req.horizon;

	lv_count++;
	printf("\nGP Recieved Local View from Robot = %d (%d, %d, %1.0lf) at Horizon = %d. #Requests = %d.\n", id, robots[id].x, robots[id].y, robots[id].theta, horizon, lv_count);
	
	this -> updateGlobalView(req);
	//this -> printWorkspace();

	if(lv_count == rob_count)
	{
		boost::thread* thr = new boost::thread(boost::bind(&GAMRCPP_MAIN::checkTotalRequests, this));
		thr->detach();
	}

	res.next_horizon = hor_id;
	printf("GP Acknowledged Local View from Robot = %d.\n\n", id);

	return true;
}


void GAMRCPP_MAIN::printWorkspace()
{
	int i, j;
	printf("Global Workspace = %d x %d\n", ws_size_x,  ws_size_y);

	for(j = ws_size_y - 1; j >= 0; j--)
	{
		printf(" %d \t", j);

		for(i = 0; i < ws_size_x; i++)
		{
			printf("%0.2lf ", ws[i][j]);
		}

		printf("\n");
	}

	printf("\n\t\t");

	for(i = 0; i < ws_size_x; i++)
		printf(" %d ", i);

	printf("\n");
}


void GAMRCPP_MAIN::updateGlobalView(gamrcpp_pkg::ShareLocalInformation::Request &req)
{
	for(uint i = 0; i < ws_size_x; i++)
		for(uint j = 0; j < ws_size_y; j++)
		{
			uint lv_index = i * ws_size_y + j;		// Local view index
			double lv = double(req.workspace[lv_index]);		// Local view

			if(lv == -1.0)		// Unexplored
				continue;
			else if((lv == 0.0) || (lv == 0.5))		// Obstacle or Goal
			{
				if(ws[i][j] == -1.0)
					ws[i][j] = lv;
			}
			else		// Covered
				ws[i][j] = lv;
		}
}


void GAMRCPP_MAIN::checkTotalRequests()
{
	int discovered_cells = this -> getDiscoveredCells();		//Sum of number of obstacle cells and covered cells

	#ifdef DEBUG_RVIZ
		rviz_obj -> update_rviz(ws);
	#endif
	
	this -> checkCompletion(discovered_cells);
	this -> generateNextPlan();
}


int GAMRCPP_MAIN::getDiscoveredCells()
{
	int completed = 0;
	int i, j;

	for(i = 0; i < ws_size_x; i++)
	{
		for(j = 0; j < ws_size_y; j++)
		{
			if(ws[i][j] == 0)		//Obstacle
			{
				completed++;
				obs_count++;
			}

			if((ws[i][j] > 0) && (ws[i][j] <= 0.15))	//Obstacle
			{
				completed++;
				obs_count++;
			}

			if((ws[i][j] >= 0.85))		//Covered
			{
				completed++;
				cov_count++;
			}

			if(ws[i][j] == 0.5)		//0.5 = Uncovered
				goal_count++;
		}
	}
	
	return completed;
}


void GAMRCPP_MAIN::checkCompletion(int discovered_cells)
{
	uint num_of_cells = ws_size_x * ws_size_y;

	if(!goal_count)
	{
		ros::Time mis_time_end = ros::Time::now();		// Ending of the mission time
		Duration mis_time = mis_time_end - mis_time_begin;

		plan_vec_t tmpPlan;
		int horizon_length = -1;
		this->sharePlan(tmpPlan, horizon_length);

		ofstream rph_file;
		std::string rph_file_path = ros::package::getPath("gamrcpp_pkg") + OUTPUT_DIR + RESULT_PER_HORIZON_FILE;
		rph_file.open(rph_file_path.c_str(), fstream::app);	
		rph_file << "Horizon = " << hor_id << ", #O = " << obs_count << ", #G = " << goal_count << ", #C = " << cov_count << ", Computation Time = 0, Horizon Length = 0, #Active Robots = 0";
		rph_file.close();

		ofstream ofp;
		std::string output_file = ros::package::getPath("gamrcpp_pkg") + OUTPUT_DIR + RESULT_FILE;
		ofp.open(output_file.c_str(), fstream::app);
		ofp << "Workspace Size = " << ws_size_x << " x " << ws_size_y << ", #Robots = " << rob_count << ", #Horizons = " << (hor_id + 1) << ", Total Computation Time = " << tot_comp_time << ", Total Horizon Length = " << tot_hor_len << ", Mission Time = " << mis_time.toSec();
		ofp.close();

		printf("\n************************************************** Coverage Completed **************************************************\n#Horizons = %d. Total Computation Time = %lf. Total Horizon Length = %d. Mission Time = %lf.\n", (hor_id + 1), tot_comp_time, tot_hor_len, mis_time.toSec());	
		ros::shutdown();
		exit(0);
	}
}


void GAMRCPP_MAIN::sharePlan(plan_vec_t &totalPlan, int horizon_length)
{
	int i = 0, j = 0;
		
	for(i = 0; i < rob_count; i++)			//Send individual plans to each robot
	{
		gamrcpp_pkg::PlanForHorizon srv;
		srv.request.total_robots = this->rob_count;
		srv.request.horizon_length = horizon_length;
		gamrcpp_pkg::PlanInstance tmp_plan;

		for(j = 0; j < totalPlan.size(); j++)
		{
			if(i == totalPlan[j].id)
			{
				tmp_plan.robot_id = int(totalPlan[j].id);
				tmp_plan.horizon_step = int(totalPlan[j].time_instance);
				tmp_plan.x = totalPlan[j].location_x;
				tmp_plan.y = totalPlan[j].location_y;
				tmp_plan.theta = totalPlan[j].theta;

				srv.request.plans.push_back(tmp_plan);
			}
		}

		std::string plan_service_name = "/robot_";
		plan_service_name += boost::lexical_cast<std::string>(i);
		plan_service_name += plan_service_name;
		plan_service_name += "/share_plan";
		ros::ServiceClient global_plan = nh->serviceClient<gamrcpp_pkg::PlanForHorizon>(plan_service_name);
		
		boost::thread* thread1 = new boost::thread(boost::bind(&GAMRCPP_MAIN::callRobot, this, global_plan, srv, plan_service_name));			//Shares plans asynchronously - 1
		thread1->detach();
	}
}


void GAMRCPP_MAIN::callRobot(ros::ServiceClient sc, gamrcpp_pkg::PlanForHorizon pfh, std::string psn)
{
	if(sc.call(pfh))
	{
		printf("%s. Next Horizon = %d.\n", psn.c_str(), (int)pfh.response.next_horizon + 1);
	}
	else
	{
		printf("%s!\n", psn.c_str());
	}
}


void GAMRCPP_MAIN::printHorizonInformation(uint active_robs_count, int horizon_length, double comp_time)
{
	ofstream ofp;
	std::string output_file = ros::package::getPath("gamrcpp_pkg") + OUTPUT_DIR + RESULT_PER_HORIZON_FILE;
	ofp.open(output_file.c_str(), fstream::app);
	ofp << "Horizon = " << hor_id << ", #O = " << obs_count << ", #G = " << goal_count << ", #C = " << cov_count <<  ", Computation Time = " << comp_time << ", Horizon Length = " << horizon_length << ", #Active Robots = " << active_robs_count << endl;
	ofp.close();

	cout << "\nHorizon = " << hor_id << " #O = " << obs_count << " #G = " << goal_count << " #C = " << cov_count <<  " Computation Time = " << comp_time << " Horizon Length = " << horizon_length << " #Active Robots = " << active_robs_count << endl;
	cout << "==================================================" << endl;

	if(!goal_count)
	{
		printf("\n\n==========\nNo Goal\n==========\n\n");
		ros::shutdown();
		exit(0);
	}
}


void GAMRCPP_MAIN::updateHorizonInformation(int horizon_length, double comp_time)
{
	hor_id++;
	tot_hor_len += horizon_length;
	tot_comp_time += comp_time;
	lv_count = 0;
	obs_count = 0;
	goal_count = 0;
	cov_count = 0;
}


void GAMRCPP_MAIN::generateNextPlan()
{
	uint active_robs_count = 0;		//Number of Active Robots
	int hor_len = 0;		//Horizon Length
	plan_vec_t totalPlan;
	Time comp_time_begin = ros::Time::now();	

	vec_vec_bool ws_graph(ws_size_x, vector<bool>(ws_size_y, true));		//Graph representation of the workspace; true = Visited (Covered), Unvisited (Uncovered); false = Unexplored, Obstacle

	vector<struct loc> goals_locs;		//Locations of Goals (a.k.a. Uncovered Cells)
	struct loc goal_loc;

	for(int i = 0; i < ws_size_x; i++)
		for(int j = 0; j < ws_size_y; j++)
			if((ws[i][j] == -1) || (ws[i][j] == 0))		//-1 = Unexplored, 0 = Obstacle
				ws_graph[i][j] = false;
			else if(ws[i][j] == 0.5)		//0.5 = Uncovered
			{
				goal_loc.x = i;
				goal_loc.y = j;
				goals_locs.push_back(goal_loc);
			}

	vector<struct loc> robs_locs;		//Locations of Robots
	struct loc rob_init_loc;		//Initial Location of a Robot
	robot_status rob_state;

	for(int robot_id = 0; robot_id < rob_count; robot_id++)
	{
		rob_state = robots[robot_id];

		rob_init_loc.x = rob_state.x;
		rob_init_loc.y = rob_state.y;
		rob_init_loc.theta = (int)rob_state.theta;

		robs_locs.push_back(rob_init_loc);
	}

	GAMRCPP gamrcpp_obj;
	vec_vec_loc trajectories = gamrcpp_obj.runGAMRCPP(ws_size_x, ws_size_y, ws_graph, rob_count, robs_locs, goals_locs.size(), goals_locs, hor_id, false, 1);		//Paths of Robots

	vector<struct loc> rob_path;		//Path of a Robot
	struct loc rob_path_loc, rob_path_start_loc, rob_path_goal_loc;		//A Location in the Path of a Robot
		motionPlan mp;

	for(int robot_id = 0; robot_id < rob_count; robot_id++)
	{
		rob_path = trajectories[robot_id];//cout << "\nP_" << robot_id << " = " << rob_path.size() - 1;
		rob_path_start_loc = rob_path[0];
		int loc_id;

		for(loc_id = 0; loc_id < rob_path.size(); loc_id++)
		{
			rob_path_loc = rob_path[loc_id];

			mp.id = robot_id;
			mp.time_instance = loc_id;
			mp.location_x = rob_path_loc.x;
			mp.location_y = rob_path_loc.y;
			mp.theta = rob_path_loc.theta;

			totalPlan.push_back(mp);
		}

		rob_path_goal_loc = rob_path[loc_id - 1];

		if(!((rob_path_start_loc.x == rob_path_goal_loc.x) && (rob_path_start_loc.y == rob_path_goal_loc.y)))		// Robot has not been assigned any goal
			active_robs_count++;
	}

	if(rob_count)
		hor_len = trajectories[0].size() - 1;

	Time comp_time_end = ros::Time::now();
	Duration comp_time = comp_time_end - comp_time_begin;		//Computation Time
	this->sharePlan(totalPlan, hor_len);
	this->printHorizonInformation(active_robs_count, hor_len, comp_time.toSec());
	this->updateHorizonInformation(hor_len, comp_time.toSec());
}


int main(int argc, char **argv)
{	
	ros::init(argc, argv, "gamrcpp_node");
	ros::NodeHandle nh("~");

	srand(time(0));
	
	GAMRCPP_MAIN *gap_main_obj = new GAMRCPP_MAIN();
	gap_main_obj -> initializeGAMRCPP(&nh);

	ros::spin();
	return 0;
}