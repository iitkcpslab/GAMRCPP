/*
Purpose : 	gamrCPP -
					1.	compute_optimal_costs()
						1a.	bfs()
					2.	hungarian()
						2a.	hopcroft_karp_karzanov()
							2aa.	hopcroft_karp_karzanov_bfs()
							2ab.	hopcroft_karp_karzanov_dfs()
					3.	compute_optimal_paths()
						3a. optimal_path_bfs()
					4.	while(true)
						5.	get_feasible_paths()
						6.	compute_partial_orders()
						7.	compute_total_order()
						8.	if(valid TO)
								break
							else
								9. break_dependency_cycles()
								10.	adjust_dependent_paths()
					11.	compute_optimal_time_offsets()
					12.	compute_optimal_trajectories()
Last updated : Feasible paths
Last updated on : 28 Jan 2022
Author : Ratijit Mitra
*/


#include <stdio.h>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <stack>


#include <ros/ros.h>
#include <ros/package.h>


#include "gamrcpp_pkg/debug.h"
#include "gamrcpp_pkg/ma.h"


using namespace std;


class GAMRCPP
{
	public:
		vec_vec_loc runGAMRCPP(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, int num_of_robs, vector<struct loc> robs_locs, int num_of_goals, vector<struct loc> goals_locs, uint horizon, bool clus_plan, uint clus_id);

		void showQueue(queue<struct loc> BFS_QUEUE);
		void showMatrix(vec_vec_int v_2d);
		
		vec_vec_int bfs(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, struct loc cur_rob_loc);
		vec_vec_int bfs_longitudinal(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, struct loc cur_rob_loc);
		vec_vec_int compute_optimal_costs(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, vector<struct loc> robs_locs, vector<struct loc> goals_locs);
		
		vector<struct loc> optimal_path_bfs(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, struct loc rob_loc, struct loc goal_loc);
		vector<struct loc> optimal_path_bfs_longitudinal(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, struct loc rob_loc, struct loc goal_loc);
		vec_vec_loc compute_optimal_paths(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, vector<struct loc> robs_locs, vector<struct loc> goals_locs, vector<int> opt_goal_assignments, uint &active_count);

		bool is_type1_crossover_path_pair(int rob_1, int rob_2, struct loc rob1_start_loc, vec_loc path_2);
		bool is_type2_crossover_path_pair(int rob_1, int rob_2, vec_loc path_1, vec_loc path_2);
		bool is_nested_path_pair(int rob_1, int rob_2, vec_loc path_1, vec_loc path_2);
		vec_loc adjust_path(struct loc init_state, vec_loc opt_path);
		bool test_path(int i, vec_loc path, vec_int goal_vec_new, vec_vec_loc feasible_paths);
		vec_vec_loc get_feasible_paths(vector<int> &opt_goal_vec, vec_vec_loc opt_paths, uint &killed_count, uint &revived_count, uint &revived_goal_count, int ws_size_x, int ws_size_y);

		vec_vec_bool compute_partial_orders(int num_of_robs, vec_vec_loc paths);
		vector<int> compute_total_order(int num_of_robs, vec_vec_bool adj, vec_vec_bool &adj_residue);
		void adjust_dependent_paths(vector<int> opt_goal_vec, vec_vec_loc &paths, uint &active_count);
		
		vector<int> compute_start_time_offsets(vector<int> total_order, vec_vec_loc paths);
		vec_vec_loc compute_collision_averted_paths(int num_of_robs, vector<int> time_offsets, vec_vec_loc paths, uint horizon, bool clus_plan, uint clus_id);
		vec_vec_loc compute_optimal_trajectories(int num_of_robs, vector<int> time_offsets, vec_vec_loc paths, uint horizon, bool clus_plan, uint clus_id);

		void monitor_paths(int ws_size_x, int ws_size_y, int num_of_robs, vec_vec_loc trajectories);
};