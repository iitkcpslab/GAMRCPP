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


#include "gamrcpp_pkg/gamrcpp.h"


//==================================================1. Optimal Costs : START
void GAMRCPP::showQueue(queue<struct loc> BFS_QUEUE)
{
	while(!BFS_QUEUE.empty())
	{
		struct loc temp_loc = BFS_QUEUE.front(); 
        BFS_QUEUE.pop();

		cout << "(" << temp_loc.x << ", " << temp_loc.y << ", " << temp_loc.theta << ") ";
	}
}


void GAMRCPP::showMatrix(vec_vec_int v_2d)
{
	int v_2d_size = v_2d[0].size();
	int i, j;

	//for(j = v_2d_size - 1; j >= 0; j--)
	for(i = 0; i < v_2d_size; i++)
	{
		//for(i = 0; i < v_2d_size; i++)
		for(j = 0; j < v_2d_size; j++)
		{
			cout << v_2d[i][j] << " ";
		}

		cout << endl;
	}
}


vec_vec_int GAMRCPP::bfs(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, struct loc rob_state)
{
	int rob_state_x = rob_state.x;
	int rob_state_y = rob_state.y;
	int rob_state_theta = rob_state.theta;
	//cout << "(" << rob_state_x << ", " << rob_state_y << ", " << rob_state_theta <<")";

	vec_vec_int opt_cost_mat(ws_size_x, vector<int>(ws_size_y, COST_INF));
	vec_vec_int direction(ws_size_x, vector<int>(ws_size_y, -1));

	opt_cost_mat[rob_state_x][rob_state_y] = 0;
	direction[rob_state_x][rob_state_y] = rob_state_theta;

	//==================================================EnQueue : START
	queue<struct loc> BFS_QUEUE;
	BFS_QUEUE.push(rob_state);

	//printf("After enqueue...\n");showQueue(BFS_QUEUE);
	//==================================================EnQueue : END

	while(!BFS_QUEUE.empty())
	{
		//==================================================DeQueue : START
		struct loc rob_state_temp = BFS_QUEUE.front(); 
    	BFS_QUEUE.pop();

		int rob_state_x_temp = rob_state_temp.x;
		int rob_state_y_temp = rob_state_temp.y;
		int rob_state_theta_temp = rob_state_temp.theta;

		//printf("After dequeue...\n");showQueue(BFS_QUEUE);
		//==================================================DeQueue : END

		if(direction[rob_state_x_temp][rob_state_y_temp] == rob_state_theta_temp)
		{
			int nbr_x = rob_state_x_temp + 1;		// Right neighbor cell
			int nbr_y = rob_state_y_temp;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				int num_of_rot = 0;		// Number of rotations
				int theta_temp = rob_state_theta_temp;

				while(theta_temp != 0)		// 0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
				{
					switch(theta_temp)		// Rotate the Robot
					{
						case 1:	theta_temp = 0;
								break;
						case 2:	theta_temp = 1;
								break;
						case 3:	theta_temp = 0;
								break;
					}
					
					num_of_rot++;
				}

				int cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
				int nbr_cost = opt_cost_mat[nbr_x][nbr_y];

				if(nbr_cost > cost_temp + num_of_rot)
				{
					opt_cost_mat[nbr_x][nbr_y] = cost_temp + num_of_rot;
					direction[nbr_x][nbr_y] = theta_temp;

					//==================================================EnQueue : START
					rob_state_temp.x = nbr_x;
					rob_state_temp.y = nbr_y;
					rob_state_temp.theta = theta_temp;
					BFS_QUEUE.push(rob_state_temp);

					//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
					//==================================================EnQueue : END
				}
			}

			nbr_x = rob_state_x_temp;		// Top neighbor cell
			nbr_y = rob_state_y_temp + 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				int num_of_rot = 0;		// Number of rotations
				int theta_temp = rob_state_theta_temp;

				while(theta_temp != 1)		// 0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
				{
					switch(theta_temp)		// Rotate the Robot
					{
						case 0:	theta_temp = 1;
								break;
						case 2:	theta_temp = 1;
								break;
						case 3:	theta_temp = 2;
								break;
					}
					
					num_of_rot++;
				}

				int cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
				int nbr_cost = opt_cost_mat[nbr_x][nbr_y];

				if(nbr_cost > cost_temp + num_of_rot)
				{
					opt_cost_mat[nbr_x][nbr_y] = cost_temp + num_of_rot;
					direction[nbr_x][nbr_y] = theta_temp;

					//==================================================EnQueue : START
					rob_state_temp.x = nbr_x;
					rob_state_temp.y = nbr_y;
					rob_state_temp.theta = theta_temp;
					BFS_QUEUE.push(rob_state_temp);

					//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
					//==================================================EnQueue : END
				}
			}

			nbr_x = rob_state_x_temp - 1;		// Left neighbor cell
			nbr_y = rob_state_y_temp;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				int num_of_rot = 0;		// Number of rotations
				int theta_temp = rob_state_theta_temp;

				while(theta_temp != 2)		// 0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
				{
					switch(theta_temp)		// Rotate the Robot
					{
						case 0:	theta_temp = 3;
								break;
						case 1:	theta_temp = 2;
								break;
						case 3:	theta_temp = 2;
								break;
					}
					
					num_of_rot++;
				}

				int cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
				int nbr_cost = opt_cost_mat[nbr_x][nbr_y];

				if(nbr_cost > cost_temp + num_of_rot)
				{
					opt_cost_mat[nbr_x][nbr_y] = cost_temp + num_of_rot;
					direction[nbr_x][nbr_y] = theta_temp;

					//==================================================EnQueue : START
					rob_state_temp.x = nbr_x;
					rob_state_temp.y = nbr_y;
					rob_state_temp.theta = theta_temp;
					BFS_QUEUE.push(rob_state_temp);

					//printf("After enqueue...\n");showQueue(BFS_QUEUE);
					//==================================================EnQueue : END
				}
			}

			nbr_x = rob_state_x_temp;		// Bottom neighbor cell
			nbr_y = rob_state_y_temp - 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				int num_of_rot = 0;		// Number of rotations
				int theta_temp = rob_state_theta_temp;

				while(theta_temp != 3)		// 0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
				{
					switch(theta_temp)		// Rotate the Robot
					{
						case 0:	theta_temp = 3;
								break;
						case 1:	theta_temp = 0;
								break;
						case 2:	theta_temp = 3;
								break;
					}
					
					num_of_rot++;
				}

				int cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
				int nbr_cost = opt_cost_mat[nbr_x][nbr_y];

				if(nbr_cost > cost_temp + num_of_rot)
				{
					opt_cost_mat[nbr_x][nbr_y] = cost_temp + num_of_rot;
					direction[nbr_x][nbr_y] = theta_temp;

					//==================================================EnQueue : START
					rob_state_temp.x = nbr_x;
					rob_state_temp.y = nbr_y;
					rob_state_temp.theta = theta_temp;
					BFS_QUEUE.push(rob_state_temp);

					//printf("After enqueue...\n");showQueue(BFS_QUEUE);
					//==================================================EnQueue : END
				}
			}
		}
	}

	//printf("Cost Matrix...\n");showMatrix(opt_cost_mat);printf("\n");

	return opt_cost_mat;
}


vec_vec_int GAMRCPP::bfs_longitudinal(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, struct loc rob_state)
{
	int rob_state_x = rob_state.x;
	int rob_state_y = rob_state.y;
	//cout << "(" << rob_state_x << ", " << rob_state_y <<")";

	vec_vec_int opt_cost_mat(ws_size_x, vector<int>(ws_size_y, COST_INF));
	opt_cost_mat[rob_state_x][rob_state_y] = 0;

	//==================================================EnQueue : START
	queue<struct loc> BFS_QUEUE;
	BFS_QUEUE.push(rob_state);

	//printf("After enqueue...\n");showQueue(BFS_QUEUE);
	//==================================================EnQueue : END

	while(!BFS_QUEUE.empty())
	{
		//==================================================DeQueue : START
		struct loc rob_state_temp = BFS_QUEUE.front(); 
    	BFS_QUEUE.pop();

		int rob_state_x_temp = rob_state_temp.x;
		int rob_state_y_temp = rob_state_temp.y;

		//printf("After dequeue...\n");showQueue(BFS_QUEUE);
		//==================================================DeQueue : END

		int nbr_x = rob_state_x_temp + 1;		// Right neighbor cell
		int nbr_y = rob_state_y_temp;

		if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
		{
			int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
			int nbr_opt_cost = opt_cost_mat[nbr_x][nbr_y];

			if(nbr_opt_cost > opt_cost_temp)
			{
				opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp;

				//==================================================EnQueue : START
				rob_state_temp.x = nbr_x;
				rob_state_temp.y = nbr_y;
				BFS_QUEUE.push(rob_state_temp);

				//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
				//==================================================EnQueue : END
			}
		}

		nbr_x = rob_state_x_temp;		// Top neighbor cell
		nbr_y = rob_state_y_temp + 1;

		if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
		{
			int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
			int nbr_opt_cost = opt_cost_mat[nbr_x][nbr_y];

			if(nbr_opt_cost > opt_cost_temp)
			{
				opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp;

				//==================================================EnQueue : START
				rob_state_temp.x = nbr_x;
				rob_state_temp.y = nbr_y;
				BFS_QUEUE.push(rob_state_temp);

				//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
				//==================================================EnQueue : END
			}
		}

		nbr_x = rob_state_x_temp - 1;		// Left neighbor cell
		nbr_y = rob_state_y_temp;

		if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
		{
			int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
			int nbr_opt_cost = opt_cost_mat[nbr_x][nbr_y];

			if(nbr_opt_cost > opt_cost_temp)
			{
				opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp;

				//==================================================EnQueue : START
				rob_state_temp.x = nbr_x;
				rob_state_temp.y = nbr_y;
				BFS_QUEUE.push(rob_state_temp);

				//printf("After enqueue...\n");showQueue(BFS_QUEUE);
				//==================================================EnQueue : END
			}
		}

		nbr_x = rob_state_x_temp;		// Bottom neighbor cell
		nbr_y = rob_state_y_temp - 1;

		if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
		{
			int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
			int nbr_opt_cost = opt_cost_mat[nbr_x][nbr_y];

			if(nbr_opt_cost > opt_cost_temp)
			{
				opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp;

				//==================================================EnQueue : START
				rob_state_temp.x = nbr_x;
				rob_state_temp.y = nbr_y;
				BFS_QUEUE.push(rob_state_temp);

				//printf("After enqueue...\n");showQueue(BFS_QUEUE);
				//==================================================EnQueue : END
			}
		}
	}

	//printf("Cost Matrix...\n");showMatrix(opt_cost_mat);printf("\n");

	return opt_cost_mat;
}


vec_vec_int GAMRCPP::compute_optimal_costs(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, vector<struct loc> robs_states, vector<struct loc> goals_locs)
{
	int num_of_robs = robs_states.size();
	int num_of_goals = goals_locs.size();
	vec_vec_int opt_cost_mat(num_of_robs, vector<int>(num_of_goals, -1));		// Optimal Cost Matrix

	for(int rob_id = 0; rob_id < num_of_robs; rob_id++)
	{
		struct loc rob_state = robs_states[rob_id];
		//cout << "\nRobot_" << rob_id << " ";

		#ifdef TURTLEBOT
			vec_vec_int opt_cost_mat_temp = bfs(ws_size_x, ws_size_y, ws_graph, rob_state);
		#else
			vec_vec_int opt_cost_mat_temp = bfs_longitudinal(ws_size_x, ws_size_y, ws_graph, rob_state);
		#endif

		for(int goal_id = 0; goal_id < num_of_goals; goal_id++)
		{
			struct loc goal_loc = goals_locs[goal_id];
			opt_cost_mat[rob_id][goal_id] = opt_cost_mat_temp[goal_loc.x][goal_loc.y];
		}

		//cout << "\nOptimal Cost Matrix for R_" << rob_id << "...\n";showMatrix(opt_cost_mat_temp);
	}

	return opt_cost_mat;
}
//==================================================1. Optimal Costs : END


//==================================================3. Optimal Paths : START
vector<struct loc> GAMRCPP::optimal_path_bfs(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, struct loc rob_state, struct loc goal_loc)
{
	int rob_state_x = rob_state.x;
	int rob_state_y = rob_state.y;
	int rob_state_theta = rob_state.theta;
	//cout << "Robot (" << rob_state_x << ", " << rob_state_y << ")\t\t";

	int goal_x = goal_loc.x;
	int goal_y = goal_loc.y;
	//cout << "Goal (" << goal_x << ", " << goal_y << ")" << endl;

	vec_vec_int opt_cost_mat(ws_size_x, vector<int>(ws_size_y, COST_INF));
	vec_vec_int predecessor_x(ws_size_x, vector<int>(ws_size_y, -1));		// Predecessor cell of a cell
	vec_vec_int predecessor_y(ws_size_x, vector<int>(ws_size_y, -1));
	vec_vec_int direction(ws_size_x, vector<int>(ws_size_y, -1));

	opt_cost_mat[rob_state_x][rob_state_y] = 0;
	direction[rob_state_x][rob_state_y] = rob_state_theta;

	//==================================================EnQueue : START
	queue<struct loc> BFS_QUEUE;
	BFS_QUEUE.push(rob_state);

	//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
	//==================================================EnQueue : END

	while(!BFS_QUEUE.empty())
	{
		//==================================================DeQueue : START
		struct loc rob_state_temp = BFS_QUEUE.front(); 
    	BFS_QUEUE.pop();

		int rob_state_x_temp = rob_state_temp.x;
		int rob_state_y_temp = rob_state_temp.y;
		int rob_state_theta_temp = rob_state_temp.theta;

		//printf("\nAfter dequeue...\n");showQueue(BFS_QUEUE);
		//==================================================DeQueue : END

		if(direction[rob_state_x_temp][rob_state_y_temp] == rob_state_theta_temp)
		{
			int nbr_x = rob_state_x_temp + 1;		// Right neighbor cell
			int nbr_y = rob_state_y_temp;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				int num_of_rot = 0;		// Number of rotations
				int temp_theta = rob_state_theta_temp;

				while(temp_theta != 0)		// 0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
				{
					switch(temp_theta)		// Rotate the Robot
					{
						case 1:	temp_theta = 0;
								break;
						case 2:	temp_theta = 1;
								break;
						case 3:	temp_theta = 0;
								break;
					}
					
					num_of_rot++;
				}

				int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
				int nbr_cost = opt_cost_mat[nbr_x][nbr_y];

				if(nbr_cost > opt_cost_temp + num_of_rot)
				{
					opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp + num_of_rot;
					predecessor_x[nbr_x][nbr_y] = rob_state_x_temp;
					predecessor_y[nbr_x][nbr_y] = rob_state_y_temp;
					direction[nbr_x][nbr_y] = temp_theta;

					//==================================================EnQueue : START
					rob_state_temp.x = nbr_x;
					rob_state_temp.y = nbr_y;
					rob_state_temp.theta = temp_theta;
					BFS_QUEUE.push(rob_state_temp);

					//printf("\nAfter enqueue1...\n");showQueue(BFS_QUEUE);
					//==================================================EnQueue : END
				}
			}

			nbr_x = rob_state_x_temp;		// Top neighbor cell
			nbr_y = rob_state_y_temp + 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				int num_of_rot = 0;		// Number of rotations
				int temp_theta = rob_state_theta_temp;

				while(temp_theta != 1)		// 0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
				{
					switch(temp_theta)		// Rotate the Robot
					{
						case 0:	temp_theta = 1;
								break;
						case 2:	temp_theta = 1;
								break;
						case 3:	temp_theta = 2;
								break;
					}
					
					num_of_rot++;
				}

				int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
				int nbr_cost = opt_cost_mat[nbr_x][nbr_y];

				if(nbr_cost > opt_cost_temp + num_of_rot)
				{
					opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp + num_of_rot;
					predecessor_x[nbr_x][nbr_y] = rob_state_x_temp;
					predecessor_y[nbr_x][nbr_y] = rob_state_y_temp;
					direction[nbr_x][nbr_y] = temp_theta;

					//==================================================EnQueue : START
					rob_state_temp.x = nbr_x;
					rob_state_temp.y = nbr_y;
					rob_state_temp.theta = temp_theta;
					BFS_QUEUE.push(rob_state_temp);

					//printf("\nAfter enqueue2...\n");showQueue(BFS_QUEUE);
					//==================================================EnQueue : END
				}
			}

			nbr_x = rob_state_x_temp - 1;		// Left neighbor cell
			nbr_y = rob_state_y_temp;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				int num_of_rot = 0;		// Number of rotations
				int temp_theta = rob_state_theta_temp;

				while(temp_theta != 2)		// 0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
				{
					switch(temp_theta)		// Rotate the Robot
					{
						case 0:	temp_theta = 3;
								break;
						case 1:	temp_theta = 2;
								break;
						case 3:	temp_theta = 2;
								break;
					}
					
					num_of_rot++;
				}

				int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
				int nbr_cost = opt_cost_mat[nbr_x][nbr_y];

				if(nbr_cost > opt_cost_temp + num_of_rot)
				{
					opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp + num_of_rot;
					predecessor_x[nbr_x][nbr_y] = rob_state_x_temp;
					predecessor_y[nbr_x][nbr_y] = rob_state_y_temp;
					direction[nbr_x][nbr_y] = temp_theta;

					//==================================================EnQueue : START
					rob_state_temp.x = nbr_x;
					rob_state_temp.y = nbr_y;
					rob_state_temp.theta = temp_theta;
					BFS_QUEUE.push(rob_state_temp);

					//printf("\nAfter enqueue3...\n");showQueue(BFS_QUEUE);
					//==================================================EnQueue : END
				}
			}

			nbr_x = rob_state_x_temp;		//Bottom neighbor
			nbr_y = rob_state_y_temp - 1;

			if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
			{
				int num_of_rot = 0;		// Number of rotations
				int temp_theta = rob_state_theta_temp;

				while(temp_theta != 3)		// 0 : +ve x-axis, 1 : +ve y-axis, 2 : -ve x-axis, 3 : -ve y-axis
				{
					switch(temp_theta)		// Rotate the Robot
					{
						case 0:	temp_theta = 3;
								break;
						case 1:	temp_theta = 0;
								break;
						case 2:	temp_theta = 3;
								break;
					}
					
					num_of_rot++;
				}

				int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
				int nbr_cost = opt_cost_mat[nbr_x][nbr_y];

				if(nbr_cost > opt_cost_temp + num_of_rot)
				{
					opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp + num_of_rot;
					predecessor_x[nbr_x][nbr_y] = rob_state_x_temp;
					predecessor_y[nbr_x][nbr_y] = rob_state_y_temp;
					direction[nbr_x][nbr_y] = temp_theta;

					//==================================================EnQueue : START
					rob_state_temp.x = nbr_x;
					rob_state_temp.y = nbr_y;
					rob_state_temp.theta = temp_theta;
					BFS_QUEUE.push(rob_state_temp);

					//printf("\nAfter enqueue4...\n");showQueue(BFS_QUEUE);
					//==================================================EnQueue : END
				}
			}
		}
	}

	//printf("Cost Matrix...\n");showMatrix(opt_cost_mat);printf("\n");
	//printf("Predecessor_X Matrix...\n");showMatrix(predecessor_x);printf("\n");
	//printf("Predecessor_Y Matrix...\n");showMatrix(predecessor_y);printf("\n");
	//printf("Direction Matrix...\n");showMatrix(direction);printf("\n");
	
	//==================================================Backtract : START
	vector<struct loc> opt_path;
	struct loc rob_state_temp;
	rob_state_temp.x = goal_x;
	rob_state_temp.y = goal_y;
	rob_state_temp.theta = direction[goal_x][goal_y];
	opt_path.push_back(rob_state_temp);

	int last_theta = rob_state_temp.theta;
	int pred_cell_x = predecessor_x[goal_x][goal_y];
	int pred_cell_y = predecessor_y[goal_x][goal_y];

	while(1)
	{
		rob_state_temp.x = pred_cell_x;
		rob_state_temp.y = pred_cell_y;
		rob_state_temp.theta = direction[pred_cell_x][pred_cell_y];

		if(last_theta != rob_state_temp.theta)		// Insert a Rotation
		{
			struct loc rot_loc;
			rot_loc.x = pred_cell_x;
			rot_loc.y = pred_cell_y;
			rot_loc.theta = last_theta;		//Single Rotation
			opt_path.push_back(rot_loc);

			if(abs(last_theta - rob_state_temp.theta) == 2)		//Double Rotation
			{
				rot_loc.theta = (last_theta + 1) % 4;
				opt_path.push_back(rot_loc);
			}
		}

		opt_path.push_back(rob_state_temp);
		last_theta = rob_state_temp.theta;
		
		if((pred_cell_x == rob_state_x) && (pred_cell_y == rob_state_y))
			break;
		else
		{
			int pred_cell_x_bkp = pred_cell_x;
			pred_cell_x = predecessor_x[pred_cell_x][pred_cell_y];
			pred_cell_y = predecessor_y[pred_cell_x_bkp][pred_cell_y];
		}
	}
	
	reverse(opt_path.begin(), opt_path.end());
	//==================================================Backtract : END

	return opt_path;
}


vector<struct loc> GAMRCPP::optimal_path_bfs_longitudinal(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, struct loc rob_state, struct loc goal_loc)
{
	int rob_state_x = rob_state.x;
	int rob_state_y = rob_state.y;
	//cout << "Robot (" << rob_state_x << ", " << rob_state_y << ")\t\t";

	int goal_loc_x = goal_loc.x;
	int goal_loc_y = goal_loc.y;
	//cout << "Goal (" << goal_loc_x << ", " << goal_loc_y << ")" << endl;

	vec_vec_int opt_cost_mat(ws_size_x, vector<int>(ws_size_y, COST_INF));
	vec_vec_int predecessor_x(ws_size_x, vector<int>(ws_size_y, -1));		// Predecessor cell of a cell
	vec_vec_int predecessor_y(ws_size_x, vector<int>(ws_size_y, -1));

	opt_cost_mat[rob_state_x][rob_state_y] = 0;

	//==================================================EnQueue : START
	queue<struct loc> BFS_QUEUE;
	BFS_QUEUE.push(rob_state);

	//printf("\nAfter enqueue...\n");showQueue(BFS_QUEUE);
	//==================================================EnQueue : END

	while(!BFS_QUEUE.empty())
	{
		//==================================================DeQueue : START
		struct loc rob_state_temp = BFS_QUEUE.front(); 
    	BFS_QUEUE.pop();

		int rob_state_x_temp = rob_state_temp.x;
		int rob_state_y_temp = rob_state_temp.y;

		//printf("\nAfter dequeue...\n");showQueue(BFS_QUEUE);
		//==================================================DeQueue : END

		int nbr_x = rob_state_x_temp + 1;		// Right neighbor cell
		int nbr_y = rob_state_y_temp;

		if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
		{
			int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
			int nbr_opt_cost = opt_cost_mat[nbr_x][nbr_y];

			if(nbr_opt_cost > opt_cost_temp)
			{
				opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp;
				predecessor_x[nbr_x][nbr_y] = rob_state_x_temp;
				predecessor_y[nbr_x][nbr_y] = rob_state_y_temp;

				//==================================================EnQueue : START
				rob_state_temp.x = nbr_x;
				rob_state_temp.y = nbr_y;
				BFS_QUEUE.push(rob_state_temp);

				//printf("\nAfter enqueue1...\n");showQueue(BFS_QUEUE);
				//==================================================EnQueue : END
			}
		}

		nbr_x = rob_state_x_temp;		// Top neighbor cell
		nbr_y = rob_state_y_temp + 1;

		if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
		{
			int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
			int nbr_opt_cost = opt_cost_mat[nbr_x][nbr_y];

			if(nbr_opt_cost > opt_cost_temp)
			{
				opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp;
				predecessor_x[nbr_x][nbr_y] = rob_state_x_temp;
				predecessor_y[nbr_x][nbr_y] = rob_state_y_temp;

				//==================================================EnQueue : START
				rob_state_temp.x = nbr_x;
				rob_state_temp.y = nbr_y;
				BFS_QUEUE.push(rob_state_temp);

				//printf("\nAfter enqueue2...\n");showQueue(BFS_QUEUE);
				//==================================================EnQueue : END
			}
		}

		nbr_x = rob_state_x_temp - 1;		// Left neighbor cell
		nbr_y = rob_state_y_temp;

		if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
		{
			int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
			int nbr_opt_cost = opt_cost_mat[nbr_x][nbr_y];

			if(nbr_opt_cost > opt_cost_temp)
			{
				opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp;
				predecessor_x[nbr_x][nbr_y] = rob_state_x_temp;
				predecessor_y[nbr_x][nbr_y] = rob_state_y_temp;

				//==================================================EnQueue : START
				rob_state_temp.x = nbr_x;
				rob_state_temp.y = nbr_y;
				BFS_QUEUE.push(rob_state_temp);

				//printf("\nAfter enqueue3...\n");showQueue(BFS_QUEUE);
				//==================================================EnQueue : END
			}
		}

		nbr_x = rob_state_x_temp;		// Bottom neighbor cell
		nbr_y = rob_state_y_temp - 1;

		if((0 <= nbr_x) && (nbr_x < ws_size_x) && (0 <= nbr_y) && (nbr_y < ws_size_y) && ws_graph[nbr_x][nbr_y])
		{
			int opt_cost_temp = opt_cost_mat[rob_state_x_temp][rob_state_y_temp] + 1;
			int nbr_opt_cost = opt_cost_mat[nbr_x][nbr_y];

			if(nbr_opt_cost > opt_cost_temp)
			{
				opt_cost_mat[nbr_x][nbr_y] = opt_cost_temp;
				predecessor_x[nbr_x][nbr_y] = rob_state_x_temp;
				predecessor_y[nbr_x][nbr_y] = rob_state_y_temp;

				//==================================================EnQueue : START
				rob_state_temp.x = nbr_x;
				rob_state_temp.y = nbr_y;
				BFS_QUEUE.push(rob_state_temp);

				//printf("\nAfter enqueue4...\n");showQueue(BFS_QUEUE);
				//==================================================EnQueue : END
			}
		}
	}

	//printf("Cost Matrix...\n");showMatrix(opt_cost_mat);printf("\n");
	//printf("Predecessor_X Matrix...\n");showMatrix(predecessor_x);printf("\n");
	//printf("Predecessor_Y Matrix...\n");showMatrix(predecessor_y);printf("\n");
	
	//==================================================Backtract : START
	vector<struct loc> opt_path;
	struct loc rob_state_temp;
	rob_state_temp.x = goal_loc_x;
	rob_state_temp.y = goal_loc_y;
	opt_path.push_back(rob_state_temp);

	int pred_cell_x = predecessor_x[goal_loc_x][goal_loc_y];
	int pred_cell_y = predecessor_y[goal_loc_x][goal_loc_y];

	while(1)
	{
		rob_state_temp.x = pred_cell_x;
		rob_state_temp.y = pred_cell_y;

		opt_path.push_back(rob_state_temp);
		
		if((pred_cell_x == rob_state_x) && (pred_cell_y == rob_state_y))
			break;
		else
		{
			int pred_cell_x_bkp = pred_cell_x;
			pred_cell_x = predecessor_x[pred_cell_x][pred_cell_y];
			pred_cell_y = predecessor_y[pred_cell_x_bkp][pred_cell_y];
		}
	}
	
	reverse(opt_path.begin(), opt_path.end());
	//==================================================Backtract : END

	return opt_path;
}


vec_vec_loc GAMRCPP::compute_optimal_paths(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, vector<struct loc> robs_states, vector<struct loc> goals_locs, vector<int> opt_goal_vec, uint &active_count)
{
	int num_of_robs = robs_states.size();
	vec_vec_loc opt_paths;		// Optimal Paths

	for(int rob_id = 0; rob_id < num_of_robs; rob_id++)
	{
		struct loc rob_state = robs_states[rob_id];
		int opt_goal_id = opt_goal_vec[rob_id];
		vector<struct loc> opt_path;

		if(opt_goal_id == -1)		// Inactive Robot
			opt_path.push_back(rob_state);
		else
		{
			struct loc goal_loc = goals_locs[opt_goal_id];

			#ifdef TURTLEBOT
				opt_path = optimal_path_bfs(ws_size_x, ws_size_y, ws_graph, rob_state, goal_loc);
			#else
				opt_path = optimal_path_bfs_longitudinal(ws_size_x, ws_size_y, ws_graph, rob_state, goal_loc);
			#endif

			active_count++;
		}

		opt_paths.push_back(opt_path);
	}
	
	return opt_paths;
}


bool GAMRCPP::is_type1_crossover_path_pair(int rob_1, int rob_2, struct loc rob1_start_loc, vec_loc path_2)
{
	int path2_len = path_2.size() - 1;		// Length of path_2

	if(path2_len > 1)
	{
		for(int i = 1; i < path2_len; i++)
		{
			struct loc path2_loc = path_2[i];

			if((path2_loc.x == rob1_start_loc.x) && (path2_loc.y == rob1_start_loc.y))
			{
				cout << "\nR_" << rob_1 << " ! R_" << rob_2;
				return true;
			}
		}

		return false;
	}
	else
		return false;
}


bool GAMRCPP::is_type2_crossover_path_pair(int rob_1, int rob_2, vec_loc path_1, vec_loc path_2)
{
	int path1_len = path_1.size() - 1;		// Length of path_1
	int path2_len = path_2.size() - 1;		// Length of path_2

	if((path1_len > 1) && (path2_len > 1))
	{
		struct loc path1_start_loc = path_1[0];
		struct loc path2_start_loc = path_2[0];
		bool flag_start1_found, flag_start2_found;
		flag_start1_found = flag_start2_found = false;

		for(int i = 1; i < path1_len; i++)
		{
			struct loc path1_loc = path_1[i];

			if((path1_loc.x == path2_start_loc.x) && (path1_loc.y == path2_start_loc.y))
			{
				flag_start2_found = true;
				break;
			}
		}

		for(int i = 1; i < path2_len; i++)
		{
			struct loc path2_loc = path_2[i];

			if((path2_loc.x == path1_start_loc.x) && (path2_loc.y == path1_start_loc.y))
			{
				flag_start1_found = true;
				break;
			}
		}

		if(flag_start1_found && flag_start2_found)
		{
			cout << "\nP_" << rob_1 << " x P_" << rob_2;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}


bool GAMRCPP::is_nested_path_pair(int rob_1, int rob_2, vec_loc path_1, vec_loc path_2)
{
	int path_1_len = path_1.size() - 1;		// Length of path_1
	int path_2_len = path_2.size() - 1;		// Length of path_2
	struct loc path_1_start_loc = path_1[0];
	struct loc path_2_start_loc = path_2[0];
	struct loc path_1_goal_loc = path_1[path_1_len];
	struct loc path_2_goal_loc = path_2[path_2_len];
	bool flag_start1_found, flag_start2_found, flag_goal1_found, flag_goal2_found, r1_in_r2, r2_in_r1;
	flag_start1_found = flag_start2_found = flag_goal1_found = flag_goal2_found = r1_in_r2 = r2_in_r1 = false;

	for(int i = 1; i < path_2_len; i++)		// Is R_1 nested in R_2?
	{
		struct loc path_2_loc = path_2[i];

		if(!flag_start1_found && (path_1_start_loc.x == path_2_loc.x) && (path_1_start_loc.y == path_2_loc.y))
			flag_start1_found = true;
		else if(!flag_goal1_found && (path_1_goal_loc.x == path_2_loc.x) && (path_1_goal_loc.y == path_2_loc.y))
			flag_goal1_found = true;

		r1_in_r2 = flag_start1_found && flag_goal1_found;

		if(r1_in_r2)
		{
			cout << "\nR_" << rob_2 << " > R_" << rob_1;
			break;
		}
	}

	for(int i = 1; i < path_1_len; i++)		// Is R_2 nested in R_1?
	{
		struct loc path_1_loc = path_1[i];

		if(!flag_start2_found && (path_2_start_loc.x == path_1_loc.x) && (path_2_start_loc.y == path_1_loc.y))
			flag_start2_found = true;
		else if(!flag_goal2_found && (path_2_goal_loc.x == path_1_loc.x) && (path_2_goal_loc.y == path_1_loc.y))
			flag_goal2_found = true;

		r2_in_r1 = flag_start2_found && flag_goal2_found;

		if(r2_in_r1)
		{
			cout << "\nR_" << rob_1 << " > R_" << rob_2;
			break;
		}
	}

	if(r1_in_r2 || r2_in_r1)
		return true;
	else
		return false;
}


vec_loc GAMRCPP::adjust_path(struct loc init_state, vec_loc opt_path)
{
	vec_loc path;
	path.push_back(init_state);
	uint path_len = opt_path.size() - 1;
	struct loc opt_path_loc;
	uint k = path_len - 1;

	do
	{
		opt_path_loc = opt_path[k];

		if((init_state.x == opt_path_loc.x) && (init_state.y == opt_path_loc.y))
			break;

		k--;
	}
	while(k > 0);

	opt_path_loc = opt_path[++k];

	if(init_state.theta != opt_path_loc.theta)			// Needs Rotations
	{
		uint tmp_theta = (init_state.theta + 1) % 4;
		uint lr_count = 1;			// Left rotation counter

		while(tmp_theta != opt_path_loc.theta)
		{
			tmp_theta = (tmp_theta + 1) % 4;
			lr_count++;
		}

		uint rr_count = 4 - lr_count;			// Right rotation counter
		uint r_count = 1;		// Rotation counter

		if(lr_count <= rr_count)		// Rotate left
		{
			tmp_theta = init_state.theta;

			while(r_count <= lr_count)
			{
				tmp_theta = (tmp_theta + 1) % 4;
				init_state.theta = tmp_theta;
				path.push_back(init_state);
				r_count++;
			}
		}
		else
		{
			tmp_theta = init_state.theta;

			while(r_count <= rr_count)
			{
				tmp_theta = (tmp_theta - 1 + 4) % 4;
				init_state.theta = tmp_theta;
				path.push_back(init_state);
				r_count++;
			}
		}
	}

	for(uint l = k; l <= path_len; l++)
		path.push_back(opt_path[l]);

	return path;
}


bool GAMRCPP::test_path(int i, vec_loc path, vec_int goal_vec_new, vec_vec_loc feasible_paths)
{
	int num_of_robs = goal_vec_new.size();

	for(int j = 0; j < num_of_robs; j++)
		if((i != j) && (goal_vec_new[j] != -1) && (is_type2_crossover_path_pair(i, j, path, feasible_paths[j]) || is_nested_path_pair(i, j, path, feasible_paths[j])))
			return false;

	return true;
}


vec_vec_loc GAMRCPP::get_feasible_paths(vector<int> &opt_goal_vec, vec_vec_loc opt_paths, uint &killed_count, uint &revived_count, uint &revived_goal_count, int ws_size_x, int ws_size_y)
{
	//================================================== Detection : START
	int num_of_robs = opt_goal_vec.size();		// Number of robots
	vec_bool is_killed_rob(num_of_robs, false);		// Is R_i killed?
	vec_int killed_robs;		// The set of killed robots
	// cout << "\nKilled";

	for(int i = 0; i < num_of_robs - 1; i++)		// Type-2 crossover path and Nested path
		for(int j = i + 1; j < num_of_robs; j++)
			if((i != j) && (opt_goal_vec[i] != -1) && (opt_goal_vec[j] != -1) && (!is_killed_rob[i] || !is_killed_rob[j]) && (is_type2_crossover_path_pair(i, j, opt_paths[i], opt_paths[j]) || is_nested_path_pair(i, j, opt_paths[i], opt_paths[j])))
			{
				if(!is_killed_rob[i])
				{
					is_killed_rob[i] = true;
					killed_robs.push_back(i);
					killed_count++;
					// cout << " R_" << i;
				}

				if(!is_killed_rob[j])
				{
					is_killed_rob[j] = true;
					killed_robs.push_back(j);
					killed_count++;
					// cout << " R_" << j;
				}
			}

	for(int i = 0; i < num_of_robs; i++)		// Adds inactive robots
		if(opt_goal_vec[i] == -1)
			killed_robs.push_back(i);
	
	while(!killed_robs.empty())
	{
		int i = killed_robs.back();
		killed_robs.pop_back();

		for(int j = 0; j < num_of_robs; j++)
			if((i != j) && (opt_goal_vec[j] != -1) && !is_killed_rob[j] && is_type1_crossover_path_pair(i, j, opt_paths[i][0], opt_paths[j]))
				{
					is_killed_rob[j] = true;
					killed_robs.push_back(j);
					killed_count++;
					// cout << " R_" << j;
				}
	}

	// cout << "\n\nKilled...";

	// for(int i = 0; i < num_of_robs; i++)
	// 	if(is_killed_rob[i])
	// 		cout << " R_" << i;
	//================================================== Detection : END
	//================================================== Correction : START
	vec_vec_int W_r(ws_size_x, vec_int(ws_size_y, -1));		// Workspace with Inactive/Killed Robot IDs
	vec_vec_bool W_killed_goals(ws_size_x, vec_bool(ws_size_y, 0));		// Goals of killed robots
	vec_int goal_vec_new(num_of_robs, -1);		// Goals of Inactive/Killed Robots
	vec_vec_loc feasible_paths;		// The set of feasible paths

	for(int i = 0; i < num_of_robs; i++)		// Initialization
	{
		vec_loc path;

		if((opt_goal_vec[i] != -1) && !is_killed_rob[i])
			path = opt_paths[i];
		else
		{
			struct loc start_state = opt_paths[i][0];
			W_r[(int)start_state.x][(int)start_state.y] = i;
			path.push_back(start_state);

			if(is_killed_rob[i])
			{
				int path_len = opt_paths[i].size() - 1;
				struct loc goal_state = opt_paths[i][path_len];
				W_killed_goals[(int)goal_state.x][(int)goal_state.y] = 1;		// Needs to be visited
			}
		}

		feasible_paths.push_back(path);
	}

	for(int i = 0; i < num_of_robs; i++)
		if(is_killed_rob[i])
		{
			// cout << "\nTry reassigning G_" << opt_goal_vec[i] << " (of R_" << i;
			vec_loc path_i = opt_paths[i];
			int path_i_len = path_i.size() - 1;

			for(int loc_id = path_i_len - 1; loc_id >= 0; loc_id--)
			{
				struct loc path_i_loc = path_i[loc_id];
				int nearest_rob = W_r[(int)path_i_loc.x][(int)path_i_loc.y];		// The nearest robot to the goal opt_goal_vec[i]

				if(nearest_rob != -1)
				{
					// cout << ") to R_" << nearest_rob;
					vec_loc path;

					if(nearest_rob == i)
						path = path_i;
					else
						path = adjust_path(opt_paths[nearest_rob][0], path_i);

					if(test_path(nearest_rob, path, goal_vec_new, feasible_paths))
					{
						W_r[(int)path_i_loc.x][(int)path_i_loc.y] = -1;		// Revived
						goal_vec_new[nearest_rob] = opt_goal_vec[i];
						
						int path_len = path.size() - 1;

						for(int loc_id = 1; loc_id <= path_len; loc_id++)
						{
							struct loc path_state = path[loc_id];
							feasible_paths[nearest_rob].push_back(path_state);
							W_killed_goals[(int)path_state.x][(int)path_state.y] = 0;		// To be visited
						}

						revived_count++;
						// cout << ": OK";
					}

					break;
				}
			}
		}
	//================================================== Correction : END
	// cout << "\n\nRevived...";

	for(int i = 0; i < num_of_robs; i++)
		if((opt_goal_vec[i] == -1) || is_killed_rob[i])
		{
			int goal_id_new = goal_vec_new[i];
			opt_goal_vec[i] = goal_id_new;

			// if(goal_id_new != -1)
				// cout << " R_" << i;
		}

	revived_goal_count = killed_count;

	for(uint i = 0; i < ws_size_x; i++)
		for(uint j = 0; j < ws_size_y; j++)
			if(W_killed_goals[i][j])
				revived_goal_count--;

	return feasible_paths;
}
//==================================================3. Optimal Paths : END


//==================================================4. Partial Order : START
vec_vec_bool GAMRCPP::compute_partial_orders(int num_of_robs, vec_vec_loc paths)
{
	vec_vec_bool partial_order(num_of_robs, vector<bool>(num_of_robs, false));		// Initialized to false

	for(int i = 0; i < num_of_robs; i++)
	{
		struct loc start_i = paths[i][0];
		int path_len_i = paths[i].size();
		struct loc goal_i = paths[i][path_len_i - 1];

		for(int j = 0; j < num_of_robs; j++)
			if(i != j)
			{
				int path_len_j = paths[j].size();

				if(path_len_j > 1)		//Robot j has been assigned a goal
					for(int k = 0; k < path_len_j; k++)
					{
						struct loc cell_j = paths[j][k];

						if((start_i.x == cell_j.x) && (start_i.y == cell_j.y))		// Check if S_j..........S_i..........G_j
						{
							partial_order[i][j] = true;
							cout << "S_" << i << " > " << "P(" << j << ")" << endl;
						}

						if((goal_i.x == cell_j.x) && (goal_i.y == cell_j.y))		//Check if S_j..........G_i..........G_j
						{
							partial_order[j][i] = true;
							cout << "G_" << i << " > " << "P(" << j << ")" << endl;
						}
					}
			}
	}

	return partial_order;
}
//==================================================4. Partial Order : END


//==================================================5. Total Order : START
vector<int> GAMRCPP::compute_total_order(int num_of_robs, vec_vec_bool adj, vec_vec_bool &adj_residue)
{
	vector<int> in_degrees(num_of_robs, 0);
	vector<bool> visited(num_of_robs, false);
	queue<int> Q;

	for(int i = 0; i < num_of_robs; i++)
	{
		for(int j = 0; j < num_of_robs; j++)
		{
			if(adj[i][j])
			{
				in_degrees[j]++;
			}
		}
	}

	//cout << "\nIn-degrees...\n";

	for(int i = 0; i < num_of_robs; i++)
	{
		//cout << in_degrees[i] << " ";

		if(!in_degrees[i])
		{
			Q.push(i);
			//cout << "Pushed R_" << i << " ";
			visited[i] = true;
		}
	}

	cout << endl;
	//vector<int> total_order;
	vector<int> total_order(num_of_robs, -1);		//Initialized with invalid robot id (-1)
	uint to_index = 0;

	while(!Q.empty()) 
    { 
        int robot_index = Q.front(); 
        Q.pop();
        //cout << "\nPopped R_" << robot_index << endl;
        
        //total_order.push_back(robot_index);
        total_order[to_index++] = robot_index;

        for(int j = 0; j < num_of_robs; j++)
        {
        	if(adj[robot_index][j] && !visited[j])
        	{
        		in_degrees[j]--;
        		adj_residue[robot_index][j] = false;		//Traversed

        		if(!in_degrees[j])
				{
					Q.push(j);
					//cout << "Pushed R_" << j << " ";
					visited[j] = true;
				}
        	}
        }

        /*cout << "\n\nIn-degrees...\n";

		for(int i = 0; i < num_of_robs; i++)
		{
			cout << in_degrees[i] << " ";
		}

		cout << endl;*/
    }

	return total_order;
}


void GAMRCPP::adjust_dependent_paths(vector<int> opt_goal_vec, vec_vec_loc &paths, uint &active_count)
{
	uint rob_count = opt_goal_vec.size();

	for(uint rob_id = 0; rob_id < rob_count; rob_id++)
		if(opt_goal_vec[rob_id] == -1)
		{
			active_count--;
			uint path_len = paths[rob_id].size() - 1;

			for(uint j = path_len; j > 0; j--)
				paths[rob_id].erase(paths[rob_id].begin() + j);
		}
}
//==================================================5. Total Order : END


//==================================================6. Timeoffsets : START
vector<int> GAMRCPP::compute_start_time_offsets(vector<int> total_order, vec_vec_loc paths)
{
	int num_of_robs = total_order.size();
	vector<int> sto_vec(num_of_robs, 0);		// Initialized to 0
	int i = 1;

	while(i < num_of_robs)
	{
		int rob_id = total_order[i];
		int sto_rob_id = 0;
		int path_i_size = paths[rob_id].size();
		int j = i - 1;

		while(j >= 0)
		{
			int pred_id = total_order[j];		// Total order predecessor robot index
			int sto_pred_id = sto_vec[pred_id];
			int path_j_size = paths[pred_id].size();

			//================================================== Same Cell Collision (SCC) : START
			int time_max = sto_rob_id + path_i_size - 1;

			if(time_max < (sto_pred_id + path_j_size - 1))
				time_max = sto_pred_id + path_j_size - 1;

			struct loc cell_i, cell_j;
			bool flag_scc = false;

			for(int time = 0; time <= time_max; time++)
			{
				if(time <= sto_rob_id)
					cell_i = paths[rob_id][0];
				else if(time <= (sto_rob_id + path_i_size - 1))
					cell_i = paths[rob_id][time - sto_rob_id];
				else
					cell_i = paths[rob_id][path_i_size - 1];

				if(time <= sto_pred_id)
					cell_j = paths[pred_id][0];
				else if(time <= (sto_pred_id + path_j_size - 1))
					cell_j = paths[pred_id][time - sto_pred_id];
				else
					cell_j = paths[pred_id][path_j_size - 1];

				if((cell_i.x == cell_j.x) && (cell_i.y == cell_j.y))		// Same Cell Collision
				{
					flag_scc = true;
					break;
				}
			}
			//================================================== Same Cell Collision (SCC) : END
			//================================================== Head-on Collision (HoC) : START
			int time_min = sto_rob_id + path_i_size - 1;

			if(time_min > (sto_pred_id + path_j_size - 1))
				time_min = sto_pred_id + path_j_size - 1;

			struct loc cell_i_prev, cell_j_prev;
			cell_i_prev = paths[rob_id][0];
			cell_j_prev = paths[pred_id][0];
			bool flag_hoc = false;

			for(int time = 1; time <= time_min; time++)
			{
				if(time <= sto_rob_id)
					cell_i = paths[rob_id][0];
				else if(time <= (sto_rob_id + path_i_size - 1))
					cell_i = paths[rob_id][time - sto_rob_id];

				if(time <= sto_pred_id)
					cell_j = paths[pred_id][0];
				else if(time <= (sto_pred_id + path_j_size - 1))
					cell_j = paths[pred_id][time - sto_pred_id];

				if((cell_i.x == cell_j_prev.x) && (cell_i.y == cell_j_prev.y) && (cell_j.x == cell_i_prev.x) && (cell_j.y == cell_i_prev.y))		// Head-on Collision
				{
					flag_hoc = true;
					break;
				}

				cell_i_prev = cell_i;
				cell_j_prev = cell_j;
			}
			//================================================== Head-on Collision (HoC) : END

			if(flag_scc || flag_hoc)
			{
				sto_rob_id++;
				j = i - 1;
				continue;
			}
			else
				j--;
		}
		
		sto_vec[rob_id] = sto_rob_id;
		i++;
	}

	return sto_vec;
}
//==================================================6. Timeoffsets : START


//==================================================7. Optimal Trajectories : START
vec_vec_loc GAMRCPP::compute_collision_averted_paths(int num_of_robs, vector<int> time_offsets, vec_vec_loc paths, uint horizon, bool clus_plan, uint clus_id)
{
	//================================================== Maximum Time : START
	std::string gamrcppPkgPath = ros::package::getPath("gamrcpp_pkg");
	ofstream capl_file;			// Collision Averted Path Lengths File
	std::string capl_filename;

	if(clus_plan)
		capl_filename = gamrcppPkgPath + OUTPUT_DIR + boost::lexical_cast<std::string>(horizon) + "_" + boost::lexical_cast<std::string>(clus_id) + OPTIMAL_PATH_LENGTHS_FILE;
	else
		capl_filename = gamrcppPkgPath + OUTPUT_DIR + boost::lexical_cast<std::string>(horizon) + OPTIMAL_PATH_LENGTHS_FILE;
	
	capl_file.open(capl_filename.c_str(), ios::out);
	int max_time = -1;

	for(int i = 0; i < num_of_robs; i++)
	{
		int sum_of_path_len_and_time_offset = (paths[i].size() - 1) + time_offsets[i];
		capl_file << (paths[i].size() - 1) << "," << time_offsets[i] << "," << sum_of_path_len_and_time_offset << endl;

		if(sum_of_path_len_and_time_offset > max_time)
			max_time = sum_of_path_len_and_time_offset;
	}

	capl_file.close();
	//cout << "\nMaximum Time = " << max_time << endl;
	//================================================== Maximum Time : END

	//================================================== Trajectories : START
	vec_vec_loc trajectories(num_of_robs, vector<struct loc>());
	struct loc robot_loc;

	for(int robot_index = 0; robot_index < num_of_robs; robot_index++)
	{
		int time_offset_robot_index = time_offsets[robot_index];
		vector<struct loc> path_robot_index = paths[robot_index];
		int path_len_robot_index = path_robot_index.size() - 1;

		robot_loc.x = path_robot_index[0].x;
		robot_loc.y = path_robot_index[0].y;
		robot_loc.theta = path_robot_index[0].theta;
		trajectories[robot_index].push_back(robot_loc);

		for(int time = 0; time < max_time; time++)
		{
			if(time >= time_offset_robot_index)
			{
				if((time - time_offset_robot_index) < path_len_robot_index)			// Copy from the path
				{
					robot_loc.x = path_robot_index[time - time_offset_robot_index + 1].x;
					robot_loc.y = path_robot_index[time - time_offset_robot_index + 1].y;
					robot_loc.theta = path_robot_index[time - time_offset_robot_index + 1].theta;
				}
				else		// Append final location
				{
					robot_loc.x = path_robot_index[path_len_robot_index].x;
					robot_loc.y = path_robot_index[path_len_robot_index].y;
					robot_loc.theta = path_robot_index[path_len_robot_index].theta;
				}
			}
			else		// Append initial location
			{
				robot_loc.x = path_robot_index[0].x;
				robot_loc.y = path_robot_index[0].y;
				robot_loc.theta = path_robot_index[0].theta;
			}

			trajectories[robot_index].push_back(robot_loc);
		}
	}
	//================================================== Trajectories : END

	return trajectories;
}


vec_vec_loc GAMRCPP::compute_optimal_trajectories(int num_of_robs, vector<int> time_offsets, vec_vec_loc paths, uint horizon, bool clus_plan, uint clus_id)
{
	//================================================== Maximum Time : START
	std::string gamrcppPkgPath = ros::package::getPath("gamrcpp_pkg");
	ofstream capl_file;			// Collision Averted Path Lengths File
	std::string capl_filename;

	if(clus_plan)
		capl_filename = gamrcppPkgPath + OUTPUT_DIR + boost::lexical_cast<std::string>(horizon) + "_" + boost::lexical_cast<std::string>(clus_id) + OPTIMAL_PATH_LENGTHS_FILE;
	else
		capl_filename = gamrcppPkgPath + OUTPUT_DIR + boost::lexical_cast<std::string>(horizon) + OPTIMAL_PATH_LENGTHS_FILE;
	
	// capl_file.open(capl_filename.c_str(), ios::out);
	// int max_time = -1;
	bool flag_hor_len_set = false;
	uint hor_len;		// Minimum active (robot) path length

	for(int i = 0; i < num_of_robs; i++)
	{
		int sum_of_path_len_and_time_offset = (paths[i].size() - 1) + time_offsets[i];
		//capl_file << sum_of_path_len_and_time_offset << endl;		//Save Collision Averted Path Length
		// capl_file << (paths[i].size() - 1) << "," << time_offsets[i] << "," << sum_of_path_len_and_time_offset << endl;

		// if(sum_of_path_len_and_time_offset > max_time)
		// 	max_time = sum_of_path_len_and_time_offset;

		if(sum_of_path_len_and_time_offset)		// Active robot
			if(flag_hor_len_set)
			{
				if(hor_len > sum_of_path_len_and_time_offset)
					hor_len = sum_of_path_len_and_time_offset;
			}
			else
			{
				hor_len = sum_of_path_len_and_time_offset;
				flag_hor_len_set = true;
			}
	}

	// capl_file.close();
	//cout << "\nMaximum Time = " << max_time << endl;
	//================================================== Maximum Time : END

	//================================================== Trajectories : START
	capl_file.open(capl_filename.c_str(), ios::out);
	vec_vec_loc trajectories(num_of_robs, vector<struct loc>());
	struct loc robot_loc;

	for(int robot_index = 0; robot_index < num_of_robs; robot_index++)
	{
		int time_offset_robot_index = time_offsets[robot_index];
		vector<struct loc> path_robot_index = paths[robot_index];
		int path_len_robot_index = path_robot_index.size() - 1;

		robot_loc.x = path_robot_index[0].x;
		robot_loc.y = path_robot_index[0].y;
		robot_loc.theta = path_robot_index[0].theta;
		trajectories[robot_index].push_back(robot_loc);

		// for(int time = 0; time < max_time; time++)
		for(int time = 0; time < hor_len; time++)
		{
			if(time >= time_offset_robot_index)
			{
				if((time - time_offset_robot_index) < path_len_robot_index)			// Copy from the path
				{
					robot_loc.x = path_robot_index[time - time_offset_robot_index + 1].x;
					robot_loc.y = path_robot_index[time - time_offset_robot_index + 1].y;
					robot_loc.theta = path_robot_index[time - time_offset_robot_index + 1].theta;
				}
				else		// Append final location
				{
					robot_loc.x = path_robot_index[path_len_robot_index].x;
					robot_loc.y = path_robot_index[path_len_robot_index].y;
					robot_loc.theta = path_robot_index[path_len_robot_index].theta;
				}
			}
			else		// Append initial location
			{
				robot_loc.x = path_robot_index[0].x;
				robot_loc.y = path_robot_index[0].y;
				robot_loc.theta = path_robot_index[0].theta;
			}

			trajectories[robot_index].push_back(robot_loc);
		}

		if(time_offset_robot_index >= hor_len)
			capl_file << "0,0,0\n";
		else
			if((time_offset_robot_index + path_len_robot_index) >= hor_len)
				capl_file << (hor_len - time_offset_robot_index) << "," << time_offset_robot_index << "," << hor_len << endl;
			else
				capl_file << path_len_robot_index << "," << time_offset_robot_index << "," << (time_offset_robot_index + path_len_robot_index) << endl;
	}

	capl_file.close();
	//================================================== Trajectories : END

	return trajectories;
}
//==================================================7. Optimal Trajectories : END


void GAMRCPP::monitor_paths(int ws_size_x, int ws_size_y, int num_of_robs, vec_vec_loc trajectories)
{
	int hor_len = trajectories[0].size() - 1;		// Horizon Length

	//================================================== Detect Same Cell Collsion : START
	for(int time = 0; time <= hor_len; time++)
	{
		vec_vec_int ws_cells(ws_size_x, vector<int>(ws_size_y, -1));

		for(int rob_id = 0; rob_id < num_of_robs; rob_id++)
		{
			struct loc rob_loc = trajectories[rob_id][time];

			if(ws_cells[rob_loc.x][rob_loc.y] == -1)
				ws_cells[rob_loc.x][rob_loc.y] = rob_id;
			else		// Same Cell Collision
			{
				cout << "\nMonitor failed! SCC R_" << rob_id << " <> R_" << ws_cells[rob_loc.x][rob_loc.y] << " (" << rob_loc.x << ", " << rob_loc.y << ") @ t = " << time << "\n";
				exit(1);
			}
		}
	}
	//================================================== Detect Same Cell Collsion : END
	//================================================== Detect Head-on Collsion : START
	for(int time = 1; time <= hor_len; time++)
	{
		for(int rob_id1 = 0; rob_id1 < num_of_robs - 1; rob_id1++)
			for(int rob_id2 = rob_id1 + 1; rob_id2 < num_of_robs; rob_id2++)
			{
				struct loc cell_1, cell_2, cell_1_prev, cell_2_prev;

				cell_1 = trajectories[rob_id1][time];
				cell_2 = trajectories[rob_id2][time];
				cell_1_prev = trajectories[rob_id1][time - 1];
				cell_2_prev = trajectories[rob_id2][time - 1];

				if((cell_1.x == cell_2_prev.x) && (cell_1.y == cell_2_prev.y) && (cell_2.x == cell_1_prev.x) && (cell_2.y == cell_1_prev.y))		// Head-on Collision
				{
					cout << "\nMonitor failed! HoC R_" << rob_id1 << " <> R_" << rob_id2 << " (" << cell_1.x << ", " << cell_1.y << ") <> (" << cell_2.x << ", " << cell_2.y << ") @ t = " << time << "\n";
					exit(1);
				}
			}
	}
	//================================================== Detect Head-on Collsion : END
}


vec_vec_loc GAMRCPP::runGAMRCPP(int ws_size_x, int ws_size_y, vec_vec_bool ws_graph, int num_of_robs, vector<struct loc> robs_states, int num_of_goals, vector<struct loc> goals_locs, uint horizon, bool clus_plan, uint clus_id)
{
	#ifdef DEBUG_WS_GRAPH
		cout << "\nWS_Graph...\n";

		for(int j = ws_size_y - 1; j >= 0; j--)
		{
			for(int i = 0; i < ws_size_x; i++)
				cout << ws_graph[i][j] << " ";
			
			cout << endl;
		}
	#endif

	#ifdef DEBUG_ROBS_STATES
		cout << "\nRobots' states...";

		for(uint i = 0; i < num_of_robs; i++)
		{
			struct loc rob_state = robs_states[i];
			cout << "\nR_" << i << " (" << rob_state.x << ", " << rob_state.y << ", " << rob_state.theta << "): Path_" << i;
		}
	#endif

	#ifdef DEBUG_GOALS_LOCS
		cout << "\n\nGoals...";

		for(uint i = 0; i < num_of_goals; i++)
		{
			struct loc goal_loc = goals_locs[i];
			cout << "\nG_" << i << " (" << goal_loc.x << ", " << goal_loc.y << ")";
		}
	#endif

	std::string gamrcppPkgPath = ros::package::getPath("gamrcpp_pkg");
	std::string debug_filename;
	ofstream debug_file;

	//==================================================1. Optimal Costs : START
	vec_vec_int opt_cost_mat = compute_optimal_costs(ws_size_x, ws_size_y, ws_graph, robs_states, goals_locs);

	#ifdef DEBUG_OC
		printf("\n\nOptimal Cost Matrix...\n");
		debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_OC_FILENAME + boost::lexical_cast<std::string>(horizon) + CSV_EXT;
		debug_file.open(debug_filename.c_str(), ios::out);

		for(int rob_index = 0; rob_index < num_of_robs; rob_index++)
		{
			for(int goal_index = 0; goal_index < num_of_goals; goal_index++)
			{
				cout << opt_cost_mat[rob_index][goal_index] << " ";
				debug_file << opt_cost_mat[rob_index][goal_index];

				if((goal_index + 1) != num_of_goals)
					debug_file << ",";
			}

			cout << endl;
			debug_file << endl;
		}

		debug_file.close();
	#endif
	//==================================================1. Optimal Costs : END

	//==================================================2. Optimal Goals : START
	MUNKRES_ALGO munkres_obj;
	// cout << "\nMunkres...\n";
	// auto clk_start = high_resolution_clock::now();
	vec_int opt_goal_vec = munkres_obj.munkres(opt_cost_mat, num_of_robs, num_of_goals);		// Optimal goal assignments
	// auto clk_stop = high_resolution_clock::now();
	// auto clk_duration = duration_cast<milliseconds>(clk_stop - clk_start);
	// cout << "\nTime = " << clk_duration.count() << " ms = " << clk_duration.count()/1000 << " s\n";

	#ifdef DEBUG_OG
		cout << "\nOptimal Goal Vector...\n";
		debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_OG_FILENAME + boost::lexical_cast<std::string>(horizon) + TXT_EXT;
		debug_file.open(debug_filename.c_str(), ios::out);

		for(int i = 0; i < num_of_robs; i++)
		{
			// cout << "R_" << i << "\t\tG_" << opt_goal_vec[i] << endl;
			debug_file << opt_goal_vec[i];

			if((i + 1) != num_of_robs)
				debug_file << ",";
		}

		// cout << endl;
		debug_file.close();
	#endif
	//==================================================2. Optimal Goals : END
	//==================================================3. Optimal Paths : START
	uint active_count = 0;
	vec_vec_loc opt_paths = compute_optimal_paths(ws_size_x, ws_size_y, ws_graph, robs_states, goals_locs, opt_goal_vec, active_count);		// Optimal Paths

	#ifdef DEBUG_OP
		cout << "\nOptimal Paths...\n";
		debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_OP_FILENAME + boost::lexical_cast<std::string>(horizon) + CSV_EXT;
		debug_file.open(debug_filename.c_str(), ios::out);

		for(int i = 0; i < num_of_robs; i++)
		{
			cout << "Path_" << i << " ";

			vector<struct loc> path_temp = opt_paths[i];

			for(int j = 0; j < path_temp.size(); j++)
			{
				cout << "(" << path_temp[j].x << ", " << path_temp[j].y << ", " << path_temp[j].theta << ") ";
				debug_file << "(" << path_temp[j].x << " " << path_temp[j].y << " " << path_temp[j].theta << "),";
			}

			cout << endl;
			debug_file << endl;
		}
		
		debug_file.close();
	#endif
	//==================================================3. Optimal Paths : END

	vec_vec_loc feasible_paths;
	vec_vec_bool partial_order;
	vector<int> total_order;
	bool flag_to_found;		// Is the Total Order found?
	uint iter_id = 0;

	while(true)
	{
		//================================================== 4. Feasible Paths : START
		uint killed_count = 0;		// Number of killed robots
		uint revived_count = 0;		// Number of revived robots
		uint revived_goal_count = 0;		// Number of goals of revived robots that will be visited. revived_count <= revived_goal_count <= killed_count
		feasible_paths = get_feasible_paths(opt_goal_vec, opt_paths, killed_count, revived_count, revived_goal_count, ws_size_x, ws_size_y);

		#ifdef DEBUG_FP
			cout << "\nFeasible Paths...\n";
			debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_FP_FILENAME + boost::lexical_cast<std::string>(horizon) + "_i" + boost::lexical_cast<std::string>(iter_id) + CSV_EXT;
			debug_file.open(debug_filename.c_str(), ios::out);

			for(int i = 0; i < num_of_robs; i++)
			{
				cout << "Path_" << i << " ";

				vector<struct loc> path_temp = feasible_paths[i];

				for(int j = 0; j < path_temp.size(); j++)
				{
					cout << "(" << path_temp[j].x << ", " << path_temp[j].y << ", " << path_temp[j].theta << ") ";
					debug_file << "(" << path_temp[j].x << " " << path_temp[j].y << " " << path_temp[j].theta << "),";
				}

				cout << endl;
				debug_file << endl;
			}

			debug_file.close();
		#endif

		debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_FP_STAT_FILENAME + TXT_EXT;
		debug_file.open(debug_filename.c_str(), ios::app);

		// cout << "\n#Killed = " << killed_count << ".\t#Revived = " << revived_count;
		debug_file << horizon << "," << iter_id << "," << active_count << "," << killed_count << "," << revived_count << "," << revived_goal_count << endl;

		debug_file.close();

		// active_count = active_count - killed_count + revived_count;
		//================================================== 4. Feasible Paths : END
		//==================================================5. Partial Orders : START
		partial_order = compute_partial_orders(num_of_robs, feasible_paths);

		#ifdef DEBUG_PO
			cout << "\nPartial Orders...\n";
			debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_PO_FILENAME + boost::lexical_cast<std::string>(horizon) + "_i" + boost::lexical_cast<std::string>(iter_id) + CSV_EXT;
			debug_file.open(debug_filename.c_str(), ios::out);

			for(int i = 0; i < num_of_robs; i++)
			{
				vector<bool> v = partial_order[i];

				for(int j = 0; j < num_of_robs; j++)
				{
					if(v[j])
					{
						cout << "1 ";
						debug_file << "1";
					}
					else
					{
						cout << "0 ";
						debug_file << "0";
					}

					if((j + 1) != num_of_robs)
						debug_file << v[j] << ",";
				}

				cout << endl;
				debug_file << endl;
			}

			debug_file.close();
		#endif
		//==================================================5. Partial Orders : END

		//==================================================6. Total Order : START
		vec_vec_bool po_residue = partial_order;		// Residue of PO
		total_order = compute_total_order(num_of_robs, partial_order, po_residue);

		flag_to_found = true;

		for(int i = 0; i < num_of_robs; i++)
			if(total_order[i] == -1)
			{
				flag_to_found = false;
				break;
			}

		if(!flag_to_found)
		{
			// cout << "\nInvalid TO\n";

			#ifdef DEBUG_TO
				debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_TO_FILENAME + boost::lexical_cast<std::string>(horizon) + "_i" + boost::lexical_cast<std::string>(iter_id) + CSV_EXT;
				debug_file.open(debug_filename.c_str(), ios::out);

				for(int i = 0; i < num_of_robs; i++)
				{
					vector<bool> v = po_residue[i];

					for(int j = 0; j < num_of_robs; j++)
					{
						if(v[j])
						{
							cout << "1 ";
							debug_file << "1";
						}
						else
						{
							cout << "0 ";
							debug_file << "0";
						}

						if((j + 1) != num_of_robs)
							debug_file << v[j] << ",";
					}

					cout << endl;
					debug_file << endl;
				}

				debug_file.close();
			#endif

			//================================================== Directed Cyclic Graph (DCG) to Directed Acyclic Graph (DAG) : START
			vector<int> opt_goal_assignments_L, opt_goal_assignments_R;		// Left & Right Vectors of opt_goal_vec
			opt_goal_assignments_L = opt_goal_assignments_R = opt_goal_vec;

			uint count_L, count_R;
			count_L = count_R = 0;

			for(int i = 0; i < num_of_robs; i++)
			{
				for(int j = 0; j < num_of_robs; j++)
				{
					if(po_residue[i][j])
					{
						// cout << "po_residue[" << i << "][" << j << "] ";

						if(i < j)
						{
							// cout << " Left\n";

							if(opt_goal_assignments_R[i] != -1)
							{
								opt_goal_assignments_R[i] = -1;		//Goal Unassigned
								count_L++;
							}
							else if(opt_goal_assignments_R[j] != -1)
							{
								opt_goal_assignments_R[j] = -1;		//Goal Unassigned
								count_L++;
							}
						}
						else if(i > j)
						{
							// cout << " Right\n";

							if(opt_goal_assignments_L[i] != -1)
							{
								opt_goal_assignments_L[i] = -1;		//Goal Unassigned
								count_R++;
							}
							else if(opt_goal_assignments_L[j] != -1)
							{
								opt_goal_assignments_L[j] = -1;		//Goal Unassigned
								count_R++;
							}
						}
					}
				}
			}

			// cout << "\nOG:\t\t";
			for(uint i = 0; i < num_of_robs; i++)
				cout << opt_goal_vec[i] << " ";

			// cout << "count_L = " << count_L << " count_R = " << count_R;

			if(count_L && count_R)
			{
				if(count_L >= count_R)
					opt_goal_vec = opt_goal_assignments_L;
				else
					opt_goal_vec = opt_goal_assignments_R;
			}
			else if(count_L)
				opt_goal_vec = opt_goal_assignments_R;
			else if(count_R)
				opt_goal_vec = opt_goal_assignments_L;

			// cout << "\nLeft:\t";
			for(uint i = 0; i < num_of_robs; i++)
				cout << opt_goal_assignments_L[i] << " ";
			
			// cout << "\nRight:\t";
			for(uint i = 0; i < num_of_robs; i++)
				cout << opt_goal_assignments_R[i] << " ";
			//================================================== Directed Cyclic Graph (DCG) to Directed Acyclic Graph (DAG) : END
			active_count = num_of_robs;		// Reset
			adjust_dependent_paths(opt_goal_vec, feasible_paths, active_count);

			opt_paths = feasible_paths;

			#ifdef DEBUG_DP
				cout << "\nAdjusted dependent paths...\n";
				debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_DP_FILENAME + boost::lexical_cast<std::string>(horizon) + "_i" + boost::lexical_cast<std::string>(iter_id) + CSV_EXT;
				debug_file.open(debug_filename.c_str(), ios::out);

				for(uint i = 0; i < num_of_robs; i++)
				{
					cout << "Path_" << i << " ";

					vector<struct loc> path_temp = opt_paths[i];

					for(uint j = 0; j < path_temp.size(); j++)
					{
						cout << "(" << path_temp[j].x << ", " << path_temp[j].y << ", " << path_temp[j].theta << ") ";
						debug_file << "(" << path_temp[j].x << " " << path_temp[j].y << " " << path_temp[j].theta << "),";
					}

					cout << endl;
					debug_file << endl;
				}
			#endif

			iter_id++;
		}
		else
		{
			#ifdef DEBUG_TO
				cout << "\nTotal Order...\n";
				debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_TO_FILENAME + boost::lexical_cast<std::string>(horizon) + "_i" + boost::lexical_cast<std::string>(iter_id) + TXT_EXT;
				debug_file.open(debug_filename.c_str(), ios::out);

				for(int i = 0; i < num_of_robs; i++)
				{
					cout << total_order[i] << " ";
					debug_file << total_order[i] << " ";
				}

				cout << endl;
				debug_file.close();
			#endif

			break;
		}
		//==================================================6. Total Order : END
			
		// int pause;cout<<"\n...Pause...\n";cin>>pause;
	}

	//==================================================7. Start-time offsets : START
	vector<int> start_time_offsets = compute_start_time_offsets(total_order, feasible_paths);

	#ifdef DEBUG_SO
		cout << "\nStart-time offsets...\n";
		debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_SO_FILENAME + boost::lexical_cast<std::string>(horizon) + TXT_EXT;				
		debug_file.open(debug_filename.c_str(), ios::out);

		for(int i = 0; i < num_of_robs; i++)
		{
			cout << start_time_offsets[i] << " ";
			debug_file << start_time_offsets[i];

			if((i + 1) != num_of_robs)
				debug_file << " ";
		}

		cout << endl;
		debug_file.close();
	#endif
	//==================================================7. Start-time offsets : END

	//==================================================8. Collision Averted Paths : START
	#ifdef HOR_LEN_MAX
		vec_vec_loc trajectories = compute_collision_averted_paths(num_of_robs, start_time_offsets, feasible_paths, horizon, clus_plan, clus_id);
	#else
		vec_vec_loc trajectories = compute_optimal_trajectories(num_of_robs, start_time_offsets, feasible_paths, horizon, clus_plan, clus_id);
	#endif

	#ifdef DEBUG_CAP
		cout << "\nTrajectories...\n";
		debug_filename = gamrcppPkgPath + OUTPUT_DIR + DEBUG_CAP_FILENAME + boost::lexical_cast<std::string>(horizon) + CSV_EXT;
		debug_file.open(debug_filename.c_str(), ios::out);

		for(int i = 0; i < num_of_robs; i++)
		{
			vector<struct loc> t = trajectories[i];

			for(int j = 0; j < t.size(); j++)
			{
				struct loc robot_loc = t[j];
				cout << "(" << robot_loc.x << ", " << robot_loc.y << ", " << robot_loc.theta << ") ";
				debug_file << "(" << robot_loc.x << " " << robot_loc.y << " " << robot_loc.theta << "),";
			}

			cout << endl;
			debug_file << endl;
		}

		debug_file.close();
	#endif
	//==================================================8. Collision Averted Paths : END

	#ifdef MONITOR_PATHS
		monitor_paths(ws_size_x, ws_size_y, num_of_robs, trajectories);
	#endif

	return trajectories;
}