/*
Purpose: Munkres Algorithm
Last updated: 
Last updated on: 24 Apr 2021
Author: Ratijit Mitra
*/


#include "gamrcpp_pkg/ma.h"


void MUNKRES_ALGO::show_int_vec(vec_int vec)
{
	for(uint i = 0; i < vec.size(); i++)
		cout << vec[i] << " ";

	cout << endl;
}


void MUNKRES_ALGO::show_int_mat(vec_vec_int mat, uint row_count, uint col_count)
{
	for(uint i = 0; i < row_count; i++)
	{
		for(uint j = 0; j < col_count; j++)
			cout << mat[i][j] << " ";

		cout << endl;
	}
}


void MUNKRES_ALGO::show_bool_vec(vec_bool vec)
{
	for(uint i = 0; i < vec.size(); i++)
		cout << vec[i] << " ";

	cout << endl;
}


void MUNKRES_ALGO::show_bool_mat(vec_vec_bool mat, uint row_count, uint col_count)
{
	for(uint i = 0; i < row_count; i++)
	{
		for(uint j = 0; j < col_count; j++)
			cout << mat[i][j] << " ";

		cout << endl;
	}
}


void MUNKRES_ALGO::show_bool_mat2(vec_vec_int cost_mat, vec_vec_bool mat, uint row_count, uint col_count)
{
	int cost;
	int tot_cost = 0;		// Total cost

	for(uint i = 0; i < row_count; i++)
		for(uint j = 0; j < col_count; j++)
			if(mat[i][j])
			{
				cost = cost_mat[i][j];

				if(cost == COST_INF)
					cout << "R_" << i << " G_-1 = 0";
				else
				{
					cout << "R_" << i << " G_" << j << " = " << cost << endl;
					tot_cost += cost;
				}
			}

	cout << "\nTotal cost = " << tot_cost << endl;
}


void MUNKRES_ALGO::row_op(vec_vec_int &cost_mat, uint row_count, uint col_count)
{
	for(uint i = 0; i < row_count; i++)
	{
		int row_min = cost_mat[i][0];

		for(uint j = 1; j < col_count; j++)
		{
			int cost = cost_mat[i][j];

			if(row_min > cost)
				row_min = cost;
		}

		// cout << "\nRow " << i << " min = " << row_min;

		for(uint j = 0; j < col_count; j++)
			cost_mat[i][j] -= row_min;
	}

	// cout << "\n\nCost matrix after row operations...\n";
	// show_int_mat(cost_mat, row_count, col_count);	
}


void MUNKRES_ALGO::col_op(vec_vec_int &cost_mat, uint row_count, uint col_count)
{
	for(uint i = 0; i < col_count; i++)
	{
		int col_min = cost_mat[0][i];

		for(uint j = 1; j < row_count; j++)
		{
			int cost = cost_mat[j][i];

			if(col_min > cost)
				col_min = cost;
		}

		// cout << "\nCol " << i << " min = " << col_min;

		for(uint j = 0; j < row_count; j++)
			cost_mat[j][i] -= col_min;
	}

	// cout << "\n\nCost matrix after column operations...\n";
	// show_int_mat(cost_mat, row_count, col_count);	
}


void MUNKRES_ALGO::zstar(vec_vec_int cost_mat, vec_vec_bool &zstar_mat, vec_bool &has_zstar_row, vec_bool &has_zstar_col, vec_int &zstar_row_col, vec_int &zstar_col_row, uint row_count, uint col_count)
{
	for(uint i = 0; i < row_count; i++)
		for(uint j = 0; j < col_count; j++)
			if(!cost_mat[i][j] && !has_zstar_row[i] && !has_zstar_col[j])
			{
				zstar_mat[i][j] = true;
				has_zstar_row[i] = true;
				has_zstar_col[j] = true;
				zstar_row_col[i] = j;
				zstar_col_row[j] = i;
			}

	// cout << "\nZstar matrix...\n";
	// show_bool_mat(zstar_mat, row_count, col_count);
}


uint MUNKRES_ALGO::cov_zstar_cols(vec_bool cov_col, uint row_count, uint col_count)
{
	// cout << "\nCov_col vector...\n";
	// show_bool_vec(cov_col);

	uint cov_col_count = 0;

	for(uint i = 0; i < col_count; i++)
		if(cov_col[i])
			cov_col_count++;

	return cov_col_count;
}


vec_int MUNKRES_ALGO::assign(vec_vec_int cost_mat, vec_vec_bool zstar_mat, uint row_count, uint col_count)
{
	vec_int opt_goal_vec(row_count, -1);		// -1: Inactive
	int cost;
	// int tot_cost = 0;		// Total cost

	for(uint i = 0; i < row_count; i++)
		for(uint j = 0; j < col_count; j++)
			if(zstar_mat[i][j])
			{
				cost = cost_mat[i][j];

				if(cost == COST_INF)
					cout << "\nR_" << i << " G_-1 = 0";
				else
				{
					opt_goal_vec[i] = j;		// Active
					cout << "\nR_" << i << " G_" << j << " = " << cost;
					// tot_cost += cost;
				}
			}

	// cout << "\nTotal cost = " << tot_cost << endl;
	return opt_goal_vec;
}


uint MUNKRES_ALGO::comp_seq(int i, int j, vec_vec_bool &zstar_mat, vec_bool &has_zstar_row, vec_bool &has_zstar_col, vec_int &zstar_row_col, vec_int &zstar_col_row, vec_vec_bool &zprime_mat, vec_bool &has_zprime_row, vec_bool &has_zprime_col, vec_int &zprime_row_col, vec_int &zprime_col_row, vec_bool &cov_row, vec_bool &cov_col, uint row_count, uint col_count)
{
	vec_int seq_row;		// Sequence
	vec_int seq_col;

	int row_id = i;
	int col_id = j;

	seq_row.push_back(row_id);
	seq_col.push_back(col_id);

	// cout << "\nSequence...\n";
	// show_int_vec(seq_row);
	// show_int_vec(seq_col);

	while(has_zstar_col[col_id])
	{
		row_id = zstar_col_row[col_id];

		seq_row.push_back(row_id);
		seq_col.push_back(col_id);

		// cout << "\nSequence...\n";
		// show_int_vec(seq_row);
		// show_int_vec(seq_col);

		if(has_zprime_row[row_id])
		{
			col_id = zprime_row_col[row_id];

			seq_row.push_back(row_id);
			seq_col.push_back(col_id);

			// cout << "\nSequence...\n";
			// show_int_vec(seq_row);
			// show_int_vec(seq_col);
		}
	}

	// cout << "\nSequence...\n";
	// show_int_vec(seq_row);
	// show_int_vec(seq_col);

	for(uint k = 0; k < seq_row.size(); k++)
		if((k % 2) != 0)		// 0* to 0
		{
			row_id = seq_row[k];
			col_id = seq_col[k];

			zstar_mat[row_id][col_id] = false;
			has_zstar_row[row_id] = false;
			has_zstar_col[col_id] = false;
			zstar_row_col[row_id] = -1;
			zstar_col_row[col_id] = -1;
		}

	for(uint k = 0; k < seq_row.size(); k++)
		if((k % 2) == 0)		// 0' to 0*
		{
			row_id = seq_row[k];
			col_id = seq_col[k];

			zstar_mat[row_id][col_id] = true;
			has_zstar_row[row_id] = true;
			has_zstar_col[col_id] = true;
			zstar_row_col[row_id] = col_id;
			zstar_col_row[col_id] = row_id;

			zprime_mat[row_id][col_id] = false;
			has_zprime_row[row_id] = false;
			has_zprime_col[col_id] = false;
			zprime_row_col[row_id] = -1;
			zprime_col_row[col_id] = -1;
		}

	// cout << "\nZstar matrix...\n";
	// show_bool_mat(zstar_mat, row_count, col_count);

	// cout << "\nZprime matrix...\n";
	// show_bool_mat(zprime_mat, row_count, col_count);

	for(uint k = 0; k < row_count; k++)
		for(uint l = 0; l < col_count; l++)
			if(zprime_mat[k][l])
			{
				zprime_mat[k][l] = false;
				has_zprime_row[k] = false;
				has_zprime_col[l] = false;
				zprime_row_col[k] = -1;
				zprime_col_row[l] = -1;
			}

	// cout << "\nZprime matrix...\n";
	// show_bool_mat(zprime_mat, row_count, col_count);

	for(uint k = 0; k < row_count; k++)
		cov_row[k] = false;

	cov_col = has_zstar_col;
	uint cov_col_count = 0;

	for(uint k = 0; k < col_count; k++)
		if(has_zstar_col[k])
			cov_col_count++;

	return cov_col_count;
}


void MUNKRES_ALGO::min_uncov_pos_op(vec_vec_int &cost_mat, vec_bool cov_row, vec_bool cov_col, uint row_count, uint col_count)
{
	int cost_min = COST_INF;
	int cost_min_row, cost_min_col;
	cost_min_row = cost_min_col = -1;

	for(uint i = 0; i < row_count; i++)
		if(!cov_row[i])
			for(uint j = 0; j < col_count; j++)
				if(!cov_col[j])
				{
					int cost = cost_mat[i][j];

					if(cost_min > cost)
					{
						cost_min = cost;
						cost_min_row = i;
						cost_min_col = j;
					}
				}

	// cout << "\nMinimum uncovered cost = " << cost_min << " @ " << cost_min_row << ", " << cost_min_col;

	for(uint i = 0; i < row_count; i++)
		if(cov_row[i])
			for(uint j = 0; j < col_count; j++)
				cost_mat[i][j] += cost_min;

	for(uint i = 0; i < col_count; i++)
		if(!cov_col[i])
			for(uint j = 0; j < row_count; j++)
				cost_mat[j][i] -= cost_min;
}


vec_int MUNKRES_ALGO::munkres(vec_vec_int cost_mat, uint row_count, uint col_count)
{
	vec_vec_int cost_mat_bkp = cost_mat;
	uint min_rc = row_count;		// Min {#rows, #cols}

	if(row_count > col_count)
	{
		col_op(cost_mat, row_count, col_count);		// Column operations
		min_rc = col_count;
	}
	else
	{
		row_op(cost_mat, row_count, col_count);		// Row operations

		if(row_count == col_count)
			col_op(cost_mat, row_count, col_count);
	}

	vec_vec_bool zstar_mat(row_count, vec_bool(col_count, false));		// 0* matrix: true = 0*, false = not 0*
	vec_bool has_zstar_row(row_count, false);		// Does the i-th row have a 0*?
	vec_bool has_zstar_col(col_count, false);		// Does the i-th col have a 0*?
	vec_int zstar_row_col(row_count, -1);		// Column index of the 0* in the i-th row
	vec_int zstar_col_row(col_count, -1);		// Row index of the 0* in the i-th column
	zstar(cost_mat, zstar_mat, has_zstar_row, has_zstar_col, zstar_row_col, zstar_col_row, row_count, col_count);		// Create 0*s

	vec_bool cov_col(col_count, false);		// Is the i-th column covered?
	cov_col = has_zstar_col;

	if(cov_zstar_cols(cov_col, row_count, col_count) == min_rc)		// Cover 0* columns
		// return zstar_mat;
		return assign(cost_mat_bkp, zstar_mat, row_count, col_count);

	vec_bool cov_row(row_count, false);		// Is the i-th row covered?
	vec_vec_bool zprime_mat(row_count, vec_bool(col_count, false));		// 0' matrix: true = 0', false = not 0'
	vec_bool has_zprime_row(row_count, false);		// Does the i-th row have a 0'?
	vec_bool has_zprime_col(col_count, false);		// Does the i-th col have a 0'?
	vec_int zprime_row_col(row_count, -1);		// Column index of the 0' in the i-th row
	vec_int zprime_col_row(col_count, -1);		// Row index of the 0' in the i-th column

	while(true)
	{
		bool flag;

		do
		{
			flag = false;

			for(uint i = 0; (i < row_count) && !flag; i++)
				if(!cov_row[i])
					for(uint j = 0; (j < col_count) && !flag; j++)
						if(!cov_col[j] && !cost_mat[i][j])
						{
							// cout << "\n0' @ " << i << ", " << j;
							zprime_mat[i][j] = true;
							has_zprime_row[i] = true;
							has_zprime_col[j] = true;
							zprime_row_col[i] = j;
							zprime_col_row[j] = i;

							// cout << "\nZprime matrix...\n";
							// show_bool_mat(zprime_mat, row_count, col_count);

							if(has_zstar_row[i])
							{
								// cout << " (0* @ " << i << ", " << zstar_row_col[i] << ")";
								cov_row[i] = true;
								cov_col[zstar_row_col[i]] = false;
								flag = true;
							}
							else
							{
								uint cov_col_count = comp_seq(i, j, zstar_mat, has_zstar_row, has_zstar_col, zstar_row_col, zstar_col_row, zprime_mat, has_zprime_row, has_zprime_col, zprime_row_col, zprime_col_row, cov_row, cov_col, row_count, col_count);
								
								if(cov_col_count == min_rc)
									// return zstar_mat;
									return assign(cost_mat_bkp, zstar_mat, row_count, col_count);
								else
								{
									// cout << "\ncov_col_count = " << cov_col_count;
									flag = true;
								}
								//================================================== Step 3: END
							}
						}

		}
		while(flag);

		min_uncov_pos_op(cost_mat, cov_row, cov_col, row_count, col_count);

		// cout << "\nCost matrix...\n";
		// show_int_mat(cost_mat, row_count, col_count);
	}
}