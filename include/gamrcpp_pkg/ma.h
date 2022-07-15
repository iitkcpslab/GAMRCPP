/*
Purpose: Munkres Algorithm
Author: Ratijit Mitra
*/


#include<iostream>
#include<vector>
#include<cstdlib>
// #include<chrono>
#include<unistd.h>


#include "gamrcpp_pkg/basics.h"


#define COST_INF 9999


using namespace std;
// using namespace std::chrono;


typedef vector<bool> vec_bool;
typedef vector<int> vec_int;


class MUNKRES_ALGO
{
	public:
		void show_int_vec(vec_int vec);
		void show_int_mat(vec_vec_int mat, uint row_count, uint col_count);
		void show_bool_vec(vec_bool vec);
		void show_bool_mat(vec_vec_bool mat, uint row_count, uint col_count);
		void show_bool_mat2(vec_vec_int cost_mat, vec_vec_bool mat, uint row_count, uint col_count);
		
		void row_op(vec_vec_int &cost_mat, uint row_count, uint col_count);
		void col_op(vec_vec_int &cost_mat, uint row_count, uint col_count);
		void zstar(vec_vec_int cost_mat, vec_vec_bool &zstar_mat, vec_bool &has_zstar_row, vec_bool &has_zstar_col, vec_int &zstar_row_col, vec_int &zstar_col_row, uint row_count, uint col_count);
		uint cov_zstar_cols(vec_bool cov_col, uint row_count, uint col_count);
		vec_int assign(vec_vec_int cost_mat, vec_vec_bool zstar_mat, uint row_count, uint col_count);
		uint comp_seq(int i, int j, vec_vec_bool &zstar_mat, vec_bool &has_zstar_row, vec_bool &has_zstar_col, vec_int &zstar_row_col, vec_int &zstar_col_row, vec_vec_bool &zprime_mat, vec_bool &has_zprime_row, vec_bool &has_zprime_col, vec_int &zprime_row_col, vec_int &zprime_col_row, vec_bool &cov_row, vec_bool &cov_col, uint row_count, uint col_count);
		void min_uncov_pos_op(vec_vec_int &cost_mat, vec_bool cov_row, vec_bool cov_col, uint row_count, uint col_count);
		vec_int munkres(vec_vec_int cost_mat, uint row_count, uint col_count);
};