/*
Purpose: Debug
Last updated: RViz
Last updated on: 19 Mar 2022
Author: Ratijit Mitra
*/


// #define TURTLEBOT		// Enabled: Turtlebot
							// Disabled: Quadcopter2D

// #define HOR_LEN_MAX		// Enabled: The horizon length is the maximum of all path lengths
							// Disabled: The horizon length is the minimum of all path lengths of the active robots

//================================================== RViz
// #define DEBUG_RVIZ

#define OUTPUT_DIR "/output/"

//================================================== Result
#define RESULT_PER_HORIZON_FILE "resultPerHorizon.txt"
#define RESULT_FILE "result.txt"

#define CSV_EXT ".csv"
#define TXT_EXT ".txt"

// #define DEBUG_WS_GRAPH
// #define DEBUG_ROBS_STATES
// #define DEBUG_GOALS_LOCS

//================================================== OC : Optimal Costs
// #define DEBUG_OC
#define DEBUG_OC_FILENAME "oc_"

//================================================== OG : Optimal Goals
// #define DEBUG_OG
#define DEBUG_OG_FILENAME "og_"

//================================================== OP : Optimal Paths
// #define DEBUG_OP
#define DEBUG_OP_FILENAME "op_"

//================================================== FP : Feasible Paths
// #define DEBUG_FP
#define DEBUG_FP_FILENAME "fp_"
#define DEBUG_FP_STAT_FILENAME "fp_stat"

//================================================== PO : Partial Orders
// #define DEBUG_PO
#define DEBUG_PO_FILENAME "po_"

//================================================== TO : Total Order
// #define DEBUG_TO
#define DEBUG_TO_FILENAME "to_"

//================================================== DP : Dependent Paths
// #define DEBUG_DP
#define DEBUG_DP_FILENAME "dp_"

//================================================== SO : Start-time Offsets
// #define DEBUG_SO
#define DEBUG_SO_FILENAME "so_"

//================================================== CAP : Collision Averted Paths
#define DEBUG_CAP
#define DEBUG_CAP_FILENAME "cap_"
#define OPTIMAL_PATH_LENGTHS_FILE "_capl.txt"

//================================================== Monitor Paths
// #define MONITOR_PATHS