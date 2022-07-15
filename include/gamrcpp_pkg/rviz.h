/*
Purpose: RViz
Last updated: 
Last updated on: 
Author: Ratijit Mitra
*/


#include <ros/ros.h>

#include <std_msgs/ColorRGBA.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define X_OFFSET 0.5
#define Y_OFFSET X_OFFSET
#define Z_OFFSET X_OFFSET
#define TRANSPARENCY 0.1

using namespace std;

typedef vector<vector<double> > double_mat;		// Matrix
typedef vector<double_mat> double_vox;		// Voxel


class cRViz
{
	public:
		uint ws_x, ws_y, ws_z;		// Workspace size
		ros::Publisher rviz_pub;
		visualization_msgs::MarkerArray rviz_marker_array;

		void init_marker(visualization_msgs::Marker& marker, int id, float x, float y, float r, float g, float b);
		void init_marker3d(visualization_msgs::Marker& marker, int id, float x, float y, float z, float r, float g, float b, float a);
		void init_rviz(uint ws_size_x, uint ws_size_y, double_mat ws, ros::NodeHandle *nh);
		void init_rviz3d(uint ws_size_x, uint ws_size_y, uint ws_size_z, double_vox ws, ros::NodeHandle *nh);
		void update_rviz(double_mat ws);
		void update_rviz3d(double_vox ws);
};