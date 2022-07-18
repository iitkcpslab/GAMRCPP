#include <vector>


using namespace std;


struct s2
{
  int x;
  int y;
};

typedef struct s2 position;

typedef std::vector<position> pos_vec_t;

struct s3
{
  int x;
  int y;
  double theta;
};

typedef struct s3 position_theta;

typedef std::vector<position_theta> pos_theta_vec_t;


struct d
{
  unsigned int length_x;
  unsigned int length_y;
};

typedef struct d dimension_t;


struct c
{
  float max_cost;
  float min_cost;
  float min2_cost;
  float min_cost_diff;
  float min2_min_cost_diff;
};

typedef struct c prim_cost_t;


struct plan
{
  int id;
  int time_instance;
  int location_x;
  int location_y;
  int location_z;
  double theta;
      
};

typedef struct plan motionPlan;

typedef std::vector<motionPlan> plan_vec_t;


struct status
{
  int id;
  int status;
  int x;
  int y;
  int z;
  double theta;   
};

typedef struct status robot_status;
typedef std::vector<robot_status> robot_status_vec_t;


typedef std::vector<int> Cluster;

//std::vector< std::vector<double> > rd;
// typedef vector<vector<double> > Rewards;
bool isObstacle (pos_vec_t , unsigned int , unsigned int );

struct loc
{
  int x;
  int y;
  int z;
  int theta;
};


struct node     //For stacks and queues
{
  int index;
  int is_robot;
};

typedef vector<double> double_vec;
typedef vector<struct loc> vec_loc;

typedef vector<vector<bool> > vec_vec_bool;
typedef vector<double_vec> double_mat;
typedef vector<vector<int> > vec_vec_int;
typedef vector<vector<struct loc> > vec_vec_loc;

typedef vector<vec_vec_int> vec_3d_int;
typedef vector<vec_vec_bool> vec_3d_bool;