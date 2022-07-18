#include <iostream>
#include <thread>


#include <unistd.h>


using namespace std;


void thread_f(string cmd)
{
	system(cmd.c_str());
}


int main(int argc, char *argv[])
{
	if(argc != 2)
	{
		cout << "Usage: rosrun gamrcpp_pkg start_robots <robot_count>\n";
		exit(1);
	}

	uint rob_count = atoi(argv[1]);
	string export_cmd = "export ROS_MASTER_URI=http://localhost:11311";	

	for(uint robot_id = 0; robot_id < rob_count; robot_id++)
	{
		string rosrun_cmd = "rosrun gamrcpp_pkg robotExecutable __name:=robot_" + to_string(robot_id) + " _rid:=" + to_string(robot_id);
		string cmd = export_cmd + " && " + rosrun_cmd;
		thread robot_t(thread_f, cmd);
		robot_t.detach();
		sleep(5);
	}

	return 0;
}