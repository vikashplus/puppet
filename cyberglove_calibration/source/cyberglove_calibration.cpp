#include <stdlib.h>
#include <windows.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "cyberGlove_utils.h"
#include "mujoco.h"
#include "viz.h"

using namespace std;

std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str)
{
	std::vector<std::string>   result;
	std::string                line;
	std::getline(str, line);

	std::stringstream          lineStream(line);
	std::string                cell;

	while (std::getline(lineStream, cell, ','))
	{
		result.push_back(cell);
	}
	// This checks for a trailing comma with no data after it.
	if (!lineStream && cell.empty())
	{
		// If there was a trailing comma then add an empty element.
		result.push_back("");
	}
	return result;
}

bool load_poses(const string& filename, vector<vector<double>>& poses, vector<string>& joint_map)
{
	ifstream ifs(filename);
	if (ifs.fail()) {
		cout << "Failed to open" << filename <<", does it exist?";
		return false;
	}

	vector<string> line_tokens;
	vector<vector<double>> joint_angles;

	//Gather joint angles for given joint, across all poses
	while ( line_tokens = getNextLineAndSplitIntoTokens(ifs), line_tokens.size() > 1 )
	{
		vector<double> angles;
		for (auto token : line_tokens)
		{
			if (strspn(token.c_str(), "-.0123456789") == token.size())
			{
				double angle = stol(token, nullptr);
				angles.push_back(angle);
			}
			else
			{
				joint_map.push_back(token);
			}
		}
		joint_angles.push_back(angles);
	}

	//Convert to vector of hand poses (all row vectors become column vectors)
	//vector<vector<double>> poses(joint_angles[0].size(), vector<double>(joint_angles.size()));
	poses.clear();
	poses.resize(joint_angles[0].size(), vector<double>(joint_angles.size()));

	for (size_t row = 0; row < joint_angles.size(); row++)
	{
		for (size_t col = 0; col < joint_angles[0].size(); col++)
		{
			poses[col][row] = joint_angles[row][col];
		}
	}
	return true;
}

// update the viz using your sensor data stream
void update_viz(double *time, double *qpos, double *qvel, int nq, int nv, void* user_ctx)
{
	vector<vector<double>>* poses = (vector<vector<double>>*)user_ctx;
	// dummy update 
	//cGlove_getData(qpos, nq);
	static int tstep = 0;
	static int pose_index = 0;

	if(0 == tstep)
		printf("Rendering pose %d\n", pose_index);

	tstep++;
	if (tstep > 30)
	{
		pose_index++;
		if (pose_index > poses->size() -1)
			pose_index = 0;
		tstep = 0;

	}

	mjrRect rect = { 0, 0, 1000, 1000 };
	mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1);
	mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, rect, "Hello  ", 0, &con);
	memcpy(qpos, &((*poses)[pose_index][0]), sizeof(double)*(*poses)[pose_index].size());

}

vector<pair<double,double>> get_ranges()
{
	const size_t num_glove_sensors = 24;
	const size_t num_seconds = 10;

	vector<pair<double, double>> joint_max_min(num_glove_sensors, pair<double, double>(-1000, 1000));


	cout << "Getting normalization data." << endl;
	cout << "Please explore joint limits for 10 seconds." << endl;

	auto stop_time = chrono::system_clock::now() + chrono::seconds(num_seconds);


	//TODO: Will the glove samples always be 24? Seems like the number of sensors in the glove would not change

	//Capture min values and max values for each sensor for 10 seconds
	double glove_samples[num_glove_sensors] = { 0 };
	while (chrono::system_clock::now() < stop_time)
	{
		cGlove_getData(&glove_samples[0], sizeof(glove_samples)/sizeof(double));


		for (size_t i = 0; i < sizeof(glove_samples) / sizeof(double); i++)
		{
			if (glove_samples[i] > joint_max_min[i].first)
				joint_max_min[i].first = glove_samples[i];

			if (glove_samples[i] < joint_max_min[i].second)
				joint_max_min[i].second = glove_samples[i];
		}

	}
	cout << "Finished capturing normalization data" << endl;

	cout << "The ranges are:" << endl;
	for (auto item : joint_max_min)
	{
		cout << "Max: " << item.first << " , Min: " << item.second << endl;
	}
	
	return joint_max_min;
}

int main()
{
	string poses_csv("C:\\Users\\adept\\Documents\\teleOp\\cyberglove_calibration\\bin\\Adroitcalib_actuatorPoses.csv");

	//MuJoCo config
	string mujocoPath = getenv("MUJOCOPATH");
	string filePath = "C:\\Users\\adept\\Documents\\teleOp\\cyberglove_calibration\\bin\\adroit\\Adroit_hand.xml";
	string licensePath = mujocoPath + "\\mjkey.txt";

	vector<vector<double>> poses;
	vector<string> joint_map;
	if (!load_poses(poses_csv, poses, joint_map))
	{
		cout << "Unable to load poses, exiting." << endl;
		return 0;
	}
	else
	{
		cout << "Loaded " << poses.size() << " poses, for " << joint_map.size() << "joints." << endl;
	}


	//CyberGlove config and init
	//Set default options
	cgOption* cg_opt = &option;
	cg_opt->glove_port = "COM3";
	cg_opt->calibFile = "..\\..\\cyberglove\\calib\\cGlove_Adroit_actuator_default.calib";
	cg_opt->userRangeFile = "..\\..\\cyberglove\\calib\\cGlove_Adroit_actuator_default.userRange";
	cg_opt->handRangeFile = "..\\..\\cyberglove\\calib\\cGlove_Adroit_actuator_default.handRange";
	cGlove_init(cg_opt);

	get_ranges();
	return 0;

	//Register the udpate callback function
	viz_register_update_cb(update_viz, (void*)&poses);

	// Fire up the viz, movie time
	printf("Staring Viz\n");
	viz_init(filePath.c_str(), licensePath.c_str());

	// Get up, do a little dance and then close
	while(true)
		Sleep(5000);

	// Close the viz, time to go home.
	viz_close();
	return 0;
}