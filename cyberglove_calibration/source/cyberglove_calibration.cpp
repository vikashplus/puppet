#include <conio.h>
#include <stdlib.h>
#include <windows.h>

#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "Eigen/Dense"

#include "cyberGlove_utils.h"
#include "mujoco.h"
#include "viz.h"

using namespace std;
using Eigen::MatrixXd;

const int kNumGloveSensors = 22;
const size_t kNumPoseSamples = 100;

struct PoseData {
	int poseIndex = -1;
	double vals[kNumPoseSamples][kNumGloveSensors] = { {0} };
};

class UpdateVizCtx {
public:
	UpdateVizCtx(MatrixXd& poses) : poses(poses)  {};

	enum VizStates {
		kVizPose,
		kVizGloveInput
	};
	VizStates state = kVizPose;
	size_t pose_idx;
	MatrixXd& poses;
};

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

bool load_poses(const string& filename, MatrixXd& poses_m, vector<string>& joint_map)
{
	vector<vector<double>> poses;

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

	poses_m = MatrixXd::Zero(joint_angles[0].size(), joint_angles.size());


	//Convert to vector of hand poses (all row vectors become column vectors)
	//vector<vector<double>> poses(joint_angles[0].size(), vector<double>(joint_angles.size()));
	poses.clear();
	poses.resize(joint_angles[0].size(), vector<double>(joint_angles.size()));

	for (size_t row = 0; row < joint_angles.size(); row++)
	{
		for (size_t col = 0; col < joint_angles[0].size(); col++)
		{
			poses[col][row] = joint_angles[row][col];
			poses_m(col, row) = joint_angles[row][col];
		}
	}

	poses_m = MatrixXd::Zero(poses.size(), poses[0].size());
	for (size_t i = 0; i < poses.size(); i++)
	{
		for (size_t j = 0; j < poses[0].size(); j++)
		{
			poses_m(i, j) = poses[i][j];
		}
	}

	return true;
}

// update the viz using your sensor data stream
void update_viz(double *time, double *qpos, double *qvel, int nq, int nv, void* user_ctx)
{
	UpdateVizCtx* viz_ctx = (UpdateVizCtx*)user_ctx;

	//Visualize glove data
	if (UpdateVizCtx::VizStates::kVizPose == viz_ctx->state && -1 != viz_ctx->pose_idx)
	{
		for (int i = 0; i < viz_ctx->poses.cols(); i++)
			qpos[i] = viz_ctx->poses(viz_ctx->pose_idx, i);
	}


	// Visualize glove data
	if (UpdateVizCtx::VizStates::kVizGloveInput)
	{
		cGlove_getData(qpos, nq);
	}
}

MatrixXd get_glove_ranges()
{
	const size_t num_seconds = 10;

	MatrixXd raw_ranges(kNumGloveSensors, 2);
	raw_ranges.col(0) = MatrixXd::Constant(raw_ranges.rows(), 1, 1000);
	raw_ranges.col(1) = MatrixXd::Constant(raw_ranges.rows(), 1, -1000);

	cout << raw_ranges;

	cout << "Getting normalization data." << endl;
	cout << "Please explore joint limits for 10 seconds." << endl;

	auto stop_time = chrono::system_clock::now() + chrono::seconds(num_seconds);

	//Capture min values and max values for each sensor for 10 seconds
	double glove_samples[kNumGloveSensors] = { 0 };
	while (chrono::system_clock::now() < stop_time)
	{
		cGlove_getRawData(glove_samples, kNumGloveSensors);

		for (size_t i = 0; i < kNumGloveSensors; i++)
		{
			if (glove_samples[i] < raw_ranges(i, 0))
				raw_ranges(i, 0) = glove_samples[i];

			if (glove_samples[i] > raw_ranges(i, 1))
				raw_ranges(i, 1) = glove_samples[i];
		}

	}
	cout << "Finished capturing normalization data" << endl;

	cout << "The ranges are:" << endl;
	cout << raw_ranges;
	
	return raw_ranges;
}


//TODO: Delete this eventually
//void capture_cal_vectors(UpdateVizCtx& ctx,
//						const vector<vector<double>>& poses,
//						vector<PoseData>& cal_vectors,
//						vector<pair<double, double>>& joint_max_min)
//{
//	cout << "Calibration: Mimic poses displayed. Hit enter to begin capture." << endl;
//
//	cal_vectors.resize(poses.size());
//
//	for (size_t i = 0; i < poses.size(); i++)
//	{
//		ctx.pose_idx = i;
//		cout << "Pose " << i << ": ";
//		cin.ignore();
//
//		for (size_t j = 0; j < kNumPoseSamples; j++)
//		{
//			double glove_samples[kNumGloveSensors] = { 0 };
//
//			cGlove_getData(glove_samples, kNumGloveSensors);
//
//			// Use this opportunity to update ranges for normalization
//			for (size_t i = 0; i < kNumGloveSensors; i++)
//			{
//				if (glove_samples[i] > joint_max_min[i].first)
//					joint_max_min[i].first = glove_samples[i];
//
//				if (glove_samples[i] < joint_max_min[i].second)
//					joint_max_min[i].second = glove_samples[i];
//			}
//
//			memcpy(&(cal_vectors[i].vals[j][0]), glove_samples, sizeof(double)*kNumGloveSensors);
//
//			if (0 == j % 5)
//				cout << ".";
//		}
//		cout << " Done capturing calibration data" << endl;
//	}
//}


MatrixXd capture_glove_data(UpdateVizCtx& ctx, const MatrixXd& poses, MatrixXd& raw_glove_ranges)
{
	cout << "Calibration: Mimic poses displayed. Hit enter to begin capture." << endl;

	MatrixXd glove_samples = MatrixXd::Zero(kNumGloveSensors, kNumPoseSamples*poses.rows());
	MatrixXd true_values =   MatrixXd::Zero(poses.cols(),     kNumPoseSamples*poses.rows());

	for (int i_pose = 0; i_pose < poses.rows(); i_pose++)
	{
		cout << "Pose " << i_pose << ": ";
		cin.ignore();

		for (size_t j_sample = 0; j_sample < kNumPoseSamples; j_sample++)
		{
			// Populate the true samples with the values from the poses CSV
			true_values.col(i_pose*kNumPoseSamples + j_sample) = poses.row(i_pose);

			//vector<double> glove_raw(kNumGloveSensors);
			Eigen::VectorXd glove_raw(kNumGloveSensors);
			cGlove_getRawData(glove_raw.data(), kNumGloveSensors);
			glove_samples.col(i_pose*kNumPoseSamples + j_sample) = glove_raw; 

			//Continue collecting ranges for normalization
			for (int i = 0; i < glove_raw.size(); i++)
			{
				raw_glove_ranges(i, 0) = (glove_raw[i] < raw_glove_ranges(i, 1)) ? glove_raw[i] : raw_glove_ranges(i, 0);
				raw_glove_ranges(i, 1) = (glove_raw[i] > raw_glove_ranges(i, 2)) ? glove_raw[i] : raw_glove_ranges(i, 1);
			}

		}
		cout << " Done capturing calibration data" << endl;
	}

	return MatrixXd();
}

MatrixXd gen_true_values_from_poses(const MatrixXd& poses)
{
	MatrixXd true_values = MatrixXd::Zero(poses.cols(), kNumPoseSamples*poses.cols());

	for (int i_pose = 0; i_pose < poses.cols(); i_pose++)
	{
		for (size_t j_sample = 0; j_sample < kNumPoseSamples; j_sample++)
		{
			// Populate the true samples with the values from the poses CSV
			true_values.col(i_pose*kNumPoseSamples + j_sample) = poses.row(i_pose);
		}
	}
	return true_values;
}

MatrixXd compute_calibration(const MatrixXd& true_values_n, const MatrixXd& glove_values_n)
{
	//Following are the mappings for teh Adroit hand

	// How the raw signals from the cyberglove map into the fingers
	std::vector<std::vector<int>> map_raw =
	{ 
		{
			{  4,  5,  6,  8, 11 },     // First Finger
			{  8,  9, 10, 11, 12, 15 }, // Second Finger
			{ 12, 13, 14, 15, 19 },     // Ring Finger
			{ 12, 16, 17, 18, 19 },     // Little finger
			{  1,  2,  3,  4, 20, 21 }, // Thumb calibrate
			{  1, 20, 21, 22 }          // Wrist Calibrate
		} 
	};

	// How the MuJoCo joints maps into the fingers
	std::vector<std::vector<int>> map_cal = 
	{
		{
			{  3,  4,  5, 6 },      // First Finger
			{  7,  8,  9, 10 },     // Second Finger
			{ 11, 12, 13, 14 },     // Ring Finger
			{ 15, 16, 17, 18, 19 }, // Little finger
			{ 20, 21, 22, 23, 24 }, // Thumb calibrate
			{  1,  2 }              // Wrist Calibrate
		}
	};

	MatrixXd calibration = MatrixXd::Zero(true_values_n.rows(), glove_values_n.rows() + 1);

	for (int i = 0; i < map_raw.size(); i++)
	{
		// Mapping for the current finger
		auto map_raw_f = map_raw[i];
		auto map_cal_f = map_cal[i];

		// calibration(map_cal_F, [map_raw_F(n_raw + 1)]) = trueNValue(map_cal_F, :) / ...
		//	[gloveNSamps(map_raw_F, :); ones(1, size(gloveNSamps, 2))];

		MatrixXd denom = MatrixXd::Zero(map_raw_f.size() + 1, glove_values_n.cols());
		denom.bottomRows(1) = MatrixXd::Ones(1, glove_values_n.cols());
		for (int i = 0; i < map_raw_f.size(); i++)
		{
			denom.row(i) = glove_values_n.row(map_raw_f[i]);
		}

		MatrixXd numer = MatrixXd::Zero(map_cal_f.size(), true_values_n.cols());
		for (int i = 0; i < map_cal_f.size(); i++)
		{
			numer.row(i) = true_values_n.row(map_cal_f[i]);
		}

		// We're solving for Xa=b, as opposed to the usual aX=b here
	   // xA = B: A ^ T x^T = B ^ T and you have the form you want.
		MatrixXd sol = (denom.transpose()).colPivHouseholderQr().solve(numer.transpose()).transpose();

		for (int row = 0; row < map_cal_f.size(); row++)
		{
			for (int col = 0; col < map_raw_f.size() + 1; col++)
			{
				if (col < map_raw_f.size())
					calibration(map_cal_f[row], map_raw_f[col]) = sol(row, col);
				else
					calibration(map_cal_f[row], glove_values_n.rows()) = sol(row,col);
			}
			
		}
	}

	return calibration;
}

bool save_calibration(const string& filename_prefix,
	const MatrixXd& glove_ranges,
	const MatrixXd& true_ranges,
	const MatrixXd& calibration)
{
	const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

	ofstream hand_range_file((filename_prefix+".handRange").c_str());
	hand_range_file << true_ranges.format(CSVFormat);

	ofstream user_range_file((filename_prefix + ".userRange").c_str());
	user_range_file << glove_ranges.format(CSVFormat);

	ofstream cal_file((filename_prefix + ".calib").c_str());
	cal_file << calibration;

	return true;
}


int main()
{
	string poses_csv("C:\\Users\\adept\\Documents\\teleOp\\cyberglove_calibration\\bin\\Adroitcalib_actuatorPoses.csv");

	//MuJoCo config
	string mujocoPath = getenv("MUJOCOPATH");
	string filePath = "C:\\Users\\adept\\Documents\\teleOp\\cyberglove_calibration\\bin\\adroit\\Adroit_hand.xml";
	string licensePath = mujocoPath + "\\mjkey.txt";

	MatrixXd poses;
	vector<string> joint_map;
	if (!load_poses(poses_csv, poses, joint_map))
	{
		cout << "Unable to load poses, exiting." << endl;
		return 0;
	}
	else
	{
		cout << "Loaded " << poses.size() << " poses, for " << joint_map.size() << " joints." << endl;
	}

	//CyberGlove config and init
	//Set default options
	cgOption* cg_opt = &option;
	cg_opt->glove_port = "COM3";
	cg_opt->calibSenor_n = 22;
	cg_opt->calibFile = "C:\\Users\\adept\\Documents\\teleOp\\cyberglove\\calib\\cGlove_Adroit_actuator_default.calib";
	cg_opt->userRangeFile = "C:\\Users\\adept\\Documents\\teleOp\\cyberglove\\calib\\cGlove_Adroit_actuator_default.userRange";
	cg_opt->handRangeFile = "C:\\Users\\adept\\Documents\\teleOp\\cyberglove\\calib\\cGlove_Adroit_actuator_default.handRange";
	cGlove_init(cg_opt);

	//Register the udpate callback function
	UpdateVizCtx viz_ctx(poses);
	viz_ctx.pose_idx = -1;
	viz_register_update_cb(update_viz, (void*)&viz_ctx);

	// Fire up the viz, movie time
	printf("Staring Viz\n");
	viz_init(filePath.c_str(), licensePath.c_str());

	// Capture sensor value ranges from the glove
	viz_ctx.state = UpdateVizCtx::kVizGloveInput;
	MatrixXd glove_ranges = get_glove_ranges();

	MatrixXd true_ranges(m->nu, 2);
	for (size_t i = 0; i < m->nu; i++)
	{
		true_ranges(i, 0) = m->actuator_ctrlrange[2 * i];
		true_ranges(i, 1) = m->actuator_ctrlrange[2 * i + 1];
	}

	// Capture calibration vectors from the glove
	// (Also continue to update ranges)
	MatrixXd glove_values = capture_glove_data(viz_ctx, poses, glove_ranges);

	// Generate true data vectors using the calibratration pose matrix
	MatrixXd true_values = gen_true_values_from_poses(poses);

	// Normalize the true values
	//val = (val - min)/(max-min)
	MatrixXd true_values_n(true_values.rows(), true_values.cols());
	true_values_n = (true_ranges - true_ranges.col(1)).cwiseProduct((true_ranges.col(2) - true_ranges.col(1)).cwiseInverse());

	//Normalized the glove samples
	MatrixXd glove_values_n(glove_values.rows(), glove_values.cols());
	glove_values_n = (glove_ranges - glove_ranges.col(1)).cwiseProduct((glove_ranges.col(2) - glove_ranges.col(1)).cwiseInverse());

	//TODO: Compute calibration
	MatrixXd calibration = compute_calibration(true_values_n, glove_values_n);

	save_calibration("output", glove_ranges, true_ranges, calibration);

	// Get up, do a little dance and then close
	while(true)
		Sleep(5000);

	// Close the viz, time to go home.
	viz_close();
	return 0;
}