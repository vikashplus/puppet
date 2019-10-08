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

#undef max
#undef min
#include "cxxopts.hpp"

#include "cyberGlove_utils.h"
#include "CyberGlove.h"
#include "mujoco.h"
#include "viz.h"

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;

const int kNumGloveSensors = 22;
const size_t kNumPoseSamples = 100;
 
CyberGlove* cg = nullptr;

class UpdateVizCtx {
public:
	UpdateVizCtx(MatrixXd* poses) : poses(poses)  {};

	enum VizStates {
		kVizPose,
		kVizGloveInput
	};
	VizStates state = kVizPose;
	size_t pose_idx;
	MatrixXd* poses = nullptr;
};

std::vector<std::string> get_next_line_and_split_into_tokens(std::istream& str)
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
	while ( line_tokens = get_next_line_and_split_into_tokens(ifs), line_tokens.size() > 1 )
	{
		vector<double> angles;
		for (auto token : line_tokens)
		{
			if (strspn(token.c_str(), "-.0123456789") == token.size())
			{
				double angle = stod(token, nullptr);
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

	//Visualize a pose
	if (UpdateVizCtx::VizStates::kVizPose == viz_ctx->state && -1 != viz_ctx->pose_idx)
	{
		for (int i = 0; i < viz_ctx->poses->cols(); i++)
			qpos[i] = (*viz_ctx->poses)(viz_ctx->pose_idx, i);
	}

	// Visualize glove data
	if (UpdateVizCtx::VizStates::kVizGloveInput == viz_ctx->state)
	{
		cGlove_getData(qpos, nq);
	}
}

MatrixXd get_glove_ranges()
{
	const size_t num_seconds = 20;

	MatrixXd raw_ranges(kNumGloveSensors, 2);
	raw_ranges.col(0) = MatrixXd::Constant(raw_ranges.rows(), 1, 1000);
	raw_ranges.col(1) = MatrixXd::Constant(raw_ranges.rows(), 1, -1000);

	cout << "Getting normalization data." << endl;
	cout << "Please explore joint limits for 20 seconds." << endl;

	auto stop_time = chrono::system_clock::now() + chrono::seconds(num_seconds);

	// It's possible to get bad samples in the buffer if you don't wait a bit.
	// TODO: Make the cyberglove return failure if there's been no update.
	Sleep(1000);

	//Capture min values and max values for each sensor for 20 seconds
	double glove_samples[kNumGloveSensors] = { 0 };
	while (chrono::system_clock::now() < stop_time)
	{
		//cGlove_getRawData(glove_samples, kNumGloveSensors);
		unsigned int samples[kNumGloveSensors];
		cg->GetSample(samples, kNumGloveSensors, NULL);
		for (int i = 0; i < kNumGloveSensors; i++)
			glove_samples[i] = samples[i];

		Sleep(20);

		for (size_t i = 0; i < kNumGloveSensors; i++)
		{
			if (glove_samples[i] < raw_ranges(i, 0))
				raw_ranges(i, 0) = glove_samples[i];

			if (glove_samples[i] > raw_ranges(i, 1))
				raw_ranges(i, 1) = glove_samples[i];
		}

	}
	cout << "Finished capturing normalization data" << endl;
	
	return raw_ranges;
}

MatrixXd capture_glove_data(UpdateVizCtx& ctx, const MatrixXd& poses, MatrixXd& raw_glove_ranges)
{
	cout << "Calibration: Mimic poses displayed. Hit enter to begin capture." << endl;

	MatrixXd glove_samples = MatrixXd::Zero(kNumGloveSensors, kNumPoseSamples*poses.rows());
	MatrixXd true_values =   MatrixXd::Zero(poses.cols(),     kNumPoseSamples*poses.rows());

	ctx.state = UpdateVizCtx::kVizPose;

	for (int i_pose = 0; i_pose < poses.rows(); i_pose++)
	{
		ctx.pose_idx = i_pose;
		cout << "Pose " << i_pose << ": ";
		cin.ignore();

		for (size_t j_sample = 0; j_sample < kNumPoseSamples; j_sample++)
		{
			// Populate the true samples with the values from the poses CSV
			true_values.col(i_pose*kNumPoseSamples + j_sample) = poses.row(i_pose);

			//vector<double> glove_raw(kNumGloveSensors);
			Eigen::VectorXd glove_raw(kNumGloveSensors);

			//cGlove_getRawData(glove_raw.data(), kNumGloveSensors);
			unsigned int samples[kNumGloveSensors];
			cg->GetSample(samples, kNumGloveSensors, NULL);
			for (int i = 0; i < kNumGloveSensors; i++)
				glove_raw(i) = samples[i];

			glove_samples.col(i_pose*kNumPoseSamples + j_sample) = glove_raw; 

			//Continue collecting ranges for normalization
			for (int i = 0; i < glove_raw.size(); i++)
			{
				raw_glove_ranges(i, 0) = (glove_raw[i] < raw_glove_ranges(i, 0)) ? glove_raw[i] : raw_glove_ranges(i, 0);
				raw_glove_ranges(i, 1) = (glove_raw[i] > raw_glove_ranges(i, 1)) ? glove_raw[i] : raw_glove_ranges(i, 1);
			}

		}
		cout << " Done capturing calibration data" << endl;
	}

	return glove_samples;
}

MatrixXd gen_true_values_from_poses(const MatrixXd& poses)
{
	MatrixXd true_values = MatrixXd::Zero(poses.cols(), kNumPoseSamples*poses.rows());

	for (int i_pose = 0; i_pose < poses.rows(); i_pose++)
	{
		for (size_t j_sample = 0; j_sample < kNumPoseSamples; j_sample++)
		{
			// Populate the true samples with the values from the poses CSV
			true_values.col(i_pose*kNumPoseSamples + j_sample) = poses.row(i_pose).transpose();
		}
	}
	return true_values;
}

void eigen_matrix_to_matlab(const MatrixXd& mat, const string& matrix_name, const string& filename)
{
	std::ofstream out(filename);

	out << matrix_name << " = [ ";

	for (int i = 0; i < mat.rows(); i++)
	{
		out << mat.row(i) << "; ";
	}

	out << "]" << endl;
	out.close();

	return;
}

template<typename M>
M load_csv(const std::string& path) {
	std::ifstream indata;
	indata.open(path);
	std::string line;
	std::vector<double> values;
	int rows = 0;
	while (std::getline(indata, line)) {
		std::stringstream lineStream(line);
		std::string cell;
		while (std::getline(lineStream, cell, ',')) {
			values.push_back(std::stod(cell));
		}
		++rows;
	}
	return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size() / rows);
}

void store_csv(const std::string& name, MatrixXd matrix)
{
	const static IOFormat CSVFormat(StreamPrecision, DontAlignCols, ", ", "\n");
	ofstream file(name.c_str());
	file << matrix.format(CSVFormat);
}

MatrixXd compute_calibration(const MatrixXd& true_values_n, const MatrixXd& glove_values_n)
{
	// Following are the mappings for the Adroit hand
	// The mappings came from matlab, and thus the indices are off by 1.
	// Just FYI

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

		MatrixXd denom = MatrixXd::Zero(map_raw_f.size() + 1, glove_values_n.cols());
		denom.bottomRows(1) = MatrixXd::Ones(1, glove_values_n.cols());
		for (int i = 0; i < map_raw_f.size(); i++)
		{
			denom.row(i) = glove_values_n.row(map_raw_f[i] - 1);
		}

		MatrixXd numer = MatrixXd::Zero(map_cal_f.size(), true_values_n.cols());
		for (int i = 0; i < map_cal_f.size(); i++)
		{
			numer.row(i) = true_values_n.row(map_cal_f[i] - 1);
		}

		// We're solving for Xa=b, as opposed to the usual aX=b here
	    // xA = B: A ^ T x^T = B ^ T and you have the form you want.
		MatrixXd sol = (denom.transpose()).colPivHouseholderQr().solve(numer.transpose()).transpose();

		// These are helpful for debugging, manually checking against the operations in do_calibration.m
		//eigen_matrix_to_matlab(numer, "numer", "numer.m");
		//eigen_matrix_to_matlab(denom, "denom", "denom.m");
		//eigen_matrix_to_matlab(sol, "sol", "sol.m");

		for (int row = 0; row < map_cal_f.size(); row++)
		{
			for (int col = 0; col < map_raw_f.size() + 1; col++)
			{
				if (col < map_raw_f.size())
					calibration(map_cal_f[row] - 1, map_raw_f[col] - 1) = sol(row, col);
				else
					calibration(map_cal_f[row] - 1, glove_values_n.rows()) = sol(row,col);
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
	hand_range_file << true_ranges.transpose();

	ofstream user_range_file((filename_prefix + ".userRange").c_str());
	user_range_file << glove_ranges.transpose();

	ofstream cal_file((filename_prefix + ".calib").c_str());
	cal_file << calibration;

	return true;
}

MatrixXd normalize_samples(const MatrixXd& samples, const MatrixXd& ranges)
{
	MatrixXd scaling_factor = (ranges.col(1) - ranges.col(0)).cwiseInverse();

	//If there's no varance, the scaling_factor can have inf values. In this case, set the scaling factor to 0.
	for (int i = 0; i < scaling_factor.rows(); i++)
		scaling_factor(i, 0) = isinf(scaling_factor(i, 0)) ? 0 : scaling_factor(i, 0);

	MatrixXd bias_corrected = (samples.colwise() - ranges.col(0));
	return scaling_factor.asDiagonal()*bias_corrected;
}

//Performs shift and scale operation
double remap(double ori_val, double ori_min, double ori_max, double new_min, double new_max)
{
	double ori_abs = ori_val - ori_min;
	double ori_max_abs = ori_max - ori_min;

	double normal = ori_abs / ori_max_abs;

	double new_max_abs = new_max - new_min;
	double new_abs = new_max_abs * normal;

	double new_val = new_abs + new_min;

	return new_val;
}

// Pose space to muJoCo joint space:
//		Poses in the input file have a range from -1 to 1.
//		We want to map these ranges from jmin to jmax
//		Note, it's more complicated than it seems, because we
//      want zero values to remain the same. Check out the following
//      graphic:
//					-1-----0-----1     from this
//                jmin-----0-----jmax  to this
MatrixXd p2j(const MatrixXd& poses, const MatrixXd& mj_ranges)
{
	MatrixXd new_poses(poses.rows(), poses.cols());

	for (int row = 0; row < poses.rows(); row++)
	{
		for (int col = 0; col < poses.cols(); col++)
		{
			double joint_min = mj_ranges(col, 0);
			double joint_max = mj_ranges(col, 1);

			if (poses(row, col) >= 0)
			{
				new_poses(row, col) = remap(poses(row, col), 0, 1, 0, joint_max);
			}
			else
			{
				new_poses(row, col) = remap(poses(row, col), -1, 0, joint_min, 0);
			}
			//new_poses(row, col) = remap(poses(row, col), -1, 1, joint_min, joint_max);
		}
	}
	return new_poses;
}

int do_calibration(const cxxopts::ParseResult& opts)
{
	string mujoco_path = opts["mj_path"].as<string>();
	string poses_csv = opts["pose_file"].as<string>();
	string com_port = opts["port"].as<string>();
	string prefix = opts["prefix"].as<string>();
	bool viz_glove_input_only = opts["viz_only"].as<bool>();
	string mujoco_xml_path = opts["xml"].as<string>();

	string mj_license_path = mujoco_path + "\\mjkey.txt";

	// Options for debugging
	bool get_glove_vals_from_csv = false;
	bool store_glove_vals_to_csv = false;
	bool use_default_calib = false;
	bool write_to_m_files = false;

	//Register the udpate callback function
	UpdateVizCtx viz_ctx(nullptr);
	viz_register_update_cb(update_viz, (void*)&viz_ctx);

	if (viz_glove_input_only)
	{
		cgOption* cg_opt = &option;
		cg_opt->glove_port = const_cast<char *>(com_port.c_str());
		cg_opt->calibSenor_n = 24;
		cg_opt->calibFile = "C:\\Users\\adept\\Documents\\teleOp\\cyberglove_calibration\\bin\\user.calib";
		cg_opt->userRangeFile = "C:\\Users\\adept\\Documents\\teleOp\\cyberglove_calibration\\bin\\user.userRange";
		cg_opt->handRangeFile = "C:\\Users\\adept\\Documents\\teleOp\\cyberglove_calibration\\bin\\user.handRange";

		cGlove_init(cg_opt);

		//Viz glove input only
		if (viz_glove_input_only)
		{
			// Fire up the viz
			cout << "Staring Viz" << endl;

			viz_ctx.state = UpdateVizCtx::kVizGloveInput;

			viz_init(mujoco_xml_path.c_str(), mj_license_path.c_str());

			cout << "Pres Space to Exit";
			char c = 0;
			while (c != ' ')
				c = _getch();

			// Close the viz
			viz_close();

			return 0;
		}
	}
	else
	{
		cg = new CyberGlove(com_port.c_str(), 115200);
	}

	MatrixXd poses;
	vector<string> joint_map;
	if (!load_poses(poses_csv, poses, joint_map))
	{
		cout << "Unable to load poses, exiting." << endl;
		return 0;
	}
	else
	{
		cout << "Loaded " << poses.rows() << " poses, for " << joint_map.size() << " joints." << endl;
	}

	if (write_to_m_files)
		eigen_matrix_to_matlab(poses, "poses", "poses.m");

	// Capture sensor value ranges from the glove
	MatrixXd glove_ranges = get_glove_ranges();

	// Fire up the Viz
	cout << "Starting Viz" << endl;
	viz_ctx.poses = &poses;
	viz_ctx.pose_idx = -1;
	viz_ctx.state = UpdateVizCtx::kVizPose;
	viz_init(mujoco_xml_path.c_str(), mj_license_path.c_str());

	MatrixXd true_ranges(m->nu, 2);
	for (size_t i = 0; i < m->nu; i++)
	{
		true_ranges(i, 0) = m->actuator_ctrlrange[2 * i];
		true_ranges(i, 1) = m->actuator_ctrlrange[2 * i + 1];
	}

	if (write_to_m_files)
		eigen_matrix_to_matlab(true_ranges, "true_ranges", "true_ranges.m");

	// Remap the poses from their original space to joint space
	poses = p2j(poses, true_ranges);

	// Generate true data vectors using the calibratration pose matrix
	MatrixXd true_values = gen_true_values_from_poses(poses);
	if (write_to_m_files)
		eigen_matrix_to_matlab(true_values, "true_values", "true_values.m");

	// Normalize the true values
	MatrixXd true_values_n(true_values.rows(), true_values.cols());
	true_values_n = normalize_samples(true_values, true_ranges);

	if (write_to_m_files)
		eigen_matrix_to_matlab(true_values_n, "true_values_n", "true_values_n.m");

	// Capture calibration vectors from the glove
	// (Also continue to update ranges)
	MatrixXd glove_values;
	if (!get_glove_vals_from_csv)
	{
		glove_values = capture_glove_data(viz_ctx, poses, glove_ranges);
	}
	else
	{
		cout << "Loaded glove values and ranges from CSV file" << endl;
		glove_values = load_csv<MatrixXd>("glove_values.csv");
		glove_ranges = load_csv<MatrixXd>("glove_ranges.csv");
	}

	if (store_glove_vals_to_csv)
	{
		cout << "Stored glove values and ranges to CSV file" << endl;
		store_csv("glove_ranges.csv", glove_ranges);
		store_csv("glove_values.csv", glove_values);
	}

	if (write_to_m_files)
	{
		eigen_matrix_to_matlab(glove_values, "glove_values", "glove_values.m");
		eigen_matrix_to_matlab(glove_ranges, "glove_ranges", "glove_ranges.m");
	}

	//Normalized the glove samples
	MatrixXd glove_values_n(glove_values.rows(), glove_values.cols());
	glove_values_n = normalize_samples(glove_values, glove_ranges);

	if (write_to_m_files)
		eigen_matrix_to_matlab(glove_values_n, "glove_samples_n", "glove_values_n.m");

	//Compute calibration
	MatrixXd calibration = compute_calibration(true_values_n, glove_values_n);

	if (write_to_m_files)
		eigen_matrix_to_matlab(calibration, "calibration", "calibration.m");

	cout << "Saving caibration" << endl;
	save_calibration(prefix, glove_ranges, true_ranges, calibration);

	viz_close();

	return 0;
}

int main(int argc, char** argv)
{
	cxxopts::Options options(argv[0], " - calibration Utility for CyberGlove III");

	char cwd[MAX_PATH];
	GetModuleFileName(NULL, cwd, MAX_PATH);

	options.add_options()
		("help", "Print help")
		("c,calfile_path", "Path to the directory containing calibration files", cxxopts::value<string>()->default_value(cwd))
		("mj_path", "MUJOCO_PATH", cxxopts::value<string>()->default_value(getenv("MUJOCOPATH")))
		("pose_file", "Pose file (CSV)", cxxopts::value<string>()->default_value("Adroitcalib_actuatorPoses.csv"))
		("p,port", "Serial port (eg. \"COM3\")", cxxopts::value<string>()->default_value("COM3"))
		("prefix", "Output calibration file prefixes", cxxopts::value<string>()->default_value("user"))
		("v,viz_only", "Visualize a calibration", cxxopts::value<bool>()->default_value("false"))
		("x,xml", "Adroit MuJoCo XML model", cxxopts::value<string>()->default_value("adroit\\Adroit_hand.xml"))
		;

	auto result = options.parse(argc, argv);

	if (result.count("help"))
	{
		cout << options.help({ "", "Group" }) << std::endl;
		exit(0);
	}

	if (!result.count("pose_file"))
	{
		cout << "No pose file provided, assuming: " << result["pose_file"].as<string>() << endl;
	}

	if (!result.count("port"))
	{
		cout << "No serial port provided, assuming: " << result["port"].as<string>() << endl;
	}
	
	DWORD attr;

	attr = GetFileAttributes(result["xml"].as<string>().c_str());
	if (INVALID_FILE_ATTRIBUTES == attr && GetLastError() == ERROR_FILE_NOT_FOUND)
	{
		cout << "Pose MuJoCo XML file not found: " << result["xml"].as<string>() << endl;
		return 1;
	}

	attr = GetFileAttributes(result["pose_file"].as<string>().c_str());
	if (INVALID_FILE_ATTRIBUTES == attr && GetLastError() == ERROR_FILE_NOT_FOUND)
	{
		cout << "Pose file not found: " << result["pose_file"].as<string>() << endl;
		return 1;
	}

	string user_range_file = result["calfile_path"].as<string>() + "/" + result["prefix"].as<string>() + ".userRange";
	attr = GetFileAttributes(user_range_file.c_str());
	if (INVALID_FILE_ATTRIBUTES == attr && GetLastError() == ERROR_FILE_NOT_FOUND)
	{
		cout << "User Range calibration file not found: " << user_range_file << endl;
		return 1;
	}

	string hand_range_file = result["calfile_path"].as<string>() + "/" + result["prefix"].as<string>() + ".handRange";
	attr = GetFileAttributes(hand_range_file.c_str());
	if (INVALID_FILE_ATTRIBUTES == attr && GetLastError() == ERROR_FILE_NOT_FOUND)
	{
		cout << "Hand Range calibration file not found: " << hand_range_file << endl;
		return 1;
	}

	string calibration_file = result["calfile_path"].as<string>() + "/" + result["prefix"].as<string>() + ".calib";
	attr = GetFileAttributes(calibration_file.c_str());
	if (INVALID_FILE_ATTRIBUTES == attr && GetLastError() == ERROR_FILE_NOT_FOUND)
	{
		cout << "Calibration file not found: " << calibration_file << endl;
		return 1;
	}

	return do_calibration(result);
}