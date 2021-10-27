#ifndef _USER_OPTIONS_H_
#define _USER_OPTIONS_H_

	typedef double cgNum;

    typedef struct _options
	{
		// Use modes
		bool USEGLOVE = true;
		bool USEFEEDBACK = true;
		bool STREAM_2_VIZ = true;
		bool STREAM_2_DRIVER = true;
		bool HIRES_DATA = false;

		// Glove variables
		char* glove_port = "COM1";
		int baudRate = 115200;
		int rawSenor_n = 22;
		bool updateRawRange = false;

		// Hand
		char* modelFile = "humanoid.xml";
		int calibSenor_n = 24;
		char* driver_ip = "128.208.4.243";
		char* driver_port = "COM1";
		char* logFile ="none";

		// Calibration
		char* calibFile = "";
		char* userRangeFile = "";
		char* handRangeFile = "";

		// Mujoco
		char* viz_ip = "128.208.4.243";
		int skip = 1;				// update teleOP every skip steps(1: updates tracking every mj_step)
		bool useIkAsPosCmd = false; // instead of snap the robot to the solved IK, send IK as position cmd

		// Inverse Kinematics
		char* ik_body_name = "panda0_link7"; // body to use for IK
		int ik_pos_tolerance = .001; // pos tolerance for IK
		int ik_max_steps = 1;	     // max mjstep for IK

		// feedback
		char* DOChan = "Dev2/port0/line0:7";
		int pulseWidth = 20; // width of feedback pulse in ms;

	}cgOption;

	// Utilities ==============================

	// write message to console, pause and exit
	void util_error(const char* msg);

	// write warning message to console
	void util_warning(const char* msg);

	// allocate memory, align
	void* util_malloc(unsigned int sz, unsigned int align);

    // multiply matrix and vector
    void util_mulMatVec(cgNum* res, const cgNum* mat, const cgNum* vec, int nr, int nc);

	// free memory
	void util_free(void* buf);

	// read configuration
	int util_config(const char *fileName, const char *iname, void *var);

	// Read options from the config file
	cgOption * readOptions(const char* filename);

#endif
