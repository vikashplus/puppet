#ifndef _CYBERGLOVE_UTILS_H_
#define _CYBERGLOVE_UTILS_H_

#include <thread>
#include <mutex>
#include "utils.h"
	typedef struct _data
	{
		bool valid = false;		// is data valid?
		unsigned int* timestamp;// data time stamp
		cgNum* rawSample;		// raw samples from the glove
		cgNum* rawSample_nrm;	// normalized raw glove samples
		cgNum* calibSample;		// Mujoco convension calibrate samples

		cgNum* calibMat;		// Calibration matrix
		cgNum* userRangeMat;	// User glove range
		cgNum* handRangeMat;	// Hand joint ranges

		std::thread glove_th;   // Glove background update thread
		bool updateGlove;		// update glove with latest data?
		std::mutex cgGlove;		// Mutex on the calibSamples
	}cgData;

	extern cgData cgdata;
	extern cgOption option;

	// Glove API ===============================

	// Get the latest data from the glove
	void cGlove_getData(cgNum *buff, const int n_buff);

	//  Clean up glove
	void cGlove_clean(char* errorInfo);

	// initialize glove
	bool cGlove_init(cgOption* options);

#endif