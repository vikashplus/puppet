#include "CyberGlove.h"
#include "CyberGlove_utils.h"
#include <stdio.h>
#include <math.h>

static CyberGlove* persistentGlove = NULL;
cgData cgdata;

// Normalize the raw sample
void cGlove_nrmRawSample(cgNum* rawSample_nrm, cgNum* rawSample,
							 cgNum* range, int rawSample_sz)
{
	int i;

	// Check range
	for(i=0; i<rawSample_sz; i++)
	{
		if(option.updateRawRange)
		{	// Update range
			if(rawSample[i]<range[i])
				range[i] = rawSample[i];
			else if (rawSample[i]>range[i+rawSample_sz])
				range[i+rawSample_sz] = rawSample[i];
		}
		else
		{	// clamp to range
			if(rawSample[i]<range[i])
				rawSample[i]=range[i];
			else if (rawSample[i]>range[i+rawSample_sz])
				rawSample[i]=range[i+rawSample_sz];
		}
		
		// Normalize rawSamples
		rawSample_nrm[i] = (rawSample[i]-range[i])/(range[i+rawSample_sz]-range[i]);
	}
	rawSample_nrm[rawSample_sz] = 1;
}

// Calibrate Glove using normalized raw sample
void cGlove_calibrateNrmSample(cgNum* calibSample, cgNum* rawSample_nrm, cgNum* AdroitRangeMat,
							 cgNum* calib, int calibSample_sz, int rawSample_sz)
{
	util_mulMatVec(calibSample, calib, rawSample_nrm, calibSample_sz, rawSample_sz+1);
	for (int i=0; i<calibSample_sz; i++)
	{	
		// clamp [0 1]
		if(calibSample[i]<0)
			calibSample[i]=0;
		else if(calibSample[i]>1)
			calibSample[i]=1;

		// Scale back Adroit joint ranges
		calibSample[i] = AdroitRangeMat[i] + (AdroitRangeMat[i+calibSample_sz]-AdroitRangeMat[i])*calibSample[i];
	}

}

// Glove API ==================================

// Allocate cgdata
void cGlove_initData(cgData* d, cgOption* o)
{
	// config buffers
	d->rawSample = (cgNum*)util_malloc(sizeof(cgNum)*o->rawSenor_n, 8);			// raw glove samples
	d->rawSample_nrm = (cgNum*)util_malloc(sizeof(cgNum)*(o->rawSenor_n+1), 8);	// normalized raw glove samples
	d->calibSample = (cgNum*)util_malloc(sizeof(cgNum)*o->calibSenor_n, 8);		// Mujoco convension calibrate samples

	// allocate and load calibration + ranges
	d->calibMat = (cgNum*)util_malloc(sizeof(cgNum)*o->calibSenor_n*(o->rawSenor_n+1), 8);
	d->userRangeMat = (cgNum*)util_malloc(sizeof(cgNum)*o->rawSenor_n*2, 8);
	d->handRangeMat = (cgNum*)util_malloc(sizeof(cgNum)*o->calibSenor_n*2, 8);
	util_readFile(o->calibFile, d->calibMat, o->calibSenor_n*(o->rawSenor_n+1));
	util_readFile(o->userRangeFile, d->userRangeMat, o->rawSenor_n*2);
	util_readFile(o->handRangeFile, d->handRangeMat, o->calibSenor_n*2);
	d->valid = true;
}


// free cgdata
void cGlove_freeData(cgData* d)
{
	d->valid = false;
	util_free(d->rawSample);
	util_free(d->rawSample_nrm);
	util_free(d->calibSample);
	util_free(d->calibMat);
	util_free(d->userRangeMat);
	util_free(d->handRangeMat);
}


// Clean up
void cGlove_clean(char* errorInfo) 
{
	printf("cGlove:>\t Cleaning up cgGlove..\n");
	
	// wait for thread
	if(cgdata.updateGlove)
	{
		printf("cGlove:>\t Waiting for glove update thread to exit\n");
		cgdata.updateGlove = false;
		if(cgdata.glove_th.joinable())
		{
			cgdata.glove_th.join();
			cgdata.glove_th.~thread();
		}
		printf("cGlove:>\t Glove update thread exited\n");
	}
	
	// remove glove
	if(persistentGlove != NULL)
		delete persistentGlove;
	persistentGlove = NULL;

	// clear cgdata
	cGlove_freeData(&cgdata);

	if( errorInfo != NULL)
		util_error(errorInfo);
	else
		printf("cGlove:>\t Clean up cgGlove :: Successful\n");
} 


// connect to glove 
void cGlove_connect(cgOption* o)
{
	printf("cGlove:>\t Trying to open glove port: %s\n", o->glove_port);
	try 
	{
		persistentGlove = new CyberGlove(o->glove_port, o->baudRate);
	}
	catch (std::runtime_error name)
	{
		printf("cGlove:>\t Connection problem: %s\n", name.what());
		printf("cGlove:>\t Retrying... \n");
		try 
		{
			persistentGlove = new CyberGlove(o->glove_port, o->baudRate);
		}
		catch (std::runtime_error name)
		{
			printf("cGlove:>\t Connection problem. %s\n", name.what());
		}
	}
	if(persistentGlove == NULL) 
	{
		cGlove_clean("Couldn't create glove interface.\n");
	}
	printf("cGlove:>\t Created interface for glove.\n");
}


// Update and calibrate cgdata from the glove
void cGlove_update(cgData* d, cgOption* o)
{
	printf("cGlove:>\t cGlove update thread started\n");

	cgNum* calibSample_local = (cgNum*)util_malloc(100000+sizeof(cgNum)*o->calibSenor_n, 8); // Mujoco convension 
	int n_samples = persistentGlove->SampleSize();
	static std::vector<unsigned int> inputSample(n_samples);

	// start streaming and start update loop
	persistentGlove->StartStreaming(o->HIRES_DATA);
	while(d->updateGlove)
	{
		try
		{
			if(o->HIRES_DATA)
				persistentGlove->GetHiResSample(&inputSample.front(),	persistentGlove->SampleSize(), d->timestamp);
			else
				persistentGlove->GetSample(&inputSample.front(), persistentGlove->SampleSize(), d->timestamp);  
		}
		catch (std::runtime_error name)
		{
			printf("cGlove:>\t Error getting sample:: %s\n", name.what());
		}

		// Note : No mutex: partial info can be read by the graphics  
		for (int s=0; s<n_samples; s++)
			d->rawSample[s] = (cgNum)inputSample[s];
		
		// ??? hack to avoid clipping. Remove when resolved
		if(o->HIRES_DATA)
			for (int s=0; s<n_samples; s++)
				d->rawSample[s] *= .1;

		// normalize and calibrate
		cGlove_nrmRawSample(d->rawSample_nrm, d->rawSample, d->userRangeMat, o->rawSenor_n);
		cGlove_calibrateNrmSample(calibSample_local, d->rawSample_nrm, d->handRangeMat, d->calibMat, o->calibSenor_n, o->rawSenor_n);

		// update
		d->cgGlove.lock();
		memcpy(d->calibSample, calibSample_local, o->calibSenor_n*sizeof(cgNum));
		d->cgGlove.unlock();
	}

	// Clear buffers
	util_free(calibSample_local);
	printf("cGlove:>\t cGlove update thread exiting\n");
}


// initialize glove using options
 bool cGlove_init(cgOption* options)
{
	// Connect to Glove
	cGlove_connect(&option);

	// make cgdata
	cGlove_initData(&cgdata, &option);

	// start thread for fast udpate
	cgdata.updateGlove = true;
	cgdata.glove_th = std::thread(cGlove_update, &cgdata, &option);

	return true;
}


// get most recent glove cgdata.
void cGlove_getData(cgNum *buff, const int n_buff)
{
	if(n_buff<option.calibSenor_n)
	{
		printf("Warning:: Buffer too small to update. Minimum size should be %d", option.calibSenor_n);
		return;
	}
	cgdata.cgGlove.lock();
	memcpy(buff, cgdata.calibSample, option.calibSenor_n*sizeof(cgNum));
	cgdata.cgGlove.unlock();
}

