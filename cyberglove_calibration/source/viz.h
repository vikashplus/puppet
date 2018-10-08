//----------------------------------//
// MuJoCo Visualizer                //
// Author Vikashplus@gmail.com      //
//----------------------------------//

#ifndef VIS_H
#define VIS_H

typedef void(*update_cb_t)(double* time, double *qpos, double *qvel, int nq, int nv, void* user_ctx);

// clsoe the viz
void viz_close();

// initialize the viz 
void viz_init(const char* modelPath, const char* licensePath);

// Registers callback allowing users to update the visualizer
void viz_register_update_cb(update_cb_t cb, void* user_ctx);

//GPU context, use at your own risk
extern mjrContext con;


#endif