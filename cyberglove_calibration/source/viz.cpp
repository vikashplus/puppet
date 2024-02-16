//----------------------------------//
// MuJoCo Visualizer                //
// Author Vikashplus@gmail.com      //
//----------------------------------//

#include <windows.h>

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "string.h"
#include <process.h>
#include "viz.h"
#include "crossplatform.h"

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// User Inputs
char model_path[1000];
char license_path[1000];

// status flags
bool mj_initialized = false;
bool mj_rendering = false; 

//Update callback
update_cb_t update_cb = NULL;
void* update_cb_user_ctx = NULL;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


void init_model_data()
{    
    // activate software
    mj_activate(license_path);

    // load and compile model
    char error[1000] = "Could not load binary model";

    m = mj_loadXML(model_path, 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);
    mj_initialized = true;
}


// rendering
void viz_render(void* args)
{
    printf("mujocoViz:> Initializing Mujoco with rendering\n");
    init_model_data();
    mj_rendering = true;
    printf("mujocoViz:> Rendering thread started\n");
    
    // init GLFW
    if( !glfwInit() )
        mju_error("****** Could not initialize GLFW *******");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1000, 1000, "MuJoCo Vizualizer", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjv_makeScene(&scn, 1000);                   // space for 1000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_100);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);


    // run main loop, ~30 fps rendering
    while(!glfwWindowShouldClose(window) && mj_initialized)
    {
        // fetch updates from the user/hardware
		if (NULL != update_cb)
			update_cb(&(d->time), d->qpos, d->qvel, m->nq, m->nv, update_cb_user_ctx);
		else
			printf("WARNING: viz update callback not registered!\n");

        mj_forward(m,d);

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

        // render at ~33 Hz
        Sleep(33); 

    }

    // close GLFW, free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // Try cean exit for glfw
    try
    {
        glfwTerminate();
    }
    catch (...) 
    {
        printf("WARNING: GLFW termination failure\n\n");
    }

    mj_rendering = false;
    printf("mujocoViz:> Rendering thread exited\n");
}

void viz_init(const char* modelPath, const char* licensePath)
{
    // set user variables
    strcpy(model_path, modelPath);
    strcpy(license_path, licensePath);

    // fire up rendering 
	_beginthread(viz_render, 0, NULL);
    while(!mj_initialized)
        Sleep(33);
    printf("mujocoViz:> Mujoco initialized\n");
}

void viz_close()
{
    printf("mujocoViz:> Requesting renderer to exit\n");
    mj_initialized = 0;

    while(mj_rendering)
    {
		Sleep(3);
    }
    printf("mujocoViz:> Rendering Exited\n");
}

void viz_register_update_cb(update_cb_t cb, void* user_ctx)
{
	update_cb = cb;
	update_cb_user_ctx = user_ctx;
}
