// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Core>

//#include <iostream>
#include "robot.h"
#include <math.h>
//#include <string>
//#include <algorithm>

char filename[] = "/home/campo074/Documents/mujoco300_linux/mujoco_menagerie/franka_emika_panda/scene2.xml";

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


double control_frequency = 1000.0; // 1kHz
Model bot;
static int state = 1; // for state machine
static int count = 0; // timer
double cube0[7] = {0.5,0.0,0.21,1,0,0,0};

void Controller(int &type);

void mycontroller(const mjModel* m, mjData* d)
{
  static double last_update = 0.0;
  static int type = 1; // joint vs operational space control

  int close_gripper_val = 30;
  double range = 0.05;
  int wait_one_sec = 300;

  if( d->time - last_update > 1.0/control_frequency )
  {
    switch(state){ // pick and place task
      case 1:
        bot.updateEEPosition(m,d);
        bot.setEETargetPosition(Eigen::Vector3d(0.5,0,0.21));
        bot.setEETargetOrientation(Eigen::Quaterniond(0,1,0,0));
        bot.OpenGripper(d);
        if((bot.x-bot.xd).norm() <= range){state = 2;}
        break;
      case 2:
        bot.CloseGripper(d,close_gripper_val);
        bot.setEETargetPosition(Eigen::Vector3d(0.5,0,0.21));
        bot.setEETargetOrientation(Eigen::Quaterniond(0,1,0,0));
        count = count + 1;
        if(count == wait_one_sec)
        {
          state = 3;
          count = 0;
        }
        break;
      case 3:
        bot.CloseGripper(d,close_gripper_val);
        bot.updateEEPosition(m,d);
        bot.setEETargetPosition(Eigen::Vector3d(0.5,0,0.51));
        bot.setEETargetOrientation(Eigen::Quaterniond(0,1,0,0));
        if((bot.x-bot.xd).norm() <= range){state = 4;}
        break;
      case 4:
        bot.updateEEPosition(m,d);
        bot.setEETargetPosition(Eigen::Vector3d(0.5,0.3,0.21));
        bot.setEETargetOrientation(Eigen::Quaterniond(0,1,0,0));
        if((bot.x-bot.xd).norm() <= range)
        {
          bot.OpenGripper(d);
          count = count + 1;
        }
        if(count == wait_one_sec)
        {
          state = 5;
          count = 0;
        }
        break;
      default:
        bot.setEETargetPosition(Eigen::Vector3d(0.5,0,0.51));
        bot.setEETargetOrientation(Eigen::Quaterniond(0,1,0,0));
        bot.OpenGripper(d);
        if((bot.x-bot.xd).norm() <= range){
            int k = 0;
            for(int i=9;i<m->nq;i++)
            {
              d->qpos[i] = cube0[k];
              k++;
            }
            state = 1;
          }
    }
    
    Controller(type);
    last_update = d->time;
  }
}

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    //mj_resetData(m, d);
    mjcb_control = mycontroller;
    state = 1;
    count = 0;
    // initial robot pose
    bot.setJointPosition(d,bot.qd);
    // initial cube pose
    int k = 0;
    for(int i=9;i<m->nq;i++)
    {
      d->qpos[i] = cube0[k];
      k++;
    }
    mj_forward(m, d);
  }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

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
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


void Controller(int &type)
{
  Eigen::MatrixXd tau(7,1);
  Eigen::MatrixXd Fq(7,1);
  Eigen::MatrixXd Fx(6,1);
  Eigen::VectorXd xdot_d(6,1);

  bot.updateJointPosition(d);
  bot.updateJointVelocity(d);
  bot.updateEEPosition(m,d);
  bot.updateEEOrientation(m,d,bot.ee);
  bot.updateEEVelocity();
  bot.updateGravityVector(d);
  bot.updateJacobian(m,d,bot.ee);

  bot.updateMassMatrix(m,d);
  bot.updateEEMassMatrix();
  bot.updateInverseJacobianMatrix();
  bot.updateNullspaceMatrix();

    switch(type){ // joint space control
      case 0:
        Fq = bot.kpx*(bot.qd - bot.q) - bot.kdx*bot.dq;
        tau = bot.M*Fq + bot.g + bot.Nbar.transpose()*bot.M*(-bot.kdx*bot.dq);
        break;
      case 1: // operational space control (pos&rot)
        xdot_d << (bot.x-bot.xd),-(bot.quatd*bot.quat.conjugate()).vec();
        xdot_d = -bot.kpx/bot.kdx*xdot_d;
        bot.nu = std::min(1.0,bot.Vmax/xdot_d.norm());
        Fx = bot.kdx*(bot.nu*xdot_d - bot.dx);
        tau = bot.J.transpose()*bot.L*Fx + bot.g + bot.Nbar.transpose()*bot.M*(-bot.kdx*bot.dq);
        break;
      default:
        tau = Eigen::MatrixXd::Zero(7, 1);
    }

  bot.setJointTorques(d,tau);
}

// main function
int main(int argc, const char** argv) {

  // load and compile model
  char error[1000] = "Could not load binary model";

  // check command-line arguments
  if (argc<2) {
    //std::printf(" USAGE:  basic modelfile\n");
    //return 0;
     m = mj_loadXML(filename, 0, error, 1000);
  }
  else
  {
    if (std::strlen(argv[1])>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
      m = mj_loadModel(argv[1], 0);
    } else {
      m = mj_loadXML(argv[1], 0, error, 1000);
    }
  }

  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  double arr_view[] = {145, -10, 2.5, 0.0, 0.0, 0.5};
  cam.azimuth = arr_view[0];
  cam.elevation = arr_view[1];
  cam.distance = arr_view[2];
  cam.lookat[0] = arr_view[3];
  cam.lookat[1] = arr_view[4];
  cam.lookat[2] = arr_view[5];

  /*
  std::cout << "Number of position coordinates: " << m->nq << std::endl;
  std::cout << "Number of degrees of freedom: " << m->nv << std::endl;
  std::cout << "Number of active constraints: " << d->nefc << std::endl;
  */
  // install the control callback
  mjcb_control = mycontroller;

  // initial robot pose
  bot.setJointPosition(d,bot.qd);

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0/60.0) {
      mj_step(m, d);
    }

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
  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 1;
}

/*
    //qd << q0[0],q0[1],q0[2],q0[3],q0[4],q0[5],q0[6];
    //qd(0) = cos(d->time);

    J = robot.getJacobian(m,d);
    std::cout << "Jacobian Matrix: " << std::endl;
    std::cout << J << std::endl << std::endl;

    q = robot.getCurrentJointPosition(d);
    std::cout << "Joint Position Vector: " << std::endl;
    std::cout << q << std::endl << std::endl;

    dq = robot.getCurrentJointVelocity(d);
    std::cout << "Joint Velocity Vector: " << std::endl;
    std::cout << dq << std::endl << std::endl;

    M = robot.getMassMatrix(m,d);
    std::cout << "Mass Matrix: " << std::endl;
    std::cout << M << std::endl << std::endl;

    g = robot.getGravityVector(d);
    std::cout << "Gravity Vector: " << std::endl;
    std::cout << g << std::endl << std::endl;

    L = robot.getEEMassMatrix();
    std::cout << "EE Mass Matrix: " << std::endl;
    std::cout << L << std::endl << std::endl;

    Jbar = robot.getInverseJacobianMatrix();
    std::cout << "Inverse Jacobian Matrix: " << std::endl;
    std::cout << Jbar << std::endl << std::endl;

    Nbar = robot.getNullspaceMatrix();
    std::cout << "Nullspace Matrix: " << std::endl;
    std::cout << Nbar << std::endl << std::endl;

    r = robot.getEEOrientation(m,d);
    std::cout << "Quaternion: " << std::endl;
    std::cout << r.coeffs() << std::endl << std::endl; 
    // Returns a vector expression of the coefficients (x,y,z,w) 

    x = robot.getEEPosition(m,d);
    std::cout << "EE Position: " << std::endl;
    std::cout << x << std::endl << std::endl; 

    dx = robot.getEEVelocity();
    std::cout << "EE Velocity: " << std::endl;
    std::cout << dx << std::endl << std::endl;
*/

   /*
    rd=Eigen::Quaterniond(
			1/sqrt(2)*sin(M_PI/4.0*cos(2.0*M_PI*d->time/T)),
			1/sqrt(2)*cos(M_PI/4.0*cos(2.0*M_PI*d->time/T)),
			1/sqrt(2)*sin(M_PI/4.0*cos(2.0*M_PI*d->time/T)),
			1/sqrt(2)*cos(M_PI/4.0*cos(2.0*M_PI*d->time/T))
		);
    */
    //xd(0) = 0.5+0.1*cos(d->time);
    //xd(1) = 0.1*sin(d->time);
    //xd(2) = 0.7+0.1*sin(d->time);
    //rd = Eigen::Quaterniond(0,1,0,0);

    /*
    for(int i=0;i<7;i++)
    {
      //d->ctrl[i] = tau(i);
      d->qpos[i] = q0[i];
    }
    */

       //d->ctrl[7] = 255; // open gripper: 255
    //d->ctrl[8] = 255;
    //d->qpos[8] = -1;//q0[i];
    //d->qpos[8] = q0[i];

      /*
    Eigen::MatrixXd J(6,7);
    Eigen::MatrixXd Jbar(7,6);
    Eigen::MatrixXd Nbar(7,7);
    Eigen::MatrixXd L(6,6);
    Eigen::MatrixXd M(7,7);
    Eigen::MatrixXd q(7,1);
    Eigen::MatrixXd dq(7,1);
    Eigen::MatrixXd g(7,1);
    Eigen::MatrixXd tau(7,1);
    Eigen::MatrixXd F(7,1);
    Eigen::Vector3d x(3,1);
    Eigen::MatrixXd dx(6,1);
    Eigen::MatrixXd x_error(6,1);
    Eigen::Quaterniond r;
    Eigen::VectorXd xdot_d(6,1);
    static double kpx = 200;
    static double kdx = 100;
    double nu;
    */
   /*
    J = robot.getJacobian(m,d,ee);
    q = robot.getJointPosition(d);
    dq = robot.getJointVelocity(d);
    M = robot.getMassMatrix(m,d);
    g = robot.getGravityVector(d);
    L = robot.getEEMassMatrix();
    Jbar = robot.getInverseJacobianMatrix();
    Nbar = robot.getNullspaceMatrix();
    r = robot.getEEOrientation(m,d,ee);
    x = robot.getEEPosition(m,d,ee);
    dx = robot.getEEVelocity();
    */
   /*
   if(state == 1) // Pick object
    {
      bot.updateEEPosition(m,d);
      bot.setEETargetPosition(Eigen::Vector3d(0.5,0,0.21));
      bot.setEETargetOrientation(Eigen::Quaterniond(0,0,1,0));
      bot.OpenGripper(d);
      if((bot.x-bot.xd).norm() <= range){state = 2;}
    }
    else if(state == 2) // closer gripper
    {
      bot.CloseGripper(d,close_gripper_val);
      bot.setEETargetPosition(Eigen::Vector3d(0.5,0,0.21));
      bot.setEETargetOrientation(Eigen::Quaterniond(0,0,1,0));
      count = count + 1;
      if(count == 100)
      {
        state = 3;
        count = 0;
      }
    }
    else if(state == 3) // Mid-point
    { 
      bot.CloseGripper(d,close_gripper_val);
      bot.updateEEPosition(m,d);
      bot.setEETargetPosition(Eigen::Vector3d(0.5,0,0.51));
      bot.setEETargetOrientation(Eigen::Quaterniond(0,0,1,0));
      if((bot.x-bot.xd).norm() <= range){state = 4;}
    }
    else if(state == 4)// drop off object
    { 
      bot.updateEEPosition(m,d);
      bot.setEETargetPosition(Eigen::Vector3d(0.5,0.3,0.21));
      bot.setEETargetOrientation(Eigen::Quaterniond(0,0,1,0));
      if((bot.x-bot.xd).norm() <= range)
      {
        bot.OpenGripper(d);
        count = count + 1;
      }
      if(count == 100)
      {
        state = 5;
        count = 0;
      }

    }
    else // Go to starting position
    {
      bot.setEETargetPosition(Eigen::Vector3d(0.5,0,0.51));
      bot.setEETargetOrientation(Eigen::Quaterniond(0,0,1,0));
      bot.OpenGripper(d);
    }
   */
