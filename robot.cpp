//#include <iostream>

#include "robot.h"
//#include <mujoco/mujoco.h>
//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Geometry>

void Model::updateMassMatrix(const mjModel* m, mjData* d)
{
    mjtNum* massMatrix = new mjtNum[m->nv*m->nv];
    mj_fullM(m, massMatrix, (d->qM) ); // returns mass matrix in row major format

    for (int i = 0; i < dof; ++i) {
        for (int j = 0; j < dof; ++j) {
          M(i,j) = massMatrix[i * m->nv  + j];
      }
    }
}

void Model::updateEEMassMatrix(void)
{
    L = (J*M.inverse()*J.transpose()).inverse();
}

void Model::updateInverseJacobianMatrix(void)
{
    Jbar = M.inverse()*J.transpose()*L;
}

void Model::updateNullspaceMatrix(void)
{
    Nbar = Eigen::MatrixXd::Identity(dof, dof) - Jbar*J;
}

void Model::updateGravityVector(mjData* d)
{
    for(int i=0;i<dof;i++)
    {
        g(i) = d->qfrc_bias[i];
    } 
}

void Model::updateJointPosition(mjData* d)
{
    for(int i=0;i<dof;i++)
    {
        q(i) = d->qpos[i];
    } 
}

void Model::updateJointVelocity(mjData* d)
{
    for(int i=0;i<dof;i++)
    {
        dq(i) = d->qvel[i];
    } 
}

void Model::updateJacobian(const mjModel* m, mjData* d,const char* &body_name)
{
    mjtNum* jacp = new mjtNum[3*m->nv];
    mjtNum* jacr = new mjtNum[3*m->nv];
    
    int body_id = mj_name2id(m, mjOBJ_BODY, body_name);//panda0_link7 franka_gripper franka_gripper_center
    mj_jacBody(m,d,jacp,jacr,body_id); //mj_jacBodyCom

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < dof; ++j) {
          J(i,j) = jacp[i * m->nv  + j];
          J(i+3,j) = jacr[i * m->nv  + j];
      }
    }
}

void Model::updateEEPosition(const mjModel* m,mjData* d)
{
    int body_id = mj_name2id(m, mjOBJ_BODY, ee); // "link7" franka_gripper franka_gripper_center
    x = Eigen::Vector3d(d->xpos[body_id*3+0],d->xpos[body_id*3+1],d->xpos[body_id*3+2]);
}

void Model::updateEEVelocity(void)
{
    dx = J*dq;
}

void Model::updateEEOrientation(const mjModel* m,mjData* d,const char* &body_name)
{
    int body_id = mj_name2id(m, mjOBJ_BODY, body_name); // franka_gripper panda0_link7 franka_gripper_center
    
    quat.w() = d->xquat[body_id*4+0];
    quat.x() = d->xquat[body_id*4+1];
    quat.y() = d->xquat[body_id*4+2];
    quat.z() = d->xquat[body_id*4+3];
}

void Model::setJointTorques(mjData* d,Eigen::MatrixXd &tau)
{
    for(int i=0;i<dof;i++)
    {
      d->ctrl[i] = tau(i);
    }
}

void Model::CloseGripper(mjData* d,int &val)
{
  d->ctrl[7] = val; // 30
}

void Model::OpenGripper(mjData* d)
{
  d->ctrl[7] = 255;
}

void Model::setEETargetPosition(Eigen::Vector3d x_des)
{
    xd << x_des[0],x_des[1],x_des[2]+offset;
}

void Model::setEETargetOrientation(Eigen::Quaterniond rd)
{
    quatd.w() = rd.w();
    quatd.x() = rd.x();
    quatd.y() = rd.y();
    quatd.z() = rd.z();
}

void Model::setJointPosition(mjData* d,Eigen::MatrixXd q0)
{
    for(int i=0;i<dof;i++)
    {
        d->qpos[i] = q0(i);
    } 
}

Model::Model()
{
    // initialize variables
    
    M = Eigen::Matrix<double,7,7>::Zero(7,7);
    L = Eigen::Matrix<double,6,6>::Zero(6,6);
    Jv = Eigen::Matrix<double,3,7>::Zero(3,7);
    J = Eigen::Matrix<double,6,7>::Zero(6,7);
    Jbar = Eigen::Matrix<double,7,6>::Zero(7,6);
    Nbar = Eigen::Matrix<double,7,7>::Zero(7,7);
    g = Eigen::Matrix<double,7,1>::Zero(7,1);
    q = Eigen::Matrix<double,7,1>::Zero(7,1);
    dq = Eigen::Matrix<double,7,1>::Zero(7,1);
    quat = Eigen::Quaterniond(1,0,0,0);
    quatd = Eigen::Quaterniond(1,0,0,0);
    x = Eigen::Matrix<double,3,1>::Zero(3,1);
    dx = Eigen::Matrix<double,6,1>::Zero(6,1);
    xd = Eigen::Vector3d::Zero(3);
    qd << 0.0, -0.5, 0.0, -2.0, 0.0, 1.7, 0.5;

} 







/*
      // Allocate memory for the Jacobian
  mjtNum* jacp = new mjtNum[3 * m->nv];
  mjtNum* jacr = new mjtNum[3 * m->nv];

  // Get the ID of the body
  int body_id = mj_name2id(m, mjOBJ_BODY, "link7");

  // Compute the Jacobian
  mj_jacBody(m, d, jacp, jacr, body_id);

  // Print the position part of the Jacobian
  std::cout << "Position Jacobian:" << std::endl;
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < m->nv; ++j) {
          std::cout << jacp[i * m->nv + j] << " ";
      }
      std::cout << std::endl;
  }

  // Print the rotation part of the Jacobian
  std::cout << "Rotation Jacobian:" << std::endl;
  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < m->nv; ++j) {
          std::cout << jacr[i * m->nv + j] << " ";
      }
      std::cout << std::endl;
      */

