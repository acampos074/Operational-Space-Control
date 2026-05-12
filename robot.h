#ifndef ROBOT_H
#define ROBOT_H

#include <mujoco/mujoco.h>
//#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

class Model
{
    public:
        // constructor / desctructor
        Model();
        void updateMassMatrix(const mjModel* m, mjData* d);
        void updateEEMassMatrix(void);
        void updateInverseJacobianMatrix(void);
        void updateNullspaceMatrix(void);
        void updateJacobian(const mjModel* m, mjData* d,const char* &body_name);
        void updateGravityVector(mjData* d);
        void updateJointPosition(mjData* d);
        void updateJointVelocity(mjData* d);
        void updateEEPosition(const mjModel* m,mjData* d);
        void updateEEVelocity(void);
        void updateEEOrientation(const mjModel* m,mjData* d,const char* &body_name);
        void setJointTorques(mjData* d,Eigen::MatrixXd &tau);
        void CloseGripper(mjData* d,int &val);
        void OpenGripper(mjData* d);
        void setEETargetPosition(Eigen::Vector3d x_des);
        void setEETargetOrientation(Eigen::Quaterniond rd);
        void setJointPosition(mjData* d,Eigen::MatrixXd q0);
        double nu; // velocity limiting coefficient
        double kpx = 200; // proportional gain for position
        double kdx = 200; // derivative gain for position
        double Vmax = 0.3; // maximum velocity of end-effector
        const char* ee = "hand"; // left_finger hand
        
        Eigen::Matrix<double,7,7,Eigen::DontAlign> M;
        Eigen::Matrix<double,6,6,Eigen::DontAlign> L;
        Eigen::Matrix<double,3,7,Eigen::DontAlign> Jv;
        Eigen::Matrix<double,6,7,Eigen::DontAlign> J;
        Eigen::Matrix<double,7,6,Eigen::DontAlign> Jbar;
        Eigen::Matrix<double,7,7,Eigen::DontAlign> Nbar;
        Eigen::Matrix<double,7,1,Eigen::DontAlign> g;
        Eigen::Matrix<double,7,1,Eigen::DontAlign> q;
        Eigen::Matrix<double,7,1,Eigen::DontAlign> dq;
        Eigen::Quaterniond quat;
        Eigen::Quaterniond quatd;
        Eigen::Matrix<double,3,1,Eigen::DontAlign> x;
        Eigen::Matrix<double,6,1,Eigen::DontAlign> dx;
        Eigen::Vector3d xd;
        Eigen::Matrix<double,7,1,Eigen::DontAlign> qd;

    private:
        int dof = 7; // number of joints
        double offset = 0.0584; // offset between gripper and end-effector
};

#endif