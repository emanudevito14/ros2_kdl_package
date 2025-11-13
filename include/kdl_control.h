#ifndef KDLControl
#define KDLControl

#include "Eigen/Dense"
#include <Eigen/SVD>
#include "kdl_robot.h"
#include "utils.h"

class KDLController
{

public:

    KDLController(KDLRobot &_robot);

    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp,
                           double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp,
                           double _Kpo,
                           double _Kdp,
                           double _Kdo);
    Eigen::VectorXd velocityCtrlNull(const Eigen::Vector3d &pos_error,
                                                const Eigen::MatrixXd &J, 
                                                const Eigen::VectorXd &q);  

private:

    KDLRobot* robot_;
    double Kp_ = 1.0;     
    double lambda_ = 0.1; 

};

#endif
