#include "kdl_control.h"
#include "utils.h"
KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{

}


Eigen::VectorXd KDLController::velocityCtrlNull(const Eigen::Vector3d &pos_error,
                                                const Eigen::MatrixXd &J, // 6 x n
                                                const Eigen::VectorXd &q)  // n x 1
{
    const double EPS = 1e-8;
    int n = q.size();

    // 1) pseudoinverse of J (n x 6)
    Eigen::MatrixXd J_pinv = pseudoinverse(J);

    // 2) build cartesian error as 6x1 (linear + zero angular if needed)
    Eigen::VectorXd cart_err(6);
    cart_err.setZero();
    cart_err.segment<3>(0) = pos_error; // linear error
    // leave angular error as zero or pass it as argument if available

    // 3) task velocity (q_task = J^+ * Kp * cart_err)
    Eigen::VectorXd q_task = J_pinv * (Kp_ * cart_err); // q_task is n x 1

    // 4) compute q0dot = -dU/dq (component-wise)  (this is the correct derivative)
    Eigen::MatrixXd jnt_limits = robot_->getJntLimits(); // n x 2: col0=min, col1=max
    Eigen::VectorXd q0dot(n);
    q0dot.setZero();

    for (int i = 0; i < n; ++i) {
        double qi = q(i);
        double qmin = jnt_limits(i,0);
        double qmax = jnt_limits(i,1);
        double A = (qmax - qmin) * (qmax - qmin);

        double d1 = qmax - qi;
        double d2 = qi - qmin;

        // protection
        if (d1 < EPS) d1 = EPS;
        if (d2 < EPS) d2 = EPS;

        double denom = (d1 * d2);
        double denom_sq = denom * denom; // ((qmax-qi)(qi-qmin))^2
        double numer = (d1 - d2); // = (qmax-qi) - (qi-qmin) = qmax + qmin - 2*qi

        // derivative of U_i: dU/dq = - (A / lambda) * numer / denom_sq
        // but q0dot = - dU/dq -> q0dot = + (A / lambda) * numer / denom_sq
        q0dot(i) = (A / lambda_) * numer / denom_sq;
    }

    // optional: saturate q0dot per-component to avoid huge values
    const double MAX_Q0 = 1.0; // rad/s or suitable limit
    for (int i = 0; i < n; ++i) {
        if (std::abs(q0dot(i)) > MAX_Q0) q0dot(i) = std::copysign(MAX_Q0, q0dot(i));
    }

    // 5) null-space projection: (I - J^+ J) * q0dot
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
    Eigen::VectorXd q_ns = (I - J_pinv * J) * q0dot;

    // 6) final joint velocity
    Eigen::VectorXd q_dot = q_task + q_ns;

    return q_dot;
}



