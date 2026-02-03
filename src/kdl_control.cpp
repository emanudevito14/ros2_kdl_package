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


// In kdl_control.cpp

Eigen::VectorXd KDLController::visionCtrl(const Eigen::Vector3d &p_o,
                                          const Eigen::MatrixXd &J_spatial,
                                          const Eigen::Matrix3d &R_base_cam,
                                          const Eigen::VectorXd &q)
{
    // Constants
    double gain = 1.5; // Matrix K in equation (3) 
    int n = q.size();
    
    // 1. Calculate s (current feature vector)
    // s = cPo / ||cPo||  [cite: 40]
    double norm_po = p_o.norm();
    if(norm_po < 1e-6) {
        // Avoid division by zero if object is too close or invalid
        return Eigen::VectorXd::Zero(n);
    }
    Eigen::Vector3d s = p_o / norm_po;

    // 2. Define desired feature s_d = [0, 0, 1]^T 
    Eigen::Vector3d s_d(0.0, 0.0, 1.0);

    // 3. Compute Interaction Matrix L(s)
    // L(s) = [ 1/||P|| * (I - s*s^T)   skew(s) ]  
    Eigen::MatrixXd L_cam(3, 6);
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    
    // Position part: 1/depth * (I - s*s')
    L_cam.block(0, 0, 3, 3) = (1.0 / norm_po) * (I - s * s.transpose());
    // Orientation part: -skew(s)
    L_cam.block(0, 3, 3, 3) = skew(s);

    // 4. Transform Jacobian to Camera Frame
    // The robot computes J in the spatial frame. We need it in the camera frame.
    // The PDF mentions R[cite: 47]. Effectively: V_cam = [R^T 0; 0 R^T] * V_spatial
    //Eigen::MatrixXd R_block = Eigen::MatrixXd::Zero(6, 6);
    //R_block.block(0, 0, 3, 3) = R_base_cam.transpose();
    //R_block.block(3, 3, 3, 3) = R_base_cam.transpose();

    // J_c = [R^T 0; 0 R^T] * J_spatial
    //Eigen::MatrixXd J_c = R_block * J_spatial;

    // 5. Compute Task Jacobian: J_task = L(s) * J_c
    Eigen::MatrixXd J_task = L_cam * J_spatial;

    // 6. Compute Pseudoinverse of Task Jacobian
    Eigen::MatrixXd J_task_pinv = pseudoinverse(J_task);

    // 7. Secondary Task: Joint Limit Avoidance (q0_dot)
    // Formula (2) [cite: 18]
    Eigen::MatrixXd jnt_limits = robot_->getJntLimits();
    Eigen::VectorXd q0_dot(n);
    q0_dot.setZero();
    const double EPS = 1e-8;

    for (int i = 0; i < n; ++i) {
        double qi = q(i);
        double qmin = jnt_limits(i, 0);
        double qmax = jnt_limits(i, 1);
        double num = (qmax - qmin) * (qmax - qmin);
        double den1 = (qmax - qi);
        double den2 = (qi - qmin);
        
        if (std::abs(den1) < EPS) den1 = (den1 >= 0) ? EPS : -EPS;
        if (std::abs(den2) < EPS) den2 = (den2 >= 0) ? EPS : -EPS;

        double term = num / (den1 * den2);
        // Gradient of the cost function U wrt q: dU/dq. q0 = -dU/dq
        // Using the exact formula derived in velocityCtrlNull:
        // derivative of U: (A/lambda) * (qmax+qmin - 2q) / (den1*den2)^2
        // We implement the repulsive velocity directly:
        double numer_deriv = (qmax - qmin) * (2*qi - qmax - qmin); // Derivative of numerator part of cost
        // Note: Re-using the logic from your velocityCtrlNull implementation is best here
        // Assuming implementation matches velocityCtrlNull logic:
        double A = pow(qmax - qmin, 2);
        double denom_sq = pow(den1 * den2, 2);
        double numer_diff = (den1 - den2); 
        q0_dot(i) = (A / lambda_) * numer_diff / denom_sq; 
    }

    // 8. Null space projector N = (I - J_task_pinv * J_task) 
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n, n) - (J_task_pinv * J_task);

    // 9. Control Law
    // q_dot = K * (L J)^+ * s_d + N * q0_dot  
    // Note: Standard IBVS typically uses error (s_d - s).
    // The PDF explicitly writes "s_d" in Eq (3), but implies a "look-at-point task".
    // To ensure convergence to the center, we use the error vector (s_d - s).
    // If strictly following the formula as written without error term, the robot might drift.
    // We assume s_d acts as the driving error term towards the desired optical axis.
    
    Eigen::VectorXd error = s-s_d ; 
    
    // We calculate the command
    Eigen::VectorXd q_dot = gain * J_task_pinv * error + N * q0_dot;

    return q_dot;
}



