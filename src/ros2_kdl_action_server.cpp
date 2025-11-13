#include <memory>
#include <chrono>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "ros2_kdl_package/action/linear_trajectory.hpp"

#include <kdl_parser/kdl_parser.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "kdl_robot.h"
#include "kdl_planner.h"
#include "kdl_control.h"
#include "utils.h"

#include "ros2_kdl_package/msg/position_error.hpp"

using namespace std::chrono_literals;
using FloatArray = std_msgs::msg::Float64MultiArray;

class KDLActionServer : public rclcpp::Node
                        
{
 public:

    using LinearTrajectory = ros2_kdl_package::action::LinearTrajectory;
    using GoalHandleLinTraj = rclcpp_action::ServerGoalHandle<LinearTrajectory>;
    KDLActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("kdl_action_server",options),node_handle_(std::shared_ptr<KDLActionServer>(this))
    {
        

        declare_parameter("cmd_interface", "position"); // default to "position"
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
     
        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
        {
            RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
        }

        // declare traj_type parameter (linear, circular)
        declare_parameter("traj_type", "linear");
        get_parameter("traj_type", traj_type_);
        RCLCPP_INFO(get_logger(),"Current trajectory type is: '%s'", traj_type_.c_str());
        if (!(traj_type_ == "linear"))
        {
            RCLCPP_INFO(get_logger(),"Selected traj type is not linear"); return;
        }

        // declare s_type parameter (trapezoidal, cubic)
        declare_parameter("s_type", "trapezoidal");
        get_parameter("s_type", s_type_);
        RCLCPP_INFO(get_logger(),"Current s type is: '%s'", s_type_.c_str());
        if (!(s_type_ == "trapezoidal" || s_type_ == "cubic"))
        {
            RCLCPP_INFO(get_logger(),"Selected s type is not valid!"); return;
        }
        // Declare numeric trajectory parameters
        declare_parameter("traj_duration", 1.5);
        declare_parameter("acc_duration", 0.5);
        declare_parameter("total_time", 1.5);
        declare_parameter("trajectory_len", 150);
        declare_parameter("Kp", 5.0);

        // Declare end position parameter as array
        std::vector<double> end_pos_default = {0.0, 0.0, 0.0};
        declare_parameter("end_position", end_pos_default);

        // Retrieve parameters from YAML
        get_parameter("Kp", Kp_);

        iteration_ = 0; t_ = 0;
        joint_state_available_ = false;
        
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
        auto parameter = parameters_client->get_parameters({"robot_description"});
        // create KDLrobot structure
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
            std::cout << "Failed to retrieve robot_description param!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);  
        
        // Create joint array
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
        q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
        robot_->setJntLimits(q_min,q_max);            
        joint_positions_.resize(nj); 
        joint_velocities_.resize(nj); 
        joint_positions_cmd_.resize(nj); 
        joint_velocities_cmd_.resize(nj); 
        joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();

        // Subscriber to jnt states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&KDLActionServer::joint_state_subscriber, this, std::placeholders::_1));

        // Wait for the joint_state topic
        while(!joint_state_available_){
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(node_handle_);
        }

        // Update KDLrobot object
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

        // Compute EE frame
       init_cart_pose_ = robot_->getEEFrame();
        // std::cout << "The initial EE pose is: " << std::endl;  
        // std::cout << init_cart_pose_ <<std::endl;

        // Compute IK
       KDL::JntArray q(nj);
       robot_->getInverseKinematics(init_cart_pose_, q);
        // std::cout << "The inverse kinematics returned: " <<std::endl; 
        // std::cout << q.data <<std::endl;

        // Initialize controller
       controller_ = std::make_shared<KDLController>(*robot_);


       if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
            
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
       else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
       else if(cmd_interface_ == "effort"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                
            
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            } 

        this->action_server_ = rclcpp_action::create_server<LinearTrajectory>(
            this,
            "linear_trajectory",
            std::bind(&KDLActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&KDLActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&KDLActionServer::handle_accepted, this, std::placeholders::_1)
        );
      // Create msg and publish
      std_msgs::msg::Float64MultiArray cmd_msg;
      cmd_msg.data = desired_commands_;
      cmdPublisher_->publish(cmd_msg);

      RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
        
        
    }

 private:
    rclcpp_action::Server<LinearTrajectory>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const LinearTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received goal request:\n s_type= %s\n traj_duration= %f\n acc_duration= %f\n total_time= %f\n traj_len= %d\n"
            "end_pos_x = %f\n end_pos_y =%f\n end_pos_z=%f", 
            goal->s_type.c_str(),
            goal->traj_duration,
            goal->acc_duration,
            goal->total_time,
            goal->trajectory_len,
            goal->end_pos[0],
            goal->end_pos[1],
            goal->end_pos[2]
        );
        if (!(goal->s_type == "trapezoidal" || goal->s_type == "cubic"))
        {
            RCLCPP_WARN(get_logger(),"Selected s type is not valid!"); 
            return rclcpp_action::GoalResponse::REJECT;
        }
        if(!(goal->traj_duration > 0 && goal->acc_duration >0 && goal->total_time >0 && goal->trajectory_len>0))
        {
            RCLCPP_WARN(get_logger(),"traj_duratio,acc_duration,total_time,trajectory_len <=0"); 
            return rclcpp_action::GoalResponse::REJECT;
        }
        if(goal->end_pos.size()!=3){
            RCLCPP_WARN(get_logger(), "end_pos vector must have exactly 3 elements (got %zu)", goal->end_pos.size()); 
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLinTraj> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        (void) goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLinTraj> goal_handle)
    {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread(&KDLActionServer::execute, this, goal_handle).detach();

    }

    void execute(const std::shared_ptr<GoalHandleLinTraj> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(10);
        auto goal = goal_handle->get_goal();
        s_type_=goal->s_type;
        traj_duration_=goal->traj_duration;
        acc_duration_=goal->acc_duration;
        total_time_=goal->total_time;
        trajectory_len_=goal->trajectory_len;
        end_position_<<goal->end_pos[0],goal->end_pos[1],goal->end_pos[2];
        
        auto feedback = std::make_shared<LinearTrajectory::Feedback>();
        auto result = std::make_shared<LinearTrajectory::Result>();
        double dt = total_time_ / static_cast<double>(trajectory_len_);
        auto & error_position= feedback->err_pos;
        // EE's trajectory initial position (just an offset)
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));

        // EE's trajectory end position (just opposite y)
        Eigen::Vector3d end_position= end_position_; 

        // Plan trajectory
        

        // Retrieve the first trajectory point
        if(traj_type_ == "linear"){
            planner_ = KDLPlanner(traj_duration_, acc_duration_, init_position, end_position); // currently using trapezoidal velocity profile
            if(s_type_ == "trapezoidal")
            {
                p_ = planner_.linear_traj_trapezoidal(t_);
            }else if(s_type_ == "cubic")
            {
                p_ = planner_.linear_traj_cubic(t_);
            }
        } 

        t_+=dt;

        while(rclcpp::ok() && t_< total_time_){

            if (goal_handle->is_canceling()) {
              result->success = false;
              result->err_pos_final=errpos_f;
              goal_handle->canceled(result);
              RCLCPP_INFO(this->get_logger(), "Goal canceled");
              return;
            }


            iteration_ = iteration_ + 1;
            t_+=dt;
            if(traj_type_ == "linear"){
               if(s_type_ == "trapezoidal")
               {
                 p_ = planner_.linear_traj_trapezoidal(t_);
               }else if(s_type_ == "cubic")
                {
                  p_ = planner_.linear_traj_cubic(t_);
                }  
            }
         
         // Compute EE frame
            KDL::Frame cartpos = robot_->getEEFrame();           

            // Compute desired Frame
            KDL::Frame desFrame; desFrame.M = cartpos.M; desFrame.p = toKDL(p_.pos); 

            // compute errors
            Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
            std::cout << "The error norm is : " << error.norm() << std::endl;
            //publish feedback to action client
            errpos_f.x= error[0]; 
            errpos_f.y=error[1]; 
            errpos_f.z=error[2];
            error_position.push_back(errpos_f);
            goal_handle->publish_feedback(feedback);

            if(cmd_interface_ == "position"){
                // Next Frame
                KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp_*error))*dt; 

                // Compute IK
                joint_positions_cmd_ = joint_positions_;
                robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
            }
            else if (cmd_interface_ == "velocity")
            {
    
                // Compute differential IK inputs
                Vector6d cartvel; 
                cartvel << p_.vel + Kp_ * error, o_error;

                const Eigen::MatrixXd J = robot_->getEEJacobian().data;
                const Eigen::VectorXd q = joint_positions_.data;

                joint_velocities_cmd_.data = pseudoinverse(J) * cartvel;
                    
                
            }

            else if(cmd_interface_ == "effort"){
                joint_efforts_cmd_.data[0] = 0.1*std::sin(2*M_PI*t_/total_time_);
            }

            // Update KDLrobot structure
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

            if(cmd_interface_ == "position"){
                // Set joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Set joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                // Set joint effort commands
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            } 

            // Create msg and publish
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

             loop_rate.sleep();

        }

        // Check if goal is done
        if (rclcpp::ok()) {
         result->success= true;
         result->err_pos_final = errpos_f;
         goal_handle->succeed(result);
         RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }

        
    }
    void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){

        joint_state_available_ = true;
        for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
            joint_positions_.data[i] = sensor_msg.position[i];
            joint_velocities_.data[i] = sensor_msg.velocity[i];
        }
    }
    double traj_duration_;
    double acc_duration_;
    double total_time_;
    int trajectory_len_;
    double Kp_;
    Eigen::Vector3d end_position_;
    ros2_kdl_package::msg::PositionError errpos_f;

    std::shared_ptr<KDLController> controller_;


    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::TimerBase::SharedPtr subTimer_;
    rclcpp::Node::SharedPtr node_handle_;

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;

    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;

    trajectory_point p_;

    int iteration_;
    bool joint_state_available_;
    double t_;
    std::string cmd_interface_;
    std::string traj_type_;
    std::string s_type_;

    KDL::Frame init_cart_pose_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KDLActionServer>());
    rclcpp::shutdown();
    return 0;
}

