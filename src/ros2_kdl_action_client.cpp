#include <functional>
#include <stdio.h>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/bool.hpp"

#include "ros2_kdl_package/action/linear_trajectory.hpp"
#include "ros2_kdl_package/action/vision.hpp"
#include "ros2_kdl_package/msg/position_error.hpp"

//using LinearTrajectory = ros2_kdl_package::action::LinearTrajectory;
using namespace std::chrono_literals;

class KDLActionClient : public rclcpp::Node
{
public:
    using LinearTrajectory = ros2_kdl_package::action::LinearTrajectory;
    using GoalHandleLinTraj = rclcpp_action::ClientGoalHandle<LinearTrajectory>;

    using Vision =ros2_kdl_package::action::Vision;
    using GoalHandleVision= rclcpp_action::ClientGoalHandle<Vision>;

    explicit KDLActionClient(const rclcpp::NodeOptions & options)
    : Node("kdl_action_client",options)
    {
      
      declare_parameter("ctrl", "velocity_ctrl");
      get_parameter("ctrl", ctrl_);
      RCLCPP_INFO(get_logger(),"Selected velocity controller: '%s'", ctrl_.c_str());
      if (!(ctrl_ == "velocity_ctrl" || ctrl_ == "velocity_ctrl_null"|| ctrl_=="vision_ctrl")) {
          RCLCPP_ERROR(get_logger(), "Invalid ctrl param! Use 'velocity_ctrl', 'velocity_ctrl_null' or 'vision_ctrl'");
          return;
      }
      if(ctrl_=="velocity_ctrl"|| ctrl_=="velocity_ctrl_null"){
          this->client1_ptr_ = rclcpp_action::create_client<LinearTrajectory>(
          this,
          "linear_trajectory");  

          declare_parameter("x", 0.4);
          declare_parameter("y", -0.3);
          declare_parameter("z", 0.6);

          last_x_=-99.0; last_y_=-99.0; last_z_=-99.0;

      }
      else{
        this->client2_ptr_=rclcpp_action::create_client<Vision>(this,"vision");
      }
      
      
      

      
      this->send_goal();

    }
    void send_goal()
    {
        using namespace std::placeholders;

        if(ctrl_=="velocity_ctrl"|| ctrl_=="velocity_ctrl_null"){
              double current_x = this->get_parameter("x").as_double();
              double current_y = this->get_parameter("y").as_double();
              double current_z = this->get_parameter("z").as_double();

              if (current_x == last_x_ && current_y == last_y_ && current_z == last_z_) {
                  RCLCPP_INFO(this->get_logger(), "Coordinate invariate [%.2f, %.2f, %.2f]. In attesa di modifiche...", 
                              current_x, current_y, current_z);
                  return;
              }

              if (!this->client1_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
              }
              
              
              auto goal_msg = LinearTrajectory::Goal();
              goal_msg.s_type = "trapezoidal";
              goal_msg.traj_duration = 1.5;
              goal_msg.acc_duration = 0.5;
              goal_msg.total_time= 1.5;
              goal_msg.trajectory_len= 150;
              goal_msg.end_pos[0]= current_x;
              goal_msg.end_pos[1]= current_y;
              goal_msg.end_pos[2]= current_z;

            



              RCLCPP_INFO(this->get_logger(), "Sending goal");

              auto send_goal_options = rclcpp_action::Client<LinearTrajectory>::SendGoalOptions();
              send_goal_options.goal_response_callback =
              std::bind(&KDLActionClient::goal_response_callback, this, _1);
              send_goal_options.feedback_callback =
              std::bind(&KDLActionClient::feedback_callback, this, _1, _2);
              send_goal_options.result_callback =
              std::bind(&KDLActionClient::result_callback, this, _1);
              this->client1_ptr_->async_send_goal(goal_msg, send_goal_options);
        }
        else{
          
          if (!this->client2_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
              }
          auto goal_msg = Vision::Goal();
          goal_msg.ctrl = ctrl_;

          RCLCPP_INFO(this->get_logger(), "Sending goal");

          auto send_goal_options = rclcpp_action::Client<Vision>::SendGoalOptions();
          send_goal_options.goal_response_callback =
          std::bind(&KDLActionClient::goal_response_vision, this, _1);
          send_goal_options.feedback_callback =
          std::bind(&KDLActionClient::feedback_vision, this, _1, _2);
          send_goal_options.result_callback =
          std::bind(&KDLActionClient::result_vision, this, _1);
          this->client2_ptr_->async_send_goal(goal_msg, send_goal_options);

        }

        
  }

private:
    rclcpp_action::Client<LinearTrajectory>::SharedPtr client1_ptr_;
    rclcpp_action::Client<Vision>::SharedPtr client2_ptr_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finish_sub_;
    double last_x_, last_y_, last_z_;
    std::string ctrl_;
   
    

  void goal_response_callback(const GoalHandleLinTraj::SharedPtr & goal_handle)
    {
     if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
     } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
     }
  }

  void feedback_callback(
    GoalHandleLinTraj::SharedPtr,
    const std::shared_ptr<const LinearTrajectory::Feedback> feedback)
   { 
        std::stringstream ss;
        ss << "Next error position in sequence received: ";
        for (auto number : feedback->err_pos) {
        ss << "(" << number.x << ", " << number.y << ", " << number.z << ") ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
   }

  void result_callback(const GoalHandleLinTraj::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    
    ss << "("
   << result.result->err_pos_final.x << ", "
   << result.result->err_pos_final.y << ", "
   << result.result->err_pos_final.z << ")";

    
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    
  }

  void goal_response_vision(const GoalHandleVision::SharedPtr & goal_handle){
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_vision(
    GoalHandleVision::SharedPtr,
    const std::shared_ptr<const Vision::Feedback> feedback){
        const std::vector<double> & velocities = feedback->velocity_command_f;

        std::stringstream ss;
        ss << "Command velocity: [ ";
        for (const auto & v : velocities) {
            ss << std::fixed << std::setprecision(4) << v << " ";
        }
        ss << "]";
        
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

  }

  void result_vision(const GoalHandleVision::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    
    const std::vector<double> & velocities_final = result.result->velocity_command;
    std::stringstream ss;
    ss << "Command velocity: [ ";
    for (const auto & v : velocities_final) {
        ss << std::fixed << std::setprecision(4) << v << " ";
    }
    ss << "]";
        
    
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    
  }


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KDLActionClient>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
    
}

