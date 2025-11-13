#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "ros2_kdl_package/action/linear_trajectory.hpp"
#include "ros2_kdl_package/msg/position_error.hpp"

using LinearTrajectory = ros2_kdl_package::action::LinearTrajectory;
using namespace std::chrono_literals;

class KDLActionClient : public rclcpp::Node
{
public:
    using LinearTrajectory = ros2_kdl_package::action::LinearTrajectory;
    using GoalHandleLinTraj = rclcpp_action::ClientGoalHandle<LinearTrajectory>;

    explicit KDLActionClient(const rclcpp::NodeOptions & options)
    : Node("kdl_action_client",options)
    {
      this->client_ptr_ = rclcpp_action::create_client<LinearTrajectory>(
      this,
      "linear_trajectory");  

      this->send_goal();

    }
    void send_goal()
    {
        using namespace std::placeholders;

        if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        }

        auto goal_msg = LinearTrajectory::Goal();
        goal_msg.s_type = "trapezoidal";
        goal_msg.traj_duration = 1.5;
        goal_msg.acc_duration = 0.5;
        goal_msg.total_time= 1.5;
        goal_msg.trajectory_len= 150;
        goal_msg.end_pos[0]= 0.4;
        goal_msg.end_pos[1]= -0.3;
        goal_msg.end_pos[2]= 0.6;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<LinearTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&KDLActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
        std::bind(&KDLActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
        std::bind(&KDLActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
    rclcpp_action::Client<LinearTrajectory>::SharedPtr client_ptr_;

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
    rclcpp::shutdown();
  }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KDLActionClient>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
    
}

