#include <chrono>
#include <cmath>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "ros_gz_interfaces/msg/entity.hpp" 
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;


geometry_msgs::msg::Quaternion toQuaternion(double roll, double pitch, double yaw) {
    double cr = cos(roll * 0.5); double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5); double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5); double sy = sin(yaw * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

int get_key() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int ch = getchar();
    if (ch == 27) { getchar(); ch = getchar(); }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("aruco_teleop_advanced");
    
    
    auto client = node->create_client<ros_gz_interfaces::srv::SetEntityPose>(
        "/world/aruco_stereo_world/set_pose");

  
    double x = -0.3, y = 0.0, z = 0.4;
    double roll = -1.57, pitch = 0.0, yaw = -1.57;
    
    double pos_step = 0.02; 
    double rot_step = 0.05; 

    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) return 0;
        std::cout << "Wating service set_entity_pose..." << std::endl;
    }

    std::cout << "Teleop Aruco actived! Using arrow for moving along X/Y, U/D for moving along Z, R/T/F/G/H/J for rotations." << std::endl;

    while (rclcpp::ok()) {
        int key = get_key();
        bool update = true;

        switch (key) {
            case 'A': x += pos_step; break;
            case 'B': x -= pos_step; break;
            case 'D': y += pos_step; break;
            case 'C': y -= pos_step; break;
            case 'u': z += pos_step; break;
            case 'd': z -= pos_step; break;
            case 'r': roll += rot_step; break;
            case 't': roll -= rot_step; break;
            case 'f': pitch += rot_step; break;
            case 'g': pitch -= rot_step; break;
            case 'h': yaw += rot_step; break;
            case 'j': yaw -= rot_step; break;
            case 'q': return 0;
            default: update = false; break;
        }

        if (update) {
            auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
            
            
            request->entity.name = "aruco_tag";
            request->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
            
            request->pose.position.x = x;
            request->pose.position.y = y;
            request->pose.position.z = z;
            request->pose.orientation = toQuaternion(roll, pitch, yaw);

            client->async_send_request(request);

            std::cout << "\r\033[K" << std::fixed << std::setprecision(2)
                      << "X:" << x << " Y:" << y << " Z:" << z 
                      << " | R:" << roll << " P:" << pitch << " Y:" << yaw << std::flush;
        }
    }
    rclcpp::shutdown();
    return 0;
}
