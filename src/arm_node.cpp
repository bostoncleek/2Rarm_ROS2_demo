/**
 * @file arm_node.cpp
 * @author Boston Cleek 
 * @brief 2R arm 
*/
// PUBLISHES: joint_states (JointState) - theta1 and theta2 corresponding to the two joint angles
// IK derivation: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/

#define _USE_MATH_DEFINES
#include <chrono>                               
#include <iostream>
#include <functional>
#include <memory>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("2R_arm_demo");


namespace simple_arm
{
class SimpleArm : public rclcpp::Node
{
public:
    SimpleArm(const rclcpp::NodeOptions& options)
     : Node("simple_arm", options)
     , h_(2.0/3.0 * (l1_+l2_))
     , t_(0.0)
     , dt_(0.01)
    {
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("eef_marker", 10);
        timer_ = create_wall_timer(10ms, std::bind(&SimpleArm::jointCallback, this));

        l1_ = declare_parameter<double>("L1", 1.0);
        l2_ = declare_parameter<double>("L2", 1.0);
        period_ = declare_parameter<double>("T", 10.0);

        RCLCPP_INFO_STREAM(LOGGER, l1_);
        RCLCPP_INFO_STREAM(LOGGER, l2_);
        RCLCPP_INFO_STREAM(LOGGER, period_);
        RCLCPP_INFO(LOGGER, "simple_arm node initialized");
    }

private:
    void jointCallback()
    {
        const auto eef_position = endEffector();
        const auto joints = jointAngles(eef_position.at(0), eef_position.at(1));

        sensor_msgs::msg::JointState joint_msg;
        joint_msg.header.stamp = now();
        joint_msg.name = {"joint1", "joint2", "end"};
        joint_msg.position = {joints.at(0), joints.at(1), 0.0};
        joint_pub_->publish(joint_msg);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "End_Effector";
        marker.header.stamp = now();
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.lifetime = rclcpp::Duration(period_/10.0);

        if (eef_position.at(0) > 0.0)
        {
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
        }
        else 
        {
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.color.r = 1.0;
            marker.color.a = 1.0;  
        }

        marker_pub_->publish(marker);
        
        // RCLCPP_INFO_STREAM(LOGGER, t_);
        // std::cout << "EEF(x,y): " << eef_position.at(0) << " " << eef_position.at(1) << std::endl;
        // std::cout << "Joints(j1,j2): " << joints.at(0) << " " << joints.at(1) << std::endl;

        t_ += dt_;

        if (t_ > period_ || std::fabs(t_ - period_) < 1e-12)
        {
            t_ = 0.0;
        }
    }

    std::vector<double> endEffector()
    {
        const auto x = 0.9 * std::cos(2.0 * M_PI * t_ / period_) * std::sqrt(std::pow(l1_ + l2_, 2) - h_*h_);
        const auto y = 2.0/3.0 * (l1_ + l2_);
        return { x, y };
    }

    std::vector<double> jointAngles(double x, double y)
    {
        const auto arg = std::clamp((x*x + y*y - l2_*l2_ - l1_*l1_) / (2.0 * l1_ * l2_), -1.0, 1.0);
        const auto theta2 = std::acos( arg );
        // std::cout << "arg: " << arg << " acos(arg): " << theta2 << std::endl;
        const auto theta1 = std::atan2(y,x) - std::atan2( l2_ * std::sin(theta2), l1_ + l2_ * std::cos(theta2) );
        return { theta1, theta2 };
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    double l1_, l2_, period_, h_, t_, dt_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(simple_arm::SimpleArm)