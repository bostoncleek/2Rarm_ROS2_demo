/**
 * @file arm_node.cpp
 * @author Boston Cleek 
 * @brief 2R arm 
*/
// PUBLISHES: joint_states (JointState) - theta1 and theta2 corresponding to the two joint angles

#define _USE_MATH_DEFINES
#include <chrono>                               
#include <iostream>
#include <functional>
#include <memory>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "simple_arm/msg/joint_state.hpp"

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
        joint_pub_ = this->create_publisher<simple_arm::msg::JointState>("joint_states", 1);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("eef_marker", 1);
        timer_ = this->create_wall_timer(100ms, std::bind(&SimpleArm::jointCallback, this));

        l1_ = this->declare_parameter<double>("L1", 1.0);
        l2_ = this->declare_parameter<double>("L2", 1.0);
        period_ = this->declare_parameter<double>("T", 10.0);

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

        simple_arm::msg::JointState joint_msg;
        joint_msg.j1 = joints.at(0);
        joint_msg.j2 = joints.at(1);
        joint_pub_->publish(joint_msg);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "END_EFFECTOR";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.lifetime = rclcpp::Duration(period_/5.0);

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

        t_ += dt_;

        if (t_ > period_ || std::fabs(t_ - period_) < 1e-12)
        {
            t_ = 0.0;
        }
    }

    std::vector<double> endEffector()
    {
        const auto x = 0.9 * std::cos(2.0 * M_PI * t_ / period_) * std::sqrt(std::pow(l1_ + l2_, 2) - std::pow(h_,2));
        const auto y = 2.0/3.0 * (l1_ + l2_);
        return { x, y };
    }

    std::vector<double> jointAngles(double x, double y)
    {
        const auto theta2 = std::acos((std::pow(x, 2) + std::pow(y, 2) - std::pow(l2_, 2) - std::pow(l2_, 2)) / (2.0 * l1_ * l2_));
        const auto theta1 = std::atan2(y,x) - std::atan2(l2_ * std::sin(theta2), l1_ + l2_ * std::cos(theta2));
        return { theta1, theta2 };
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<simple_arm::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    double l1_, l2_, period_, h_, t_, dt_;
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(simple_arm::SimpleArm)