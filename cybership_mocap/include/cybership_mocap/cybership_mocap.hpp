#pragma once

#include "chrono"
#include "functional"
#include "memory"
#include "string"

#include "rclcpp/rclcpp.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class CybershipMocap : public rclcpp::Node {
private:

    struct Config {
        std::string world_frame;

        std::string base_frame;

        std::string rigid_body_name;

        std::string topic_name;

        void declare(CybershipMocap && node);

        void update(CybershipMocap && node);
    };

    Config m_config;

    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr m_mocap_sub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_pub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    void f_mocapCallback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);

public:
    CybershipMocap();


};