#pragma once

#include "chrono"
#include "functional"
#include "memory"
#include "string"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/wrench.hpp"

enum class ThrusterType {
    VOITH_SCHNEIDER,
    FIXED,
    AZIMUTH,
};

float linear_interpolate(float x, float x0, float x1, float y0, float y1);


class ThrusterBase {
protected:

    rclcpp::Node::SharedPtr m_node;

    std::string m_name;

public:
    ThrusterBase(rclcpp::Node::SharedPtr node, std::string name) : m_node(node), m_name(name) {}

};



class CybershipThruster : public rclcpp::Node {

private:

    std::vector<std::shared_ptr<ThrusterBase>> m_thrusters;

    void f_initialize();

public:
    CybershipThruster();

    void initialize();

};


class VoithSchneider : public ThrusterBase {

    struct Config {

        std::string name;

        std::string force_topic;

        std::string arm_x_topic;

        std::string arm_y_topic;

        std::string rpm_topic;

        float force_max;

        float force_min;

        float rpm_cmd;

        bool arm_x_inverted;

        bool arm_y_inverted;

        void declare(rclcpp::Node::SharedPtr node);

        void update(rclcpp::Node::SharedPtr node);
    };

    bool m_enabled = false;

    Config m_config;

    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr m_wrench_sub;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_arm_x_pub;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_arm_y_pub;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_rpm_pub;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_enable_service;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_disable_service;

    void f_enable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                           std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void f_disable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                            std::shared_ptr<std_srvs::srv::Empty::Response> response);

    void f_force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);

public:
    VoithSchneider(rclcpp::Node::SharedPtr node, std::string thruster_name);

};

class Fixed : public ThrusterBase {

    struct Config {

        std::string name;

        std::string force_topic_name;

        void declare(rclcpp::Node::SharedPtr node);

        void update(rclcpp::Node::SharedPtr node);
    };

    Config m_config;

    void f_force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);

public:
    Fixed(rclcpp::Node::SharedPtr node, std::string thruster_name);

};

class Azimuth : public ThrusterBase {

    struct Config {
        std::string name;

        std::string force_topic_name;

        void declare(rclcpp::Node::SharedPtr node);

        void update(rclcpp::Node::SharedPtr  node);
    };

    Config m_config;

    void f_force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);

public:
    Azimuth(rclcpp::Node::SharedPtr node, std::string thruster_name);

};