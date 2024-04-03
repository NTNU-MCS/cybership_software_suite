#include "cybership_thrusters/cybership_thrusters.hpp"

Azimuth::Azimuth(rclcpp::Node::SharedPtr node, std::string name) : ThrusterBase(node, name)
{
    m_config.name = name;
    m_config.declare(m_node);

    m_config.update(m_node);

}

void Azimuth::f_force_callback(geometry_msgs::msg::Wrench::SharedPtr msg)
{

}

void Azimuth::Config::declare(rclcpp::Node::SharedPtr node)
{
    node->declare_parameter("force_topic_name", force_topic_name);
}

void Azimuth::Config::update(rclcpp::Node::SharedPtr node)
{
    node->get_parameter("force_topic_name", force_topic_name);
}