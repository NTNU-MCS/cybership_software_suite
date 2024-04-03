#include "cybership_thrusters/cybership_thrusters.hpp"

Fixed::Fixed(rclcpp::Node::SharedPtr node, std::string name) : ThrusterBase(node, name)
{

    m_config.name = name;
    m_config.declare(m_node);
    m_config.update(m_node);

}

void Fixed::f_force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{

}

void Fixed::Config::declare(rclcpp::Node::SharedPtr node)
{
    node->declare_parameter(name + ".force_topic", force_topic_name);
}

void Fixed::Config::update(rclcpp::Node::SharedPtr node)
{
    node->get_parameter(name + "force_topic", force_topic_name);
}