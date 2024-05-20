#include "cybership_thrusters/cybership_thrusters.hpp"

Fixed::Fixed(rclcpp::Node::SharedPtr node, std::string name) : ThrusterBase(node, name)
{
    m_config.name = name;


    m_config.declare(m_node);
    m_config.update(m_node);


    m_wrench_sub = m_node->create_subscription<geometry_msgs::msg::Wrench>(
        m_config.force_topic, 1,
        std::bind(&Fixed::f_force_callback, this, std::placeholders::_1)
    );

    m_signal_pub = m_node->create_publisher<std_msgs::msg::Float32>(m_config.signal_topic, 1);

    m_enable_service = m_node->create_service<std_srvs::srv::Empty>("thruster/enable",
        std::bind(&Fixed::f_enable_callback, this, std::placeholders::_1, std::placeholders::_2));

    m_disable_service = m_node->create_service<std_srvs::srv::Empty>("thruster/disable",
        std::bind(&Fixed::f_disable_callback, this, std::placeholders::_1, std::placeholders::_2));

}

void Fixed::f_enable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                               std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void) request;
    (void) response;
    m_enabled = true;
}

void Fixed::f_disable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void) request;
    (void) response;
    m_enabled = false;
}

void Fixed::f_force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    if (!m_enabled)
    {
        std_msgs::msg::Float32 zero_msg;
        zero_msg.data = 0.0;
        m_signal_pub->publish(zero_msg);
        return;
    }


    auto filtered_input = *msg;

    if (filtered_input.force.x > m_config.force_max) {
        filtered_input.force.x = m_config.force_max;
    } else if (filtered_input.force.x < m_config.force_min) {
        filtered_input.force.x = m_config.force_min;
    }

    std_msgs::msg::Float32 signal_msg;

    auto linear_interpolate = [] (float x, float x0, float x1, float y0, float y1) -> float {
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    };

    signal_msg.data = m_config.signal_inverted ? -filtered_input.force.x : filtered_input.force.x;

    if (signal_msg.data > 0) {
        signal_msg.data = linear_interpolate(signal_msg.data, 0, m_config.force_max, 0, 1);
    } else {
        signal_msg.data = linear_interpolate(signal_msg.data, 0, m_config.force_min, 0, -1);
    }

    m_signal_pub->publish(signal_msg);



}

void Fixed::Config::declare(rclcpp::Node::SharedPtr  node)
{

}

void Fixed::Config::update(rclcpp::Node::SharedPtr node)
{
    node->get_parameter("thrusters." + name + ".force_topic", force_topic);
    node->get_parameter("thrusters." + name + ".force_max", force_max);
    node->get_parameter("thrusters." + name + ".force_min", force_min);
    node->get_parameter("thrusters." + name + ".signal.topic", signal_topic);
    node->get_parameter("thrusters." + name + ".signal.inverted", signal_inverted);
}