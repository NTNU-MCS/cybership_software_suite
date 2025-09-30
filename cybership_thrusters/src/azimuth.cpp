#include "cybership_thrusters/cybership_thrusters.hpp"
#include "cmath"


Azimuth::Azimuth(rclcpp::Node::SharedPtr node, std::string name) : ThrusterBase(node, name)
{
    m_config.name = name;


    m_config.declare(m_node);
    m_config.update(m_node);

    m_wrench_sub = m_node->create_subscription<geometry_msgs::msg::Wrench>(
        m_config.force_topic, 1,
        std::bind(&Azimuth::f_force_callback, this, std::placeholders::_1)
    );

    m_angle_pub = m_node->create_publisher<std_msgs::msg::Float32>(m_config.angle_topic, 1);

    m_rpm_pub = m_node->create_publisher<std_msgs::msg::Float32>(m_config.rpm_topic, 1);

    m_enable_service = m_node->create_service<std_srvs::srv::Empty>("thruster/enable",
        std::bind(&Azimuth::f_enable_callback, this, std::placeholders::_1, std::placeholders::_2));

    m_disable_service = m_node->create_service<std_srvs::srv::Empty>("thruster/disable",
        std::bind(&Azimuth::f_disable_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Safety watchdog setup
    m_node->get_parameter_or<double>("thrusters." + m_config.name + ".safety_timeout", m_safety_timeout_sec, 2.0);
    m_last_cmd_time = m_node->now();
    m_watchdog_timer = m_node->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Azimuth::f_watchdog_check, this));

}

void Azimuth::f_enable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                               std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void) request;
    (void) response;
    m_enabled = true;
}

void Azimuth::f_disable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void) request;
    (void) response;
    m_enabled = false;
    f_publish_zero();
}

void Azimuth::f_force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    m_last_cmd_time = m_node->now();
    if (!m_enabled)
    {
        std_msgs::msg::Float32 zero_msg;
        zero_msg.data = 0.0;
        m_angle_pub->publish(zero_msg);
        m_rpm_pub->publish(zero_msg);
        return;
    }


    auto filtered_input = *msg;

    std_msgs::msg::Float32 angle_msg, rpm_msg;

    auto angle = std::atan2(msg->force.y, msg->force.x);
    auto rpm = std::sqrt(std::pow(msg->force.y,2) + std::pow(msg->force.x,2));

    if (m_config.angle_inverted) {
        angle = -angle;
    }

    angle_msg.data = angle;
    rpm_msg.data = rpm;

    m_rpm_pub->publish(rpm_msg);
    if (rpm == 0) {
        return;
    }
    m_angle_pub->publish(angle_msg);

}

void Azimuth::f_watchdog_check()
{
    auto now = m_node->now();
    auto elapsed = (now - m_last_cmd_time).seconds();
    if (elapsed > m_safety_timeout_sec) {
        f_publish_zero();
    }
}

void Azimuth::f_publish_zero()
{
    std_msgs::msg::Float32 zero_msg;
    zero_msg.data = 0.0f;
    m_angle_pub->publish(zero_msg);
    m_rpm_pub->publish(zero_msg);
}

void Azimuth::Config::declare(rclcpp::Node::SharedPtr  node)
{
    // node->declare_parameter("thrusters." + name +".force_topic", rclcpp::PARAMETER_STRING);
    // node->declare_parameter("thrusters." + name +".force_max", rclcpp::PARAMETER_DOUBLE);
    // node->declare_parameter("thrusters." + name +".force_min", rclcpp::PARAMETER_DOUBLE);
    // node->declare_parameter("thrusters." + name +".arm_x.topic", rclcpp::PARAMETER_STRING);
    // node->declare_parameter("thrusters." + name +".arm_y.topic", rclcpp::PARAMETER_STRING);
    // node->declare_parameter("thrusters." + name +".arm_x.inverted", rclcpp::PARAMETER_BOOL);
    // node->declare_parameter("thrusters." + name +".arm_y.inverted", rclcpp::PARAMETER_BOOL);
    // node->declare_parameter("thrusters." + name +".rpm.topic", rclcpp::PARAMETER_STRING);
    // node->declare_parameter("thrusters." + name +".rpm.cmd", rclcpp::PARAMETER_DOUBLE);
}

void Azimuth::Config::update(rclcpp::Node::SharedPtr node)
{
    node->get_parameter_or<std::string>("thrusters." + name + ".force_topic", force_topic, "");
    node->get_parameter_or<std::string>("thrusters." + name + ".angle.topic", angle_topic, "");
    node->get_parameter_or<bool>("thrusters." + name + ".angle.inverted", angle_inverted, false);
    node->get_parameter_or<std::string>("thrusters." + name + ".rpm.topic", rpm_topic, "");
    node->get_parameter_or<bool>("thrusters." + name + ".rpm.inverted", rpm_inverted, false);
    node->get_parameter_or<float>("thrusters." + name + ".force_max", force_max, 0.0);
    node->get_parameter_or<float>("thrusters." + name + ".force_min", force_min, 0.0);
}