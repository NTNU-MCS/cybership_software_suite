#include "cybership_thrusters/cybership_thrusters.hpp"

VoithSchneider::VoithSchneider(rclcpp::Node::SharedPtr node, std::string name) : ThrusterBase(node, name)
{
    m_config.name = name;


    m_config.declare(m_node);
    m_config.update(m_node);

    std::cout << "topic names are " << m_config.force_topic << " " << m_config.arm_x_topic << " " << m_config.arm_y_topic << " " << m_config.rpm_topic << std::endl;

    m_wrench_sub = m_node->create_subscription<geometry_msgs::msg::Wrench>(
        m_config.force_topic, 10,
        std::bind(&VoithSchneider::f_force_callback, this, std::placeholders::_1)
    );

    m_arm_x_pub = m_node->create_publisher<std_msgs::msg::Float32>(m_config.arm_x_topic, 10);

    m_arm_y_pub = m_node->create_publisher<std_msgs::msg::Float32>(m_config.arm_y_topic, 10);

    m_rpm_pub = m_node->create_publisher<std_msgs::msg::Float32>(m_config.rpm_topic, 10);

    m_enable_service = m_node->create_service<std_srvs::srv::Empty>("enable",
        std::bind(&VoithSchneider::f_enable_callback, this, std::placeholders::_1, std::placeholders::_2));

    m_disable_service = m_node->create_service<std_srvs::srv::Empty>("disable",
        std::bind(&VoithSchneider::f_disable_callback, this, std::placeholders::_1, std::placeholders::_2));

}

void VoithSchneider::f_enable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                               std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void) request;
    (void) response;
    m_enabled = true;
}

void VoithSchneider::f_disable_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                                                std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void) request;
    (void) response;
    m_enabled = false;
}

void VoithSchneider::f_force_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    if (!m_enabled)
    {
        std_msgs::msg::Float32 zero_msg;
        zero_msg.data = 0.0;
        m_arm_x_pub->publish(zero_msg);
        m_arm_y_pub->publish(zero_msg);
        m_rpm_pub->publish(zero_msg);
    }


    auto filtered_input = *msg;

    if (filtered_input.force.x > m_config.force_max) {
        filtered_input.force.x = m_config.force_max;
    } else if (filtered_input.force.x < m_config.force_min) {
        filtered_input.force.x = m_config.force_min;
    }

    if (filtered_input.force.y > m_config.force_max) {
        filtered_input.force.y = m_config.force_max;
    } else if (filtered_input.force.y < m_config.force_min) {
        filtered_input.force.y = m_config.force_min;
    }

    std_msgs::msg::Float32 arm_x_msg, arm_y_msg, rpm_msg;

    auto linear_interpolate = [] (float x, float x0, float x1, float y0, float y1) -> float {
        return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    };

    arm_x_msg.data = m_config.arm_x_inverted ? -filtered_input.force.x : filtered_input.force.x;
    arm_y_msg.data = m_config.arm_y_inverted ? -filtered_input.force.y : filtered_input.force.y;

    if (arm_x_msg.data > 0) {
        arm_x_msg.data = linear_interpolate(arm_x_msg.data, 0, m_config.force_max, 0, 1);
    } else {
        arm_x_msg.data = linear_interpolate(arm_x_msg.data, 0, m_config.force_min, 0, -1);
    }

    if (arm_y_msg.data > 0) {
        arm_y_msg.data = linear_interpolate(arm_y_msg.data, 0, m_config.force_max, 0, 1);
    } else {
        arm_y_msg.data = linear_interpolate(arm_y_msg.data, 0, m_config.force_min, 0, -1);
    }

    rpm_msg.data = m_config.rpm_cmd;

    m_arm_x_pub->publish(arm_x_msg);
    m_arm_y_pub->publish(arm_y_msg);
    m_rpm_pub->publish(rpm_msg);



}

void VoithSchneider::Config::declare(rclcpp::Node::SharedPtr  node)
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

void VoithSchneider::Config::update(rclcpp::Node::SharedPtr node)
{
    node->get_parameter("thrusters." + name + ".force_topic", force_topic);
    node->get_parameter("thrusters." + name + ".arm_x.topic", arm_x_topic);
    node->get_parameter("thrusters." + name + ".arm_x_inverted", arm_x_inverted);
    node->get_parameter("thrusters." + name + ".arm_y_inverted", arm_y_inverted);
    node->get_parameter("thrusters." + name + ".arm_y.topic", arm_y_topic);
    node->get_parameter("thrusters." + name + ".force_max", force_max);
    node->get_parameter("thrusters." + name + ".force_min", force_min);
    node->get_parameter("thrusters." + name + ".rpm.topic", rpm_topic);
    node->get_parameter("thrusters." + name + ".rpm.cmd", rpm_cmd);
}