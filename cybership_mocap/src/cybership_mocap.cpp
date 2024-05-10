#include "cybership_mocap/cybership_mocap.hpp"

CybershipMocap::CybershipMocap() : Node("cybership_mocap")
{

    m_config.declare(std::move(*this));

    m_config.update(std::move(*this));

    m_mocap_sub = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
        "rigid_bodies", 10, std::bind(&CybershipMocap::f_mocapCallback, this, std::placeholders::_1));

    m_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(m_config.topic_name, 10);

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


}

void CybershipMocap::f_mocapCallback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
{

    for (const auto &rb : msg->rigidbodies)
    {
        if(rb.rigid_body_name != m_config.rigid_body_name) {
            continue;
        }

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = m_config.world_frame;
        transform.child_frame_id = m_config.base_frame;
        transform.transform.translation.x = rb.pose.position.x;
        transform.transform.translation.y = rb.pose.position.y;
        transform.transform.translation.z = rb.pose.position.z;
        transform.transform.rotation.x = rb.pose.orientation.x;
        transform.transform.rotation.y = rb.pose.orientation.y;
        transform.transform.rotation.z = rb.pose.orientation.z;
        transform.transform.rotation.w = rb.pose.orientation.w;
        m_tf_broadcaster->sendTransform(transform);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = msg->header.stamp;
        pose.header.frame_id = m_config.world_frame;
        pose.pose = rb.pose;
        m_pose_pub->publish(pose);

    }
}

void CybershipMocap::Config::declare(CybershipMocap && node)
{
    if(!node.has_parameter("world_frame")) {
        node.declare_parameter("world_frame", world_frame);
    }
    if(!node.has_parameter("base_frame")) {
        node.declare_parameter("base_frame", base_frame);
    }
    if(!node.has_parameter("rigid_body_name")) {
        node.declare_parameter("rigid_body_name", rigid_body_name);
    }
    if(!node.has_parameter("topic")) {
        node.declare_parameter("topic", std::string("rigid_bodies"));
    }
}

void CybershipMocap::Config::update(CybershipMocap && node)
{
    node.get_parameter("world_frame", world_frame);
    node.get_parameter("base_frame", base_frame);
    node.get_parameter("rigid_body_name", rigid_body_name);
}