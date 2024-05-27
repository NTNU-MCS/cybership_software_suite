#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class NEDToENUPublisher : public rclcpp::Node
{


private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ned_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr enu_pub_;

    std::string frame_id_;

    void ned_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Transform the pose from NED to ENU frame
        geometry_msgs::msg::PoseWithCovarianceStamped enu_msg;
        enu_msg.header = msg->header;

        enu_msg.header.frame_id = frame_id_;

        // Position transformation
        enu_msg.pose.pose.position.x = msg->pose.pose.position.y;
        enu_msg.pose.pose.position.y = msg->pose.pose.position.x;
        enu_msg.pose.pose.position.z = -msg->pose.pose.position.z;

        // Orientation transformation (quaternion)
        tf2::Quaternion q_ned(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);


        tf2::Matrix3x3 m(q_ned);
        double yaw, pitch, roll;
        m.getRPY(yaw, pitch, roll);

        tf2::Quaternion q_enu;

        q_enu.setRPY(roll, pitch, -yaw + M_PI_2);

        enu_msg.pose.pose.orientation.x = q_enu.x();
        enu_msg.pose.pose.orientation.y = q_enu.y();
        enu_msg.pose.pose.orientation.z = q_enu.z();
        enu_msg.pose.pose.orientation.w = q_enu.w();

        // Copy the covariance matrix as it is (assuming it remains the same)
        enu_msg.pose.covariance = msg->pose.covariance;

        // Publish the transformed message
        enu_pub_->publish(enu_msg);
    }

public:
    NEDToENUPublisher()
        : Node("ned_to_enu_publisher")
    {
        // // Subscriber to the NED frame PoseWithCovarianceStamped messages
        ned_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "in_pose", 10, std::bind(&NEDToENUPublisher::ned_callback, this, std::placeholders::_1));

        // Publisher for the transformed ENU frame PoseWithCovarianceStamped messages
        enu_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("out_pose", 10);

        // Read `frame_id` parameter
        this->declare_parameter<std::string>("frame_id", "world");
        this->get_parameter("frame_id", frame_id_);
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NEDToENUPublisher>());
    rclcpp::shutdown();
    return 0;
}