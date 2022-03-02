#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>

class PoseTranslatorNode : public rclcpp::Node {
public:
    PoseTranslatorNode()
        : rclcpp::Node("pose_translator")
    {
        m_pose_subscriber = create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&PoseTranslatorNode::subscribe_pose, this, std::placeholders::_1));

        m_odom_publisher = create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
        m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void subscribe_pose(const turtlesim::msg::Pose::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;
        tf2::Quaternion orientation;
        orientation.setRPY(0, 0, msg->theta);

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.frame_id = "odom";
        odom_msg.header.stamp = now();

        odom_msg.pose.pose.position.x = msg->x;
        odom_msg.pose.pose.position.y = msg->y;

        odom_msg.pose.pose.orientation.x = orientation.x();
        odom_msg.pose.pose.orientation.y = orientation.y();
        odom_msg.pose.pose.orientation.z = orientation.z();
        odom_msg.pose.pose.orientation.w = orientation.w();

        odom_msg.twist.twist.linear.x = msg->linear_velocity;
        odom_msg.twist.twist.angular.z = msg->angular_velocity;

        m_odom_publisher->publish(odom_msg);

        t.header.stamp = now();
        t.header.frame_id = "world";
        t.child_frame_id = "odom";

        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = orientation.x();
        t.transform.rotation.y = orientation.y();
        t.transform.rotation.z = orientation.z();
        t.transform.rotation.w = orientation.w();

        m_tf_broadcaster->sendTransform(t);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_pose_subscriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_publisher;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PoseTranslatorNode>());

    rclcpp::shutdown();
    return 0;
}