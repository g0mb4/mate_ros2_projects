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
        geometry_msgs::msg::TransformStamped map_transform;
        geometry_msgs::msg::TransformStamped odom_transform;
        geometry_msgs::msg::TransformStamped base_link_transform;
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

        // world -> map, world = map
        map_transform.header.stamp = now();
        map_transform.header.frame_id = "world";
        map_transform.child_frame_id = "map";

        m_tf_broadcaster->sendTransform(map_transform);

        // map -> odom, map = odom
        odom_transform.header.stamp = now();
        odom_transform.header.frame_id = "map";
        odom_transform.child_frame_id = "odom";

        m_tf_broadcaster->sendTransform(odom_transform);

        // odom -> base_link
        base_link_transform.header.stamp = now();
        base_link_transform.header.frame_id = "odom";
        base_link_transform.child_frame_id = "base_link";

        base_link_transform.transform.translation.x = msg->x;
        base_link_transform.transform.translation.y = msg->y;
        base_link_transform.transform.translation.z = 0.0;

        base_link_transform.transform.rotation.x = orientation.x();
        base_link_transform.transform.rotation.y = orientation.y();
        base_link_transform.transform.rotation.z = orientation.z();
        base_link_transform.transform.rotation.w = orientation.w();

        m_tf_broadcaster->sendTransform(base_link_transform);
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