#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomToTFNode : public rclcpp::Node
{
public:
    OdomToTFNode() : Node("odom_to_tf_node")
    {
        // Create a subscriber to the /base_pose_ground_truth topic
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/base_pose_ground_truth", 10,
            std::bind(&OdomToTFNode::odometryCallback, this, std::placeholders::_1));

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        // Extract relevant information from the Odometry message
        geometry_msgs::msg::Pose pose = odom_msg->pose.pose;
        std::string frame_id = odom_msg->header.frame_id;
        std::string child_frame_id = odom_msg->child_frame_id;

        // Convert the Pose to a TransformStamped message
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = odom_msg->header.stamp;
        transform_stamped.header.frame_id = frame_id;
        transform_stamped.child_frame_id = child_frame_id;
        transform_stamped.transform.translation.x = pose.position.x;
        transform_stamped.transform.translation.y = pose.position.y;
        transform_stamped.transform.translation.z = pose.position.z;
        transform_stamped.transform.rotation = pose.orientation;

        // Broadcast the Transform
        tf_broadcaster_->sendTransform(transform_stamped);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomToTFNode>();
    rclcpp::spin(node);
    return 0;
}