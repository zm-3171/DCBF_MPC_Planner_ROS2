#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <eigen3/Eigen/Dense>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace Eigen;

class GlobalPathPublisher : public rclcpp::Node
{
public:
    GlobalPathPublisher() : Node("global_path_publisher")
    {
        destination_set = false;

        base_link = "base_link";

        step_ = 0.1;

        tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1);
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 1, std::bind(&GlobalPathPublisher::goalpose_cb, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GlobalPathPublisher::timerCallback, this));
    }

private:
    void timerCallback()
    {
        if (destination_set == true)
        {
            nav_msgs::msg::Path global_path;
            global_path.header.stamp = this->now();
            global_path.header.frame_id = "world";

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose.orientation.x = 0;
            pose_stamped.pose.orientation.y = 0;
            pose_stamped.pose.orientation.z = 0;
            pose_stamped.pose.orientation.w = 1;

            for (double i = 0.0; i < dist_; i += step_)
            {
                Vector2d waypoint = start_pos_ + i * diff_;
                pose_stamped.pose.position.x = waypoint.x();
                pose_stamped.pose.position.y = waypoint.y();
                pose_stamped.pose.position.z = 0;

                global_path.poses.push_back(pose_stamped);
            }

            global_path_pub_->publish(global_path);
        }
    }

    void goalpose_cb(const geometry_msgs::msg::PoseStamped msg)
    {
        target_pos_.x() = msg.pose.position.x;
        target_pos_.y() = msg.pose.position.y;

        getpose();

        dist_ = (target_pos_ - start_pos_).norm();
        diff_ = (target_pos_ - start_pos_) / dist_;
        destination_set = true;
    }

    void getpose()
    {
        while (true)
        {
            try
            {
                geometry_msgs::msg::TransformStamped base_link_transformStamped =
                    tf_buffer->lookupTransform("world", base_link, rclcpp::Time(0));
                start_pos_.x() = base_link_transformStamped.transform.translation.x;
                start_pos_.y() = base_link_transformStamped.transform.translation.y;

                RCLCPP_INFO(this->get_logger(), "Transform successfully received.");
                return; 
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
                rclcpp::Rate(1).sleep();
            }
        }
    }

    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::string base_link;

    Vector2d start_pos_;
    Vector2d target_pos_;
    Vector2d diff_;
    bool destination_set;
    double step_;
    double dist_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}