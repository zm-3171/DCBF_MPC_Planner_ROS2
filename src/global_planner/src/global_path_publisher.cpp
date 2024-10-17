#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class GlobalPathPublisher : public rclcpp::Node
{
public:
    GlobalPathPublisher() : Node("global_path_publisher")
    {
        global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 1);
        this->declare_parameter<double>("start_x", -10.0);
        this->declare_parameter<double>("start_y", 0.0);
        this->declare_parameter<double>("end_x", 9.0);
        this->declare_parameter<double>("end_y", 0.0);

        start_pos_ = Vector2d(this->get_parameter("start_x").get_value<double>(), this->get_parameter("start_y").get_value<double>());
        target_pos_ = Vector2d(this->get_parameter("end_x").get_value<double>(), this->get_parameter("end_y").get_value<double>());

        dist_ = (target_pos_ - start_pos_).norm();
        diff_ = (target_pos_ - start_pos_) / dist_;

        RCLCPP_INFO(this->get_logger(), "start from (%f, %f) to (%f, %f)", start_pos_.x(), start_pos_.y(), target_pos_.x(), target_pos_.y());
        RCLCPP_INFO(this->get_logger(), "step size = (%f, %f)", diff_.x(), diff_.y());

        step_ = 0.1;
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&GlobalPathPublisher::timerCallback, this));
    }

private:
    void timerCallback()
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

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    Vector2d start_pos_;
    Vector2d target_pos_;
    Vector2d diff_;
    double step_;
    double dist_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPathPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}