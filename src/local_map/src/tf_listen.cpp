#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"

class TfListenerNode : public rclcpp::Node
{
public:
    TfListenerNode() : Node("tf_listener_node")
    {
        tf_buffer_ = std::make_shared<tf2::Buffer>();
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Specify the target frame and time in the future to look for a transform
        target_frame_ = "base_link";                                       // Change this to the frame you're interested in
        future_time_ = rclcpp::Time(this->now()) + rclcpp::Duration(2, 0); // 2 seconds in the future

        // Using a timer to check for transforms periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&TfListenerNode::checkTransform, this));
    }

private:
    void checkTransform()
    {
        try
        {
            auto transform = tf_buffer_->lookupTransform("map", target_frame_, future_time_);

            RCLCPP_INFO(this->get_logger(), "Received transform from %s to %s",
                        transform.header.frame_id.c_str(),
                        target_frame_.c_str());
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_INFO(this->get_logger(), "No transform available: %s", ex.what());
        }
    }

    std::shared_ptr<tf2::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::string target_frame_;
    rclcpp::Time future_time_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}