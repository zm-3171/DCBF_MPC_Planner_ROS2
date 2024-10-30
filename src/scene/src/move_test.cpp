#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "moving_cylinder.hpp"

int MovingCylinder::id_ = 0;

using namespace std::chrono_literals;

class MovingCylinderNode : public rclcpp::Node
{
public:
    MovingCylinderNode() : Node("move_test")
    {
        initCylinders();

        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "cmd_move", 10, std::bind(&MovingCylinderNode::cmdCallback, this, std::placeholders::_1));
        client_ = this->create_client<gazebo_msgs::srv::SetEntityState>("/gazebo/set_entity_state");
        auto duration = std::chrono::seconds(interval);
        timer_ = this->create_wall_timer(duration, std::bind(&MovingCylinderNode::updateCylinders, this));

        RCLCPP_INFO(this->get_logger(), "Moving test init successfully");
    }

private:
    void cmdCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        is_move_ = msg->data;
    }

    void initCylinders()
    {
        cylinders_.clear();

        int cylinder_num = 0;
        this->declare_parameter<int>("cylinder_num", 0);
        this->get_parameter<int>("cylinder_num", cylinder_num);

        for (int i = 0; i < cylinder_num; i++)
        {
            double x, y, z;
            double vx, vy;

            this->declare_parameter<double>("x_" + std::to_string(i), 0.0);
            this->get_parameter<double>("x_" + std::to_string(i), x);

            this->declare_parameter<double>("y_" + std::to_string(i), 0.0);
            this->get_parameter<double>("y_" + std::to_string(i), y);

            this->declare_parameter<double>("z_" + std::to_string(i), 0.0);
            this->get_parameter<double>("z_" + std::to_string(i), z);

            this->declare_parameter<double>("vx_" + std::to_string(i), 0.0);
            this->get_parameter<double>("vx_" + std::to_string(i), vx);

            this->declare_parameter<double>("vy_" + std::to_string(i), 0.0);
            this->get_parameter<double>("vy_" + std::to_string(i), vy);

            auto cylinder_i = std::make_shared<MovingCylinder>();
            cylinder_i->setPosition(x, y, z);
            cylinder_i->setVel(vx, vy);
            cylinders_.push_back(cylinder_i);
        }
        RCLCPP_INFO(this->get_logger(), "total cylinder num = %d", int(cylinders_.size()));
    }

    void updateCylinders()
    {
        static int cnt = 0;
        for(auto &cylinder: cylinders_)
        {
            for(int i = 0; i < interval; ++i)
                cylinder->updateState();
        }
        if(cnt == change_interval)
        {
            cnt = 0;
            for(auto &cylinder: cylinders_)
                cylinder->reverseVel();
        }

        std::vector<std::shared_future<std::shared_ptr<gazebo_msgs::srv::SetEntityState::Response>>> futures;

        for (auto &cylinder : cylinders_)
        {
            auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
            request->state.name = cylinder->model_name;
            request->state.pose.position.x = cylinder->position_x;
            request->state.pose.position.y = cylinder->position_y;
            request->state.pose.position.z = cylinder->position_z;
            request->state.twist.linear.x = cylinder->vel_x;  
            request->state.twist.linear.y = cylinder->vel_y;

            // 设置姿态（orientation），假设圆柱体没有旋转，使用单位四元数
            request->state.pose.orientation.x = 0.0;
            request->state.pose.orientation.y = 0.0;
            request->state.pose.orientation.z = 0.0;
            request->state.pose.orientation.w = 1.0;

            // 异步发送服务请求，并将 future 存储到 futures 向量中
            if (client_->service_is_ready())
            {
                auto future_result = client_->async_send_request(request);
                futures.push_back(future_result); // 保存 future 对象
            }
        }

        // 等待所有的服务请求完成
        for (auto &future : futures)
        {
            if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
            {
                auto result = future.get();
            }
        }

        cnt ++;
    }

    std::vector<std::shared_ptr<MovingCylinder>> cylinders_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    int interval = 1;
    int change_interval = 30;
    bool is_move_ = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MovingCylinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}