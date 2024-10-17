#ifndef MOVING_CYLINDER_H
#define MOVING_CYLINDER_H

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gazebo_msgs/srv/set_model_state.hpp>

class MovingCylinder
{
public:
    MovingCylinder()
    {
        model_name = "cylinder_" + std::to_string(id_);
        id_++;
    }

    void setPosition(double px, double py, double pz)
    {
        position_x = px;
        position_y = py;
        position_z = pz;
    }

    void setVel(double vx, double vy)
    {
        vel_x = vx;
        vel_y = vy;
    }

    void updateState()
    {
        position_x += vel_x;
        position_y += vel_y;
    }

    std::string model_name;
    double position_x, position_y, position_z;
    double vel_x, vel_y;

private:
    static int id_;
};

#endif