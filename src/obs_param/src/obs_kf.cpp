#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <eigen3/Eigen/Dense>

#include "kalman.h"

using namespace std;
using namespace Eigen;

const size_t obs_kf_buffer_size = 100;

// 椭圆定义
struct Ellipse
{
    double semimajor; // Length of semi-major axis
    double semiminor; // Length of semi-minor axis
    double cx;        // x-coordinate of center
    double cy;        // y-coordinate of center
    double theta;     // Rotation angle
};

// 障碍物定义
struct obs_param
{
    float x = 0.0;
    float y = 0.0;
    float a = 0.0;
    float b = 0.0;
    float theta = 0.0;
    float mea_cov;
};

// 增加一个未使用次数计数，长时间不使用则重置
class obs_kf
{
public:
    int N;
    int lastTimeUsed;
    float T;

    Kalman ka;
    std::vector<obs_param> param_list;
    std::vector<obs_param> pred_list;
    Eigen::VectorXd x;
    Eigen::MatrixXd P0;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd H;
    Eigen::VectorXd z0;
    Eigen::VectorXd u_v;

    void obs_predict();
    obs_kf();
    void reset();
} _obs_kf[obs_kf_buffer_size];

obs_kf::obs_kf() : N(25)
{
    ka.m_StateSize = 9;
    ka.m_MeaSize = 5;
    ka.m_USize = 9;

    lastTimeUsed = 0;

    T = 0.1;

    x.resize(9);
    x.setZero();

    float P0_cov = 0.025;

    P0.resize(9, 9);
    P0.setIdentity();
    P0 *= P0_cov;

    float Q_cov = 0.00008;

    Q.resize(9, 9);
    Q.setIdentity();
    Q *= Q_cov;

    float R_cov = 0.000003; // 仿真 0.00005  实物  0.000003

    R.resize(5, 5);
    R.setZero();

    R(0, 0) = R_cov;
    R(1, 1) = R_cov;
    R(2, 2) = 10 * R_cov;
    R(3, 3) = 10 * R_cov;
    R(4, 4) = 0.00000001 * R_cov;

    A.resize(9, 9);
    A.setIdentity();
    A(0, 5) = A(1, 6) = T;

    B.resize(9, 9);
    B.setZero();

    H.resize(5, 9);
    H.setIdentity();

    z0.resize(5);
    z0.setZero();

    u_v.resize(9);
    u_v.setZero();

    ka.Init_Par(x, P0, R, Q, A, B, H, u_v);
}

void obs_kf::reset()
{
    ka.m_StateSize = 9;
    ka.m_MeaSize = 5;
    ka.m_USize = 9;

    lastTimeUsed = 0;

    param_list.clear();
    pred_list.clear();

    T = 0.1;

    x.resize(9);
    x.setZero();

    float P0_cov = 0.025;

    P0.resize(9, 9);
    P0.setIdentity();
    P0 *= P0_cov;

    float Q_cov = 0.00008;

    Q.resize(9, 9);
    Q.setIdentity();
    Q *= Q_cov;

    float R_cov = 0.000003; // 仿真 0.00005  实物  0.000003

    R.resize(5, 5);
    R.setZero();

    R(0, 0) = R_cov;
    R(1, 1) = R_cov;
    R(2, 2) = 10 * R_cov;
    R(3, 3) = 10 * R_cov;
    R(4, 4) = 0.00000001 * R_cov;

    A.resize(9, 9);
    A.setIdentity();
    A(0, 5) = A(1, 6) = T;

    B.resize(9, 9);
    B.setZero();

    H.resize(5, 9);
    H.setIdentity();

    z0.resize(5);
    z0.setZero();

    u_v.resize(9);
    u_v.setZero();

    ka.Init_Par(x, P0, R, Q, A, B, H, u_v);
}

void obs_kf::obs_predict()
{
    int num = param_list.size();
    if (num == 0)
        return;

    if (num == 1)
    {
        ka.m_x << param_list[0].x, param_list[0].y, param_list[0].a, param_list[0].b, param_list[0].theta, 0, 0, 0, 0;
        ka.m_P = P0;
    }
    else
    {
        float _mea_cov_min = 0.005;
        float _mea_cov_max = 0.05;
        float _k_cov_min = 1;
        float _k_cov_max = 3000;
        float _mea_cov = min(max(param_list.back().mea_cov, _mea_cov_min), _mea_cov_max);
        float _k = log10(_mea_cov / _mea_cov_min) / log10(_mea_cov_max / _mea_cov_min);
        float _k_cov = pow(_k_cov_max, _k) * pow(_k_cov_min, 1 - _k);

        ka.m_R(0, 0) = _k_cov * R(0, 0);
        ka.m_R(1, 1) = _k_cov * R(1, 1);

        VectorXd z(5);
        z << param_list.back().x, param_list.back().y, param_list.back().a, param_list.back().b, param_list.back().theta;
        param_list.erase(param_list.begin());

        ka.Predict_State();
        ka.Predict_Cov();
        ka.Mea_Resd(z);
        ka.Cal_Gain();
        ka.Update_State();
        ka.Update_Cov();

        Kalman ka_tmp = ka;

        pred_list.clear();
        for (int i = 0; i < N; i++)
        {
            ka_tmp.Predict_State();
            ka_tmp.Predict_Cov();
            obs_param pred;
            float x_conf = 0.95 * sqrt(ka_tmp.m_P(0, 0));
            float y_conf = 0.95 * sqrt(ka_tmp.m_P(1, 1));
            float a_conf = 0.95 * sqrt(ka_tmp.m_P(2, 2));
            float b_conf = 0.95 * sqrt(ka_tmp.m_P(3, 3));
            float pos_conf = max(x_conf, y_conf);
            float ab_conf = max(a_conf, b_conf);

            pred.x = ka_tmp.m_x(0);
            pred.y = ka_tmp.m_x(1);
            pred.a = ka_tmp.m_x(2) + pos_conf + ab_conf;
            pred.b = ka_tmp.m_x(3) + pos_conf + ab_conf;
            pred.theta = ka.m_x(4);

            pred_list.push_back(pred);
        }
    }
}

class ObsKfNode : public rclcpp::Node
{
public:
    ObsKfNode() : Node("obs_param_node")
    {
        unused_clear_time = 3;
        obs_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/for_obs_track", 1, std::bind(&ObsKfNode::obscb, this, std::placeholders::_1));
        obs_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/obs_predict_pub", 1);
        obs_vis_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obs_predict_vis_pub", 1);
    }

private:
    void obscb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "[node] receive the obs track");
        if (msg->data.empty())
            return;

        int num = msg->data.size() / 7;
        RCLCPP_INFO(this->get_logger(), "total %d obstacle", num);

        std_msgs::msg::Float32MultiArray obs_pub; // 处理后的obstacles信息

        // visualization_msgs::msg::MarkerArray ellipses_array;
        vector<Ellipse> ellipses_array;

        for(int i = 0; i < obs_kf_buffer_size; ++i)
        {_obs_kf[i].lastTimeUsed ++;}

        for (int i = 0; i < num; i++)
        {
            int flag = msg->data[7 * i + 5] - 1;
            if(flag >= obs_kf_buffer_size || flag < 0)
                RCLCPP_INFO(this->get_logger(), "label overflow %d !!!!!", flag);

            obs_param _obs_tmp;
            _obs_tmp.x = msg->data[7 * i];
            _obs_tmp.y = msg->data[7 * i + 1];
            _obs_tmp.a = msg->data[7 * i + 2];
            _obs_tmp.b = msg->data[7 * i + 3];
            _obs_tmp.theta = msg->data[7 * i + 4];
            _obs_tmp.mea_cov = msg->data[7 * i + 6];

            if(_obs_kf[flag].lastTimeUsed > unused_clear_time)
                _obs_kf[flag].reset();
            else
                _obs_kf[flag].lastTimeUsed = 0;
            _obs_kf[flag].param_list.push_back(_obs_tmp);

            curve_fitting(_obs_kf[flag], obs_pub, ellipses_array);
        }

        obs_pub_->publish(obs_pub);
        visEllipse(ellipses_array);
    }

    void curve_fitting(obs_kf &obs, std_msgs::msg::Float32MultiArray &obs_pub, vector<Ellipse> &ellipses_array)
    {
        if (obs.param_list.empty())
            return;

        obs.obs_predict();

        for (int i = 0; i < obs.pred_list.size(); i++)
        {
            Ellipse ellipse;
            ellipse.cx = obs.pred_list[i].x;
            ellipse.cy = obs.pred_list[i].y;
            ellipse.semimajor = obs.pred_list[i].a;
            ellipse.semiminor = obs.pred_list[i].b;
            ellipse.theta = obs.pred_list[i].theta;
            ellipses_array.push_back(ellipse);

            obs_pub.data.push_back(obs.pred_list[i].x);
            obs_pub.data.push_back(obs.pred_list[i].y);
            obs_pub.data.push_back(obs.pred_list[i].a);
            obs_pub.data.push_back(obs.pred_list[i].b);
            obs_pub.data.push_back(obs.pred_list[i].theta);
        }
    }

    void visEllipse(const std::vector<Ellipse> &obs_ellipses)
    {
        visualization_msgs::msg::MarkerArray ellipse_vis;

        for (const auto &ellipse : obs_ellipses)
        {
            visualization_msgs::msg::Marker mk;
            mk.header.frame_id = "world";
            mk.header.stamp = this->now(); 
            mk.ns = "ellipse";
            mk.id = ellipse_vis.markers.size();
            mk.type = visualization_msgs::msg::Marker::CYLINDER;
            mk.action = visualization_msgs::msg::Marker::ADD;
            mk.lifetime = rclcpp::Duration::from_seconds(0.5); 

            mk.pose.position.x = ellipse.cx;
            mk.pose.position.y = ellipse.cy;
            mk.pose.position.z = -0.3;
            mk.pose.orientation.x = 0.0;
            mk.pose.orientation.y = 0.0;
            mk.pose.orientation.z = sin(ellipse.theta / 2);
            mk.pose.orientation.w = cos(ellipse.theta / 2);

            mk.scale.x = 2 * ellipse.semimajor;
            mk.scale.y = 2 * ellipse.semiminor;
            mk.scale.z = 0.7;

            mk.color.a = 0.4;
            mk.color.r = 0.0;
            mk.color.g = 1.0;
            mk.color.b = 0.0;

            ellipse_vis.markers.push_back(mk);
        }

        obs_vis_pub_->publish(ellipse_vis);
    }

    // Publishers and subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr obs_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr obs_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_vis_pub_;
    int unused_clear_time;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObsKfNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}