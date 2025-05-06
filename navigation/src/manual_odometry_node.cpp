#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"  // NUEVO
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rosgraph_msgs/msg/clock.hpp"

using namespace std::chrono_literals;

class ManualOdometryNode : public rclcpp::Node
{
public:
    ManualOdometryNode()
    : Node("manual_odometry_node"),
      x_(0.0), y_(0.0), theta_(0.0),
      kv_(2.4), kv1_(2.4), kdelta_(0.5), beta_(0.001), beta2_(0.2), L_(0.32)
    {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ManualOdometryNode::cmdVelCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // NUEVO: crear publisher para joint_states
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Publisher de reloj simulado
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        last_time_ = this->now();
        timer_ = this->create_wall_timer(50ms, std::bind(&ManualOdometryNode::updatePose, this));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_ = *msg;
    }

    void updatePose()
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double u = last_cmd_.linear.x;
        double u2 = last_cmd_.angular.z;

        // Ecuaciones del modelo
        double dx = kv_ * u * std::cos(theta_ + beta_) * dt;
        double dy = kv1_ * u * std::sin(theta_ + beta2_) * dt;
        double dtheta = (u * std::tan(u2 * kdelta_) / L_) * dt;

        x_ += dx;
        y_ += dy;
        theta_ += dtheta;
        normalizeAngle(theta_);

        // NUEVO: publicar joint_states
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = current_time;

        static double wheel_pos = 0.0; // Simula rotación acumulada de la rueda
        wheel_pos += u * dt;           // Si tienes radio real: wheel_pos += (u * dt) / r;

        joint_state.name = {
            "front_left_wheel_joint", "front_right_wheel_joint",
            "back_left_wheel_joint", "back_right_wheel_joint",
            "front_left_mount_joint", "front_right_mount_joint",
            "back_left_mount_joint", "back_right_mount_joint"  // Añadir los dos faltantes
        };

        joint_state.position = {
            wheel_pos, wheel_pos,  // Posición de las ruedas
            wheel_pos, wheel_pos,  // Posición de las ruedas
            u2, u2,               // Posición de los joints de dirección
            0.0, 0.0              // Añadir los dos joints faltantes con posición 0
        };

        joint_state_pub_->publish(joint_state);

        // Publicar la transformada tf de odom -> base_link
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);

        // Publicar reloj simulado
        rosgraph_msgs::msg::Clock clk;
        clk.clock = current_time;
        clock_pub_->publish(clk);
    }

    void normalizeAngle(double &angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
    }

    // Variables de estado
    double x_, y_, theta_;
    double kv_, kv1_, kdelta_, beta_, beta2_, L_;

    geometry_msgs::msg::Twist last_cmd_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // NUEVO:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManualOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
