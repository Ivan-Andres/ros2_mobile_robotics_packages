#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
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
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        last_time_ = this->now();
        timer_ = this->create_wall_timer(50ms, std::bind(&ManualOdometryNode::updatePose, this));

        publishStaticTransform();
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

        double dx = kv_ * u * std::cos(theta_ + beta_) * dt;
        double dy = kv1_ * u * std::sin(theta_ + beta2_) * dt;
        double dtheta = (u * std::tan(u2 * kdelta_) / L_) * dt;

        x_ += dx;
        y_ += dy;
        theta_ += dtheta;
        normalizeAngle(theta_);

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

        rosgraph_msgs::msg::Clock clk;
        clk.clock = current_time;
        clock_pub_->publish(clk);
    }

    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = this->now();
        static_tf.header.frame_id = "base_link";
        static_tf.child_frame_id = "laser_scan";

        static_tf.transform.translation.x = 0.0;
        static_tf.transform.translation.y = 0.0;
        static_tf.transform.translation.z = 0.1;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        static_tf.transform.rotation.x = q.x();
        static_tf.transform.rotation.y = q.y();
        static_tf.transform.rotation.z = q.z();
        static_tf.transform.rotation.w = q.w();

        static_broadcaster_->sendTransform(static_tf);
    }

    void normalizeAngle(double &angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
    }

    double x_, y_, theta_;
    double kv_, kv1_, kdelta_, beta_, beta2_, L_;

    geometry_msgs::msg::Twist last_cmd_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
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
