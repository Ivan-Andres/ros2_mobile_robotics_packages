#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class ManualOdometryNode : public rclcpp::Node
{
public:
    ManualOdometryNode()
    : Node("manual_odometry_node"),
      x_(0.0), y_(0.0), theta_(0.0)
    {
        // Suscripción al /scan original
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&ManualOdometryNode::scanCallback, this, std::placeholders::_1));
        // Publisher para reenviar a /scan1
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan1", 10);
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/robot1/pose", 10,
            std::bind(&ManualOdometryNode::poseCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        timer_ = this->create_wall_timer(50ms, std::bind(&ManualOdometryNode::publishOdometry, this));
        
        
    }

private:
    // Callback de /scan: almacena el último mensaje recibido
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = *msg;
    }

    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        x_ = msg->position.x/1000;
        y_ = msg->position.y/1000;

        tf2::Quaternion q;
        tf2::fromMsg(msg->orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, theta_);
        orientation_ = msg->orientation;
    }

    void publishOdometry()
    {
        publishStaticTransform();
        rclcpp::Time now = this->now();

        // Publicar TF dinámico odom -> base_link
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = orientation_;
        tf_broadcaster_->sendTransform(tf_msg);

        // Publicar mensaje de odometría
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = orientation_;

        // Rellenar con ceros si no hay velocidad
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        odom_pub_->publish(odom_msg);

        // Publicar reloj
        rosgraph_msgs::msg::Clock clk;
        clk.clock = now;
        clock_pub_->publish(clk);

        if (!last_scan_.ranges.empty()) {
            // Actualizar timestamp para coherencia
            last_scan_.header.stamp = now;
            scan_pub_->publish(last_scan_);
        }
    }

    void publishStaticTransform()
    {
        geometry_msgs::msg::TransformStamped static_tf;
        static_tf.header.stamp = this->now();
        static_tf.header.frame_id = "base_link";
        static_tf.child_frame_id = "laser";

        static_tf.transform.translation.x = 0.0;
        static_tf.transform.translation.y = 0.0;
        static_tf.transform.translation.z = 0.1;

        tf2::Quaternion q;
        q.setRPY(0, 0, -M_PI_2);
        static_tf.transform.rotation = tf2::toMsg(q);

        static_broadcaster_->sendTransform(static_tf);
    }

    double x_, y_, theta_;
    geometry_msgs::msg::Quaternion orientation_;
    sensor_msgs::msg::LaserScan last_scan_;

    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
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

