#include "rclcpp/rclcpp.hpp"
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "../include/robot_diff_wheel/message.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
using std::placeholders::_1;

double odom_pose_covariance_[] = {
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9};

double odom_twist_covariance[] = {1e-9, 0, 0, 0, 0, 0, 0, 1e-3, 1e-9, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e-9};

void move2m(int f) {
    if(f)
        Send_ChassisData(300.0, 300.0, 0b00000101);
    else 
        Send_ChassisData(0.0, 0.0, 0b00000000);
}

class OdometryPublisher : public rclcpp::Node
{

public:
    OdometryPublisher() : Node("odometry_publisher")
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 订阅键盘控制话题
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&OdometryPublisher::cmd_vel_callback, this, _1));
    }
    // 发布坐标变换信息
    void publish_tf()
    {

        // 获取底盘信息
        Receive_ChassisData(&leftMotorSpeed, &rightMotorSpeed, &yaw, &control);

        // RCLCPP_INFO(this->get_logger(), "Speed, left:%lf, right:%lf, yaw:%lf, control:%#X",
        //             leftMotorSpeed, rightMotorSpeed, yaw, control);
        // RCLCPP_INFO(this->get_logger(), "left:%lf, right:%lf, yaw:%lf", leftMotorSpeed, rightMotorSpeed, yaw);

        linerSpeed = (leftMotorSpeed + rightMotorSpeed) / 2 / 1000;
        angularSpeed = (yaw - last_yaw) * (M_PI / 180.0) / LOOPPERIOD;
        theta += (yaw - last_yaw) * (M_PI / 180.0);
        // theta = (theta > 360)?(theta - 360):theta;
        position_x += ( linerSpeed * cos(theta) ) * LOOPPERIOD / FACTOR;
        position_y += ( linerSpeed * sin(theta) ) * LOOPPERIOD / FACTOR;
        last_yaw = yaw;

        double seconds = this->now().seconds();
        odom_msg_.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
        odom_msg_.header.frame_id = "odom";
        odom_msg_.child_frame_id = "base_link";

        // 更新odom_msg_的姿态信息
        odom_msg_.pose.pose.position.x = position_x;
        odom_msg_.pose.pose.position.y = position_y;
        odom_msg_.pose.pose.position.z = 0.0;
        for (int i = 0; i < 36; ++i)
        {
            odom_msg_.pose.covariance[i] = odom_pose_covariance_[i];
        }

        tf2::Quaternion odom_quat_tf2;
        odom_quat_tf2.setRPY(0, 0, theta);

        odom_msg_.pose.pose.orientation = tf2::toMsg(odom_quat_tf2);

        odom_msg_.twist.twist.linear.x = linerSpeed;
        odom_msg_.twist.twist.linear.y = 0.0;
        odom_msg_.twist.twist.angular.z = angularSpeed;
        for (int i = 0; i < 36; ++i)
        {
            odom_msg_.twist.covariance[i] = odom_twist_covariance[i];
        }

        publisher_->publish(odom_msg_);

        geometry_msgs::msg::TransformStamped transform;
        seconds = this->now().seconds();
        transform.header.stamp = rclcpp::Time(static_cast<uint64_t>(seconds * 1e9));
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_footprint";

        transform.transform.translation.x = odom_msg_.pose.pose.position.x;
        transform.transform.translation.y = odom_msg_.pose.pose.position.y;
        transform.transform.translation.z = odom_msg_.pose.pose.position.z;
        transform.transform.rotation.x = odom_msg_.pose.pose.orientation.x;
        transform.transform.rotation.y = odom_msg_.pose.pose.orientation.y;
        transform.transform.rotation.z = odom_msg_.pose.pose.orientation.z;
        transform.transform.rotation.w = odom_msg_.pose.pose.orientation.w;

        // 广播坐标变换信息
        tf_broadcaster_->sendTransform(transform);
    }

    // 订阅者回调函数
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "L:%lf R:%lf Y%lf", leftMotorSpeed, rightMotorSpeed, yaw);
        double linear_vel_x = (msg->linear.x) * 1000,
               angular_vel_th = (msg->angular.z) * 100,
               rightTargetSpeed = 0.0,
               leftTargetSpeed = 0.0;
        rightTargetSpeed = (linear_vel_x + angular_vel_th * 9.5);
        leftTargetSpeed = (linear_vel_x - angular_vel_th * 9.5);
        RCLCPP_INFO(this->get_logger(), "Target:%lf %lf\n", leftTargetSpeed, rightTargetSpeed);
        unsigned char targetControl = 0b00000000;
        if(leftTargetSpeed > 0.0) {
            targetControl += 0b00000100;
        } else if(leftTargetSpeed < 0.0) {
            targetControl += 0b00001000;
        }
        if(rightTargetSpeed > 0.0) {
            targetControl += 0b00000001;
        } else if(rightTargetSpeed < 0.0) {
            targetControl += 0b00000010;
        }
        // RCLCPP_INFO(this->get_logger(), "TargetSend: %lf, %lf, %#X", leftTargetSpeed, rightTargetSpeed, targetControl);
        Send_ChassisData(leftTargetSpeed, rightTargetSpeed, targetControl);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry odom_msg_;
    rclcpp::Time time;
    double LOOPPERIOD = 0.02f, FACTOR = 0.87;
    double rightMotorSpeed, leftMotorSpeed, linerSpeed, angularSpeed;
    double yaw, last_yaw, theta;
    double position_o, position_x, position_y, last_x, last_y;
    unsigned char control;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    Serial_Init();

    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryPublisher>();

    // 设置循环频率
    rclcpp::WallRate loop_rate(50.0);
    // move2m(1);
    while (rclcpp::ok())
    {
        // 处理回调函数
        rclcpp::spin_some(node);

        // 发布坐标变换信息
        node->publish_tf();

        // 控制循环频率
        loop_rate.sleep();
    }

    // 关闭ROS节点
    rclcpp::shutdown();
    return 0;
}