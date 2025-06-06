#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/mount_control.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <iomanip>

#include "yaw_controller.hpp"

using std::placeholders::_1;
using namespace std::chrono;

class OffboardVelocityYawControlNode : public rclcpp::Node {
public:
    OffboardVelocityYawControlNode()
        : Node("offboard_velocity_yaw_node"),
          yaw_controller_(1.0 / 30.0) {

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        state_sub_ = create_subscription<mavros_msgs::msg::State>(
            "mavros/uas_1/state", 10, std::bind(&OffboardVelocityYawControlNode::state_cb, this, _1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "mavros/uas_1/local_position/odom", qos, std::bind(&OffboardVelocityYawControlNode::odom_cb, this, _1));
        target_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/target_odometry", 10, std::bind(&OffboardVelocityYawControlNode::target_cb, this, _1));
        gimbal_sub_ = create_subscription<mavros_msgs::msg::MountControl>(
            "/mavros/uas_1/mount_control/command", 10, std::bind(&OffboardVelocityYawControlNode::gimbal_cb, this, _1));

        setpoint_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
            "mavros/uas_1/setpoint_raw/local", 10);

        arming_client_ = create_client<mavros_msgs::srv::CommandBool>("mavros/uas_1/cmd/arming");
        set_mode_client_ = create_client<mavros_msgs::srv::SetMode>("mavros/uas_1/set_mode");

        wait_for_services();
        last_req_ = now();
        timer_ = create_wall_timer(50ms, std::bind(&OffboardVelocityYawControlNode::timer_callback, this));

        std::filesystem::create_directories(log_dir_);
        csv_file_.open(log_path_, std::ios::out);
        csv_file_ << "timestamp,drone_x,drone_y,drone_z,target_x,target_y,target_z,"
                  << "los_yaw_deg,drone_yaw_deg,gimbal_yaw_deg,yaw_error_deg,yaw_rate_deg\n";
    }

    ~OffboardVelocityYawControlNode() {
        if (csv_file_.is_open())
            csv_file_.close();
    }

private:
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_, target_sub_;
    rclcpp::Subscription<mavros_msgs::msg::MountControl>::SharedPtr gimbal_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    mavros_msgs::msg::State current_state_;
    Eigen::Vector3d drone_pos_, target_pos_;
    Eigen::Quaterniond drone_quat_;
    bool has_pos_ = false, has_quat_ = false, has_target_ = false;
    double gimbal_yaw_ = 0.0;
    rclcpp::Time last_req_;

    YawController yaw_controller_;

    const double target_altitude_ = 10.0;
    const double max_speed_ = 2.0;
    const double Kp_z_ = 1.0;

    const std::string log_dir_ = std::string(std::getenv("HOME")) + "/yaw_tracking_logs/";
    const std::string log_path_ = log_dir_ + "yaw_debug_log.csv";
    std::ofstream csv_file_;

    void wait_for_services() {
        while (!arming_client_->wait_for_service(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for arming service...");
        }
        while (!set_mode_client_->wait_for_service(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for set_mode service...");
        }
    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg) {
        current_state_ = *msg;
    }

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        drone_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        drone_quat_ = Eigen::Quaterniond(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
        );
        has_pos_ = has_quat_ = true;
    }

    void target_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        target_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
        has_target_ = true;
    }

    void gimbal_cb(const mavros_msgs::msg::MountControl::SharedPtr msg) {
        gimbal_yaw_ = msg->yaw * M_PI / 180.0;
    }

    void timer_callback() {
        if (current_state_.mode != "OFFBOARD" && (now() - last_req_).seconds() > 5.0) {
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "OFFBOARD";
            set_mode_client_->async_send_request(req);
            last_req_ = now();
        } else if (!current_state_.armed && (now() - last_req_).seconds() > 5.0) {
            auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            req->value = true;
            arming_client_->async_send_request(req);
            last_req_ = now();
        }

        if (!(has_pos_ && has_quat_ && has_target_)) return;

        Eigen::Vector3d los_vec = target_pos_ - drone_pos_;
        if (los_vec.norm() < 1e-3) return;

        Eigen::Vector3d euler = drone_quat_.toRotationMatrix().eulerAngles(0, 1, 2);
        double drone_yaw = euler.z();
        double los_yaw = std::atan2(los_vec.y(), los_vec.x());

        double yaw_rate = yaw_controller_.update(drone_yaw, los_yaw);
        double yaw_error = yaw_controller_.get_last_yaw_error();
        double vz = std::clamp(Kp_z_ * (target_altitude_ - drone_pos_.z()), -0.8, 0.8);

        mavros_msgs::msg::PositionTarget msg;
        msg.header.stamp = now();
        msg.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_PX |
            mavros_msgs::msg::PositionTarget::IGNORE_PY |
            mavros_msgs::msg::PositionTarget::IGNORE_PZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW;

        msg.velocity.x = 0.0;
        msg.velocity.y = 0.0;
        msg.velocity.z = vz;
        msg.yaw_rate = yaw_rate;
        setpoint_pub_->publish(msg);

        auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        csv_file_ << std::put_time(std::localtime(&t), "%FT%T") << ","
                  << drone_pos_.x() << "," << drone_pos_.y() << "," << drone_pos_.z() << ","
                  << target_pos_.x() << "," << target_pos_.y() << "," << target_pos_.z() << ","
                  << los_yaw * 180.0 / M_PI << "," << drone_yaw * 180.0 / M_PI << ","
                  << gimbal_yaw_ * 180.0 / M_PI << "," << yaw_error * 180.0 / M_PI << ","
                  << yaw_rate * 180.0 / M_PI << "\n";
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardVelocityYawControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
