// =============================================================
//  📌 节点名称: fastlio_bridge_node.cpp
//  📝 节点功能: FAST-LIO2 与比赛控制系统的坐标变换与话题桥接节点
//  💡 核心逻辑:
//      1. 实时订阅来自 FAST-LIO2 的里程计数据 `/Odometry`（ENU 坐标系）；
//      2. 按方案一，将 camera_init/odom 的标准 ENU 数据作为项目 map 数据；
//      3. 发布统一 ENU/map 语义的 `/race/odom` 和 `/race/pose`；
//      4. 广播静态与动态 TF 变换 (`map` -> `base_link`) 以便在 RViz2 中完美显示。
// =============================================================

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

class FastLioBridgeNode : public rclcpp::Node {
public:
    FastLioBridgeNode() : Node("fastlio_bridge_node") {
        // 1️⃣ 订阅 FAST-LIO 里程计话题
        //
        // 默认 /Odometry：这是本机 FAST-LIO（~/livox_mid360_env/ws_fastlio）实际
        // 发布的话题名。原先硬编码的 /fast_lio/odometry 在本工作区不存在，会让
        // 本节点一条消息都收不到，/race/odom 与 /race/pose 永不发布 —— 而且不报错。
        // 参数化以便别的 FAST-LIO 分支使用不同话题名。
        odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/Odometry");
        body_to_sensor_x_m_ = this->declare_parameter<double>("body_to_sensor_x_m", 0.0);
        body_to_sensor_y_m_ = this->declare_parameter<double>("body_to_sensor_y_m", 0.0);
        body_to_sensor_z_m_ = this->declare_parameter<double>("body_to_sensor_z_m", 0.08);
        body_to_fastlio_yaw_rad_ = this->declare_parameter<double>(
            "body_to_fastlio_yaw_rad", 0.0);
        world_yaw_alignment_rad_ = this->declare_parameter<double>(
            "world_yaw_alignment_rad", 0.0);
        if (!std::isfinite(body_to_sensor_x_m_) || !std::isfinite(body_to_sensor_y_m_) ||
            !std::isfinite(body_to_sensor_z_m_) || !std::isfinite(body_to_fastlio_yaw_rad_) ||
            !std::isfinite(world_yaw_alignment_rad_)) {
            throw std::invalid_argument("FAST-LIO body installation parameters must be finite");
        }
        // FR-LIO publishes raw odometry with SensorDataQoS (BEST_EFFORT).
        // A BEST_EFFORT subscription is compatible with both that stream and
        // the Reliable output of the relocalization health gate; a default
        // Reliable subscription rejects the former and leaves /race/odom empty.
        const auto odom_qos = rclcpp::SensorDataQoS().keep_last(5);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, odom_qos,
            std::bind(&FastLioBridgeNode::odom_callback, this, std::placeholders::_1));

        // 2️⃣ 创建统一 ENU/map 语义的里程计和位姿发布器
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/race/odom", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/race/pose", 10);

        // 3️⃣ 初始化 TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(
            this->get_logger(),
            "FAST-LIO2 ENU/map -> base_link bridge: %s -> /race/odom, /race/pose "
            "lever=(%.3f, %.3f, %.3f) installation_yaw=%.6f world_yaw=%.6f",
            odom_topic_.c_str(), body_to_sensor_x_m_, body_to_sensor_y_m_,
            body_to_sensor_z_m_, body_to_fastlio_yaw_rad_, world_yaw_alignment_rad_);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // FAST-LIO publishes camera_init/world ENU pose with child frame `body`.
        // Convert that child pose to the control-center `base_link` once here;
        // RViz's Odometry display consumes the pose quaternion directly and does
        // not provide a second installation-rotation step.
        const auto &sensor_pose = msg->pose.pose;
        tf2::Quaternion q_map_body;
        tf2::fromMsg(sensor_pose.orientation, q_map_body);
        const tf2::Quaternion q_body_to_fastlio =
            tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), body_to_fastlio_yaw_rad_);
        tf2::Quaternion q_map_base = q_map_body * q_body_to_fastlio;
        q_map_base.normalize();

        // body_to_sensor is expressed in base_link FLU. Rotate it into FAST-LIO
        // child axes first, then into map to recover the control-center pose.
        const double c = std::cos(body_to_fastlio_yaw_rad_);
        const double s = std::sin(body_to_fastlio_yaw_rad_);
        const tf2::Vector3 lever_fastlio(
            c * body_to_sensor_x_m_ - s * body_to_sensor_y_m_,
            s * body_to_sensor_x_m_ + c * body_to_sensor_y_m_,
            body_to_sensor_z_m_);
        const tf2::Vector3 lever_map = tf2::Matrix3x3(q_map_body) * lever_fastlio;
        const auto &sensor_position = sensor_pose.position;
        const tf2::Vector3 base_position_camera_init(
            sensor_position.x - lever_map.x(),
            sensor_position.y - lever_map.y(),
            sensor_position.z - lever_map.z());

        // Convert FAST-LIO's local world (camera_init) into the project map
        // exactly once.  Position and attitude use the same world yaw.
        const double cw = std::cos(world_yaw_alignment_rad_);
        const double sw = std::sin(world_yaw_alignment_rad_);
        const tf2::Vector3 base_position(
            cw * base_position_camera_init.x() - sw * base_position_camera_init.y(),
            sw * base_position_camera_init.x() + cw * base_position_camera_init.y(),
            base_position_camera_init.z());
        const tf2::Quaternion q_world_alignment(
            tf2::Vector3(0.0, 0.0, 1.0), world_yaw_alignment_rad_);
        q_map_base = q_world_alignment * q_map_base;
        q_map_base.normalize();

        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = msg->header.stamp;
        odom_msg->header.frame_id = "map";
        odom_msg->child_frame_id = "base_link";
        odom_msg->pose = msg->pose;
        odom_msg->pose.pose.position.x = base_position.x();
        odom_msg->pose.pose.position.y = base_position.y();
        odom_msg->pose.pose.position.z = base_position.z();
        odom_msg->pose.pose.orientation = tf2::toMsg(q_map_base);
        // Pose covariance uses [x,y,z,roll,pitch,yaw]. Apply the same world
        // alignment to both linear and angular XY blocks.
        double covariance_rotation[6][6]{};
        for (std::size_t index = 0; index < 6; ++index) {
            covariance_rotation[index][index] = 1.0;
        }
        covariance_rotation[0][0] = cw;
        covariance_rotation[0][1] = -sw;
        covariance_rotation[1][0] = sw;
        covariance_rotation[1][1] = cw;
        covariance_rotation[3][3] = cw;
        covariance_rotation[3][4] = -sw;
        covariance_rotation[4][3] = sw;
        covariance_rotation[4][4] = cw;
        for (std::size_t row = 0; row < 6; ++row) {
            for (std::size_t column = 0; column < 6; ++column) {
                double value = 0.0;
                for (std::size_t left = 0; left < 6; ++left) {
                    for (std::size_t right = 0; right < 6; ++right) {
                        value += covariance_rotation[row][left] *
                            msg->pose.covariance[left * 6 + right] *
                            covariance_rotation[column][right];
                    }
                }
                odom_msg->pose.covariance[row * 6 + column] = value;
            }
        }
        odom_msg->twist = msg->twist;
        // Odometry twist is expressed in the input child frame (`body`).
        // Rotate its axes into base_link when the installation yaw is nonzero;
        // the optional angular-lever-arm correction is intentionally left to
        // the dedicated vision-speed bridge.
        const tf2::Matrix3x3 fastlio_to_base(q_body_to_fastlio.inverse());
        const auto linear_base = fastlio_to_base * tf2::Vector3(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);
        const auto angular_base = fastlio_to_base * tf2::Vector3(
            msg->twist.twist.angular.x,
            msg->twist.twist.angular.y,
            msg->twist.twist.angular.z);
        odom_msg->twist.twist.linear.x = linear_base.x();
        odom_msg->twist.twist.linear.y = linear_base.y();
        odom_msg->twist.twist.linear.z = linear_base.z();
        odom_msg->twist.twist.angular.x = angular_base.x();
        odom_msg->twist.twist.angular.y = angular_base.y();
        odom_msg->twist.twist.angular.z = angular_base.z();

        odom_pub_->publish(*odom_msg);

        // --- 4. 打包并发布 /race/pose ---
        auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        pose_msg->header = odom_msg->header;
        pose_msg->pose = odom_msg->pose.pose;

        pose_pub_->publish(*pose_msg);

        const double x_enu = base_position.x();
        const double y_enu = base_position.y();
        const double z_enu = base_position.z();

        // --- 5. 广播 map -> base_link TF 变换 ---
        // The TF and /race/odom now carry the same ENU/map pose.  This avoids
        // the previous invalid combination of an ENU TF and NED data labelled
        // as map.
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";

        tf_msg.transform.translation.x = base_position.x();
        tf_msg.transform.translation.y = base_position.y();
        tf_msg.transform.translation.z = base_position.z();

        tf_msg.transform.rotation = tf2::toMsg(q_map_base);

        tf_broadcaster_->sendTransform(tf_msg);

        // 原第 7 步（发布 /fmu/in/vehicle_visual_odometry）已删除。本工作区的
        // 外部视觉定位由唯一 EV 写者 fastlio_mavros_vision_bridge 经
        // /mavros/vision_pose/pose_cov 提供，不允许第二个 EV 写者。见 坐标转换.md。

        // Keep the NED equivalent in the diagnostic log without publishing it
        // under the ENU/map frame.
        const double x_ned = y_enu;
        const double y_ned = x_enu;
        const double z_ned = -z_enu;

        // 隔秒限制打印，保证终端清洁
        if (this->get_clock()->now().nanoseconds() % 2000000000ULL < 100000000ULL) {
            RCLCPP_INFO(this->get_logger(), "已桥接并转换里程计: ENU(%.2f, %.2f, %.2f) -> NED(%.2f, %.2f, %.2f)",
                        x_enu, y_enu, z_enu, x_ned, y_ned, z_ned);
        }
    }

    // ROS2 订阅器与发布器
    std::string odom_topic_;
    double body_to_sensor_x_m_{0.0};
    double body_to_sensor_y_m_{0.0};
    double body_to_sensor_z_m_{0.08};
    double body_to_fastlio_yaw_rad_{0.0};
    double world_yaw_alignment_rad_{0.0};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    // TF 广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastLioBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
