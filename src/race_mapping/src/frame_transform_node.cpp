/**
 * frame_transform_node.cpp — 坐标系变换发布节点
 *
 * 功能:
 *  1. 发布静态 TF: map -> odom 用于 RViz 坐标帧对齐
 *  2. 实时广播 PX4 本地坐标系 (NED) 与 FAST-LIO2 坐标系 (ENU/camera_init) 的转换
 *  3. 提供 /get_transform 服务, 查询任意两帧间当前变换
 *
 * 背景:
 *  PX4 使用 NED (北东地), FAST-LIO2 使用 ENU (东北天),
 *  Gazebo 世界坐标系可能与两者都不对齐。
 *  此节点确保所有坐标系在 RViz 中正确可视化。
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <stdexcept>

class FrameTransformNode : public rclcpp::Node
{
public:
    FrameTransformNode()
    : Node("frame_transform_node")
    {
        broadcast_map_to_odom_ = this->declare_parameter<bool>("broadcast_map_to_odom", true);
        this->declare_parameter("broadcast_odom_to_base_link", false);  // fastlio_bridge does this
        publish_camera_init_tf_ = this->declare_parameter<bool>("publish_camera_init_tf", true);
        world_yaw_alignment_rad_ =
            this->declare_parameter<double>("world_yaw_alignment_rad", 0.0);
        if (!std::isfinite(world_yaw_alignment_rad_)) {
            throw std::invalid_argument("world_yaw_alignment_rad must be finite");
        }

        // Static TF Broadcaster
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        // 启动后发送静态变换 (map -> odom 单位变换, 让 RViz 不对齐报错)
        sendStaticTransforms();

        RCLCPP_INFO(this->get_logger(), "frame_transform_node started");
        RCLCPP_INFO(
            this->get_logger(), "  Static TF map -> odom (identity): %s",
            broadcast_map_to_odom_ ? "enabled" : "disabled");
        if (publish_camera_init_tf_) {
            RCLCPP_INFO(
                this->get_logger(),
                "  Published static TF: map -> camera_init yaw=%.6f rad",
                world_yaw_alignment_rad_);
        } else {
            RCLCPP_INFO(this->get_logger(), "  Fixed map -> camera_init TF disabled");
        }

        // 定期重新发布静态 TF (防止某些工具未收到初始广播)
        retransmit_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&FrameTransformNode::sendStaticTransforms, this));
    }

private:
    void sendStaticTransforms()
    {
        auto now = this->now();

        // Only publish this in a non-relocalized stack.  During relocalization
        // it would falsely label raw FAST-LIO's local odom frame as map.
        if (broadcast_map_to_odom_) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "map";
            t.child_frame_id = "odom";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;
            map_to_odom_ = t;
        }

        if (publish_camera_init_tf_) {
            // Static TF 2: map -> camera_init. In tf2, the stored parent->child
            // rotation maps child coordinates into the parent, hence +yaw here
            // implements p_map = Rz(+yaw) * p_camera_init.
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = now;
            t.header.frame_id = "map";
            t.child_frame_id = "camera_init";
            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, world_yaw_alignment_rad_);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();
            camera_init_to_map_ = t;
        }

        // Do not publish a guessed base_link -> laser_link transform.  The
        // driver uses livox_frame, and the physical LiDAR-to-aircraft origin
        // and full attitude must be measured before adding that TF.
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        if (broadcast_map_to_odom_) {
            transforms.push_back(map_to_odom_);
        }
        if (publish_camera_init_tf_) {
            transforms.push_back(camera_init_to_map_);
        }
        static_tf_broadcaster_->sendTransform(transforms);
    }

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr retransmit_timer_;

    geometry_msgs::msg::TransformStamped map_to_odom_;
    geometry_msgs::msg::TransformStamped camera_init_to_map_;
    double world_yaw_alignment_rad_{0.0};
    bool broadcast_map_to_odom_{true};
    bool publish_camera_init_tf_{true};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameTransformNode>());
    rclcpp::shutdown();
    return 0;
}
