/**
 * race_mapping_demo_node.cpp — 建图功能集成演示节点
 *
 * 功能:
 *  1. 测试用: 在没有无人机飞行的场景下, 生成虚拟点云发布到 /saved_map
 *  2. 验证 RViz 可以正确接收并显示点云
 *  3. 可被 launch 文件条件性启动, 用于调试地图加载链路
 *
 * 使用:
 *  当没有真实 PCD 文件但想测试 RViz 显示时, 启动此节点即可看到随机点云。
 *  正式比赛用: 不启动此节点, 由 map_io_node 加载真实 scans.pcd。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <random>

class RaceMappingDemoNode : public rclcpp::Node
{
public:
    RaceMappingDemoNode()
    : Node("race_mapping_demo_node")
    {
        this->declare_parameter("num_points", 5000);
        this->declare_parameter("publish_rate_hz", 1.0);  // 只发一次然后停
        this->declare_parameter("radius", 20.0);
        this->declare_parameter("height", 5.0);

        int num_points = this->get_parameter("num_points").as_int();
        double publish_rate = this->get_parameter("publish_rate_hz").as_double();
        radius_ = this->get_parameter("radius").as_double();
        height_ = this->get_parameter("height").as_double();

        pub_demo_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/saved_map", rclcpp::QoS(1).transient_local());

        RCLCPP_INFO(this->get_logger(),
                    "race_mapping_demo_node started (DEBUG ONLY)");
        RCLCPP_INFO(this->get_logger(),
                    "  Generating %d points in cylinder (r=%.1f, h=%.1f)",
                    num_points, radius_, height_);

        // 生成并发布一次
        generateAndPublish(num_points, radius_, height_);

        // 每 publish_rate 秒重新发布一次
        if (publish_rate > 0)
        {
            pub_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / publish_rate),
                [this, num_points]() {
                    generateAndPublish(num_points, radius_, height_);
                });
        }
    }

private:
    void generateAndPublish(int num_points, double radius, double height)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = this->now();
        cloud.header.frame_id = "camera_init";

        // 设置 PointCloud2 结构
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(num_points);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(cloud, "intensity");

        // 随机数生成器: 在圆柱体内生成点 (赛道形状)
        std::mt19937 gen(rd_());
        std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * M_PI);
        std::uniform_real_distribution<double> radius_dist(0.0, 1.0);
        std::uniform_real_distribution<double> height_dist(-height / 2.0, height / 2.0);

        for (int i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
        {
            double angle = angle_dist(gen);
            double r = radius * std::sqrt(radius_dist(gen));  // sqrt 保证均匀分布
            *iter_x = r * std::cos(angle);
            *iter_y = r * std::sin(angle);
            *iter_z = height_dist(gen);
            *iter_i = 100.0f;  // 固定强度
        }

        pub_demo_->publish(cloud);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Demo cloud published: %d points", num_points);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_demo_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    std::random_device rd_;
    double radius_;
    double height_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RaceMappingDemoNode>());
    rclcpp::shutdown();
    return 0;
}