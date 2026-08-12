/**
 * map_io_node.cpp — PCD 地图保存/加载与发布节点
 *
 * 功能:
 *  1. 启动时自动加载已保存的 PCD 地图并发布到 /saved_map 话题（供 RViz 订阅）
 *  2. 提供 /load_map 服务, 运行时切换地图文件
 *  3. 提供 /save_map 服务, 订阅其他节点发布的点云并保存为 PCD
 *
 * 设计理由:
 *  FAST-LIO2 本身只保存地图, 不加载地图。仿真重启后 RViz 看不到之前扫的点云,
 *  此节点填补了"已有 PCD → 发布到 ROS2 → RViz 显示"的链路。
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <string>
#include <fstream>

namespace fs = std::filesystem;

class MapIONode : public rclcpp::Node
{
public:
    MapIONode()
    : Node("map_io_node")
    {
        // 参数声明
        this->declare_parameter("map_file", "");
        this->declare_parameter("auto_load", true);
        this->declare_parameter("publish_topic", "/saved_map");
        this->declare_parameter("save_subscription_topic", "/cloud_registered_body");
        this->declare_parameter("frame_id", "camera_init");
        this->declare_parameter("display_leaf_size", 0.03);
        this->declare_parameter("max_display_points", 3000000);

        map_file_path_ = this->get_parameter("map_file").as_string();
        bool auto_load = this->get_parameter("auto_load").as_bool();
        publish_topic_ = this->get_parameter("publish_topic").as_string();
        save_subscription_topic_ = this->get_parameter("save_subscription_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        display_leaf_size_ = this->get_parameter("display_leaf_size").as_double();
        max_display_points_ = this->get_parameter("max_display_points").as_int();

        // Publisher: 发布加载的 PCD 地图
        pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            publish_topic_, rclcpp::QoS(1).transient_local());

        // Subscriber: 订阅点云用于保存 (默认订阅 FAST-LIO2 的 body-frame 点云)
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            save_subscription_topic_, rclcpp::QoS(10),
            std::bind(&MapIONode::cloudCallback, this, std::placeholders::_1));

        // Services
        srv_load_ = this->create_service<std_srvs::srv::Trigger>(
            "/load_map",
            std::bind(&MapIONode::loadMapService, this,
                      std::placeholders::_1, std::placeholders::_2));

        srv_save_ = this->create_service<std_srvs::srv::Trigger>(
            "/save_map_pcd",
            std::bind(&MapIONode::saveMapService, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "map_io_node started");
        RCLCPP_INFO(this->get_logger(), "  map_file: %s", map_file_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "  publish_topic: %s", publish_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  display_leaf_size: %.2f m", display_leaf_size_);
        RCLCPP_INFO(this->get_logger(), "  max_display_points: %d", max_display_points_);

        // 自动加载: 每秒重试一次, 直到成功加载并发布。
        // 这样可避免仿真 (use_sim_time) 启动时 /clock 尚未就绪、
        // 第一帧时间戳异常被 RViz 丢弃后“再也不发”的间歇性问题。
        if (auto_load)
        {
            load_timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                [this]() {
                    if (loadAndPublishPCD(map_file_path_))
                    {
                        load_timer_->cancel();  // 加载成功后停止重试
                        startPeriodicRepublish();
                    }
                });
        }
    }

    /**
     * @brief 加载成功后, 以低频周期性重发缓存地图。
     *        作用: 晚连接的 RViz、或仿真/RViz 重启后, 都能重新收到地图,
     *        不必依赖 transient_local 的单次补发。
     */
    void startPeriodicRepublish()
    {
        republish_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                if (display_msg_valid_)
                {
                    display_msg_.header.stamp = this->now();
                    pub_map_->publish(display_msg_);
                }
            });
    }


private:
    /**
     * @brief 加载 PCD 文件并发布
     */
    bool loadAndPublishPCD(const std::string& file_path)
    {
        if (!fs::exists(file_path))
        {
            RCLCPP_WARN(this->get_logger(), "PCD file not found: %s", file_path.c_str());
            return false;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD: %s", file_path.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu points from %s",
                    cloud->size(), file_path.c_str());

        pcl::PointCloud<pcl::PointXYZI>::Ptr display_cloud = cloud;
        if (max_display_points_ > 0 && cloud->size() > static_cast<size_t>(max_display_points_))
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr sampled(new pcl::PointCloud<pcl::PointXYZI>);
            size_t stride = std::max<size_t>(1, cloud->size() / static_cast<size_t>(max_display_points_));
            sampled->reserve(cloud->size() / stride + 1);
            for (size_t i = 0; i < cloud->size(); i += stride)
            {
                sampled->push_back((*cloud)[i]);
            }
            sampled->width = sampled->size();
            sampled->height = 1;
            sampled->is_dense = cloud->is_dense;
            display_cloud = sampled;
            RCLCPP_INFO(this->get_logger(), "Sampled display map: %zu -> %zu points (stride %zu)",
                        cloud->size(), display_cloud->size(), stride);
        }
        if (display_leaf_size_ > 0.0 && !cloud->empty())
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
            voxel_filter.setInputCloud(display_cloud);
            voxel_filter.setLeafSize(display_leaf_size_, display_leaf_size_, display_leaf_size_);
            voxel_filter.filter(*filtered);
            size_t before_filter_size = display_cloud->size();
            display_cloud = filtered;
            RCLCPP_INFO(this->get_logger(), "Downsampled display map: %zu -> %zu points (leaf %.2f m)",
                        before_filter_size, display_cloud->size(), display_leaf_size_);
        }

        // 转换为 ROS2 PointCloud2
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*display_cloud, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = frame_id_;

        pub_map_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published map to %s (frame: %s)",
                    publish_topic_.c_str(), frame_id_.c_str());

        // 缓存最近加载的点云与可显示消息 (供周期性重发使用)
        latest_cloud_ = cloud;
        display_msg_ = cloud_msg;
        display_msg_valid_ = true;
        return true;
    }

    /**
     * @brief 加载地图服务回调
     */
    void loadMapService(
        const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        res->success = loadAndPublishPCD(map_file_path_);
        res->message = res->success ? "Map loaded from " + map_file_path_ : "Load failed";
    }

    /**
     * @brief 保存地图服务回调 (保存缓存的最新一帧点云)
     */
    void saveMapService(
        const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        // 优先尝试调用 FAST-LIO2 的 /map_save 服务，这里做 fallback
        if (!latest_cloud_ || latest_cloud_->empty())
        {
            res->success = false;
            res->message = "No cloud data cached. Make sure FAST-LIO2 is running and publishing.";
            RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
            return;
        }

        fs::path dir = fs::path(map_file_path_).parent_path();
        if (!fs::exists(dir))
            fs::create_directories(dir);

        if (pcl::io::savePCDFileBinary(map_file_path_, *latest_cloud_) == -1)
        {
            res->success = false;
            res->message = "Failed to write PCD to " + map_file_path_;
            RCLCPP_ERROR(this->get_logger(), "%s", res->message.c_str());
            return;
        }

        res->success = true;
        res->message = "Map saved to " + map_file_path_ + " (" +
                       std::to_string(latest_cloud_->size()) + " points)";
        RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
    }

    /**
     * @brief 点云订阅回调: 缓存最新点云用于保存
     */
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);
        latest_cloud_ = cloud;
    }

    // Publishers & Subscribers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_load_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;

    // Timers: 延迟自动加载重试 + 加载成功后的周期性重发
    rclcpp::TimerBase::SharedPtr load_timer_;
    rclcpp::TimerBase::SharedPtr republish_timer_;

    // State
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud_;
    sensor_msgs::msg::PointCloud2 display_msg_;
    bool display_msg_valid_ = false;
    std::string map_file_path_;
    std::string publish_topic_;
    std::string save_subscription_topic_;
    std::string frame_id_;
    double display_leaf_size_ = 0.03;
    int max_display_points_ = 3000000;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapIONode>());
    rclcpp::shutdown();
    return 0;
}
