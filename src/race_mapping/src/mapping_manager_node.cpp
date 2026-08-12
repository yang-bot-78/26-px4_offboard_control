/**
 * mapping_manager_node.cpp — 建图管理节点
 *
 * 功能:
 *  1. 监听无人机起飞/降落状态, 自动触发地图保存
 *  2. 提供 /mapping/start, /mapping/stop, /mapping/save 等服务
 *  3. 在仿真重启后, 自动调用 map_io_node 加载已有地图
 *
 * 使用场景:
 *  仿真刚重启 → 自动加载 scans.pcd → RViz 立刻看到之前扫的地图
 *  无人机起飞 → FAST-LIO2 建图 → 降落后自动保存
 */

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

class MappingManagerNode : public rclcpp::Node
{
public:
    MappingManagerNode()
    : Node("mapping_manager_node")
    {
        this->declare_parameter("auto_load_on_start", true);
        this->declare_parameter("auto_save_on_land", true);
        this->declare_parameter("load_delay_seconds", 4.0);  // 比 map_io_node 晚一点

        bool auto_load = this->get_parameter("auto_load_on_start").as_bool();
        auto_save_on_land_ = this->get_parameter("auto_save_on_land").as_bool();

        // 订阅起飞/降落状态 (来自 offboard_waypoint_node)
        // 这个 topic 由 offboard_waypoint_node 发布, 表示无人机是否在飞行中
        flight_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/race/flight_state", rclcpp::QoS(1),
            std::bind(&MappingManagerNode::flightStateCallback, this, std::placeholders::_1));

        // 服务: 触发地图加载
        srv_load_ = this->create_service<std_srvs::srv::Trigger>(
            "/mapping/load_map",
            std::bind(&MappingManagerNode::loadMapService, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 服务: 触发地图保存
        srv_save_ = this->create_service<std_srvs::srv::Trigger>(
            "/mapping/save_map",
            std::bind(&MappingManagerNode::saveMapService, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 服务: 查询地图状态
        srv_status_ = this->create_service<std_srvs::srv::Trigger>(
            "/mapping/status",
            std::bind(&MappingManagerNode::statusService, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "mapping_manager_node started");
        RCLCPP_INFO(this->get_logger(), "  auto_load_on_start: %s", auto_load ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "  auto_save_on_land: %s", auto_save_on_land_ ? "true" : "false");

        // 启动后自动加载地图 (延迟, 等 fast_lio 和 map_io_node 就绪)
        if (auto_load)
        {
            load_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(
                    this->get_parameter("load_delay_seconds").as_double()),
                [this]() {
                    auto_load_on_start_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "Auto-load trigger: call /load_map service...");
                    // 这里通过创建 client 调用 map_io_node 的 /load_map 服务
                    auto client = this->create_client<std_srvs::srv::Trigger>("/load_map");
                    if (client->wait_for_service(std::chrono::seconds(5)))
                    {
                        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                        auto future = client->async_send_request(req);
                        // fire-and-forget: map_io_node 会打印日志
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(),
                                    "/load_map service not available (map_io_node not running?)");
                    }
                    load_timer_->cancel();
                });
        }

        // 定时器: 定期打印状态 (每30秒)
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(30),
            [this]() {
                RCLCPP_INFO(this->get_logger(),
                            "Mapping status: in_flight=%s, map_loaded=%s, map_saved=%s",
                            in_flight_ ? "true" : "false",
                            map_loaded_ ? "true" : "false",
                            map_saved_ ? "true" : "false");
            });
    }

private:
    /**
     * @brief 飞行状态回调: 检测起飞/降落切换
     */
    void flightStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool was_in_flight = in_flight_;
        in_flight_ = msg->data;

        // 从飞行切换到降落 (着陆检测)
        if (was_in_flight && !in_flight_ && auto_save_on_land_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Landing detected! Auto-saving map...");

            // 调用 FAST-LIO2 的 /map_save 服务
            auto client = this->create_client<std_srvs::srv::Trigger>("/map_save");
            if (client->wait_for_service(std::chrono::seconds(3)))
            {
                auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto future = client->async_send_request(req);

                // 简单等待结果 (实际项目中应该用回调)
                rclcpp::spin_until_future_complete(
                    this->shared_from_this(), future, std::chrono::seconds(5));

                if (future.get()->success)
                {
                    map_saved_ = true;
                    RCLCPP_INFO(this->get_logger(), "Map auto-saved successfully");
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(),
                            "FAST-LIO2 /map_save service not available, trying fallback...");
                // Fallback: 调用 map_io_node 的保存服务
                auto client2 = this->create_client<std_srvs::srv::Trigger>("/save_map_pcd");
                if (client2->wait_for_service(std::chrono::seconds(2)))
                {
                    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
                    client2->async_send_request(req);
                }
            }
        }

        if (!was_in_flight && in_flight_)
        {
            RCLCPP_INFO(this->get_logger(), "Takeoff detected! FAST-LIO2 will accumulate new points.");
            map_saved_ = false;
        }
    }

    void loadMapService(
        const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        auto client = this->create_client<std_srvs::srv::Trigger>("/load_map");
        if (!client->wait_for_service(std::chrono::seconds(3)))
        {
            res->success = false;
            res->message = "/load_map service not available";
            return;
        }

        auto req_srv = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client->async_send_request(req_srv);

        rclcpp::spin_until_future_complete(
            this->shared_from_this(), future, std::chrono::seconds(5));

        auto result = future.get();
        res->success = result->success;
        res->message = result->message;
        map_loaded_ = result->success;
    }

    void saveMapService(
        const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        // 优先用 FAST-LIO2 原生服务
        auto client = this->create_client<std_srvs::srv::Trigger>("/map_save");
        if (client->wait_for_service(std::chrono::seconds(2)))
        {
            auto req_srv = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = client->async_send_request(req_srv);

            rclcpp::spin_until_future_complete(
                this->shared_from_this(), future, std::chrono::seconds(5));

            auto result = future.get();
            res->success = result->success;
            res->message = result->message;
            map_saved_ = result->success;
            return;
        }

        // Fallback: map_io_node 的保存
        auto client2 = this->create_client<std_srvs::srv::Trigger>("/save_map_pcd");
        if (client2->wait_for_service(std::chrono::seconds(2)))
        {
            auto req_srv = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = client2->async_send_request(req_srv);

            rclcpp::spin_until_future_complete(
                this->shared_from_this(), future, std::chrono::seconds(5));

            auto result = future.get();
            res->success = result->success;
            res->message = result->message;
            map_saved_ = result->success;
            return;
        }

        res->success = false;
        res->message = "No save service available (/map_save or /save_map_pcd)";
    }

    void statusService(
        const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
        std_srvs::srv::Trigger::Response::SharedPtr res)
    {
        res->success = true;
        res->message = std::string("in_flight: ") + (in_flight_ ? "true" : "false") +
                       ", map_loaded: " + (map_loaded_ ? "true" : "false") +
                       ", map_saved: " + (map_saved_ ? "true" : "false");
    }

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flight_state_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_load_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_status_;

    // Timers
    rclcpp::TimerBase::SharedPtr load_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // State
    bool in_flight_ = false;
    bool map_loaded_ = false;
    bool map_saved_ = false;
    bool auto_save_on_land_ = true;
    bool auto_load_on_start_ = true;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingManagerNode>());
    rclcpp::shutdown();
    return 0;
}