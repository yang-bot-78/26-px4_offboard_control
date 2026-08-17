#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <chrono>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

class LidarValidationGate : public rclcpp::Node
{
public:
  LidarValidationGate()
  : Node("frlio_validation_lidar_gate")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/validation/livox/lidar");
    service_name_ =
      declare_parameter<std::string>("service_name", "/frlio_validation/lidar_gate");
    delay_service_name_ = declare_parameter<std::string>(
      "delay_service_name", "/frlio_validation/lidar_delay");
    delay_ms_ = declare_parameter<int>("delay_ms", 300);
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/frlio_validation/lidar_gate_enabled");

    const auto data_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    const auto status_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output_topic_, data_qos);
    status_publisher_ = create_publisher<std_msgs::msg::Bool>(status_topic_, status_qos);
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_topic_, data_qos,
      std::bind(&LidarValidationGate::on_lidar, this, std::placeholders::_1));
    service_ = create_service<std_srvs::srv::SetBool>(
      service_name_,
      std::bind(
        &LidarValidationGate::set_enabled, this, std::placeholders::_1,
        std::placeholders::_2));
    delay_service_ = create_service<std_srvs::srv::SetBool>(
      delay_service_name_,
      std::bind(
        &LidarValidationGate::set_delayed, this, std::placeholders::_1,
        std::placeholders::_2));
    flush_timer_ = create_wall_timer(
      std::chrono::milliseconds(1), std::bind(&LidarValidationGate::flush_due, this));

    publish_status();
    RCLCPP_INFO(
      get_logger(), "LiDAR validation gate enabled: %s -> %s; service=%s delay_service=%s delay_ms=%d",
      input_topic_.c_str(), output_topic_.c_str(), service_name_.c_str(),
      delay_service_name_.c_str(), delay_ms_);
  }

private:
  void on_lidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr message)
  {
    received_.fetch_add(1, std::memory_order_relaxed);
    if (!enabled_.load(std::memory_order_relaxed)) {
      return;
    }
    if (!delayed_.load(std::memory_order_relaxed)) {
      publisher_->publish(*message);
      forwarded_.fetch_add(1, std::memory_order_relaxed);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      delayed_queue_.push_back({std::chrono::steady_clock::now(), message});
    }
  }

  void set_delayed(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    delayed_.store(request->data, std::memory_order_relaxed);
    if (!request->data) {
      flush_all();
    }
    response->success = true;
    response->message = std::string("LiDAR delay ") +
      (request->data ? "enabled" : "disabled") + "; delay_ms=" + std::to_string(delay_ms_);
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void flush_due()
  {
    const auto deadline = std::chrono::steady_clock::now() -
      std::chrono::milliseconds(delay_ms_);
    std::deque<QueuedMessage> ready;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      while (!delayed_queue_.empty() && delayed_queue_.front().arrival <= deadline) {
        ready.push_back(std::move(delayed_queue_.front()));
        delayed_queue_.pop_front();
      }
    }
    for (const auto & item : ready) {
      publisher_->publish(*item.message);
      forwarded_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void flush_all()
  {
    std::deque<QueuedMessage> ready;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      ready.swap(delayed_queue_);
    }
    for (const auto & item : ready) {
      publisher_->publish(*item.message);
      forwarded_.fetch_add(1, std::memory_order_relaxed);
    }
  }

  struct QueuedMessage
  {
    std::chrono::steady_clock::time_point arrival;
    livox_ros_driver2::msg::CustomMsg::SharedPtr message;
  };

  void set_enabled(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    enabled_.store(request->data, std::memory_order_relaxed);
    publish_status();
    response->success = true;
    response->message = std::string("LiDAR forwarding ") +
      (request->data ? "enabled" : "paused") + "; received=" +
      std::to_string(received_.load(std::memory_order_relaxed)) + "; forwarded=" +
      std::to_string(forwarded_.load(std::memory_order_relaxed));
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void publish_status()
  {
    std_msgs::msg::Bool message;
    message.data = enabled_.load(std::memory_order_relaxed);
    status_publisher_->publish(message);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string service_name_;
  std::string delay_service_name_;
  int delay_ms_{300};
  std::string status_topic_;
  std::atomic_bool enabled_{true};
  std::atomic_bool delayed_{false};
  std::atomic<std::uint64_t> received_{0};
  std::atomic<std::uint64_t> forwarded_{0};
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr delay_service_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  std::mutex queue_mutex_;
  std::deque<QueuedMessage> delayed_queue_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarValidationGate>());
  rclcpp::shutdown();
  return 0;
}
