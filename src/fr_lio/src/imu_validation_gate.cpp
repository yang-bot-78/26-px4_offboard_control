#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

// Pausable IMU relay, mirroring LidarValidationGate.
//
// Exists to make Test C executable: "predictor really stops for >150 ms must
// yield STATE_UNUSABLE / ev_usable=false". The LiDAR gate can only starve the
// posterior, which by design is L2 PLANNER_UNUSABLE and keeps EV alive. Only
// interrupting the IMU can drive the predictor axis to L3, so without this node
// the one gate that is still allowed to take EV away had no integration test.
//
// Deliberately NOT part of the flight graph: FR-LIO subscribes to /livox/imu
// directly. A validation run remaps FR-LIO onto the gated topic.
class ImuValidationGate : public rclcpp::Node
{
public:
  ImuValidationGate()
  : Node("frlio_validation_imu_gate")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/imu");
    output_topic_ =
      declare_parameter<std::string>("output_topic", "/validation/livox/imu");
    service_name_ =
      declare_parameter<std::string>("service_name", "/frlio_validation/imu_gate");
    status_topic_ = declare_parameter<std::string>(
      "status_topic", "/frlio_validation/imu_gate_enabled");

    // IMU is 200 Hz and best-effort in the driver; match it so the relay does
    // not itself become the bottleneck under test.
    const auto data_qos = rclcpp::SensorDataQoS().keep_last(200);
    const auto status_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, data_qos);
    status_publisher_ = create_publisher<std_msgs::msg::Bool>(status_topic_, status_qos);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_, data_qos,
      std::bind(&ImuValidationGate::on_imu, this, std::placeholders::_1));
    service_ = create_service<std_srvs::srv::SetBool>(
      service_name_,
      std::bind(
        &ImuValidationGate::set_enabled, this, std::placeholders::_1,
        std::placeholders::_2));

    publish_status();
    RCLCPP_WARN(
      get_logger(),
      "IMU validation gate: %s -> %s; service=%s. Pausing this starves the "
      "high-rate predictor and must drive FR-LIO to STATE_UNUSABLE.",
      input_topic_.c_str(), output_topic_.c_str(), service_name_.c_str());
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    received_.fetch_add(1, std::memory_order_relaxed);
    if (!enabled_.load(std::memory_order_relaxed)) {
      return;
    }
    // Forward the sample unchanged: same timestamp, same payload. Never
    // re-stamp or synthesize a substitute, or the test would be measuring the
    // gate's fabrication rather than FR-LIO's response to missing data.
    publisher_->publish(*message);
    forwarded_.fetch_add(1, std::memory_order_relaxed);
  }

  void set_enabled(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    enabled_.store(request->data, std::memory_order_relaxed);
    publish_status();
    response->success = true;
    response->message = std::string("IMU forwarding ") +
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
  std::string status_topic_;
  std::atomic_bool enabled_{true};
  std::atomic<std::uint64_t> received_{0};
  std::atomic<std::uint64_t> forwarded_{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuValidationGate>());
  rclcpp::shutdown();
  return 0;
}
