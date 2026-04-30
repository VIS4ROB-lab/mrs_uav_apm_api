#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

static constexpr uint16_t MAV_CMD_SET_MESSAGE_INTERVAL = 511;

// (message_id, rate_hz)
static const std::vector<std::pair<uint32_t, double>> STREAMS = {
    {0, 100.0},   // HEARTBEAT
    {1, 10.0},    // SYS_STATUS
    {27, 100.0},  // RAW_IMU
    {30, 100.0},  // ATTITUDE
    {32, 100.0},  // LOCAL_POSITION_NED
    {33, 10.0},   // GLOBAL_POSITION_INT
    {65, 10.0},   // RC_CHANNELS
    {83, 10.0},   // ATTITUDE_TARGET
    {147, 10.0},  // BATTERY_STATUS
    {152, 10.0},  // MEM_INFO
    {165, 10.0},  // HWSTATUS
    {173, 10.0},  // RANGEFINDER
    {245, 10.0},  // EXTENDED_SYS_STATE
};

class MavrosStreamEnforcer : public rclcpp::Node {
 public:
  MavrosStreamEnforcer() : Node("mavros_stream_enforcer") {
    client_ = this->create_client<mavros_msgs::srv::CommandLong>(
        "mavros/cmd/command");
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 1, [this](const mavros_msgs::msg::State::SharedPtr) {
          state_received_ = true;
          if (!state_received_at_) {
            state_received_at_ = std::chrono::steady_clock::now();
          }
        });

    timer_ = this->create_wall_timer(
        100ms, std::bind(&MavrosStreamEnforcer::enforce_tick, this));

    RCLCPP_INFO(get_logger(),
                "MAVROS stream enforcer started (serialized requests)");
  }

 private:
  void enforce_tick() {
    if (!state_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for mavros/state messages...");
      return;
    }

    if (state_received_at_ &&
        (std::chrono::steady_clock::now() - *state_received_at_) < 1s) {
      return;
    }

    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for mavros/cmd/command...");
      return;
    }

    if (next_stream_idx_ >= STREAMS.size()) {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "MAVROS streams enforced, timer stopped");
      return;
    }

    if (request_in_flight_) {
      return;
    }

    const auto& stream = STREAMS[next_stream_idx_];
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();

    req->command = MAV_CMD_SET_MESSAGE_INTERVAL;
    req->param1 = static_cast<float>(stream.first);
    req->param2 = static_cast<float>(1e6 / stream.second);
    req->param3 = 0.0f;
    req->param4 = 0.0f;
    req->param5 = 0.0f;
    req->param6 = 0.0f;
    req->param7 = 0.0f;

    request_in_flight_ = true;
    client_->async_send_request(
        req, [this, stream](
                 rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture
                     future) {
          request_in_flight_ = false;

          const auto& res = future.get();
          if (!res->success) {
            RCLCPP_WARN(get_logger(),
                        "Failed to set MAVLink message interval for msg id %u",
                        stream.first);
          }

          ++next_stream_idx_;
        });
  }

  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t next_stream_idx_{0};
  bool request_in_flight_{false};
  bool state_received_{false};
  std::optional<std::chrono::steady_clock::time_point> state_received_at_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavrosStreamEnforcer>());
  rclcpp::shutdown();
  return 0;
}