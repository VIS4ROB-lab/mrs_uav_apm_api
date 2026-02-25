#include <chrono>
#include <memory>
#include <vector>

#include "mavros_msgs/srv/command_long.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

static constexpr uint16_t MAV_CMD_SET_MESSAGE_INTERVAL = 511;

// (message_id, rate_hz)
static const std::vector<std::pair<uint32_t, double>> STREAMS = {
    {30, 100.0},   // ATTITUDE
    {83, 100.0},   // ATTITUDE_TARGET
    {27, 100.0},   // HIGHRES_IMU
    {65, 10.0},    // RC_CHANNELS
    {32, 100.0},   // LOCAL_POSITION_NED
    {33, 100.0},   // GLOBAL_POSITION_INT
    {1, 10.0},     // SYS_STATUS
    {132, 100.0},  // DISTANCE_SENSOR
    {0, 100.0},    // HEARTBEAT
    {245, 10.0},   // EXTENDED_SYS_STATE
};

class MavrosStreamEnforcer : public rclcpp::Node {
 public:
  MavrosStreamEnforcer() : Node("mavros_stream_enforcer") {
    client_ = this->create_client<mavros_msgs::srv::CommandLong>(
        "mavros/cmd/command");

    timer_ = this->create_wall_timer(
        5s, std::bind(&MavrosStreamEnforcer::enforce_streams, this));

    RCLCPP_INFO(get_logger(),
                "MAVROS stream enforcer started (reassert every 5s)");
  }

 private:
  void enforce_streams() {
    if (!client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for mavros/cmd/command...");
      return;
    }

    for (const auto& s : STREAMS) {
      auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();

      req->command = MAV_CMD_SET_MESSAGE_INTERVAL;
      req->param1 = static_cast<float>(s.first);
      req->param2 = static_cast<float>(1e6 / s.second);
      req->param3 = 0.0f;
      req->param4 = 0.0f;
      req->param5 = 0.0f;
      req->param6 = 0.0f;
      req->param7 = 0.0f;

      client_->async_send_request(req);
    }
  }

  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavrosStreamEnforcer>());
  rclcpp::shutdown();
  return 0;
}