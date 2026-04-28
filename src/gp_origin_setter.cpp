#include <memory>

#include "geographic_msgs/msg/geo_point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

class GpOriginSetter : public rclcpp::Node {
 public:
  GpOriginSetter() : Node("gp_origin_setter") {
    publisher_ = create_publisher<geographic_msgs::msg::GeoPointStamped>(
        "mavros/global_position/set_gp_origin", 1);

    subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        "mavros/global_position/global", 1,
        std::bind(&GpOriginSetter::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Listening on mavros/global_position/global and publishing "
                "mavros/global_position/set_gp_origin when GPS status is -1");
  }

 private:
  void callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    if (msg->status.status == -1 && msg->altitude != 0) {
      geographic_msgs::msg::GeoPointStamped origin_msg;
      origin_msg.header = msg->header;
      origin_msg.position.latitude = msg->latitude;
      origin_msg.position.longitude = msg->longitude;
      origin_msg.position.altitude = msg->altitude;

      publisher_->publish(origin_msg);

      RCLCPP_INFO(get_logger(), "Published set_gp_origin due to GPS status -1");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Received one GPS message with status %d, not publishing",
                  msg->status.status);
    }

    rclcpp::shutdown();
  }

  rclcpp::Publisher<geographic_msgs::msg::GeoPointStamped>::SharedPtr
      publisher_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpOriginSetter>());
  rclcpp::shutdown();
  return 0;
}
