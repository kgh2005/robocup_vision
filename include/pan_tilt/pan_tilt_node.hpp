#include <rclcpp/rclcpp.hpp>

//#include "dynamixel_rdk_msgs/msg/dynamixel_msgs.hpp"
#include "robocup_vision/msg/pan_tilt.hpp"

class PanTiltNode : public rclcpp::Node
{
public:
  PanTiltNode(); // 생성자

private:
  int pan_Pos_ = 0, tilt_Pos_ = 0;
  int mode = 2;

  // ===== ROS 통신 =====

  //rclcpp::Publisher<dynamixel_rdk_msgs::msg::DynamixelMsgs>::SharedPtr Motor_Pub;
  robocup_vision::msg::PanTilt pan_tilt;
  rclcpp::Publisher<robocup_vision::msg::PanTilt>::SharedPtr pan_tilt_pub_;

  // ===== Callback =====

  // ===== Data Processing =====
  void pan_tilt_mode();
  void pan_tilt_publish();
};
