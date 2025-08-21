#include "pan_tilt/pan_tilt_node.hpp"

PanTiltNode::PanTiltNode() : rclcpp::Node("pan_tilt_node")
{
  pan_tilt_sub_ = this->create_subscription<robocup_vision::msg::PanTilt>(
      "/PanTilt", 10,
      std::bind(&PanTiltNode::pan_tilt_Callback, this, std::placeholders::_1));
  // Motor_Pub = this->create_publisher<dynamixel_rdk_msgs::msg::DynamixelMsgs>("pan_tilt_dxl", 10);
  pan_tilt_pub_ = this->create_publisher<robocup_vision::msg::PanTiltMsgs>("/camera1/pan_tilt", 10);
  // pan_tilt_sub_ = this->create_subscription<intelligent_humanoid_interfaces::msg::Master2VisionMsg>(
  //     "/master/pan_tilt", 10,
  //     std::bind(&PanTiltNode::pantiltCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "pan_tilt_node started.");
}

void PanTiltNode::pan_tilt_publish()
{
  pan_tilt.pan = pan_Pos_;
  pan_tilt.tilt = tilt_Pos_;
  pan_tilt_pub_->publish(pan_tilt);

  // RCLCPP_INFO(this->get_logger(), "========== Pan_Tilt ==========");
  // RCLCPP_INFO(this->get_logger(), "Pan: %d, Tilt: %d", pan_Pos_, tilt_Pos_);
  // RCLCPP_INFO(this->get_logger(), " ");
}

void PanTiltNode::pan_tilt_mode()
{
  switch (mode)
  {
  case 0: // init
  {
    pan_Pos_ = 0;
    tilt_Pos_ = 0;
    pan_tilt_publish();
    break;
  }

  case 1: // tilt 45도
  {
    pan_Pos_ = 0;
    tilt_Pos_ = -150000;
    pan_tilt_publish();
    break;
  }

  default:
    RCLCPP_ERROR(this->get_logger(), "===== Pan_Tilt ERROR!! =====");
    break;
  }
}

void PanTiltNode::pan_tilt_Callback(const robocup_vision::msg::PanTilt::SharedPtr msg)
{
  mode = msg->mode;

  pan_tilt_mode();
}

// void PanTiltNode::pantiltCallback(const intelligent_humanoid_interfaces::msg::Master2VisionMsg::SharedPtr msg)
// {
//   mode = msg->tilt;
//   pan_tilt_mode();
// }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PanTiltNode>());
  rclcpp::shutdown();
  return 0;
}
