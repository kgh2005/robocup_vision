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

  pan_tilt.pan_deg = pan_deg;
  pan_tilt.tilt_deg = tilt_deg;

  pan_tilt_pub_->publish(pan_tilt);

  RCLCPP_INFO(this->get_logger(), "========== Pan_Tilt ==========");
  RCLCPP_INFO(this->get_logger(), "Pan: %d, Tilt: %d", pan_deg, tilt_deg);
  RCLCPP_INFO(this->get_logger(), " ");
}

// insta 36000 => 10도
void PanTiltNode::pan_tilt_mode()
{
  RCLCPP_INFO(this->get_logger(), "%d", mode);

  switch (mode)
  {
  case 0: // init
  {
    pan_Pos_ = 0;
    tilt_Pos_ = (-1) * INST_PER_DEG * 50;

    pan_deg = 0;
    tilt_deg = -50;

    pan_tilt_publish();
    break;
  }

  case 1: // no ball
  {
    switch (tilt_state)
    {
    case TiltState::DOWN:
    {
      if (pan_count <= 1)
      {
        tilt_count = 0;

        // 먼저 실행
        // ==================================
        pan_count += 1;

        pan_Pos_ = pan[pan_count] * INST_PER_DEG;
        tilt_Pos_ = tilt[tilt_count] * INST_PER_DEG;

        pan_deg = pan[pan_count];
        tilt_deg = tilt[tilt_count];
        // ==================================
      }
      else
      {
        pan_count = 3;
        tilt_count = 1;

        // 먼저 실행
        // ==================================
        pan_count -= 1;

        pan_Pos_ = pan[pan_count] * INST_PER_DEG;
        tilt_Pos_ = tilt[tilt_count] * INST_PER_DEG;

        pan_deg = pan[pan_count];
        tilt_deg = tilt[tilt_count];
        // ==================================
        tilt_state = TiltState::UP;
      }
      break;
    }

    case TiltState::UP:
    {
      if (pan_count >= 1)
      {
        tilt_count = 1;

        // 먼저 실행
        // ==================================
        pan_count -= 1;

        pan_Pos_ = pan[pan_count] * INST_PER_DEG;
        tilt_Pos_ = tilt[tilt_count] * INST_PER_DEG;

        pan_deg = pan[pan_count];
        tilt_deg = tilt[tilt_count];
        // ==================================
      }
      else
      {
        pan_count = -1;
        tilt_count = 0;

        // 먼저 실행
        // ==================================
        pan_count += 1;

        pan_Pos_ = pan[pan_count] * INST_PER_DEG;
        tilt_Pos_ = tilt[tilt_count] * INST_PER_DEG;

        pan_deg = pan[pan_count];
        tilt_deg = tilt[tilt_count];
        // ==================================
        tilt_state = TiltState::DOWN;
      }
      break;
    }
    }

    pan_tilt_publish();
    break;
  }

  case 2: // yes ball
  {
    // if (pan_count >= 2)
    // {
    //   pan_count = 2;
    // }
    // if (pan_count <= 0)
    // {
    //   pan_count = 0;
    // }
    // pan_Pos_ = pan[pan_count] / 10 * 36000;
    // tilt_Pos_ = tilt[tilt_count] / 10 * 36000;

    // pan_tilt_publish();
    // break;

    track_ball_roi(ball_cam_X, ball_cam_Y);
    break;
  }

  default:
    RCLCPP_ERROR(this->get_logger(), "===== Pan_Tilt ERROR!! =====");
    break;
  }
}

void PanTiltNode::track_ball_roi(int bx, int by)
{
  // 유효성 체크
  if (bx < 0 || by < 0 || bx >= img_w_ || by >= img_h_)
    return;

  const int cx = img_w_ / 2; // 320
  const int cy = img_h_ / 2; // 240

  const int x0 = cx - roi_w_ / 2;
  const int x1 = cx + roi_w_ / 2;
  const int y0 = cy - roi_h_ / 2;
  const int y1 = cy + roi_h_ / 2;

  // ROI 안이면 유지(움직이지 않음)
  if (bx >= x0 && bx <= x1 && by >= y0 && by <= y1)
    return;

  // 픽셀 오차 (오른쪽/아래쪽이면 +)
  double err_x = static_cast<double>(bx - cx);
  double err_y = static_cast<double>(by - cy);

  // P 제어 (픽셀 -> 도)
  double d_pan = pan_sign_ * kp_pan_ * err_x;
  double d_tilt = tilt_sign_ * kp_tilt_ * err_y;

  // 스텝 제한
  d_pan = std::clamp(d_pan, -max_step_deg_, max_step_deg_);
  d_tilt = std::clamp(d_tilt, -max_step_deg_, max_step_deg_);

  // 누적 업데이트
  pan_deg += d_pan;
  tilt_deg += d_tilt;

  // 한계 제한
  pan_deg = std::clamp(pan_deg, pan_min_deg_, pan_max_deg_);
  tilt_deg = std::clamp(tilt_deg, tilt_min_deg_, tilt_max_deg_);

  // 인스타 단위 변환
  pan_Pos_ = static_cast<int>(std::round(pan_deg * INST_PER_DEG));
  tilt_Pos_ = static_cast<int>(std::round(tilt_deg * INST_PER_DEG));

  pan_tilt_publish();
}

void PanTiltNode::pan_tilt_Callback(const robocup_vision::msg::PanTilt::SharedPtr msg)
{
  mode = msg->mode;
  ball_cam_X = msg->ball_x;
  ball_cam_Y = msg->ball_y;

  pan_tilt_mode();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PanTiltNode>());
  rclcpp::shutdown();
  return 0;
}
