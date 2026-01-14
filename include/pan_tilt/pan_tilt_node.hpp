#include <rclcpp/rclcpp.hpp>

// #include "dynamixel_rdk_msgs/msg/dynamixel_msgs.hpp"
#include "robocup_vision/msg/pan_tilt_msgs.hpp"
#include "robocup_vision/msg/pan_tilt.hpp"

class PanTiltNode : public rclcpp::Node
{
public:
  PanTiltNode(); // 생성자

private:
  // 인스타 변환: 10도=36000 => 1도=3600
  static constexpr double INST_PER_DEG = 3600.0;

  int pan_Pos_ = 0, tilt_Pos_ = 0;
  int pan_deg = 0, tilt_deg = 0;
  int mode = 0;
  int ball_cam_X = 0;
  int ball_cam_Y = 0;

  int pan[3] = {-50, 0, 50};
  int tilt[2] = {-50, 0};

  int pan_count = -1;
  int tilt_count = 0;

  enum class TiltState : int
  {
    DOWN = 0,
    UP = 1,
  };
  TiltState tilt_state = TiltState::DOWN;

  // === tracking params ===
  int img_w_ = 640;
  int img_h_ = 480;

  int roi_w_ = 200; // 중앙 ROI 가로(px)
  int roi_h_ = 150; // 중앙 ROI 세로(px)

  double kp_pan_ = 0.05; // deg per pixel (튜닝)
  double kp_tilt_ = 0.05;

  double max_step_deg_ = 5.0; // 1 callback당 최대 변화(도)

  // 기구 한계(너 장비에 맞게 조절)
  int pan_min_deg_ = -90.0;
  int pan_max_deg_ = 90.0;
  int tilt_min_deg_ = -80.0;
  int tilt_max_deg_ = 80.0;

  // 축 방향(한 번만 맞추면 됨)
  double pan_sign_ = 1.0;
  double tilt_sign_ = -1.0;

  // ===== ROS 통신 =====
  rclcpp::Subscription<robocup_vision::msg::PanTilt>::SharedPtr pan_tilt_sub_;

  robocup_vision::msg::PanTiltMsgs pan_tilt;
  rclcpp::Publisher<robocup_vision::msg::PanTiltMsgs>::SharedPtr pan_tilt_pub_;

  // ===== Callback =====
  void pan_tilt_Callback(const robocup_vision::msg::PanTilt::SharedPtr msg);

  // ===== Data Processing =====
  void pan_tilt_mode();
  void pan_tilt_publish();

  void track_ball_roi(int bx, int by);
};
