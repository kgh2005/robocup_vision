#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <cmath>

#include "utils/utils.hpp"
#include "robocup_vision/msg/bounding_box.hpp"
#include "robocup_vision/msg/pan_tilt_msgs.hpp"
#include "robocup_vision/msg/pan_tilt.hpp"

#include "humanoid_interfaces/msg/master2vision25.hpp"
#include "humanoid_interfaces/msg/robocupvision25.hpp"
#include "humanoid_interfaces/msg/robocupvision25feature.hpp"

struct DetectionResult
{
  int class_id;
  float score;
  cv::Rect bbox;
};

struct PAN_TILT
{
  double target_x;
  double target_y;
  double target_absx;
  double target_absy;
  struct
  {
    double PAN_POSITION;  // 팬 각도
    double TILT_POSITION; // 틸트 각도
  } ptpos;
};

class RefinerNode : public rclcpp::Node
{
public:
  RefinerNode();

private:
  // ===== 카메라 =====
  cv::Mat K_M, D_M, R_M, P_M, NEW_K_M;
  cv::Point2d focalLen;
  cv::Point2d prncPt;
  int width = 0, height = 0;

  // cv::Mat bgr_image;
  // ==========

  // ===== timer =====
  int filter_cnt = 0;
  int nice_cnt = 0;
  double fst_filter_x = 0, fst_filter_y = 0;
  int fst_filter_cnt = 0;
  double sec_filter_x = 0, sec_filter_y = 0;
  int sec_filter_cnt = 0;
  double final_filter_x = 0;
  double final_filter_y = 0;
  double ball_speed_vec_x = 0;
  double ball_speed_vec_y = 0;
  double ball_speed_level = 0;
  // ==========

  // ===== Ball =====
  cv::Rect remove_rect{0, 0, 0, 0};
  int ball_cam_X = 0;
  int ball_cam_Y = 0;
  int ball_filter_x[30] = {
      0,
  };
  int ball_filter_y[30] = {
      0,
  };
  int ball_filter_cnt = 0;
  int ball_filter_idx = 0;
  // ==========

  // ===== robot =====
  int robot_absx = 0;
  int robot_absy = 0;
  // ==========

  int remove_space_dis = 3000;

  // ===== 삼각측량법 =====
  int ROBOT_HEIGHT;
  int TILT_L;
  // ==========

  enum class Mode : int
  {
    INIT = 0,
    BALL_NO = 1,
    BALL_YES = 2,
  };
  Mode mode = Mode::INIT;


  int flag = 1;
  // ==========
  std::vector<cv::Point2f> ball_pts;
  std::vector<cv::Point2f> robot_pts;
  std::vector<cv::Point2f> line_pts, line_condis;
  ObjectPos ballPos;
  ObjectPos robotPos;
  ObjectPos line_Pos;
  PAN_TILT pan_tilt;
  std::vector<DetectionResult> Detections_ball_;
  std::vector<DetectionResult> Detections_robot_;
  std::vector<DetectionResult> Detections_line_;
  // ==========

  // ===== ROS 통신 =====
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<robocup_vision::msg::BoundingBox>::SharedPtr bbox_sub_;
  rclcpp::Subscription<robocup_vision::msg::PanTiltMsgs>::SharedPtr pan_tilt_sub_;
  rclcpp::Subscription<humanoid_interfaces::msg::Master2vision25>::SharedPtr visionSub;
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  humanoid_interfaces::msg::Robocupvision25 visionMsg;
  humanoid_interfaces::msg::Robocupvision25feature vision_feature_Msg;
  robocup_vision::msg::PanTilt PanTilt;
  rclcpp::Publisher<humanoid_interfaces::msg::Robocupvision25>::SharedPtr visionPub;
  rclcpp::Publisher<humanoid_interfaces::msg::Robocupvision25feature>::SharedPtr vision_feature_Pub;
  rclcpp::Publisher<robocup_vision::msg::PanTilt>::SharedPtr pan_tilt_pub_;

  // ===== Callback =====
  void bboxCallback(const robocup_vision::msg::BoundingBox::SharedPtr msg);
  void pan_tilt_Callback(const robocup_vision::msg::PanTiltMsgs::SharedPtr msg);
  void master_callback(const humanoid_interfaces::msg::Master2vision25::SharedPtr msg);
  // void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // ===== 좌표 =====
  void bboxProcessing();

  // ==========
  void publish_vision_msg();
  void publish_localization_msg();

  void pan_tilt_publish();

  // ===== timer =====
  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();

  // ===== param =====
  void get_param();
  // ==========
};
