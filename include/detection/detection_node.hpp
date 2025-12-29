#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <openvino/openvino.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <vector>

#include "robocup_vision/msg/bounding_box.hpp"


// 클래스별 confidence threshold 설정
const std::map<int, float> CONFIDENCE_THRESHOLDS = {
    {0, 0.5f},
    {1, 0.5f},
    {2, 0.5f},
    {3, 0.5f},
};

// 클래스별 바운딩 박스 색상 설정 (BGR)
const std::map<int, cv::Scalar> COLORS = {
    {0, cv::Scalar(0, 0, 255)},   // 빨간색
    {1, cv::Scalar(0, 255, 0)},   // 초록색
    {2, cv::Scalar(0, 255, 255)}, // 노란색
    {3, cv::Scalar(255, 0, 0)},   // 파란색
};

struct DetectionResult
{
  int class_id;
  float score;
  int x1;
  int y1;
  int x2;
  int y2;
};


class DetectionNode : public rclcpp::Node
{
public:
  DetectionNode();

private:
  cv::Mat bgr_image;

  int flag = 1;

  double ball_best_confidence = 0.0;

  std::vector<DetectionResult> Detections_ball_;

  // int ball_class_id = 0; float ball_confidence = 0.0; 
  // int ball_x1 = 0; int ball_y1 = 0; int ball_x2 = 0; int ball_x2 = 0;


  // OpenVINO 엔진
  ov::Core core_;                    // OpenVINO 런타임 코어
  std::shared_ptr<ov::Model> model_; // 그래프(모델) 객체
  ov::CompiledModel compiled_model_; // 컴파일된 모델 객체
  ov::InferRequest infer_request_;   // 추론 요청 객체

  robocup_vision::msg::BoundingBox bbox;

  // ===== ROS 통신 =====
  rclcpp::Publisher<robocup_vision::msg::BoundingBox>::SharedPtr bbox_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // ===== Callback =====
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // ===== Data Processing =====
  void imageProcessing();
};
