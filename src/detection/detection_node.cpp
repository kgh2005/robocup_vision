#include "detection/detection_node.hpp"

DetectionNode::DetectionNode() : Node("detection_node")
{
  declare_parameter<std::string>("model_xml");
  std::string model_path;
  get_parameter("model_xml", model_path);

  try
  {
    // 모델 로드 및 컴파일 (read_model returns shared_ptr<ov::Model>)
    auto model = core_.read_model(model_path);
    compiled_model_ = core_.compile_model(model, "AUTO"); // CPU -> AUTO로 변경
    RCLCPP_INFO(get_logger(), "Loaded model: %s", model_path.c_str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(get_logger(), "Model load failed: %s", e.what());
    rclcpp::shutdown();
    return;
  }
  bbox_pub_ = this->create_publisher<robocup_vision::msg::BoundingBox>("/Bounding_box", 10);
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/camera/image_raw", 10,
      std::bind(&DetectionNode::imageCallback, this, std::placeholders::_1));
}

void DetectionNode::imageProcessing()
{
  if (bgr_image.empty())
    return;

  int original_height = bgr_image.rows;
  int original_width = bgr_image.cols;

  // ---- 1. 입력 이미지 전처리 ----
  auto input_port = compiled_model_.input(0);
  auto input_shape = input_port.get_shape();
  int model_input_height = static_cast<int>(input_shape[2]);
  int model_input_width = static_cast<int>(input_shape[3]);

  cv::Mat resized_image;
  cv::resize(bgr_image, resized_image, cv::Size(model_input_width, model_input_height));
  cv::Mat rgb_image;
  cv::cvtColor(resized_image, rgb_image, cv::COLOR_BGR2RGB);

  cv::Mat normalized_image;
  rgb_image.convertTo(normalized_image, CV_32F, 1.0f / 255.0f);

  ov::Tensor input_tensor(input_port.get_element_type(), input_shape);
  float *input_data = input_tensor.data<float>();
  std::vector<cv::Mat> color_channels(3);
  cv::split(normalized_image, color_channels);
  int pixels_per_channel = model_input_height * model_input_width;
  for (int c = 0; c < 3; ++c)
  {
    std::memcpy(input_data + c * pixels_per_channel,
                color_channels[c].ptr<float>(),
                pixels_per_channel * sizeof(float));
  }

  // ---- 2. 모델 추론 ----
  auto infer_request = compiled_model_.create_infer_request();
  infer_request.set_tensor(input_port, input_tensor);
  infer_request.infer();
  // infer_request.start_async();
  // infer_request.wait();

  auto output_tensor = infer_request.get_tensor(compiled_model_.output(0));
  const float *output_data = output_tensor.data<float>();
  auto shape = output_tensor.get_shape(); // [1, N, 6] - NMS=True 출력
  size_t num_detections = shape[1];

  // bbox 초기화 (매번 새로 시작)
  bbox.class_ids.clear();
  bbox.score.clear();
  bbox.x1.clear();
  bbox.y1.clear();
  bbox.x2.clear();
  bbox.y2.clear();

  // ---- 3. detection 후처리 ----
  for (size_t detection_idx = 0; detection_idx < num_detections; detection_idx++)
  {
    // NMS=True 출력: [x1, y1, x2, y2, confidence, class_id]
    const float *det = &output_data[detection_idx * 6];

    float x1 = det[0];
    float y1 = det[1];
    float x2 = det[2];
    float y2 = det[3];
    float confidence = det[4];
    int class_id = static_cast<int>(det[5]);

    int bx1, by1, bx2, by2;

    // 픽셀 좌표를 입력 해상도 기준으로 스케일링
    bx1 = static_cast<int>(x1 / model_input_width * original_width);
    by1 = static_cast<int>(y1 / model_input_height * original_height);
    bx2 = static_cast<int>(x2 / model_input_width * original_width);
    by2 = static_cast<int>(y2 / model_input_height * original_height);

    // 좌표 클리핑
    bx1 = std::clamp(bx1, 0, original_width - 1);
    by1 = std::clamp(by1, 0, original_height - 1);
    bx2 = std::clamp(bx2, 0, original_width - 1);
    by2 = std::clamp(by2, 0, original_height - 1);

    // 유효한 바운딩 박스인지 확인
    if (bx2 <= bx1 || by2 <= by1)
    {
      continue;
    }

    float threshold = 0.5f;
    threshold = CONFIDENCE_THRESHOLDS.at(class_id);

    if (confidence < threshold)
    {
      continue;
    }

    // 저장
    bbox.class_ids.push_back(class_id);
    bbox.score.push_back(confidence);
    bbox.x1.push_back(bx1);
    bbox.y1.push_back(by1);
    bbox.x2.push_back(bx2);
    bbox.y2.push_back(by2);
    // 시각화
    cv::Point pt1(bx1, by1);
    cv::Point pt2(bx2, by2);

    cv::rectangle(bgr_image, cv::Rect(pt1, pt2), COLORS.at(class_id), 2);
  }

  bbox_pub_->publish(bbox);
  cv::imshow("Detection", bgr_image);
  cv::waitKey(1);
}

void DetectionNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  try
  {
    bgr_image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    imageProcessing();
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
