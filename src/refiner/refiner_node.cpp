#include "refiner/refiner_node.hpp"

RefinerNode::RefinerNode() : Node("refiner_node")
{
  visionPub = this->create_publisher<humanoid_interfaces::msg::Robocupvision25>(
      "vision", 10);
  vision_feature_Pub =
      this->create_publisher<humanoid_interfaces::msg::Robocupvision25feature>(
          "vision_feature", 10);
  visionSub =
      this->create_subscription<humanoid_interfaces::msg::Master2vision25>(
          "master2vision", 10,
          std::bind(&RefinerNode::master_callback, this,
                    std::placeholders::_1));
  bbox_sub_ = this->create_subscription<robocup_vision::msg::BoundingBox>(
      "/Bounding_box", 10,
      std::bind(&RefinerNode::bboxCallback, this, std::placeholders::_1));
  pan_tilt_sub_ = this->create_subscription<robocup_vision::msg::PanTilt>(
      "/PanTilt", 10,
      std::bind(&RefinerNode::pan_tilt_Callback, this, std::placeholders::_1));
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/camera/image_raw", 10,
      std::bind(&RefinerNode::imageCallback, this, std::placeholders::_1));
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera1/info", 10,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
      {
        K_M = cv::Mat(3, 3, CV_64F, (void *)msg->k.data()).clone();
        D_M = cv::Mat(1, msg->d.size(), CV_64F, (void *)msg->d.data()).clone();
        R_M = cv::Mat(3, 3, CV_64F, (void *)msg->r.data()).clone();
        P_M = cv::Mat(3, 4, CV_64F, (void *)msg->p.data()).clone();

        focalLen.x = K_M.at<double>(0, 0);
        focalLen.y = K_M.at<double>(1, 1);
        prncPt.x = K_M.at<double>(0, 2);
        prncPt.y = K_M.at<double>(1, 2);

        width = msg->width;   // 1920
        height = msg->height; // 1080

        NEW_K_M = getOptimalNewCameraMatrix(K_M, D_M, cv::Size(width, height), 0.3, cv::Size(width, height), 0);
      });

  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RefinerNode::timerCallback, this)); // Modify this line

  std::stringstream ss;
  ss << NEW_K_M;
  RCLCPP_INFO(this->get_logger(), "New Camera Matrix:\n%s", ss.str().c_str());

  RCLCPP_INFO(this->get_logger(), "RefinerNode initialized.");
}

void RefinerNode::timerCallback()
{
  // PRE CONDITION : filter_cnt, fst_filter_cnt, fst_filter_N, ballPos.dist
  // POST CONDITION : ball_speed_vec_N, ball_speed_level
  // PURPOSE : 이동평균 필터를 적용하여 캠의 오차 값을 보정하고 이동중인 물체를 포착
  if (filter_cnt >= 9)
  {
    // 해당 함수는 100ms마다 실행되고 총 10번 실행시 fps계산
    RCLCPP_INFO(this->get_logger(), "fps_cnt : %d, nice_cnt : %d", fps_cnt, nice_cnt);
    // if (fps_cnt == 0)
    // {
    //     ocam_error = 1;
    // }
    // else
    // {
    //     ocam_error = 0;
    // }
    fps_cnt = 0;
    filter_cnt = 0;
    nice_cnt = 0;
  }
  if (fst_filter_cnt != 0)
  {
    sec_filter_x += fst_filter_x / fst_filter_cnt;
    sec_filter_y += fst_filter_y / fst_filter_cnt;
    sec_filter_cnt += 1;
    fst_filter_x = 0;
    fst_filter_y = 0;
    fst_filter_cnt = 0;
  }

  if (filter_cnt == 0 || filter_cnt == 3 || filter_cnt == 6)
  {
    if (sec_filter_cnt != 0)
    {
      ball_speed_vec_x = final_filter_x;
      ball_speed_vec_y = final_filter_y;

      final_filter_x = sec_filter_x / sec_filter_cnt;
      final_filter_y = sec_filter_y / sec_filter_cnt;
      sec_filter_x = 0;
      sec_filter_y = 0;
      sec_filter_cnt = 0;

      ball_speed_vec_x = ball_speed_vec_x - final_filter_x;
      ball_speed_vec_y = ball_speed_vec_y - final_filter_y;
      double dis = sqrt(pow(ball_speed_vec_x, 2) + pow(ball_speed_vec_y, 2));
      if (dis != 0)
      {
        ball_speed_vec_x /= dis;
        ball_speed_vec_y /= dis;
      }
      else
      {
        ball_speed_vec_x = 0;
        ball_speed_vec_y = 0;
      }
      if (ballPos.dist == 0)
      {
        ball_speed_level = 0;
      }
      else
      {
        double tempdist = ballPos.dist;
        if (tempdist > 1500)
        {
          tempdist = 1500;
        }
        ball_speed_level = (int)(dis / 5) * abs(1500 - tempdist) / 500;
      }
    }
  }
  RCLCPP_INFO(this->get_logger(), "Filter count: %d", filter_cnt);
  filter_cnt += 1;
}

void RefinerNode::pan_tilt_Callback(const robocup_vision::msg::PanTilt::SharedPtr msg)
{
  tilt_deg = msg->tilt;
}

void RefinerNode::publish_localization_msg()
{
  // PRE CONDITION : vision_feature_Msg
  // POST CONDITION : vision_feature_Msg
  // PURPOSE : 로컬 노드에 특징점의 데이터를 보내기 위해 데이터 PUBLISH

  // std::cout << "publish_localization_msg" << std::endl;
  vision_feature_Pub->publish(vision_feature_Msg);

  vision_feature_Msg.confidence.clear();
  vision_feature_Msg.distance.clear();
  vision_feature_Msg.point_vec_x.clear();
  vision_feature_Msg.point_vec_y.clear();
}

void RefinerNode::publish_vision_msg()
{
  // PRE CONDITION : visionMsg
  // POST CONDITION : visionMsg
  // PURPOSE : 로컬 노드에 로봇의 데이터를 보내기 위해 데이터 PUBLISH

  visionMsg.ball_cam_x = ball_cam_X;
  visionMsg.ball_cam_y = ball_cam_Y;
  if (ballPos.dist == 0)
  {
    visionMsg.ball_2d_x = 0;
    visionMsg.ball_2d_y = 0;
  }
  else
  {
    visionMsg.ball_2d_x = final_filter_x;
    visionMsg.ball_2d_y = final_filter_y;
  }
  visionMsg.ball_d = ballPos.dist;
  visionMsg.pan = pan_tilt.ptpos.PAN_POSITION;
  visionMsg.tilt = pan_tilt.ptpos.TILT_POSITION;
  visionMsg.ball_speed_x = ball_speed_vec_x;
  visionMsg.ball_speed_y = ball_speed_vec_y;
  visionMsg.ball_speed_level = ball_speed_level;
  // visionMsg.scan_mode = scan_value;

  visionPub->publish(visionMsg);

  visionMsg.robot_vec_x.clear();
  visionMsg.robot_vec_y.clear();
}

void RefinerNode::bboxProcessing()
{
  // 바운딩 박스 Callback 받은 후 값 가공
  //
  // ===== Ball 관련 =====
  if (Detections_ball_.size() > 0)
  {
    nice_cnt += 1; // 공이 검출될 경우 nice_cnt 에 +1

    const auto &bbox = Detections_ball_[0].bbox;
    int ball_center_X = bbox.x + bbox.width / 2;
    int ball_center_Y = bbox.y + bbox.height / 2;

    remove_rect = cv::Rect(bbox.x, bbox.y, bbox.width, bbox.height);

    // ======================
    // 거리 및 절대 좌표 계산
    // ======================
    // 공의 이미지 좌표를 `Point2f` 형식으로 생성하여 리스트에 저장
    ball_pts.push_back(cv::Point2f(ball_center_X, ball_center_Y));

    // 카메라 렌즈 왜곡 보정
    undistortPoints(ball_pts, ball_pts, K_M, D_M, cv::Mat(), NEW_K_M);

    ballPos = calcObjectDistance(
        pan_tilt.ptpos.TILT_POSITION,
        ROBOT_HEIGHT + TILT_L * (cos(pan_tilt.ptpos.TILT_POSITION * DEG2RAD) - 1),
        focalLen,
        prncPt,
        cv::Point2f(ball_pts[0].x, ball_pts[0].y));

    // 공에 이동평균 필터 적용
    // 상대 좌표 계산
    pan_tilt.target_x = ballPos.dist * sin(ballPos.theta * M_PI / 180);
    pan_tilt.target_y = ballPos.dist * cos(ballPos.theta * M_PI / 180);

    // 로봇 기준 절대 좌표로 변환
    pan_tilt.target_absx = pan_tilt.target_x * cos((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180) - pan_tilt.target_y * sin((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180);
    pan_tilt.target_absy = pan_tilt.target_x * sin((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180) + pan_tilt.target_y * cos((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180);

    if (ball_filter_cnt < 30)
    {

      ball_filter_x[ball_filter_cnt] = ball_center_X;
      ball_filter_y[ball_filter_cnt] = ball_center_Y;
      ball_cam_X = ball_center_X;
      ball_cam_Y = ball_center_Y;
      ball_filter_cnt++;
    }
    else
    {
      if (ball_center_X == 0 && ball_center_Y == 0)
      {
        ball_cam_X = ball_center_X;
        ball_cam_Y = ball_center_Y;
      }
      else
      {
        ball_filter_x[ball_filter_idx] = ball_center_X;
        ball_filter_y[ball_filter_idx] = ball_center_Y;
        for (int i = 0; i < 30; i++)
        {
          ball_cam_X += ball_filter_x[i];
          ball_cam_Y += ball_filter_y[i];
        }
        ball_cam_X /= 30;
        ball_cam_Y /= 30;
      }

      if (ball_filter_idx > 29)
      {
        ball_filter_idx = 0;
      }
      else
      {
        ball_filter_idx++;
      }
    }

    // 필터 적용 후 최종적으로 나온 데이터 저장
    ball_cam_X = ball_center_X;
    ball_cam_Y = ball_center_Y;

    // 캠의 이동평균 필터를 위한 변수 저장
    fst_filter_x += pan_tilt.target_absx;
    fst_filter_y += pan_tilt.target_absy;
    fst_filter_cnt += 1;
  }
  else // 공이 카운트 되지 않았을 경우 실행
  {
    ballPos.dist = 0;
    pan_tilt.target_x = 0;
    pan_tilt.target_y = 0;
    pan_tilt.target_absx = 0;
    pan_tilt.target_absy = 0;
    ball_cam_X = 0;
    ball_cam_Y = 0;
  }

  // ===== line 관련 =====
  if (Detections_line_.size() > 0)
  {
    for (size_t i = 0; i < Detections_line_.size(); i++)
    {
      // 라인의 바운딩 박스 좌표를 사용하여 중심점 계산
      const auto &bbox = Detections_line_[i].bbox;
      double line_center_X = bbox.x + bbox.width / 2;
      double line_center_Y = bbox.y + bbox.height / 2;

      if ((line_center_X > remove_rect.x + remove_rect.width || line_center_X < remove_rect.x) ||
          (line_center_Y > remove_rect.y + remove_rect.height || line_center_Y < remove_rect.y))
      {
        line_pts.push_back(cv::Point2f(line_center_X, line_center_Y));
        line_condis.push_back(cv::Point2f(Detections_line_[i].score, 0));
      }
    }

    if (!line_pts.empty())
    {
      // 카메라 렌즈 왜곡 보정
      cv::undistortPoints(line_pts, line_pts, K_M, D_M, cv::Mat(), NEW_K_M);
    }

    for (size_t i = 0; i < line_pts.size(); i++) // 저장된 데이터 수 만큼 반복
    {
      // 특징점의 거리 계산
      linePos = calcObjectDistance(
          pan_tilt.ptpos.TILT_POSITION,
          ROBOT_HEIGHT + TILT_L * (cos(pan_tilt.ptpos.TILT_POSITION * DEG2RAD) - 1),
          focalLen,
          prncPt,
          cv::Point2f(line_pts[i].x, line_pts[i].y));

      double line_x = linePos.dist * sin(linePos.theta * M_PI / 180);
      double line_y = linePos.dist * cos(linePos.theta * M_PI / 180);

      double line_absx = line_x * cos((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180) - line_y * sin((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180);
      double line_absy = line_x * sin((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180) + line_y * cos((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180);

      // 일정 거리 이상에 존재하는 특징점은 예외 처리
      int remove_space_dis = 3000;

      if (linePos.dist < remove_space_dis)
      {
        // 로컬 노드에 특징점의 데이터를 보내기 위해 데이터 PUBLISH
        vision_feature_Msg.confidence.push_back(line_condis[i].x);
        vision_feature_Msg.distance.push_back(linePos.dist);
        vision_feature_Msg.point_vec_x.push_back(line_absx);
        vision_feature_Msg.point_vec_y.push_back(line_absy);
      }
    }

    // 벡터 초기화
    line_pts.clear();
    line_condis.clear();

    // publish_localization_msg();
  }

  // ===== robot 관련 =====
  if (Detections_robot_.size() > 0)
  {
    for (size_t i = 0; i < Detections_robot_.size(); i++)
    {
      const auto &bbox = Detections_robot_[i].bbox;

      // 로봇의 중심 좌표 계산
      // 바운딩 박스의 “중심점(centroid)”을 아래쪽(바닥 쪽)으로 얼마나 더 내릴지 결정하는 픽셀 오프셋
      double tilt_param = bbox.height / 2 * (1 - (double(bbox.y) / height));
      double robot_center_X = bbox.x + bbox.width / 2;
      double robot_center_Y = bbox.y + bbox.height / 2 + tilt_param;

      // 로봇 좌표 추가
      robot_pts.push_back(cv::Point2f(robot_center_X, robot_center_Y));
    }

    // visionMsg의 로봇 벡터 컨테이너 초기화
    visionMsg.robot_vec_x.clear();
    visionMsg.robot_vec_y.clear();

    // 렌즈 왜곡 보정
    if (!robot_pts.empty())
    {
      undistortPoints(robot_pts, robot_pts, K_M, D_M, cv::Mat(), NEW_K_M);
    }
    for (size_t i = 0; i < robot_pts.size(); i++)
    {
      // 로봇의 거리 및 절대 좌표 계산
      robotPos = calcObjectDistance(
          pan_tilt.ptpos.TILT_POSITION,
          ROBOT_HEIGHT + TILT_L * (cos(pan_tilt.ptpos.TILT_POSITION * DEG2RAD) - 1),
          focalLen,
          prncPt,
          cv::Point2f(robot_pts[i].x, robot_pts[i].y));

      double robot_x = robotPos.dist * sin(robotPos.theta * M_PI / 180);
      double robot_y = robotPos.dist * cos(robotPos.theta * M_PI / 180);

      robot_absx = robot_x * cos((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180) - robot_y * sin((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180);
      robot_absy = robot_x * sin((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180) + robot_y * cos((-1) * pan_tilt.ptpos.PAN_POSITION * M_PI / 180);

      // visionMsg의 로봇 벡터 컨테이너에 데이터 저장
      visionMsg.robot_vec_x.push_back(robot_absx);
      visionMsg.robot_vec_y.push_back(robot_absy);
    }
  }

  else // 로봇(obstacle)이 카운트 되지 않았을 경우 실행
  {
    // RCLCPP_INFO(this->get_logger(), "No robot detected.");
    visionMsg.robot_vec_x.clear();
    visionMsg.robot_vec_y.clear();
    robot_absx = 0;
    robot_absy = 0;
  }

  publish_vision_msg();
}

void RefinerNode::bboxCallback(const robocup_vision::msg::BoundingBox::SharedPtr msg)
{
  Detections_ball_.clear();
  Detections_line_.clear();
  Detections_robot_.clear();

  ball_most_confidence = 0;

  size_t num_boxes = msg->class_ids.size();

  for (size_t i = 0; i < num_boxes; i++)
  {
    DetectionResult det;
    det.class_id = msg->class_ids[i];
    det.score = msg->score[i];
    det.bbox = cv::Rect(msg->x1[i], msg->y1[i], msg->x2[i] - msg->x1[i], msg->y2[i] - msg->y1[i]);

    if (det.class_id == 0)
    {
      if (ball_most_confidence < det.score)
      {
        Detections_ball_.clear();
        Detections_ball_.push_back(det);
        ball_most_confidence = det.score;
      }
    }
    else if (det.class_id == 1)
    {
      Detections_line_.push_back(det);
    }
    else if (det.class_id == 2)
    {
      Detections_robot_.push_back(det);
    }
  }

  if (num_boxes != 0)
  {
    bboxProcessing();
  }
}

void RefinerNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  try
  {
    bgr_image = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    //bboxProcessing();
    fps_cnt += 1;
  }
  catch (const cv_bridge::Exception &e)
  {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void RefinerNode::master_callback(const humanoid_interfaces::msg::Master2vision25::SharedPtr msg)
{
  // if (scan_value == 4) {
  //   pan_tilt.ptpos.PAN_POSITION = msg->pan;
  //   pan_tilt.ptpos.TILT_POSITION = TILT_D;
  // }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RefinerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
