#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> 

#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)

struct ObjectPos
{
  double dist;
  double theta;
};

// 2D → 3D 변환(거리 + 방향 추정)
ObjectPos calcObjectDistance(double tilt, double h, const cv::Point2d &focalLength, const cv::Point2d &principalPt, const cv::Point2d &pixelPt)
{

  //    cout << "********* calcObjectDistance *********" << endl;
  //    cout << "tilt = " << tilt << endl;
  //    cout << "h = " << h << endl;

  // ObjectPos 객체 선언
  ObjectPos pos;

  cv::Point2d normPt;
  //    cout << "object: " << pixelPt.x << " " << pixelPt.y << endl;
  //    cout << "principal: " << principalPt.x << " " << principalPt.y <<
  //    endl; cout << "focal length: " << focalLength.x << " " <<
  //    focalLength.y << endl;
  normPt.x = (pixelPt.x - principalPt.x) / focalLength.x;
  normPt.y = (pixelPt.y - principalPt.y) / focalLength.y;
  double u = normPt.x;
  double v = normPt.y;

  //    cout << "u = " << u << endl;
  //    cout << "v = " << v << endl;

  //    cout << "atan(v) = " << atan(v) * RAD2DEG << endl;

  double CC_ = h;
  double C_P_ = CC_ * tan((M_PI / 2) + tilt * DEG2RAD - atan(v));
  double CP_ = sqrt(CC_ * CC_ + C_P_ * C_P_);
  double Cp_ = sqrt(1 + v * v);
  double PP_ = u * CP_ / Cp_;

  //    cout << "CC_ = " << CC_ << endl;
  //    cout << "C_P_ = " << C_P_ << endl;
  //    cout << "CP_ = " << CP_ << endl;
  //    cout << "Cp_ = " << Cp_ << endl;
  //    cout << "PP_ = " << PP_ << endl;

  pos.dist = sqrt(C_P_ * C_P_ + PP_ * PP_);
  pos.theta = -atan2(PP_, C_P_) * RAD2DEG;

  //    cout << "obj.dist = " << obj.dist << endl;

  return pos;
}