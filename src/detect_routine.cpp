#include <iostream>
#include <cmath>
#include <vector>
#include <numeric>
#include <algorithm>
#include <opencv2/aruco.hpp>

#include "detect_routine.hpp"
#include "globals.hpp"

size_t bench_4_count = 0;
size_t bench_count = 1;

constexpr float X_BOUND = 1.0;
constexpr float Y_BOUND = 1.603774;
const double PLANE_MATRIX_VALUES[] = {
  -X_BOUND, Y_BOUND, 0.0,
  -X_BOUND, -Y_BOUND, 0.0,
  X_BOUND, -Y_BOUND, 0.0,
  X_BOUND, Y_BOUND, 0.0
};

inline double distance(const cv::Point2i a, const cv::Point2i b) {
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

bool detect_marker(
    const cv::Mat& frame,
    const cv::Mat& camera_matrix,
    const std::vector<double>& dist_coef,
    std::vector<double>& out_rvec,
    std::vector<double>& out_tvec)
{
  cv::Mat gray;
  cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  const cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  cv::aruco::detectMarkers(gray, dict, corners, ids);

  if (corners.empty()) return false;

  cv::Mat world_points(4, 3, CV_64FC1, (double*) PLANE_MATRIX_VALUES);
  cv::solvePnP(world_points, corners[0], camera_matrix, dist_coef, out_rvec, out_tvec);

  return true;
}

bool detect_threshold(
    const cv::Mat& frame,
    const int min_hue,
    const int max_hue,
    const int epsilon,
    const cv::Mat& camera_matrix,
    const std::vector<double>& dist_coef,
    std::vector<double>& out_rvec,
    std::vector<double>& out_tvec,
    std::vector<cv::Point2f>& dbg_corners
    )
{
  cv::Mat frame_hsv;
  cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);
  
  cv::Mat thresh;
  cv::inRange(
      frame_hsv,
      cv::Scalar(min_hue * 180 / 360,  0 * 255 / 100, 0 * 255 / 100),
      cv::Scalar(max_hue * 180 / 360, 100 * 255 / 100, 100 * 255 / 100),
      thresh
      );

  std::vector<std::vector<cv::Point2i>> contours;
  cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) return false;

  int max_area = 0;
  int largest_contour_id = 0;
  cv::Rect largest_rect;
  cv::Rect bounding_rect;
  for (size_t i = 0; i < contours.size(); ++i) {
    bounding_rect = cv::boundingRect(contours[i]);
    if (bounding_rect.area() > max_area) {
      max_area = bounding_rect.area();
      largest_contour_id = i;
      largest_rect = bounding_rect;
    }
  }

  std::vector<cv::Point2i> hull;
  cv::convexHull(contours[largest_contour_id], hull);

  std::vector<cv::Point2i> approx_curve;
  cv::approxPolyDP(hull, approx_curve, epsilon, true);

  ++bench_count;
  std::cerr << "hit chance: " << (float) bench_4_count / bench_count << '\r';
  if (approx_curve.size() < 4) return false;

  bench_4_count += (approx_curve.size() == 4);

  cv::Point2i centroid(0, 0);
  for (const cv::Point2i& point : approx_curve) {
    centroid += point;
  }
  centroid /= (int) approx_curve.size();

  std::vector<double> angles, distances;
  for (const cv::Point2i& point : approx_curve) {
    angles.push_back(std::atan2(point.y - centroid.y, point.x - centroid.x));
    distances.push_back(distance(centroid, point));
  }

  // Sort points according to the distance from the centroid
  std::vector<size_t> idx(angles.size());
  std::vector<cv::Point2f> corners;
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&distances] (size_t i, size_t j) { return distances[i] > distances[j]; });

  // Sort points CCW
  std::sort(idx.begin(), idx.begin() + 4, [&angles] (size_t i, size_t j) { return angles[i] < angles[j]; });
  std::vector<cv::Point2f> sorted_points;
  for (size_t i = 0; i < 4; ++i) {
    sorted_points.push_back(approx_curve[idx[i]]);
  }

  // Solve PnP
  cv::Mat world_points(4, 3, CV_64FC1, (double*) PLANE_MATRIX_VALUES);
  cv::solvePnP(world_points, sorted_points, camera_matrix, dist_coef, out_rvec, out_tvec);

  dbg_corners = std::vector<cv::Point2f>(sorted_points);
  
  return true;
}
