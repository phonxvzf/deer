#ifndef _DETECT_ROUTINE_HPP
#define _DETECT_ROUTINE_HPP

#include <opencv2/opencv.hpp>

bool detect_marker(
    const cv::Mat& frame,
    const cv::Mat& camera_matrix,
    const std::vector<double>& dist_coef,
    std::vector<double>& out_rvec,
    std::vector<double>& out_tvec);

bool detect_threshold(
    const cv::Mat& frame,
    const int min_hue,
    const int max_hue,
    const int epsilon,
    const cv::Mat& camera_matrix,
    const std::vector<double>& dist_coef,
    std::vector<double>& out_rvec,
    std::vector<double>& out_tvec
    );

#endif /* _DETECT_ROUTINE_HPP */
