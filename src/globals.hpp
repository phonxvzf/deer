#ifndef _GLOBALS_HPP
#define _GLOBALS_HPP

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

const cv::String MONITOR_TITLE = "Monitor";

const double CAM_MATRIX_VALUES[] = {
  6.6843589070435712e+02, 0., 3.1930978114340189e+02,
  0., 6.6644989049880701e+02, 2.3191340221795991e+02,
  0., 0., 1.
};

const std::vector<double> DIST_COEF = {
  2.6757419447697561e-02, -3.7005079720994571e-01,
  -9.6922955423738855e-03, 4.0017843969807613e-04,
  5.9342104527500028e-01
};

#endif /* _GLOBALS_HPP */
