#include <cstdint>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "main.hpp"
#include "globals.hpp"
#include "shader.hpp"
#include "glad.h"
#include "model.hpp"
#include "detect_routine.hpp"

void draw_scene(
    cv::Mat& frame,
    const cv::Mat& world_points,
    const cv::Mat& camera_matrix,
    const std::vector<double>& dist_coef,
    const std::vector<double>& rvec,
    const std::vector<double>& tvec)
{
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(world_points, rvec, tvec, camera_matrix, dist_coef, image_points);
  const cv::Scalar line_color(255, 0, 0);
  for (size_t i = 0; i < 3; ++i) {
    cv::line(frame, image_points[i], image_points[i+1], line_color, 5);
  }
  cv::line(frame, image_points[3], image_points[0], line_color, 5);
  for (size_t i = 4; i < 7; ++i) {
    cv::line(frame, image_points[i], image_points[i+1], line_color, 5);
  }
  cv::line(frame, image_points[7], image_points[4], line_color, 5);
  for (size_t i = 0; i < 4; ++i) {
    cv::line(frame, image_points[i], image_points[i+4], line_color, 5);
  }
  const cv::Scalar point_color(0, 0, 255);
  for (size_t i = 0; i < image_points.size(); ++i) {
    cv::circle(frame, image_points[i], 5, point_color, cv::FILLED);
  }
}

void add_alpha_channel(const cv::Mat& mat, cv::Mat& dst) {
  std::vector<cv::Mat> mat_channels;
  cv::split(mat, mat_channels);
  cv::Mat alpha(mat.rows, mat.cols, CV_8UC1, cv::Scalar(255));
  mat_channels.push_back(alpha);
  cv::merge(mat_channels, dst);
}

GLuint frame_texture;
GLuint frame_vao, frame_vbo, frame_ebo;
shader *frame_shader, *model_shader;
model *deer_model;
void init_gl(const int tx_width, const int tx_height) {
  //glEnable(GL_DEPTH_TEST);

  // Initialize frame VAO
  glGenVertexArrays(1, &frame_vao);
  glBindVertexArray(frame_vao);

  // Initialize frame VBO
  glGenBuffers(1, &frame_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, frame_vbo);
  const GLfloat vertices[] = {
    1.0f, 1.0f, 0.0f, 1.0f, 1.0f,
    1.0f, -1.0f, 0.0f, 1.0f, 0.0f,
    -1.0f, -1.0f, 0.0f, 0.0f, 0.0f,
    -1.0f, 1.0f, 0.0f, 0.0f, 1.0f
  };
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STREAM_READ | GL_STREAM_DRAW);

  // Initialize frame EBO
  glGenBuffers(1, &frame_ebo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, frame_ebo);
  const GLuint order[] = {
    0, 1, 2, 3, 0, 2
  };
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(order), order, GL_STREAM_READ | GL_STREAM_DRAW);
  
  // Initialize shader program
  frame_shader = new shader("shader/frame.vert", "shader/frame.frag");
  model_shader = new shader("shader/model.vert", "shader/model.frag");
  
  // Enable vertex attributes
  // [v][v][v][t][t]
  frame_shader->use();
  GLint vert_position_loc = glGetAttribLocation(frame_shader->program(), "in_position");
  GLint tex_coords_loc = glGetAttribLocation(frame_shader->program(), "in_tex_coords");
  const size_t stride = 5 * sizeof(GLfloat);
  glVertexAttribPointer(vert_position_loc, 3, GL_FLOAT, GL_FALSE, stride, 0);
  glVertexAttribPointer(tex_coords_loc, 2, GL_FLOAT, GL_FALSE, stride, 0);
  glEnableVertexAttribArray(vert_position_loc);
  glEnableVertexAttribArray(tex_coords_loc);

  // Unbind stuff
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

  // Initialize frame texture
  glGenTextures(1, &frame_texture);
  glBindTexture(GL_TEXTURE_2D, frame_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexImage2D(
      GL_TEXTURE_2D,
      0,
      GL_RGBA,
      tx_width,
      tx_height,
      0,
      GL_BGRA,
      GL_UNSIGNED_INT_8_8_8_8_REV,
      nullptr
      );
  glBindTexture(GL_TEXTURE_2D, 0);

  // Initialize deer model
  deer_model = new model("res/model/lowpolydeer/deer.obj", *model_shader);
}

void clean_gl() {
  deer_model->clean();
  frame_shader->clean();
  model_shader->clean();
  glDeleteTextures(1, &frame_texture);
  glDeleteBuffers(1, &frame_vao);
  glDeleteBuffers(1, &frame_vbo);
  glDeleteBuffers(1, &frame_ebo);
  delete frame_shader;
}

struct draw_data {
  cv::Mat *frame;
  cv::Mat *extrinsic_rot;
  float *extrinsic_trans;
  bool ready = false;
};

void draw_gl(void *params) {
  struct draw_data data = *static_cast<struct draw_data*>(params);

  cv::Mat frame = *data.frame;
  cv::flip(frame, frame, 0); // OpenGL will flip the texture vertically
  cv::Mat frame_walpha(frame.cols, frame.rows, CV_8UC4);
  add_alpha_channel(frame, frame_walpha);

  // feed frame texture to GPU
  glBindTexture(GL_TEXTURE_2D, frame_texture);
  glTexSubImage2D(
      GL_TEXTURE_2D,
      0,
      0,
      0,
      frame.cols,
      frame.rows,
      GL_BGRA,
      GL_UNSIGNED_INT_8_8_8_8_REV,
      frame_walpha.data
  );

  glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // draw frame
  glBindVertexArray(frame_vao);
  frame_shader->use();
  const GLint u_tex_sampler_loc = glGetUniformLocation(frame_shader->program(), "u_tex_sampler");
  glUniform1i(u_tex_sampler_loc, 0);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
  frame_shader->detach();
  glBindVertexArray(0);
  glBindTexture(GL_TEXTURE_2D, 0);

  // draw deer
  if (!data.ready) return;
  float extrinsic_rot_pod[9];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      extrinsic_rot_pod[3*i + j] = static_cast<float>(data.extrinsic_rot->at<double>(i, j));
    }
  }

  //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  model_shader->use();
  const GLint u_extrinsic_rot_loc = glGetUniformLocation(model_shader->program(), "u_extrinsic_rot");
  const GLint u_model_loc = glGetUniformLocation(model_shader->program(), "u_model");
  const GLint u_view_loc = glGetUniformLocation(model_shader->program(), "u_view");
  const GLint u_extrinsic_trans_loc = glGetUniformLocation(model_shader->program(), "u_extrinsic_trans");

  glm::mat4 model = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f));
  //glm::mat4 view = glm::lookAt(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  //glm::mat4 model = glm::mat4(1.0f);
  glm::mat4 view = glm::mat4(1.0f);
  glUniformMatrix4fv(u_model_loc, 1, GL_FALSE, glm::value_ptr(model));
  glUniformMatrix4fv(u_view_loc, 1, GL_FALSE, glm::value_ptr(view));
  glUniformMatrix3fv(u_extrinsic_rot_loc, 1, GL_TRUE, extrinsic_rot_pod);
  glUniform3fv(u_extrinsic_trans_loc, 1, data.extrinsic_trans);

  deer_model->draw();
  model_shader->detach();

  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void draw_debug(
    const cv::Mat& frame,
    const cv::Mat& world_points,
    const cv::Mat& camera_matrix,
    const std::vector<double>& dist_coef,
    const std::vector<double>& rvec,
    const std::vector<double>& tvec)
{
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(world_points, rvec, tvec, camera_matrix, dist_coef, image_points);
  for (const cv::Point2d& p : image_points) {
    cv::circle(frame, p, 2, cv::Scalar(0, 0, 255), 2, cv::FILLED);
  }
}

int main() {
  cv::namedWindow(MONITOR_TITLE, cv::WINDOW_FREERATIO | cv::WINDOW_OPENGL);
  cv::resizeWindow(MONITOR_TITLE, cv::Size(640, 480));
  cv::setOpenGlContext(MONITOR_TITLE);
  if (gladLoadGL()) {
    std::cout 
      << "OpenGL Loaded Successfully\n"
      << "=========================="
      << std::endl;
    std::cout << "Vendor\t\t: " << glGetString(GL_VENDOR) << std::endl;
    std::cout << "Renderer\t: " << glGetString(GL_RENDERER) << std::endl;
    std::cout << "GL Version\t: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "GLSL Version\t: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
  } else {
    std::cerr << "Unable to load GL" << std::endl;
    return 1;
  }

  cv::Mat camera_matrix(3, 3, CV_64FC1, (double*) CAM_MATRIX_VALUES);
  cv::VideoCapture cap(cv::CAP_V4L2);
  if (!cap.isOpened()) {
    std::cerr << "[ERROR] Webcam problems" << std::endl;
    return 1;
  }

  const int frame_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  const int frame_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  cv::resizeWindow(MONITOR_TITLE, frame_width, frame_height);
  init_gl(frame_width, frame_height);

  std::vector<double> rvec, tvec;
  double world_points_values[] = {
    // base
    -1.0, -1.0, 0.0,
    -1.0, 1.0, 0.0,
    1.0, 1.0, 0.0,
    1.0, -1.0, 0.0,
    // top
    -1.0, -1.0, -1.0,
    -1.0, 1.0, -1.0,
    1.0, 1.0, -1.0,
    1.0, -1.0, -1.0
  };

  const cv::Mat world_points(8, 3, CV_64FC1, world_points_values);
  std::vector<double> deer_positions_values;
  for (auto &v : deer_model->meshes().at(0).vertices()) {
    deer_positions_values.push_back(v.position[0] / 100);
    deer_positions_values.push_back(v.position[2] / 100);
    deer_positions_values.push_back(-v.position[1] / 100);
  }
  cv::Mat deer_positions(deer_positions_values.size() / 3, 3, CV_64FC1, deer_positions_values.data());
  draw_data params;
  cv::Mat frame;
  cv::Mat rvec_mat;
  float tvec_pod[3];
  int min_hue = 130, max_hue = 160, epsilon = 40;
  cv::setOpenGlDrawCallback(MONITOR_TITLE, draw_gl, &params);
  cv::createTrackbar("Min hue", MONITOR_TITLE, &min_hue, 360);
  cv::createTrackbar("Max hue", MONITOR_TITLE, &max_hue, 360);
  cv::createTrackbar("Epsilon", MONITOR_TITLE, &epsilon, 100);
  while (cap.isOpened()) {
    cap >> frame;
    params.frame = &frame;
    //if (detect_marker(frame, camera_matrix, DIST_COEF, rvec, tvec)) {
    if (detect_threshold(frame, min_hue, max_hue, epsilon, camera_matrix, DIST_COEF, rvec, tvec)) {
      // draw_scene(frame, world_points, camera_matrix, DIST_COEF, rvec, tvec);
      // draw_debug(frame, deer_positions, camera_matrix, DIST_COEF, rvec, tvec);
      cv::Rodrigues(rvec, rvec_mat); // convert rotation vector to rotation matrix
      tvec_pod[0] = tvec[0];
      tvec_pod[1] = tvec[1];
      tvec_pod[2] = tvec[2];
      params.extrinsic_rot = &rvec_mat;
      params.extrinsic_trans = tvec_pod;
      params.ready = true;
    } else {
      params.ready = false;
    }
    cv::updateWindow(MONITOR_TITLE);
    if (cv::waitKey(25) == 'q') break;
  }

  cv::setOpenGlDrawCallback(MONITOR_TITLE, nullptr);
  cv::destroyWindow(MONITOR_TITLE);
  clean_gl();
  return 0;
}
