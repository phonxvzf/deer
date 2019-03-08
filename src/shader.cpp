#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <cstring>

#include "shader.hpp"

shader::shader(const std::string& vert_path, const std::string& frag_path) {
  m_vertex_shader = compile_shader(vert_path, shader_type::VERTEX_SHADER);
  m_fragment_shader = compile_shader(frag_path, shader_type::FRAGMENT_SHADER);
  
  if (m_vertex_shader > 0 && m_fragment_shader > 0) {
    m_program = glCreateProgram();
    glAttachShader(m_program, m_vertex_shader);
    glAttachShader(m_program, m_fragment_shader);
    glLinkProgram(m_program);
  } else {
    std::cerr << "[ERROR] Error during shader program creation" << std::endl;
    std::exit(1);
  }
}

int shader::compile_shader(const std::string& file_path, shader_type type) {
  std::ifstream ifs(file_path.c_str());
  int compile_status = false;
  int info_log_length;
  GLuint shader = glCreateShader(type);

  std::cout << "[INFO] Compiling " << file_path << std::endl;

  if (ifs.is_open()) {
    std::stringstream shader_code;
    shader_code << ifs.rdbuf();
    ifs.close();

    char *codes[1];
    codes[0] = new char[shader_code.str().length() + 1];
    std::memcpy(codes[0], shader_code.str().c_str(), shader_code.str().length());
    codes[0][shader_code.str().length()] = 0;

    const int code_size = shader_code.str().length() + 1;
    glShaderSource(shader, 1, codes, &code_size);
    glCompileShader(shader);

    delete[] codes[0];

    glGetShaderiv(shader, GL_COMPILE_STATUS, &compile_status);

    if (!compile_status) {
      glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &info_log_length);
      char compilation_log[info_log_length];
      glGetShaderInfoLog(shader, info_log_length, nullptr, compilation_log);
      std::cerr << "[ERROR] Failed to compile shader: " << compilation_log;
      return -1;
    }
  } else {
    std::cerr << "[ERROR] Cannot open shader source: " << file_path << std::endl;
    return -1;
  }

  return shader;
}

int shader::program() const {
  return m_program;
}

void shader::use() const {
  glUseProgram(m_program);
}

void shader::detach() const {
  glUseProgram(0);
}

void shader::clean() {
  glDeleteProgram(m_program);
  glDeleteShader(m_vertex_shader);
  glDeleteShader(m_fragment_shader);
}
