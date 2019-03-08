#ifndef _SHADER_HPP
#define _SHADER_HPP

#include <string>

#include "glad.h"

class shader {
  public:
    enum shader_type {
      VERTEX_SHADER = GL_VERTEX_SHADER,
      FRAGMENT_SHADER = GL_FRAGMENT_SHADER
    };

    shader() {};
    shader(const std::string& vert_path, const std::string& frag_path);

    int compile_shader(const std::string& file_path, shader_type type);
    int program() const;
    void use() const;
    void detach() const;
    void clean();

  private:
    GLint m_vertex_shader;
    GLint m_fragment_shader;
    GLuint m_program;
};

#endif /* _SHADER_HPP */
