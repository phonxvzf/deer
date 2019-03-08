#ifndef _MODEL_HPP
#define _MODEL_HPP

#include <vector>
#include <string>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "glad.h"
#include "shader.hpp"

struct vertex {
  GLfloat position[3];
  GLfloat normal[3];
};

class mesh {
  private:
    GLuint m_vao, m_vbo, m_ebo;
    std::vector<struct vertex> m_vertices;
    std::vector<GLuint> m_indices;
    shader m_shader;
    
  public:
    mesh(
        const std::vector<struct vertex>& vertices,
        const std::vector<GLuint>& indices,
        const shader& model_shader
        );
    void draw();
    void clean();
    std::vector<struct vertex> vertices() { return m_vertices; }
};

class model {
  private:
    shader m_shader;
    std::vector<mesh> m_meshes;
    void process_node(aiNode *node, const aiScene *scene);
    mesh process_mesh(aiMesh *mesh);

  public:
    model(const std::string& file_path, const shader& model_shader);
    void draw();
    void clean();
    std::vector<mesh> meshes() { return m_meshes; }
};

#endif /* _MODEL_HPP */
