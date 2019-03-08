#include <cstdlib>
#include <iostream>

#include "model.hpp"

mesh::mesh(
    const std::vector<struct vertex>& vertices,
    const std::vector<GLuint>& indices,
    const shader& model_shader
    )
  : m_vertices(vertices), m_indices(indices), m_shader(model_shader)
{
  m_vertices = vertices;
  m_indices = indices;
  m_shader = model_shader;
  glGenVertexArrays(1, &m_vao);
  glGenBuffers(1, &m_vbo);
  glGenBuffers(1, &m_ebo);

  glBindVertexArray(m_vao);
  glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
  glBufferData(
      GL_ARRAY_BUFFER,
      sizeof(struct vertex) * vertices.size(),
      vertices.data(),
      GL_STATIC_DRAW
      );
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
  glBufferData(
      GL_ELEMENT_ARRAY_BUFFER,
      sizeof(GLuint) * indices.size(),
      indices.data(),
      GL_STATIC_DRAW
      );
  model_shader.use();
  GLint in_position_loc = glGetAttribLocation(model_shader.program(), "in_position");
  GLint in_normal_loc = glGetAttribLocation(model_shader.program(), "in_normal");
  glVertexAttribPointer(
      in_position_loc,
      3,
      GL_FLOAT,
      GL_FALSE,
      sizeof(struct vertex),
      nullptr
      );
  glVertexAttribPointer(
      in_normal_loc,
      3,
      GL_FLOAT,
      GL_FALSE,
      sizeof(struct vertex),
      nullptr
      );
  glEnableVertexAttribArray(in_position_loc);
  glEnableVertexAttribArray(in_normal_loc);
  model_shader.detach();
  glBindVertexArray(0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void mesh::draw() {
  m_shader.use();
  glBindVertexArray(m_vao);
  glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
  m_shader.detach();
}

void mesh::clean() {
  glDeleteVertexArrays(1, &m_vao);
  glDeleteBuffers(1, &m_vbo);
  glDeleteBuffers(1, &m_ebo);
}

model::model(const std::string& file_path, const shader& model_shader)
  : m_shader(model_shader)
{
  Assimp::Importer importer;
  const aiScene *scene = importer.ReadFile(file_path, aiProcess_Triangulate | aiProcess_FlipUVs);
  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
    std::cerr << "[ERROR] Assimp error: " << importer.GetErrorString() << std::endl;
    std::exit(1);
  } else {
    process_node(scene->mRootNode, scene);
  }
}

void model::process_node(aiNode *node, const aiScene *scene) {
  for (size_t i = 0; i < node->mNumMeshes; ++i) {
    aiMesh *m = scene->mMeshes[node->mMeshes[i]];
    m_meshes.push_back(process_mesh(m));
  }

  for (size_t i = 0; i < node->mNumChildren; ++i) {
    process_node(node->mChildren[i], scene);
  }
}

mesh model::process_mesh(aiMesh *a_mesh) {
  std::vector<struct vertex> vertices;
  std::vector<GLuint> indices;

  for (size_t i = 0; i < a_mesh->mNumVertices; ++i) {
    vertices.push_back({
        {a_mesh->mVertices[i].x, a_mesh->mVertices[i].y, a_mesh->mVertices[i].z},
        {0, 0, 0}
        });
  }

  for (size_t i = 0; i < a_mesh->mNumFaces; ++i) {
    aiFace face = a_mesh->mFaces[i];
    for (size_t j = 0; j < face.mNumIndices; ++j) {
      indices.push_back(face.mIndices[j]);
    }
  }

  return mesh(vertices, indices, m_shader);
}

void model::draw() {
  for (mesh& m : m_meshes) {
    m.draw();
  }
}

void model::clean() {
  for (mesh& m : m_meshes) {
    m.clean();
  }
}
