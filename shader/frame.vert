#version 130

in vec3 in_position;
in vec2 in_tex_coords;

out vec2 p_tex_coords;

void main() {
  gl_Position = vec4(in_position * 2 + vec3(1.0, 1.0, 0.0), 1.0);
  p_tex_coords = in_tex_coords;
}
