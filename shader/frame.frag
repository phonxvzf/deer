#version 130

in vec2 p_tex_coords;

out vec4 out_frag_color;

uniform sampler2D u_tex_sampler;

void main() {
  out_frag_color = texture(u_tex_sampler, p_tex_coords);
}
