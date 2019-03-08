#version 130

in vec3 in_position;
in vec3 in_normal;

uniform mat3 u_extrinsic_rot;
uniform mat4 u_model;
uniform mat4 u_view;
uniform vec3 u_extrinsic_trans;

mat3 intrinsic = mat3(
    6.6843589070435712e+02, 0, 0, // col 0 (fx)
    0, 6.6644989049880701e+02, 0, // col 1 (fy)
    3.1930978114340189e+02, 2.3191340221795991e+02, 1 // col 2 (cx, cy)
    );

// tangential distortion coefficients
vec2 p = vec2(-9.6922955423738855e-03, 4.0017843969807613e-04);
// radial distortion coefficients
vec3 k = vec3(2.6757419447697561e-02, -3.7005079720994571e-01, 5.9342104527500028e-01);

void main() {
  vec4 world = u_view * u_model * vec4(in_position, 1.0);
  float tmp = world.z;
  world.z = -world.y;
  world.y = tmp;
  vec3 proj = u_extrinsic_rot * world.xyz / 300.0 + u_extrinsic_trans;
  vec2 proj2d = proj.xy / proj.z;
  float rsq = proj2d.x * proj2d.x + proj2d.y * proj2d.y;
  float rquad = rsq * rsq;
  float rhex = rquad * rsq;
  float radial_dist = 1 + k[0] * rsq + k[1] * rquad + k[2] * rhex;

  vec3 pos;
  pos.x = proj2d.x * radial_dist + 2 * p[0] * proj2d.x * proj2d.y + p[1] * (rsq + 2 * proj2d.x * proj2d.x);
  pos.y = proj2d.y * radial_dist + 2 * p[1] * proj2d.x * proj2d.y + p[0] * (rsq + 2 * proj2d.y * proj2d.y);
  pos.z = 1.0;

  // undistort
  pos = intrinsic * pos;

  // convert to GL coordinates
  pos.x = (pos.x - 320) / 320;
  pos.y = (240 - pos.y) / 240;
  pos.z = 0.0;

  gl_Position = vec4(pos, 1.0);
}
