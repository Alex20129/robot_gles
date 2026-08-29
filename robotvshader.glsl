uniform mat4 mvp_matrix;
uniform mat3 normal_matrix;
attribute vec4 a_position;
attribute vec3 a_normal;
varying vec3 v_normal;

void main()
{
	gl_Position = mvp_matrix * a_position;
	v_normal = normalize(normal_matrix * a_normal);
}
