#ifdef GL_ES
precision mediump int;
precision mediump float;
#endif

varying vec3 v_normal;
varying vec3 v_position;

// Материал
uniform vec3 u_materialColor;
uniform float u_shininess;

uniform vec3 u_lightDirection;
uniform vec3 u_lightColor;
uniform float u_ambientStrength;

void main()
{
	vec3 n = normalize(v_normal);
	vec3 viewDir = normalize(-v_position);
	vec3 lightDir = normalize(u_lightDirection);

	// ------------------
	//   Ambient
	// ------------------
	vec3 ambient = u_ambientStrength * u_lightColor;

	// ------------------
	//   Diffuse
	// ------------------
	float diff = max(dot(n, lightDir), 0.0);
	vec3 diffuse = diff * u_lightColor;

	// ------------------
	//   Specular
	// ------------------
	vec3 reflectDir = reflect(-lightDir, n);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), u_shininess);
	vec3 specular = spec * u_lightColor;

	vec3 result = (ambient + diffuse + specular) * u_materialColor;

	gl_FragColor = vec4(result, 1.0);
}
