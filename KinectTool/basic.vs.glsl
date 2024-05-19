#version 330

layout(location = 0) in vec3 iPos;

uniform mat4 uProj = mat4(1.0);
uniform mat4 uView = mat4(1.0);
uniform mat4 uModel = mat4(1.0);

void main(void)
{
	gl_Position = uProj * uView * uModel * vec4(iPos, 1.0);
}