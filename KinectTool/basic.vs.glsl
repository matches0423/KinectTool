#version 330

layout(location = 0) in vec3 iPosition;

void main(void)
{
	gl_Position = vec4(iPosition, 1.0f);
}