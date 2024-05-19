#version 330

out vec4 FragColor;

uniform vec4 uColor = vec4(1.0, 0.0, 0.0, 1.0);

void main(void)
{
	FragColor = uColor;
}