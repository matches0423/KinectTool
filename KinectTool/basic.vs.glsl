#version 330

layout(location = 0) in vec3 iPos;
layout(location = 1) in int iCon;

uniform mat4 uProj = mat4(1.0);
uniform mat4 uView = mat4(1.0);
uniform mat4 uModel = mat4(1.0);

uniform vec4 uColor = vec4(1.0, 0.0, 0.0, 1.0);
uniform int uMode = 0;

out vec4 oColor;

void main(void)
{
	gl_Position = uProj * uView * uModel * vec4(iPos, 1.0);

	if(uMode == 1)
	{
		gl_PointSize = 5;
		if(iCon > 0)
		{
			oColor = vec4(0.0f, 1.0f, 0.0f, 1.0f);
		}
		else
		{
			oColor = vec4(1.0f, 0.0f, 0.0f, 1.0f);
		}
	}
	else if(uMode == 2)
	{
		gl_PointSize = 10;
		oColor = vec4(1.0f, 0.0f, 0.0f, 1.0f);
	}
	else
	{
		gl_PointSize = 1;
		oColor = uColor;
	}
}