// imgui
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

// opengl
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// glm
#include <glm/glm.hpp>
#include <glm/ext.hpp>

// std
#include <cstdio>
#include <fstream>
#include <string>
#include <sstream>

// my classes
#include "MySkeleton.h"

int lastKey = 0;

/*************************************************************************************************/
/*                                     OpenGL CallBacks                                          */
/*************************************************************************************************/
static void glfw_error_callback(int error, const char* description)
{
	fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

static void cursor_position_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (ImGui::GetIO().WantCaptureMouse)
		return;

}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (ImGui::GetIO().WantCaptureKeyboard)
		return;

	if (action == GLFW_PRESS)
	{
		lastKey = key;
	}
}

/*************************************************************************************************/
/*                                     Helper Functions                                          */
/*************************************************************************************************/
static std::string get_file_contents(const char* filename) {
	std::ifstream ifs(filename, std::ios::in | std::ios::binary);
	if (ifs.is_open()) {
		std::ostringstream contents;
		contents << ifs.rdbuf();
		ifs.close();
		return(contents.str());
	}
	else
		printf("Cann't open file: %s!\n", filename);

	return "";
}

static void ShaderLog(GLuint shader)
{
	GLint isCompiled = 0;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &isCompiled);
	if (isCompiled == GL_FALSE)
	{
		GLint maxLength = 0;
		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &maxLength);

		// The maxLength includes the NULL character
		GLchar* errorLog = new GLchar[maxLength];
		glGetShaderInfoLog(shader, maxLength, &maxLength, &errorLog[0]);

		printf("%s\n", errorLog);
		delete[] errorLog;
		exit(1);
	}
}

static void RenderTriangle()
{
	static const std::array<float, 9> vertices = {
		-10.0f, -10.0f, 0.0f,
		 10.0f, -10.0f, 0.0f,
		 0.0f,  10.0f, 0.0f
	};

	GLuint VBO;
	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices.data());

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glDrawArrays(GL_TRIANGLES, 0, 3);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDeleteBuffers(1, &VBO);
}

/*************************************************************************************************/
/*                                     Main Function                                             */
/*************************************************************************************************/
int main(int, char**)
{
	glfwSetErrorCallback(glfw_error_callback);

	if (!glfwInit())
		return 1;

	// GL 3.0 + GLSL 130
	const char* glsl_version = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

	// Create window with graphics context
	GLFWwindow* window = glfwCreateWindow(1280, 720, "Kinect to Keyboard", nullptr, nullptr);
	if (!window)
	{
		glfwTerminate();
		return 1;
	}

	glfwMakeContextCurrent(window);
	gladLoadGL();

#ifdef _DEBUG
	glfwSwapInterval(1); // Enable vsync
#endif

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glEnable(GL_PROGRAM_POINT_SIZE);

	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetKeyCallback(window, key_callback);

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsLight();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	// shader program
	GLuint shaderProgram = glCreateProgram();
	std::string buf = "";

	// vertex shader
	GLuint vs = glCreateShader(GL_VERTEX_SHADER);
	buf = get_file_contents("basic.vs.glsl");
	const char* vsSource = buf.c_str();
	glShaderSource(vs, 1, &vsSource, NULL);
	glCompileShader(vs);
	ShaderLog(vs);
	glAttachShader(shaderProgram, vs);

	// fragment shader
	GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
	buf = get_file_contents("basic.fs.glsl");
	const char* fsSource = buf.c_str();
	glShaderSource(fs, 1, &fsSource, NULL);
	glCompileShader(fs);
	ShaderLog(fs);
	glAttachShader(shaderProgram, fs);

	glLinkProgram(shaderProgram);
	glDeleteShader(vs);
	glDeleteShader(fs);

	// my skeleton class
	MySkeleton* skeleton = new MySkeleton();
	skeleton->Init(window);
	skeleton->Start();

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		static int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		{
			// ImGui inputs
			static char str[128] = "";
			static char input_path[128] = "";
			static char output_path[128] = "";
			static std::array<bool, JOINTS>& checkList = skeleton->getCheckList();
			static float thresh = 0.5f;
			static int guiMode = RECORD;
			static const char* modeName[MODE_COUNT] = { "Record", "Execute" };

			// setup gui
			static const ImGuiWindowFlags windowsFlags =
				ImGuiWindowFlags_NoTitleBar |
				ImGuiWindowFlags_NoResize |
				ImGuiWindowFlags_NoMove |
				ImGuiWindowFlags_NoCollapse |
				ImGuiWindowFlags_NoSavedSettings;
			ImGui::Begin("Window", NULL, windowsFlags);
			ImGui::SetWindowPos(ImVec2(0, 0));
			ImGui::SetWindowSize(ImVec2(display_w, -1));

			ImGui::SliderInt("Mode", &guiMode, 0, MODE_COUNT - 1, modeName[guiMode]);
			skeleton->setMode(guiMode);

			if (guiMode == RECORD)
			{
				// row 1
				if (ImGui::Button("Import"))
				{
					skeleton->Import(input_path);
					std::swap(input_path, output_path);
					std::memset(input_path, 0, sizeof(input_path));
				}
				ImGui::SameLine(); ImGui::InputTextWithHint("Import Dir", "file.csv", input_path, sizeof(input_path));

				// row 2
				const char* keyName = glfwGetKeyName(lastKey, 0);
				snprintf(str, sizeof(str), "Bind to key[%s]", keyName);
				if (ImGui::Button(str))
				{
					skeleton->Save(lastKey);
					printf("Bind key: %s\n", keyName);
				}
				ImGui::SameLine();
				if (ImGui::Button("Clear"))
					skeleton->Clear();
				ImGui::SameLine();
				snprintf(str, sizeof(str), "Clear All %zu", skeleton->getSavedAmount());
				if (ImGui::Button(str))
					skeleton->ClearAll();

				if (ImGui::CollapsingHeader("Compare by orientation"))
				{
					ImGui::SliderFloat("Threshhold", &thresh, 0.0f, 2.0f);
#if defined(K4A)
					if (ImGui::BeginTable("split", 4))
					{
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_PELVIS", &checkList[K4ABT_JOINT_PELVIS]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_SPINE_NAVEL", &checkList[K4ABT_JOINT_SPINE_NAVEL]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_SPINE_CHEST", &checkList[K4ABT_JOINT_SPINE_CHEST]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_NECK", &checkList[K4ABT_JOINT_NECK]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_CLAVICLE_LEFT", &checkList[K4ABT_JOINT_CLAVICLE_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_SHOULDER_LEFT", &checkList[K4ABT_JOINT_SHOULDER_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_ELBOW_LEFT", &checkList[K4ABT_JOINT_ELBOW_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_WRIST_LEFT", &checkList[K4ABT_JOINT_WRIST_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_HAND_LEFT", &checkList[K4ABT_JOINT_HAND_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_HANDTIP_LEFT", &checkList[K4ABT_JOINT_HANDTIP_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_THUMB_LEFT", &checkList[K4ABT_JOINT_THUMB_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_CLAVICLE_RIGHT", &checkList[K4ABT_JOINT_CLAVICLE_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_SHOULDER_RIGHT", &checkList[K4ABT_JOINT_SHOULDER_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_ELBOW_RIGHT", &checkList[K4ABT_JOINT_ELBOW_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_WRIST_RIGHT", &checkList[K4ABT_JOINT_WRIST_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_HAND_RIGHT", &checkList[K4ABT_JOINT_HAND_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_HANDTIP_RIGHT", &checkList[K4ABT_JOINT_HANDTIP_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_THUMB_RIGHT", &checkList[K4ABT_JOINT_THUMB_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_HIP_LEFT", &checkList[K4ABT_JOINT_HIP_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_KNEE_LEFT", &checkList[K4ABT_JOINT_KNEE_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_ANKLE_LEFT", &checkList[K4ABT_JOINT_ANKLE_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_FOOT_LEFT", &checkList[K4ABT_JOINT_FOOT_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_HIP_RIGHT", &checkList[K4ABT_JOINT_HIP_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_KNEE_RIGHT", &checkList[K4ABT_JOINT_KNEE_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_ANKLE_RIGHT", &checkList[K4ABT_JOINT_ANKLE_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_FOOT_RIGHT", &checkList[K4ABT_JOINT_FOOT_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_HEAD", &checkList[K4ABT_JOINT_HEAD]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_NOSE", &checkList[K4ABT_JOINT_NOSE]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_EYE_LEFT", &checkList[K4ABT_JOINT_EYE_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_EAR_LEFT", &checkList[K4ABT_JOINT_EAR_LEFT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_EYE_RIGHT", &checkList[K4ABT_JOINT_EYE_RIGHT]);
						ImGui::TableNextColumn(); ImGui::Checkbox("K4ABT_JOINT_EAR_RIGHT", &checkList[K4ABT_JOINT_EAR_RIGHT]);
						ImGui::EndTable();
			}
#elif defined(K4W)
					if (ImGui::BeginTable("split", 4))
					{
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_SpineBase", &checkList[JointType_SpineBase]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_SpineMid", &checkList[JointType_SpineMid]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_Neck", &checkList[JointType_Neck]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_Head", &checkList[JointType_Head]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_ShoulderLeft", &checkList[JointType_ShoulderLeft]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_ElbowLeft", &checkList[JointType_ElbowLeft]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_WristLeft", &checkList[JointType_WristLeft]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_HandLeft", &checkList[JointType_HandLeft]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_ShoulderRight", &checkList[JointType_ShoulderRight]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_ElbowRight", &checkList[JointType_ElbowRight]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_WristRight", &checkList[JointType_WristRight]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_HandRight", &checkList[JointType_HandRight]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_HipLeft", &checkList[JointType_HipLeft]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_KneeLeft", &checkList[JointType_KneeLeft]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_AnkleLeft", &checkList[JointType_AnkleLeft]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_FootLeft", &checkList[JointType_FootLeft]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_HipRight", &checkList[JointType_HipRight]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_KneeRight", &checkList[JointType_KneeRight]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_AnkleRight", &checkList[JointType_AnkleRight]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_FootRight", &checkList[JointType_FootRight]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_SpineShoulder", &checkList[JointType_SpineShoulder]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_HandTipLeft", &checkList[JointType_HandTipLeft]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_ThumbLeft", &checkList[JointType_ThumbLeft]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_HandTipRight", &checkList[JointType_HandTipRight]);
						ImGui::TableNextColumn(); ImGui::Checkbox("JointType_ThumbRight", &checkList[JointType_ThumbRight]);
						ImGui::EndTable();
					}
#endif
		}
				skeleton->setThresh(thresh);

				// row
				if (ImGui::Button("Export"))
				{
					skeleton->Export(output_path);
				}
				ImGui::SameLine(); ImGui::InputTextWithHint("Export Dir", "file.csv", output_path, sizeof(output_path));
			}
			else
			{
				ImGui::SliderFloat("Threshhold", &thresh, 0.0f, 2.0f);
				skeleton->setThresh(thresh);
			}

			// row 
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
			ImGui::End();

			//ImGui::ShowDemoWindow();
		}

		// ImGui Rendering
		ImGui::Render();

		// GL stuff
		glViewport(0, 0, display_w, display_h);
		if (skeleton->hasMatch())
			glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
		else
			glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// parameter for mvp matrix
		static const glm::vec3 camPos(0.0f, 0.0f, 20.0f);
		static const glm::vec3 camTarget(0.0f, 0.0f, 0.0f);
		static const glm::vec3 camUp(0.0f, 1.0f, 0.0f);
		static const float fov = glm::radians(90.0f);

		// calc mvp matrix
		glm::mat4 projMatrix = glm::perspective(fov, (float)display_w / (float)display_h, 0.1f, 100.0f);
		glm::mat4 viewMatrix = glm::lookAt(camPos, camTarget, camUp);

		// statr shader program
		glUseProgram(shaderProgram);

		// - send mvp matrix
		glUniformMatrix4fv(
			glGetUniformLocation(shaderProgram, "uProj"),
			1,
			GL_FALSE,
			glm::value_ptr(projMatrix)
		);
		glUniformMatrix4fv(
			glGetUniformLocation(shaderProgram, "uView"),
			1,
			GL_FALSE,
			glm::value_ptr(viewMatrix)
		);

		// - render skeleton
		skeleton->Load2Shader();
		skeleton->Render(shaderProgram);
		//RenderTriangle();

		// - end shader program
		glUseProgram(0);

		// render gui last to show on top
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();

		// check for error
		GLenum error = glGetError();
		if (error != GL_NO_ERROR)
			printf("OpenGL error code�G0x%x\n", error);
	}

	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glDeleteProgram(shaderProgram);
	glfwDestroyWindow(window);
	glfwTerminate();

	skeleton->Stop();
	delete skeleton;

	return 0;
}