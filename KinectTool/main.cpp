// imgui
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

// opengl
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// std
#include <stdio.h>
#include <fstream>
#include <string>
#include <sstream>

// my classes
#include "MySkeleton.h"

bool testFlag = false;
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
	glfwSwapInterval(1); // Enable vsync

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

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

	// my classes
	MySkeleton* skeleton = new MySkeleton();
	skeleton->Init(window);
	skeleton->Start();

	// Main loop
	while (!glfwWindowShouldClose(window))
	{
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		{
			// ImGui inputs
			static char buf[128] = "";
			static bool doLoad = false;
			static char input_path[128] = "imgui.ini";
			static bool doSave = false;
			static char output_path[128] = "";
			static int holdTime = 0;
			static bool isClick = true;

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

			// row 1
			doLoad = ImGui::Button("Import"); ImGui::SameLine();
			ImGui::InputTextWithHint("Import Dir", "Import Dir", input_path, sizeof(input_path));

			// row 2
			snprintf(buf, sizeof(buf), "Bind to key[%s]", glfwGetKeyName(lastKey, 0));
			ImGui::InputInt("Hold Time", &holdTime); ImGui::SameLine();
			ImGui::Checkbox("Click Input?", &isClick); ImGui::SameLine();
			if(ImGui::Button(buf))
			{
				skeleton->Save(holdTime, lastKey, isClick);
				printf("Bind key: %s\n", glfwGetKeyName(lastKey, 0));
			}

			// row
			if (ImGui::Button("Clear"))
				skeleton->Clear();
			ImGui::SameLine();
			snprintf(buf, sizeof(buf), "Clear All %zu", skeleton->getSavedAmount());
			if (ImGui::Button(buf))
				skeleton->ClearAll();

			// row
			doSave = ImGui::Button("Export"); ImGui::SameLine();
			ImGui::InputTextWithHint("Export Dir", "Export Dir", output_path, sizeof(output_path));

			// row 
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
			ImGui::End();

			// logic
			if (doLoad)
			{
				skeleton->Import(input_path);
				std::swap(input_path, output_path);
				std::memset(input_path, 0, sizeof(input_path));
				doLoad = false;
			}
			 
			if (doSave)
			{
				skeleton->Export(output_path);
				doSave = false;
			}

			//ImGui::ShowDemoWindow();
		}

		// ImGui Rendering
		ImGui::Render();

		// GL stuff
		glViewport(0, 0, display_w, display_h);
		if (skeleton->hasData())
		{
			if (skeleton->isMatch())
				glClearColor(0.45f, 0.55f, 0.60f, 1.0f);
			else
				glClearColor(1.0f, 0.55f, 0.60f, 1.0f);
		}
		else
		{
			glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		}
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glUseProgram(shaderProgram);
		skeleton->Render();
		glUseProgram(0);

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

	glfwDestroyWindow(window);
	glfwTerminate();

	skeleton->Stop();
	delete(skeleton);

	return 0;
}