#pragma once
// glfw
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define K4A
//#define K4W

#ifdef K4A
#include <k4a/k4a.h>
#include <k4abt.h>
#elif K4W

#endif

// std
#include <thread>
#include <vector>
#include <array>
#include <queue>
#include <mutex>

class MySkeleton {
public:		// data structures
	struct data {
		k4abt_skeleton_t pose;		// joint oreantion
		int holdTime;				// how long the user should hold this pose
		int key;					// bind to which key
		bool isClick;				// click input or hold input
	};

private:	// variables

	// main brain
	GLFWwindow* m_window;
	std::thread* m_thread;

	// camera
#ifdef K4A
	k4a_device_t m_device;
	k4abt_tracker_t m_tracker;
#elif K4W

#endif

	// poses data
	std::queue<k4abt_skeleton_t> m_poseLog;
	k4abt_skeleton_t* m_currentPose;
	std::vector<data> m_savedPose;

	// data
	bool m_isMatch;

	// render
	GLuint m_vao;
	GLuint m_vbo;
	GLuint m_ebo;
	std::mutex m_bufferBusy;

public:		// functions

	// constructer
	MySkeleton();
	~MySkeleton();

	// operations
	void Init(GLFWwindow* window);
	void Start();
	void Update();
	void Stop();

	// get data
	bool hasData();
	bool isMatch();
	size_t getSavedAmount();

	// operations for poses
	void Clear();
	void ClearAll();
	void Save(int holdTime, int key, bool isClick);
	void Import(const char* path);
	bool Export(const char* path);

	// render functions
	void Load2Shader(const k4abt_skeleton_t& skeleton);
	void Render();

	// tools
	static bool CompareJoint(const k4abt_skeleton_t& lhs, const k4abt_skeleton_t& rhs, float thresh = 1.0);
};