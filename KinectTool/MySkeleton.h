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

class MySkeleton {
public:		// data structures
	struct data {
		k4abt_skeleton_t pose;		// joint oreantion
		int key;					// bind to which key
	};

private:	// variables

	// main brain
	GLFWwindow* m_window;
	std::thread* m_thread;

	// camera
	k4a_device_t m_device;
	k4abt_tracker_t m_tracker;

	// poses data
	std::queue<k4abt_skeleton_t> m_skeletonLog;
	k4abt_skeleton_t* m_currentSkeleton;
	k4abt_skeleton_t* m_matchPose;
	std::vector<data> m_savedPose;
	std::array<bool, K4ABT_JOINT_COUNT> m_checkList;
	int m_failed;
	float m_jointThresh;

	// GL
	GLuint m_vbo;
	GLuint m_ebo;
	GLuint m_vbo_confidence;

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
	size_t getSavedAmount();

	// set data
	void setCheckList(const std::array<bool, K4ABT_JOINT_COUNT>& checkList);
	void setThresh(const float& thresh);

	// operations for poses
	void Clear();
	void ClearAll();
	void Save(int key);
	void Import(const char* path);
	bool Export(const char* path);

	// render functions
	void Load2Shader();
	void Render(const GLuint& program);

	// tools
	int CompareJoint(const k4abt_skeleton_t& lhs, const k4abt_skeleton_t& rhs);
};