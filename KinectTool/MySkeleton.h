#pragma once
// glfw
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define K4W
// kinect for azure
#if defined(K4A)
#include <k4a/k4a.h>
#include <k4abt.h>
#define JOINTS (int)K4ABT_JOINT_COUNT
typedef k4abt_skeleton_t skeleton_data;

// kinect for windows
#elif defined(K4W)
#include <Kinect.h>
#define JOINTS (int)JointType_Count
typedef struct {
	Joint joints[JOINTS];
	JointOrientation orientations[JOINTS];
} skeleton_data;
#endif

// std
#include <thread>
#include <vector>
#include <array>
#include <queue>

class MySkeleton {
public:		// data structures
	struct data {
		skeleton_data skeleton;		// joint oreantion
		int key;					// bind to which key
	};

private:	// variables

	// main brain
	GLFWwindow* m_window;
	std::thread* m_thread;

	// camera
#if defined(K4A)
	k4a_device_t m_device;
	k4abt_tracker_t m_tracker;
#elif defined(K4W)
	IKinectSensor* m_sensor;
	IBodyFrameReader* m_reader;
#endif

	// poses data
	std::queue<skeleton_data> m_skeletonLog;
	skeleton_data* m_currentSkeleton;
	skeleton_data* m_matchPose;
	std::vector<data> m_savedPose;
	std::array<bool, JOINTS> m_checkList;
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
	std::array<bool, JOINTS>& getCheckList();
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
	int CompareJoint(const skeleton_data& lhs, const skeleton_data& rhs);
};