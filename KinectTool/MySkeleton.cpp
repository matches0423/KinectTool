#include "MySkeleton.h"

#include <fstream>
#include <cmath>

#define VERIFY(result, error)                                                                            \
	if (result != K4A_RESULT_SUCCEEDED)                                                                  \
	{                                                                                                    \
		printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
		exit(1);                                                                                         \
	}

#ifdef K4A
static const int indices[] = {
	K4ABT_JOINT_SPINE_NAVEL,		K4ABT_JOINT_PELVIS,
	K4ABT_JOINT_SPINE_CHEST,		K4ABT_JOINT_SPINE_NAVEL,
	K4ABT_JOINT_NECK,				K4ABT_JOINT_SPINE_CHEST,
	K4ABT_JOINT_CLAVICLE_LEFT,		K4ABT_JOINT_SPINE_CHEST,
	K4ABT_JOINT_SHOULDER_LEFT,		K4ABT_JOINT_CLAVICLE_LEFT,
	K4ABT_JOINT_ELBOW_LEFT,			K4ABT_JOINT_SHOULDER_LEFT,
	K4ABT_JOINT_WRIST_LEFT,			K4ABT_JOINT_ELBOW_LEFT,
	K4ABT_JOINT_HAND_LEFT,			K4ABT_JOINT_WRIST_LEFT,
	K4ABT_JOINT_HANDTIP_LEFT,		K4ABT_JOINT_HAND_LEFT,
	K4ABT_JOINT_THUMB_LEFT,			K4ABT_JOINT_WRIST_LEFT,
	K4ABT_JOINT_CLAVICLE_RIGHT,		K4ABT_JOINT_SPINE_CHEST,
	K4ABT_JOINT_SHOULDER_RIGHT,		K4ABT_JOINT_CLAVICLE_RIGHT,
	K4ABT_JOINT_ELBOW_RIGHT,		K4ABT_JOINT_SHOULDER_RIGHT,
	K4ABT_JOINT_WRIST_RIGHT,		K4ABT_JOINT_ELBOW_RIGHT,
	K4ABT_JOINT_HAND_RIGHT,			K4ABT_JOINT_WRIST_RIGHT,
	K4ABT_JOINT_HANDTIP_RIGHT,		K4ABT_JOINT_HAND_RIGHT,
	K4ABT_JOINT_THUMB_RIGHT,		K4ABT_JOINT_WRIST_RIGHT,
	K4ABT_JOINT_HIP_LEFT,			K4ABT_JOINT_PELVIS,
	K4ABT_JOINT_KNEE_LEFT,			K4ABT_JOINT_HIP_LEFT,
	K4ABT_JOINT_ANKLE_LEFT,			K4ABT_JOINT_KNEE_LEFT,
	K4ABT_JOINT_FOOT_LEFT,			K4ABT_JOINT_ANKLE_LEFT,
	K4ABT_JOINT_HIP_RIGHT,			K4ABT_JOINT_PELVIS,
	K4ABT_JOINT_KNEE_RIGHT,			K4ABT_JOINT_HIP_RIGHT,
	K4ABT_JOINT_ANKLE_RIGHT,		K4ABT_JOINT_KNEE_RIGHT,
	K4ABT_JOINT_FOOT_RIGHT,			K4ABT_JOINT_ANKLE_RIGHT,
	K4ABT_JOINT_HEAD,				K4ABT_JOINT_NECK,
	K4ABT_JOINT_NOSE,				K4ABT_JOINT_HEAD,
	K4ABT_JOINT_EYE_LEFT,			K4ABT_JOINT_HEAD,
	K4ABT_JOINT_EAR_LEFT,			K4ABT_JOINT_HEAD,
	K4ABT_JOINT_EYE_RIGHT,			K4ABT_JOINT_HEAD,
	K4ABT_JOINT_EAR_RIGHT,			K4ABT_JOINT_HEAD
};
#elif K4W
static const int indices[] = {

};
#endif

MySkeleton::MySkeleton()
{
	this->m_window = nullptr;
	this->m_thread = nullptr;
	this->m_currentPose = nullptr;
	this->m_isMatch = false;
	this->m_vao = NULL;

#ifdef K4A
	this->m_device = NULL;
	this->m_tracker = NULL;
#elif K4W

#endif
}

MySkeleton::~MySkeleton() {}

void MySkeleton::Init(GLFWwindow *window)
{
	this->m_window = window;

#ifdef K4A
	// Setup camera
	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

	VERIFY(k4a_device_open(0, &this->m_device), "Open K4A Device failed!");
	VERIFY(k4a_device_start_cameras(this->m_device, &device_config), "Start K4A cameras failed!");

	k4a_calibration_t sensor_calibration;
	VERIFY(k4a_device_get_calibration(this->m_device, device_config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration),
		   "Get depth camera calibration failed!");

	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &m_tracker), "Body tracker initialization failed!");
#elif K4W

#endif

	// create objects
	glGenVertexArrays(1, &this->m_vao);
	glGenBuffers(1, &this->m_vbo);
	glGenBuffers(1, &this->m_ebo);

	// setup vao
	glBindVertexArray(this->m_vao);

	// - vbo
	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	// - ebo
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->m_ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_DYNAMIC_DRAW);

	// unbind buffers
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	printf("Done Init!\n");
}

void MySkeleton::Start()
{
	if (!this->m_window || this->m_thread)
		return;

	this->m_thread = new std::thread(&MySkeleton::Update, this);
}

void MySkeleton::Update()
{
	while (this->m_window && !glfwWindowShouldClose(this->m_window))
	{
#ifdef K4A
		k4a_capture_t sensor_capture;
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(this->m_device, &sensor_capture, K4A_WAIT_INFINITE);
		if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
		{
			k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(m_tracker, sensor_capture, K4A_WAIT_INFINITE);

			k4a_capture_release(sensor_capture);
			if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				// It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Add capture to tracker process queue timeout!\n");
				break;
			}
			else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
			{
				printf("Error! Add capture to tracker process queue failed!\n");
				break;
			}

			k4abt_frame_t body_frame = NULL;
			k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(m_tracker, &body_frame, K4A_WAIT_INFINITE);
			if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
			{
				if (k4abt_frame_get_num_bodies(body_frame) > 0)
				{
					printf("Get!\n");

					k4abt_body_t body;
					VERIFY(k4abt_frame_get_body_skeleton(body_frame, 0, &body.skeleton), "Get body from body frame failed!");
					body.id = k4abt_frame_get_body_id(body_frame, 0);

					this->Load2Shader(body.skeleton);

					this->m_isMatch = false;
					if (this->m_currentPose)
					{
						this->m_isMatch = MySkeleton::CompareJoint(body.skeleton, *this->m_currentPose);
					}
					else
					{
						this->m_poseLog.push(body.skeleton);
					}
				}

				k4abt_frame_release(body_frame);
			}
			else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				//  It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Pop body frame result timeout!\n");
				break;
			}
			else
			{
				printf("Pop body frame result failed!\n");
				break;
			}
		}
		else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
		{
			// It should never hit time out when K4A_WAIT_INFINITE is set.
			printf("Error! Get depth frame time out!\n");
			break;
		}
		else
		{
			printf("Get depth capture returned error: %d\n", get_capture_result);
			break;
		}
#elif K4W

#endif

		// try to get pose
		if (!this->m_currentPose)
		{
			// �p�Gt1 ~ t30��pose����t0��pose�~�t�Ȥp��thresh hold, ����t0
			if (this->m_poseLog.size() > 30)
			{
				this->m_currentPose = new k4abt_skeleton_t(this->m_poseLog.front());
			}
			else
			{
				while (this->m_poseLog.size() > 0)
				{
					if (MySkeleton::CompareJoint(this->m_poseLog.front(), this->m_poseLog.back()))
					{
						printf("Matched Frames: %zu\n", this->m_poseLog.size());
						break;
					}
					else
					{
						this->m_poseLog.pop();
					}
				}
			}
		}
	}

#ifdef K4A
	// Close camera
	k4abt_tracker_shutdown(this->m_tracker);
	k4abt_tracker_destroy(this->m_tracker);
	k4a_device_stop_cameras(this->m_device);
	k4a_device_close(this->m_device);
#elif K4W

#endif

	printf("Stopped.\n");
}

void MySkeleton::Stop()
{
	if (!this->m_thread)
	{
		printf("No workers running.\n");
		return;
	}

	printf("Stopping skeleton worker...\n");
	this->m_window = nullptr;
	this->m_thread->join();
	delete (this->m_thread);
	this->m_thread = nullptr;
}

bool MySkeleton::hasData()
{
	if (this->m_currentPose)
		return true;
	else
		return false;
}

bool MySkeleton::isMatch()
{
	return this->m_isMatch;
}

size_t MySkeleton::getSavedAmount()
{
	return this->m_savedPose.size();
}

void MySkeleton::Clear()
{
	if (!this->m_currentPose)
		return;

	delete (this->m_currentPose);
	this->m_currentPose = nullptr;

	while (!this->m_poseLog.empty())
	{
		this->m_poseLog.pop();
	}
}

void MySkeleton::ClearAll()
{
	this->m_savedPose.clear();
	this->Clear();
}

void MySkeleton::Save(int holdTime, int key, bool isClick)
{
	if (!this->m_currentPose)
		return;

	MySkeleton::data buf;
	buf.pose = *this->m_currentPose;
	buf.holdTime = holdTime;
	buf.key = key;
	buf.isClick = isClick;

	this->m_savedPose.push_back(buf);
	this->Clear();
}

void MySkeleton::Import(const char *path)
{
	std::ifstream ifs;
	ifs.open(path);
	if (ifs.is_open())
	{
		std::string buf = "";
		while (std::getline(ifs, buf))
		{
			printf("%s\n", buf.c_str());
		}

		ifs.close();
	}
}

bool MySkeleton::Export(const char *path)
{
	std::ofstream ofs;
	ofs.open(path);
	if (ofs.is_open())
	{
		ofs << "Hello world" << std::endl;
		ofs.close();
		return true;
	}
	else
		return false;
}

#ifdef K4A
void MySkeleton::Load2Shader(const k4abt_skeleton_t& skeleton)
{
	std::vector<float> vertices;
	for (int i = 0; i < (int)K4ABT_JOINT_COUNT; ++i)
	{
		vertices.push_back(skeleton.joints[i].position.v[0]);
		vertices.push_back(-skeleton.joints[i].position.v[1]);		// reverse y coordinate
		vertices.push_back(-skeleton.joints[i].position.v[2]);		// reverse z coordinate
	}

	if (this->m_bufferBusy.try_lock())
	{
		printf("Updating...\n");
		glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), &vertices[0], GL_DYNAMIC_DRAW);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		this->m_bufferBusy.unlock();
	}
}
#endif

void MySkeleton::Render()
{
	if (this->m_bufferBusy.try_lock())
	{
		printf("Drawing...\n");
		glBindVertexArray(this->m_vao);
		glDrawElements(GL_LINES, sizeof(indices) / sizeof(int), GL_UNSIGNED_INT, 0);
		glBindVertexArray(0);

		this->m_bufferBusy.unlock();
	}
}


bool MySkeleton::CompareJoint(const k4abt_skeleton_t& lhs, const k4abt_skeleton_t& rhs, float thresh)
{
	for (int i = 0; i < K4ABT_JOINT_HIP_LEFT; ++i)
	{
		// �p��H�߭ȱo���Ҽ{
		if (lhs.joints[i].confidence_level < K4ABT_JOINT_CONFIDENCE_MEDIUM ||
			rhs.joints[i].confidence_level < K4ABT_JOINT_CONFIDENCE_MEDIUM)
		{
			continue;
		}

		if (i == K4ABT_JOINT_HAND_LEFT ||
			i == K4ABT_JOINT_HANDTIP_LEFT ||
			i == K4ABT_JOINT_THUMB_LEFT ||
			i == K4ABT_JOINT_HAND_RIGHT ||
			i == K4ABT_JOINT_HANDTIP_RIGHT ||
			i == K4ABT_JOINT_THUMB_RIGHT)
		{
			continue;
		}

		float diff[4] = {0, 0, 0, 0};
		diff[0] = lhs.joints[i].orientation.v[0] - rhs.joints[i].orientation.v[0];
		diff[1] = lhs.joints[i].orientation.v[1] - rhs.joints[i].orientation.v[1];
		diff[2] = lhs.joints[i].orientation.v[2] - rhs.joints[i].orientation.v[2];
		diff[3] = lhs.joints[i].orientation.v[3] - rhs.joints[i].orientation.v[3];

		float mag = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2] + diff[3] * diff[3];
		mag = std::sqrt(mag);

		if (mag > thresh)
		{
			printf("Failed ad joint[%d] with error of: %.3f\n", i, mag);
			return false;
		}
	}
	return true;
}