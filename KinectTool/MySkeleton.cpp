#include "MySkeleton.h"

#include <fstream>
#include <cmath>
#include <boost/tokenizer.hpp>

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

	this->m_device = nullptr;
	this->m_tracker = nullptr;

	this->m_currentSkeleton = nullptr;
	this->m_matchPose = nullptr;

	this->m_checkList.fill(1);
	this->m_failed = 0;
	this->m_jointThresh = 1.0f;

	this->m_ebo = NULL;
	this->m_vbo = NULL;
	this->m_vbo_confidence = NULL;

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

	// - vbo for vertices position
	glGenBuffers(1, &this->m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * K4ABT_JOINT_COUNT, NULL, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// - vbo for confidence level
	glGenBuffers(1, &this->m_vbo_confidence);
	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo_confidence);
	glBufferData(GL_ARRAY_BUFFER, sizeof(int) * K4ABT_JOINT_COUNT, NULL, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// - ebo
	glGenBuffers(1, &this->m_ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->m_ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	// Setup camera
#ifdef K4A
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
				this->m_currentSkeleton = nullptr;
				if (k4abt_frame_get_num_bodies(body_frame) > 0)
				{
					//printf("Get!\n");

					k4abt_body_t body;
					VERIFY(k4abt_frame_get_body_skeleton(body_frame, 0, &body.skeleton), "Get body from body frame failed!");
					body.id = k4abt_frame_get_body_id(body_frame, 0);

					this->m_currentSkeleton = &body.skeleton;
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
		if (this->m_currentSkeleton)
		{
			if (!this->m_matchPose)
			{
				this->m_skeletonLog.push(*this->m_currentSkeleton);

				// check the difference between the first skeleton in the queue and the current skeleton,
				// pop the queue until finds a match.
				while (this->m_skeletonLog.size() > 0)
				{
					if (MySkeleton::CompareJoint(this->m_skeletonLog.front(), this->m_skeletonLog.back()) > 0)
					{
						this->m_skeletonLog.pop();
					}
					else
					{
						printf("Matched Frames: %zu\n", this->m_skeletonLog.size());
						break;
					}
				}

				// if the last 30 frames all matched, save this skeleton
				if (this->m_skeletonLog.size() > 30)
				{
					delete this->m_matchPose;
					this->m_matchPose = new k4abt_skeleton_t(this->m_skeletonLog.front());
				}
			}
			else
			{

				this->m_failed = MySkeleton::CompareJoint(*this->m_matchPose, *this->m_currentSkeleton);
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

size_t MySkeleton::getSavedAmount()
{
	return this->m_savedPose.size();
}

void MySkeleton::setCheckList(const std::array<bool, K4ABT_JOINT_COUNT>& checkList)
{
	this->m_checkList = checkList;
}

void MySkeleton::setThresh(const float& thresh)
{
	this->m_jointThresh = thresh;
}

void MySkeleton::Clear()
{
	if (!this->m_matchPose)
		return;

	delete (this->m_matchPose);
	this->m_matchPose = nullptr;

	this->m_failed = 0;

	while (!this->m_skeletonLog.empty())
	{
		this->m_skeletonLog.pop();
	}
}

void MySkeleton::ClearAll()
{
	this->m_savedPose.clear();
	this->Clear();
}

void MySkeleton::Save(int key)
{
	if (!this->m_matchPose)
		return;

	this->m_savedPose.push_back({
		*this->m_matchPose,
		key
		});

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
			MySkeleton::data data;
			data.key = std::stoi(buf);

			for (int i = 0; i < K4ABT_JOINT_COUNT; ++i)
			{
				data.pose.joints[i].confidence_level = K4ABT_JOINT_CONFIDENCE_MEDIUM;
				std::memset(data.pose.joints[i].orientation.v, 0, sizeof(data.pose.joints[i].orientation.v));

				std::getline(ifs, buf);
				boost::escaped_list_separator<char> sep;
				boost::tokenizer<boost::escaped_list_separator<char>> tok(buf, sep);
				auto it = tok.begin();
				data.pose.joints[i].orientation.v[0] = std::stof(*it); ++it;
				data.pose.joints[i].orientation.v[1] = std::stof(*it); ++it;
				data.pose.joints[i].orientation.v[2] = std::stof(*it); ++it;
				data.pose.joints[i].orientation.v[3] = std::stof(*it);
			}

			this->m_savedPose.push_back(data);
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
		for (const MySkeleton::data& data : this->m_savedPose)
		{
			ofs << data.key << std::endl;
			for (int i = 0; i < K4ABT_JOINT_COUNT; ++i)
			{
				ofs << data.pose.joints[i].orientation.v[0] << ',';
				ofs << data.pose.joints[i].orientation.v[1] << ',';
				ofs << data.pose.joints[i].orientation.v[2] << ',';
				ofs << data.pose.joints[i].orientation.v[3] << '\n';
			}
		}
		ofs.close();
		return true;
	}
	else
		return false;
}

#ifdef K4A
void MySkeleton::Load2Shader()
{
	if (!this->m_currentSkeleton)
		return;

	// copy to prevent async update from altering data
	k4abt_skeleton_t data(*this->m_currentSkeleton);
	m_currentSkeleton = nullptr;

	static std::array<float, 3 * K4ABT_JOINT_COUNT> vertices;
	static std::array<int, K4ABT_JOINT_COUNT> confidence;
	vertices.fill(0.0f);
	confidence.fill(0);
	for (int i = 0; i < (int)K4ABT_JOINT_COUNT; ++i)
	{
		vertices[i * 3 + 0] = (-data.joints[i].position.v[0] / 100.0f);
		vertices[i * 3 + 1] = (-data.joints[i].position.v[1] / 100.0f);
		vertices[i * 3 + 2] = (0.0f);

		confidence[i] = data.joints[i].confidence_level < K4ABT_JOINT_CONFIDENCE_MEDIUM ? 0 : 1;
	}

	//printf("Chest at %5.3f, %5.3f, %5.3f\n",
	//	vertices[K4ABT_JOINT_SPINE_CHEST * 3 + 0],
	//	vertices[K4ABT_JOINT_SPINE_CHEST * 3 + 1],
	//	vertices[K4ABT_JOINT_SPINE_CHEST * 3 + 2]);
	//printf("Pelvis at %5.3f, %5.3f, %5.3f\n",
	//	vertices[K4ABT_JOINT_PELVIS * 3 + 0],
	//	vertices[K4ABT_JOINT_PELVIS * 3 + 1],
	//	vertices[K4ABT_JOINT_PELVIS * 3 + 2]);

	//printf("Updating...\n");
	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices.data());
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo_confidence);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(confidence), confidence.data());
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}
#endif

void MySkeleton::Render(const GLuint& program)
{
	//printf("Drawing...\n");

	// get uniform to control color
	GLint uColor = glGetUniformLocation(program, "uColor");
	GLint uMode = glGetUniformLocation(program, "uMode");

	// bind buffers
	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo_confidence);
	glVertexAttribIPointer(1, 1, GL_INT, 0, 0);
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->m_ebo);

	// 1. draw skeleton
	glUniform1i(uMode, 0);
	glUniform4f(uColor, 0.0f, 0.0f, 0.0f, 1.0f);
	glDrawElements(GL_LINES, 62, GL_UNSIGNED_INT, 0);

	// 2. draw joint confident
	glUniform1i(uMode, 1);
	glDrawArrays(GL_POINTS, 0, K4ABT_JOINT_COUNT);

	// 3. draw which joint failed the test
	if (this->m_failed > 0)
	{
		glUniform1i(uMode, 2);
		glDrawArrays(GL_POINTS, this->m_failed, 1);
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}


int MySkeleton::CompareJoint(const k4abt_skeleton_t& lhs, const k4abt_skeleton_t& rhs)
{
	// skip joints below hip for now
	for (int i = 0; i < K4ABT_JOINT_COUNT; ++i)
	{
		// skip these joints for now
		if (!this->m_checkList[i])
		{
			continue;
		}
		// return false if can not capture joint
		else if (lhs.joints[i].confidence_level < K4ABT_JOINT_CONFIDENCE_MEDIUM || 
			rhs.joints[i].confidence_level < K4ABT_JOINT_CONFIDENCE_MEDIUM)
		{
			return i;
		}

		float diff[4] = {0, 0, 0, 0};
		diff[0] = lhs.joints[i].orientation.v[0] - rhs.joints[i].orientation.v[0];
		diff[1] = lhs.joints[i].orientation.v[1] - rhs.joints[i].orientation.v[1];
		diff[2] = lhs.joints[i].orientation.v[2] - rhs.joints[i].orientation.v[2];
		diff[3] = lhs.joints[i].orientation.v[3] - rhs.joints[i].orientation.v[3];

		float mag = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2] + diff[3] * diff[3];
		mag = std::sqrt(mag);

		if (mag > this->m_jointThresh)
		{
			//printf("Failed ad joint[%d] with error of: %.3f\n", i, mag);
			return i;
		}
	}
	return 0;
}