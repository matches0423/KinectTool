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

#if defined(K4A)
static const int indices[] = {
	// joint						parent
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
#elif defined(K4W)
static const int indices[] = {
	// joint					parent
	JointType_SpineMid,			JointType_SpineBase,
	JointType_SpineShoulder,	JointType_SpineMid,
	JointType_Neck,				JointType_SpineShoulder,
	JointType_ShoulderLeft,		JointType_SpineShoulder,
	JointType_ElbowLeft,		JointType_ShoulderLeft,
	JointType_WristLeft,		JointType_ElbowLeft,
	JointType_HandLeft,			JointType_WristLeft,
	JointType_HandTipLeft,		JointType_HandLeft,
	JointType_ThumbLeft,		JointType_WristLeft,
	JointType_ShoulderRight,	JointType_SpineShoulder,
	JointType_ElbowRight,		JointType_ShoulderRight,
	JointType_WristRight,		JointType_ElbowRight,
	JointType_HandRight,		JointType_WristRight,
	JointType_HandTipRight,		JointType_HandRight,
	JointType_ThumbRight,		JointType_WristRight,
	JointType_HipLeft,			JointType_SpineBase,
	JointType_KneeLeft,			JointType_HipLeft,
	JointType_AnkleLeft,		JointType_KneeLeft,
	JointType_FootLeft,			JointType_AnkleLeft,
	JointType_HipRight,			JointType_SpineBase,
	JointType_KneeRight,		JointType_HipRight,
	JointType_AnkleRight,		JointType_KneeRight,
	JointType_FootRight,		JointType_AnkleRight,
	JointType_Head,				JointType_Neck,
};
#endif

MySkeleton::MySkeleton()
{
	this->m_window = nullptr;
	this->m_thread = nullptr;

#if defined(K4A)
	this->m_device = NULL;
	this->m_tracker = NULL;
#elif defined(K4W)
	this->m_sensor = nullptr;
	this->m_reader = nullptr;
#endif

	this->m_currentSkeleton = nullptr;
	this->m_matchPose = nullptr;

	this->m_checkList.fill(1);
	this->m_failed = -1;
	this->m_jointThresh = 1.0f;

	this->m_ebo = NULL;
	this->m_vbo = NULL;
	this->m_vbo_confidence = NULL;

	this->m_checkList.fill(1);
}

MySkeleton::~MySkeleton() {}

void MySkeleton::Init(GLFWwindow *window)
{
	this->m_window = window;

	// - vbo for vertices position
	glGenBuffers(1, &this->m_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo);
	glBufferData(GL_ARRAY_BUFFER, 3 * sizeof(float) * JOINTS, NULL, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// - vbo for confidence level
	glGenBuffers(1, &this->m_vbo_confidence);
	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo_confidence);
	glBufferData(GL_ARRAY_BUFFER, sizeof(int) * JOINTS, NULL, GL_DYNAMIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	// - ebo
	glGenBuffers(1, &this->m_ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, this->m_ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	// Setup camera
#if defined(K4A)
	k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

	VERIFY(k4a_device_open(0, &this->m_device), "Open K4A Device failed!");
	VERIFY(k4a_device_start_cameras(this->m_device, &device_config), "Start K4A cameras failed!");

	k4a_calibration_t sensor_calibration;
	VERIFY(k4a_device_get_calibration(this->m_device, device_config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration),
		"Get depth camera calibration failed!");

	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
	VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &m_tracker), "Body tracker initialization failed!");
#elif defined(K4W)
	if (GetDefaultKinectSensor(&this->m_sensor) != S_OK)
	{
		printf("Get Sensor failed\n");
		exit(1);
	}

	if (this->m_sensor->Open() != S_OK)
	{
		printf("Can't open sensor\n");
		exit(1);
	}

	IBodyFrameSource* source = NULL;
	if (this->m_sensor->get_BodyFrameSource(&source) != S_OK)
	{
		printf("Can't open BodyFrameSource\n");
		exit(1);
	}

	if (source->OpenReader(&this->m_reader) != S_OK)
	{
		printf("Can't open reader\n");
		exit(1);
	}
	source->Release();
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
#if defined(K4A)
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
#elif defined(K4W)

		delete this->m_currentSkeleton;
		this->m_currentSkeleton = nullptr;

		IBodyFrame* bodyFrame = nullptr;
		if (this->m_reader->AcquireLatestFrame(&bodyFrame) == S_OK)
		{
			IBody* bodies[6] = { 0 };
			if (bodyFrame->GetAndRefreshBodyData(6, bodies) == S_OK)
			{
				for (int i = 0; i < 6; ++i)
				{
					IBody* body = bodies[i];
					if (body)
					{
						BOOLEAN tracked = false;
						if (body->get_IsTracked(&tracked) == S_OK && tracked)
						{
							this->m_currentSkeleton = new skeleton_data;
							body->GetJoints(JOINTS, this->m_currentSkeleton->joints);
							body->GetJointOrientations(JOINTS, this->m_currentSkeleton->orientations);
							Sleep(10);
							break;
						}
					}
				}

				for (int i = 0; i < 6; ++i)
				{
					bodies[i]->Release();;
				}
			}
		}
		if (bodyFrame)
			bodyFrame->Release();
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
					if (MySkeleton::CompareJoint(this->m_skeletonLog.front(), this->m_skeletonLog.back()) >= 0)
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
					this->m_matchPose = new skeleton_data(this->m_skeletonLog.front());
				}
			}
			else
			{
				this->m_failed = MySkeleton::CompareJoint(*this->m_matchPose, *this->m_currentSkeleton);
			}
		}
	}

#if defined(K4A)
	// Close camera
	k4abt_tracker_shutdown(this->m_tracker);
	k4abt_tracker_destroy(this->m_tracker);
	k4a_device_stop_cameras(this->m_device);
	k4a_device_close(this->m_device);
#elif defined(K4W)
	this->m_reader->Release();
	this->m_reader = nullptr;
	this->m_sensor->Close();
	this->m_sensor->Release();
	this->m_sensor = nullptr;
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
	delete this->m_thread;
	this->m_thread = nullptr;
}

size_t MySkeleton::getSavedAmount()
{
	return this->m_savedPose.size();
}

std::array<bool, JOINTS>& MySkeleton::getCheckList()
{
	return this->m_checkList;
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

	this->m_failed = -1;

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

#if defined(K4A)
			for (int i = 0; i < JOINTS; ++i)
			{
				data.skeleton.joints[i].confidence_level = K4ABT_JOINT_CONFIDENCE_MEDIUM;
				std::memset(data.skeleton.joints[i].orientation.v, 0, sizeof(data.skeleton.joints[i].orientation.v));

				std::getline(ifs, buf);
				boost::escaped_list_separator<char> sep;
				boost::tokenizer<boost::escaped_list_separator<char>> tok(buf, sep);
				auto it = tok.begin();
				data.skeleton.joints[i].orientation.v[0] = std::stof(*it); ++it;
				data.skeleton.joints[i].orientation.v[1] = std::stof(*it); ++it;
				data.skeleton.joints[i].orientation.v[2] = std::stof(*it); ++it;
				data.skeleton.joints[i].orientation.v[3] = std::stof(*it);
			}
#elif defined(K4W)
			for (int i = 0; i < JOINTS; ++i)
			{
				data.skeleton.joints->TrackingState = TrackingState_Tracked;

				std::getline(ifs, buf);
				boost::escaped_list_separator<char> sep;
				boost::tokenizer<boost::escaped_list_separator<char>> tok(buf, sep);
				auto it = tok.begin();
				data.skeleton.orientations[i].Orientation.w = stof(*it); ++it;
				data.skeleton.orientations[i].Orientation.x = stof(*it); ++it;
				data.skeleton.orientations[i].Orientation.y = stof(*it); ++it;
				data.skeleton.orientations[i].Orientation.z = stof(*it);
			}
#endif

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
			for (int i = 0; i < JOINTS; ++i)
			{
#if defined(K4A)
				ofs << data.skeleton.joints[i].orientation.v[0] << ',';
				ofs << data.skeleton.joints[i].orientation.v[1] << ',';
				ofs << data.skeleton.joints[i].orientation.v[2] << ',';
				ofs << data.skeleton.joints[i].orientation.v[3] << '\n';
#elif defined(K4W)
				ofs << data.skeleton.orientations[i].Orientation.w << ',';
				ofs << data.skeleton.orientations[i].Orientation.x << ',';
				ofs << data.skeleton.orientations[i].Orientation.y << ',';
				ofs << data.skeleton.orientations[i].Orientation.z << '\n';
#endif
			}
		}
		ofs.close();
		return true;
	}
	else
		return false;
}

void MySkeleton::Load2Shader()
{
	skeleton_data data;
	if (this->m_currentSkeleton)
		data = skeleton_data(*this->m_currentSkeleton);
	else
		return;

	static std::array<float, 3 * JOINTS> vertices;
	static std::array<int, JOINTS> confidence;
	vertices.fill(0.0f);
	confidence.fill(0);

#if defined(K4A)
	// copy to prevent async update from altering data
	m_currentSkeleton = nullptr;
	for (int i = 0; i < JOINTS; ++i)
	{
		// unit = millimeter
		vertices[i * 3 + 0] = (-data.joints[i].position.v[0] / 100.0f);
		vertices[i * 3 + 1] = (-data.joints[i].position.v[1] / 100.0f);
		vertices[i * 3 + 2] = (0.0f);

		confidence[i] = data.joints[i].confidence_level < K4ABT_JOINT_CONFIDENCE_MEDIUM ? 0 : 1;
	}
#elif defined(K4W)
	//this->m_currentSkeleton = nullptr;
	for (int i = 0; i < JOINTS; ++i)
	{
		// unit = meter
		vertices[i * 3 + 0] = data.joints[i].Position.X * 10.0f;
		vertices[i * 3 + 1] = data.joints[i].Position.Y * 10.0f;
		vertices[i * 3 + 2] = 0;

		confidence[i] = data.joints[i].TrackingState < TrackingState_Tracked ? 0 : 1;
	}
#endif

	//printf("Updating...\n");
	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices.data());
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, this->m_vbo_confidence);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(confidence), confidence.data());
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

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
	glDrawArrays(GL_POINTS, 0, JOINTS);

	// 3. draw which joint failed the test
	if (this->m_failed >= 0)
	{
		glUniform1i(uMode, 2);
		glDrawArrays(GL_POINTS, this->m_failed, 1);
	}

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

int MySkeleton::CompareJoint(const skeleton_data& lhs, const skeleton_data& rhs)
{
	// skip joints below hip for now
	for (int i = 0; i < JOINTS; ++i)
	{
		// skip these joints for now
		if (!this->m_checkList[i])
		{
			continue;
		}
#if defined(K4A)
		// if can not capture joint
		if (lhs.joints[i].confidence_level < K4ABT_JOINT_CONFIDENCE_MEDIUM || 
			rhs.joints[i].confidence_level < K4ABT_JOINT_CONFIDENCE_MEDIUM)
		{
			return i;
		}
#elif defined(K4W)
		if (lhs.joints[i].TrackingState < TrackingState_Tracked ||
			rhs.joints[i].TrackingState < TrackingState_Tracked)
		{
			return i;
		}
#endif

		float diff[4] = {0, 0, 0, 0};
#if defined(K4A)
		diff[0] = lhs.joints[i].orientation.v[0] - rhs.joints[i].orientation.v[0];
		diff[1] = lhs.joints[i].orientation.v[1] - rhs.joints[i].orientation.v[1];
		diff[2] = lhs.joints[i].orientation.v[2] - rhs.joints[i].orientation.v[2];
		diff[3] = lhs.joints[i].orientation.v[3] - rhs.joints[i].orientation.v[3];
#elif defined(K4W)
		diff[0] = lhs.orientations[i].Orientation.w - rhs.orientations[i].Orientation.w;
		diff[1] = lhs.orientations[i].Orientation.x - rhs.orientations[i].Orientation.x;
		diff[2] = lhs.orientations[i].Orientation.y - rhs.orientations[i].Orientation.y;
		diff[3] = lhs.orientations[i].Orientation.z - rhs.orientations[i].Orientation.z;
#endif

		float mag = diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2] + diff[3] * diff[3];
		mag = std::sqrt(mag);

		if (mag > this->m_jointThresh)
		{
			//printf("Failed ad joint[%d] with error of: %.3f\n", i, mag);
			return i;
		}
	}
	return -1;
}