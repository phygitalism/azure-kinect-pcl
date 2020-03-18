#include "kinect_reader.h"


k4a_playback_t azure::io::open_reader(const std::string& path_to_file)
{
	k4a_playback_t playback_handle{ nullptr };

	const char* path_to_mkv = path_to_file.c_str();

	if (k4a_playback_open(path_to_mkv, &playback_handle) == K4A_RESULT_FAILED)
	{
		return nullptr;
	}
	else
	{
		return playback_handle;
	}
}


void azure::io::start_reading(const k4a_playback_t playback_handle, azure::LockFreeQueue & queue, std::atomic_bool & is_stop)
{
	using std::cerr;
	using std::cout;
	using std::endl;
	using namespace std::chrono;

	k4a_transformation_t transform{ nullptr };
	k4a_capture_t capture{ nullptr };

	try
	{
		k4a_stream_result_t result{ K4A_STREAM_RESULT_SUCCEEDED };
		k4a_calibration_t calibration{};


		if (k4a_playback_get_calibration(playback_handle, &calibration) == K4A_RESULT_FAILED)
		{
			is_stop = true;
			return;
		}

		transform = k4a_transformation_create(&calibration);

		if (transform == nullptr)
		{
			is_stop = true;
			return;
		}

		while (result == K4A_STREAM_RESULT_SUCCEEDED && !is_stop)
		{
			result = k4a_playback_get_next_capture(playback_handle, &capture);

			if (result == K4A_STREAM_RESULT_SUCCEEDED)
			{
				k4a_image_t depth_image = k4a_capture_get_depth_image(capture);

				if (depth_image != nullptr)
				{
					k4a_image_t point_cloud = nullptr;

					azure::pcl::depth_image_to_point_cloud(transform, K4A_CALIBRATION_TYPE_DEPTH, depth_image, &point_cloud);

					if (point_cloud != nullptr)
					{
						while (!queue.push(point_cloud) && !is_stop)
						{
							//std::this_thread::sleep_for(1ms);
						}
					}

					k4a_image_release(depth_image);
				}

				k4a_capture_release(capture);
			}

		}
	}
	catch (...)
	{
		k4a_transformation_destroy(transform);
		transform = nullptr;
	}

	is_stop = true;
	k4a_transformation_destroy(transform);
}
