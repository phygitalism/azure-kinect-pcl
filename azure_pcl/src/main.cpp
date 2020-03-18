
#include <pcl/visualization/cloud_viewer.h>

#include "common.h"
#include "kinect_reader.h"
#include "pcl_clustering.h"


int main(int argc, char** argv)
{
	using std::cout;
	using std::cerr;
	using std::endl;
	using std::string;
	using std::thread;
	using namespace azure;

    string path_to_file;

	if (argc == 2)
	{
		path_to_file = argv[1];
	}
	else
	{
		cerr << "You need to set path to .mkv file" << endl;
		return 1;
	}

	const string window_name{ "Point cloud" };

	LockFreeQueue queue(20);

	std::atomic_bool is_stop = false;

	k4a_playback_t playback = azure::io::open_reader(path_to_file);

	if (playback == nullptr)
	{
		cerr << "Cannot open: " << path_to_file << endl;
		return 1;
	}

	auto length = k4a_playback_get_recording_length_usec(playback);

	// You can start from origin
	if (k4a_playback_seek_timestamp(playback, length / 3, K4A_PLAYBACK_SEEK_BEGIN) != K4A_STREAM_RESULT_SUCCEEDED)
	{
		k4a_playback_close(playback);
		playback = nullptr;
		cerr << "Cannot seek" << endl;
	}

	try
	{

		::pcl::visualization::CloudViewer viewer("Azure point cloud");

		color::ColourManager::Init_ColourManager();

		thread reader_thread(azure::io::start_reading, playback, std::ref(queue), std::ref(is_stop));

		k4a_image_t raw_points{};

		PointCloud::Ptr kinect_points{ new PointCloud };

		PointCloud::Ptr all_poinst{ new PointCloud };

		while (!is_stop && !viewer.wasStopped())
		{
			while (!queue.pop(raw_points) && !is_stop)
			{

			}

			azure::pcl::convert_to_pcl(raw_points, kinect_points);
			k4a_image_release(raw_points);
			auto clustered_point_cloud{ azure::cluster::euclidean_clustering(kinect_points) };
			// You can see all points if pass kinect_points
			viewer.showCloud(clustered_point_cloud);
		}

		is_stop = true;

		reader_thread.join();
	}
	catch (...)
	{
		k4a_playback_close(playback);
		playback = nullptr;
	}

	k4a_playback_close(playback);
	playback = nullptr;

	cout << "Stop playback" << endl;

	return 0;
}