#include "common.h"
#include "kinect_reader.h"
#include "pcl_clustering.h"

int main(int argc, char ** argv)
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

    std::atomic_bool is_stop{false};

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
        return 1;
    }

    thread reader_thread(azure::io::start_reading, playback, std::ref(queue), std::ref(is_stop));

    try
    {
        color::ColourManager::Init_ColourManager();
        k4a_image_t raw_points{};

        PointCloud::Ptr kinect_points{ new PointCloud };
        PointCloud::Ptr clustered_point_cloud{ new PointCloud };

        ::pcl::visualization::PCLVisualizer viewer("Azure point cloud");

        viewer.resetCamera();
        viewer.setCameraPosition(-0.454925, -1.63636, -6.02044, 0.0148022, -0.984551, 0.174471);
        viewer.setShowFPS(true);
        viewer.addPointCloud(clustered_point_cloud);

        while (!is_stop && !viewer.wasStopped())
        {
            while (!queue.pop(raw_points) && !is_stop)
            {
            }

            azure::pcl::convert_to_pcl(raw_points, kinect_points);
            k4a_image_release(raw_points);
            clustered_point_cloud = azure::cluster::euclidean_clustering(kinect_points);
            // You can see all points if pass kinect_points

            viewer.updatePointCloud(clustered_point_cloud);
            viewer.spinOnce(1);
        }

        is_stop = true;

        reader_thread.join();
    }
    catch (...)
    {
        k4a_playback_close(playback);
        playback = nullptr;
        is_stop = true;
        reader_thread.join();
        return 1;
    }

    k4a_playback_close(playback);
    playback = nullptr;

    cout << "Stop playback" << endl;

    return 0;
}