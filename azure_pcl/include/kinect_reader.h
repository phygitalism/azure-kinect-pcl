#pragma once

#include "common.h"
#include "point_cloud_utils.h"

namespace azure
{
    namespace io
    {
        k4a_playback_t open_reader(const std::string & path_to_file);

        void start_reading(const k4a_playback_t playback_handle, LockFreeQueue & queue, std::atomic_bool & is_stop);
    }
}
