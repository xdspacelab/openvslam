#include "time_sync.h"

// Time diff of gps timestamp from video. return value >0.0 if gps is ahead of video.
milliseconds time_sync::is_gps_caught_up_video() {
    if (time_sync::gps_timestamp > time_sync::video_timestamp)
        return time_sync::gps_timestamp - time_sync::video_timestamp;
    else
        return time_sync::video_timestamp - time_sync::gps_timestamp;
}

// Time diff of video timestamp from gps. return >0.0 if video is ahead of gps.
milliseconds time_sync::is_video_caught_up_gps() {
    if (time_sync::gps_timestamp < time_sync::video_timestamp)
        return time_sync::video_timestamp - time_sync::gps_timestamp;
    else
        return time_sync::gps_timestamp - time_sync::video_timestamp;
}