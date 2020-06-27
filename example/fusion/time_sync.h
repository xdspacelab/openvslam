#pragma once

#include <string>
#include <thread>
#include <chrono>

using namespace std;
using namespace chrono;

class time_sync {

	public:
		time_sync() {
            this->video_timestamp = this->gps_timestamp = milliseconds(0);
	}
		// video timestamp since start - of the play of video (in ms)
		milliseconds video_timestamp;
		// gps timestamp since start - from the parsing process (in ms)
        milliseconds gps_timestamp;
        milliseconds is_gps_caught_up_video();
        milliseconds is_video_caught_up_gps();

};