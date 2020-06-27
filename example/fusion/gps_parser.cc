#pragma once

#include "gps_parser.h"


// Continuously parses the gps text and updated with respect to timestamp. Run in separate thead!
void gps_parser::start_reading(time_sync& time_s) {
    this->lat = this->lon = this->alt = "0.0";
    this->terminate = false;

    ifstream fin;
    fin.open(this->file_path, fstream::in);

    if (!fin.fail()) {
        this->is_valid = true;
        spdlog::info("Parsing gps file @ " + file_path);
        string msg;
        string txt[4];
        bool valid;

        chrono::milliseconds sleep_time(0);
        long long last_stamp = 0;

        chrono::milliseconds wait_for_vid_thread(0);
        auto tp_1 = std::chrono::steady_clock::now();

        while (getline(fin, msg)) {
            
            if (terminate)
                break;

            //spdlog::info("parser: " + msg);
            if (msg.find("#") != string::npos)
                continue;

            std::stringstream msg_stream(msg);
            valid = false;

            if (getline(msg_stream, txt[0], ','))
                if (getline(msg_stream, txt[1], ','))
                    if (getline(msg_stream, txt[2], ','))
                        if (getline(msg_stream, txt[3], ','))
                            valid = true;

            if (valid) {
                this->timestamp = stoll(txt[0]);
                this->lat = txt[1];
                this->lon = txt[2];
                this->alt = txt[3];
            }
            else {
                spdlog::warn(msg);
            }

            if (last_stamp == 0) {
                last_stamp = this->timestamp;
            }
            
             
			const auto tp_2 = std::chrono::steady_clock::now();
			// double (s)
            const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
            
			sleep_time = chrono::milliseconds(this->timestamp - (last_stamp + (long long)(track_time * 1000)));
            spdlog::info("dt: "+to_string(this->timestamp - last_stamp) +" track: "+ to_string((long long)(track_time * 1000)));
			
        time_s.gps_timestamp += chrono::milliseconds(this->timestamp - last_stamp);
			
        wait_for_vid_thread = time_s.is_gps_caught_up_video();
            if (wait_for_vid_thread + sleep_time > chrono::milliseconds(10)) {
            spdlog::info("sleeping gps parser for: vid_w:" + to_string(wait_for_vid_thread.count()) + ", s_t: " + to_string(sleep_time.count()));
            if (wait_for_vid_thread>sleep_time)
                std::this_thread::sleep_for(wait_for_vid_thread);
            else
                std::this_thread::sleep_for(sleep_time);
            }

            last_stamp = this->timestamp;
            //sleep_time = 0.0;
            tp_1 = tp_2;
        }
        spdlog::info("reached end of gps input txt file");
    }
    else {
        this->is_valid = false;
        spdlog::critical("Failed to read gps file @ " + file_path);
    }
}

void gps_parser::terminate_process() {
    this->terminate = true;
    spdlog::info("Terminating gps parser");
}