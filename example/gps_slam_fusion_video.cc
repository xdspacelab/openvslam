
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>

#ifdef USE_PANGOLIN_VIEWER
#include "pangolin_viewer/viewer.h"
#endif

#include <opencv2/core/core.hpp>
#include "opencv2/highgui.hpp"
#include "openvslam/system.h"
#include "openvslam/config.h"
#include <popl.hpp>
#include <spdlog/spdlog.h>

#include "gps_network.h"
#include "gps_parser.h"
#include "time_sync.h"

using namespace std;

std::thread slam_th;
std::thread gps_th;
fstream gps_out;
openvslam::Mat44_t camera_pose;
gps_parser* gps;
geo_location* curr_geo;
time_sync* time_s;
cv::VideoCapture cap;
bool slam_gps_running;
bool enable_3d_view;

void run_slam(const std::string& vocab_path, const std::shared_ptr<openvslam::config>& cam_cfg,
              const std::string& vid_path, const std::string& map_db_path) {
    cap = cv::VideoCapture(vid_path);

    if (!cap.isOpened()) {
        spdlog::critical("Unable to open video from path @ " + vid_path);
        slam_gps_running = false;
        return;
    }

    const auto frame_cnt = cap.get(CV_CAP_PROP_FRAME_COUNT);
	// ideal time for per frame ms
    const milliseconds ideal_timestep_ms((long long)(1000 / cap.get(CV_CAP_PROP_FPS)));
	//total video time in millisecond
    const double video_time_ms = (frame_cnt / cap.get(CV_CAP_PROP_FPS)) * 1000.0;

    spdlog::info("video frame count: " + to_string(frame_cnt));

    // build a SLAM system
    openvslam::system SLAM(cam_cfg, vocab_path);
    // startup the SLAM process
    SLAM.startup();

    // create a viewer object
    // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
    pangolin_viewer::viewer viewer(cam_cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

    double timestamp = 0.0;
    std::vector<double> track_times;
    track_times.reserve(frame_cnt);
    spdlog::info("SLAM starting");
    cv::Mat img;
    chrono::milliseconds wait_for_gps_thread(0);
    auto tp_1 = std::chrono::steady_clock::now();
    // run the SLAM in another thread
    std::thread thread([&]() {
    for (unsigned int i = 0; i < frame_cnt; ++i) {
        

        cap.read(img);

        if (!img.empty()) {
            // input the current frame and estimate the camera pose
            camera_pose = SLAM.feed_monocular_frame(img, timestamp);
        }

        timestamp += 1.0 / cam_cfg->camera_->fps_;
        time_s->video_timestamp += ideal_timestep_ms;

        const auto tp_2 = std::chrono::steady_clock::now();
        const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();

        track_times.push_back(track_time);

        wait_for_gps_thread = time_s->is_video_caught_up_gps();
        if (wait_for_gps_thread > chrono::milliseconds(10)) {
            //spdlog::info("waiting for gps thread:" + to_string(wait_for_gps_thread.count()));
            std::this_thread::sleep_for(wait_for_gps_thread - chrono::milliseconds((tp_2 - tp_1).count()));
        }

        // check if the termination of SLAM system is requested or not
        if (SLAM.terminate_is_requested() || !slam_gps_running) {
            break;
        }

        tp_1 = tp_2;
    }

    // wait until the loop BA is finished
    while (SLAM.loop_BA_is_running()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

// automatically close the viewer at the end
#ifdef USE_PANGOLIN_VIEWER
    if (enable_3d_view)
        viewer.request_terminate();
#endif
    });

	
    // run the viewer in the current thread
#ifdef  USE_PANGOLIN_VIEWER
	 if (enable_3d_view) {
        viewer.run();
        spdlog::info("enabling 3D view");
    }
    else {
        //viewer.request_terminate();
        spdlog::info("running slam without 3D view");
    }

#endif //  USE_PANGOLIN_VIEWER

	 thread.join();

    SLAM.shutdown();

	if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }
    spdlog::info("exiting slam thread");
    slam_gps_running = false;

	
    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time = std::accumulate(track_times.begin(), track_times.end(), 0.0);
    std::cout << "median tracking time: " << track_times.at(track_times.size() / 2) << "[s]" << std::endl;
    std::cout << "mean tracking time: " << total_track_time / track_times.size() << "[s]" << std::endl;
}

void run_gps(string gps_path) {
    gps = new gps_parser(gps_path);

    spdlog::info("Starting gps parser");

    gps->start_reading(*time_s);

    spdlog::info("gps parsing thread stopped");
    slam_gps_running = false;
}

void fuse_gps_slam(const string& crr_gps_path, int freq = 1) {
    //KF(camera_pose + curr_geo) -> corrected gps location
    gps_out.open(crr_gps_path, fstream::out);

    double iter_time = 1000.0 / (double)freq;

    auto tp_1 = std::chrono::steady_clock::now();
    while (slam_gps_running) {
        const auto tp_2 = std::chrono::steady_clock::now();

        const auto track_time = std::chrono::duration_cast<std::chrono::duration<double>>(tp_2 - tp_1).count();
        const auto sleep_time = iter_time - track_time;

        if (gps->is_valid) {
            curr_geo->update_value(*gps);
            spdlog::info("fusion: " + curr_geo->value() + "  gps time: " + to_string(time_s->gps_timestamp.count()) + "  vid: " + to_string(time_s->video_timestamp.count()));
            gps_out << curr_geo->value() + "\n";
            cout << "fusion cam: " << camera_pose << endl;
        }
        else {
            spdlog::warn("fusion: invalid gps");
        }

        if (sleep_time > 0.0) {
            spdlog::info("fusion thread sleep for: " + to_string(static_cast<unsigned int>(sleep_time)));
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned int>(sleep_time)));

            tp_1 = tp_2;
        }
    }

    gps->terminate_process();
    gps_out.close();
    spdlog::info("Finished gps slam fusion");
}

int main(int argc, char* argv[]) {
    // create options
    popl::OptionParser op("Allowed options for gps client:");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "config file path");
    auto video_path = op.add<popl::Value<std::string>>("i", "video", "input video path");
    auto gps_path = op.add<popl::Value<std::string>>("g", "gps", "input gps txt path");
    auto crr_gps_path = op.add<popl::Value<std::string>>("o", "cgps", "corrected gps txt path");
    auto st_lat = op.add<popl::Value<double>>("l", "lat", "Start latitude");
    auto st_lon = op.add<popl::Value<double>>("m", "lon", "Start longitude");
    auto st_alt = op.add<popl::Value<double>>("a", "alt", "Start altitude", 0.0);
    auto map_db_path = op.add<popl::Value<std::string>>("p", "map-db", "store a map database at this path after SLAM", "");
    auto show_3d = op.add<popl::Value<int>>("s", "show", "show pangolin 3d view", 0);
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");

    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    if (!video_path->is_set() || !gps_path->is_set() || !crr_gps_path->is_set() || !st_lat->is_set() || !st_lon->is_set() || !vocab_file_path->is_set() || !config_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(config_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    if (show_3d->value() != 0)
        enable_3d_view = true;

    //set start gps location
    curr_geo = &geo_location(st_lat->value(), st_lon->value(), st_alt->value());

    //start slam and gps processes
    slam_gps_running = true;
    //time_sync::gps_timestamp = time_sync::video_timestamp = chrono::milliseconds(0);
    time_s = new time_sync();

    slam_th = thread(run_slam, vocab_file_path->value(), cfg, video_path->value(), map_db_path->value());
    gps_th = thread(run_gps, gps_path->value());

    fuse_gps_slam(crr_gps_path->value());

    //slam_th.join();
    //gps_th.join();
    spdlog::info("Exiting");

    return EXIT_SUCCESS;
}