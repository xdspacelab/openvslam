
#include <iostream>
#include <string>
#include <fstream>
#include <thread>
#include <chrono>
#include <ctime>
#include <popl.hpp>
#include <spdlog/spdlog.h>

#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include "SFML/Network.hpp"

using namespace std;
using namespace cv;

// gps_path out file stream
ofstream fout;
//OCV video capture
VideoCapture cap;
VideoWriter vid_writer;
// video update thread
thread vid_th;
// GPS update thread
thread gps_th;
//latitude
//double lat;
//longitude
//double lon;
//altitude
//double alt;

// string lat,long,alt
string msg;
sf::TcpSocket socket;
sf::Socket::Status status;
//in Hz
int update_freq;
bool terminate_gps_thread;
bool terminate_vid_thread;

void gps_update_thread() {
    if (status != sf::Socket::Done) {
        spdlog::critical("TCP: error connecting");
        return;
    }

    char data[200];
    std::size_t received;
    //std::chrono::duration<double, std::ratio<1>> freq((double)(1.0 / (double)this->update_freq));

    // TCP socket:
    while (true) {
        if (terminate_gps_thread || terminate_vid_thread)
            break;

        if (socket.receive(data, 200, received) != sf::Socket::Done) {
            terminate_gps_thread = true;
            spdlog::critical("TCP: error receiving data");
            //?break;
        }
        else {
            msg = string(data);
            spdlog::info("TCP: " + msg);
            msg = msg.substr(msg.find_first_of('(') + 1, msg.find_first_of(')') - msg.find_first_of('(') - 1);
            fout << msg + "\n";
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        //std::cout << "\nSleep: " << 1.0/update_freq << std::endl;
        //std::this_thread::sleep_for(freq);
    }
}

void video_update_thread() {
    Mat curr_frame;
    int frame = 0;

	while (true) {
        if (terminate_vid_thread || terminate_gps_thread)
            break;

        cap.read(curr_frame);

        //if (curr_frame.empty())
        //continue;
        frame++;
        vid_writer << curr_frame;
//        fout << msg + "\n";
        //imshow("Video", curr_frame);
    }
    
}

void start_recording(string ip, int port, int freq, string gps_path, string video_path, int width, int height, int cam_ind) {
    fout.open(gps_path, fstream::out);

    if (fout.fail()) {
        spdlog::critical("Faile to open gps file. path: " + gps_path);
        return;
    }
    fout.close();
    fout.open(gps_path, fstream::app);
    fout << "# $<time,lat,lon,alt>\n";

    //cap = VideoCapture(cam_ind);
    //if (!cap.isOpened()) {
    //    spdlog::critical("Failed to start camera index: " + to_string(cam_ind));
    //    return;
    //}
    //cap.set(CAP_PROP_FRAME_WIDTH, width);
    //cap.set(CAP_PROP_FRAME_HEIGHT, height);
    //cap.set(CAP_PROP_FPS, freq);

    //vid_writer.open(video_path, static_cast<int>(cap.get(CAP_PROP_FOURCC)), freq, Size(width, height), true);

    //if (!vid_writer.isOpened()) {
    //    spdlog::critical("Failed to open video writer. FOURCC- " + to_string(static_cast<int>(cap.get(CAP_PROP_FOURCC))));
    //    return;
    //}

    //Mat curr_frame;

    //initialize connection with GPS server
    status = socket.connect(ip, port);
    if (status != sf::Socket::Done) {
        spdlog::critical("Failed to initialize TCP connection. server: " + ip + ":" + to_string(port));
        return;
    }
    //start gps update loop
    gps_th = std::thread(gps_update_thread);
    //vid_th = std::thread(video_update_thread);
    cv::namedWindow("Video", WINDOW_NORMAL);
    std::cout << "\nwait period: " << 1000 / freq << endl;
    //int frame = 0;
	time_t t1 = time(nullptr);
    while (true) {
        if (waitKey(30) == 27 || terminate_gps_thread)
            break;

        //cap.read(curr_frame);

        //if (curr_frame.empty())
        //    continue;
        ////frame++;
        //vid_writer << curr_frame;
        //fout << msg + "\n";
        //cout << frame<<"_";
        //imshow("Video", curr_frame);
    }

    //Close active thread and return
    terminate_vid_thread = true;
    terminate_gps_thread = true;
    gps_th.join();
    //vid_th.join();
    //close fstream
    fout.close();
    vid_writer.release();
    cap.release();
    cout << (time(nullptr) - t1) << " sec" << endl;
}

int main(int argc, char* argv[]) {
    // create options
    popl::OptionParser op("Options for recording video_gps test data:");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto ip = op.add<popl::Value<std::string>>("i", "ip", "IP address");
    auto port = op.add<popl::Value<int>>("p", "port", "Port address", 20175);
    auto freq = op.add<popl::Value<int>>("f", "freq", "update frequency for video and gps");
    auto gps_path = op.add<popl::Value<std::string>>("g", "gps", "GPS record path", "gps.txt");
    auto video_path = op.add<popl::Value<std::string>>("v", "vid", "video record path", "video.mp4");
    auto width = op.add<popl::Value<int>>("x", "width", "video width", 640);
    auto height = op.add<popl::Value<int>>("y", "height", "video height", 480);
    auto cam_num = op.add<popl::Value<int>>("c", "cam", "camera index", 0);
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

    if (!ip->is_set() || !freq->is_set()) {
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

    msg = "";

    start_recording(ip->value(), port->value(), freq->value(), gps_path->value(), video_path->value(), width->value(), height->value(), cam_num->value());

    return EXIT_SUCCESS;
}