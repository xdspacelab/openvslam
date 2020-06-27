#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <vector>
#include <fstream>

#include <popl.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;

//Capture Button
//Rect capture_btn;
//int capture_click = 0;
//Mat capture_btn_img;
//
////Calibrate Button
//Rect calib_btn;
//int calib_click = 0;
//Mat calib_btn_img;

//Camera config params
cv::Mat intrinsic;  //3x3
cv::Mat distCoeffs; //5x1; k1, k2, p1, p2, k3 : radial(k) and tangential(p)
cv::Size frame_sz;
int cam_fps = 0;

void serialize_calib_params(string config_path) {
    ofstream fout;
    fout.open(config_path, ios::out);
    spdlog::info("Seralizing camera params  @ " + config_path);

    YAML::Emitter y_out;
    y_out << YAML::Comment("#Perspective model\n\n#Camera Model Params");
    y_out << YAML::BeginMap;
    y_out << YAML::Key << "Camera.name";
    y_out << YAML::Value << "__Test__";
    y_out << YAML::Key << "Camera.setup";
    y_out << YAML::Value << "monocular";
    y_out << YAML::Key << "Camera.model";
    y_out << YAML::Value << "perspective";
    y_out << YAML::Key << "Camera.fx";
    y_out << YAML::Value << intrinsic.at<double>(0, 0);
    y_out << YAML::Key << "Camera.fy";
    y_out << YAML::Value << intrinsic.at<double>(1, 1);
    y_out << YAML::Key << "Camera.cx";
    y_out << YAML::Value << intrinsic.at<double>(0, 2);
    y_out << YAML::Key << "Camera.cy";
    y_out << YAML::Value << intrinsic.at<double>(1, 2);
    y_out << YAML::Key << "Camera.k1";
    y_out << YAML::Value << ((distCoeffs.cols > 0) ? distCoeffs.at<double>(0, 0) : 0.0);
    y_out << YAML::Key << "Camera.k2";
    y_out << YAML::Value << ((distCoeffs.cols > 1) ? distCoeffs.at<double>(0, 1) : 0.0);
    y_out << YAML::Key << "Camera.p1";
    y_out << YAML::Value << ((distCoeffs.cols > 2) ? distCoeffs.at<double>(0, 2) : 0.0);
    y_out << YAML::Key << "Camera.p2";
    y_out << YAML::Value << ((distCoeffs.cols > 3) ? distCoeffs.at<double>(0, 3) : 0.0);
    y_out << YAML::Key << "Camera.k3";
    y_out << YAML::Value << ((distCoeffs.cols > 4) ? distCoeffs.at<double>(0, 4) : 0.0);
    y_out << YAML::Key << "Camera.fps";
    y_out << YAML::Value << cam_fps;
    y_out << YAML::Key << "Camera.cols";
    y_out << YAML::Value << frame_sz.width;
    y_out << YAML::Key << "Camera.rows";
    y_out << YAML::Value << frame_sz.height;
    y_out << YAML::Key << "Camera.color_order";
    y_out << YAML::Value << "RGB";

    // y_out << YAML::Comment("# ORB Params");

    y_out << YAML::Key << "Feature.max_num_keypoints";
    y_out << YAML::Value << 2000;
    y_out << YAML::Key << "Feature.scale_factor";
    y_out << YAML::Value << 1.2;
    y_out << YAML::Key << "Feature.num_levels";
    y_out << YAML::Value << 8;
    y_out << YAML::Key << "Feature.ini_fast_threshold";
    y_out << YAML::Value << 20;
    y_out << YAML::Key << "Feature.min_fast_threshold";
    y_out << YAML::Value << 7;
    double mas[4][4] = {{0.0, 1.0, 0.0, 0.1},
                        {0.0, 1.0, 0.84, 1.0},
                        {0.0, 0.2, 0.7, 1.0},
                        {0.8, 1.0, 0.7, 1.0}};
    vector<vector<double>> mask = {vector<double>(begin(mas[0]), end(mas[0])),
                                   vector<double>(begin(mas[1]), end(mas[1])),
                                   vector<double>(begin(mas[2]), end(mas[2])),
                                   vector<double>(begin(mas[3]), end(mas[3]))};
    y_out << YAML::Key << "Feature.mask_rectangles";
    y_out << YAML::BeginSeq;
    y_out << YAML::Flow << mask[0];
    y_out << YAML::Flow << mask[1];
    y_out << YAML::Flow << mask[2];
    y_out << YAML::Flow << mask[3];
    y_out << YAML::EndSeq;
    y_out << YAML::EndMap;

    try {
        fout << y_out.c_str();
        fout.close();
    }
    catch (Exception e) {
        spdlog::critical("Exception creating config file: " + string(e.what()));
        return;
    }
    spdlog::info("Camera params serialized successfully @ " + config_path);
    return;
}

void initiate_camera_calibration(const string& video_path, const float sq_size, const string& config_file_path, int skip, string& out_vid_path) {
    int numCornersHor = 9;
    int numCornersVer = 6;
    int numSquares = numCornersHor * numCornersVer;

    cv::Size board_sz = Size(numCornersHor, numCornersVer);

    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> image_points;
    vector<Point2f> corners;

    //correctly captured frame count
    int cap_frame = 0;

    object_points.clear();
    image_points.clear();
    corners.clear();

    auto video = cv::VideoCapture(video_path);

    if (!video.isOpened()) {
        spdlog::critical("cannot open a video", video_path);
        return;
    }

    //video.set(CAP_PROP_FRAME_WIDTH,640);
    //video.set(CAP_PROP_FRAME_HEIGHT,480);

    namedWindow("Calibration", cv::WINDOW_NORMAL);
    //setMouseCallback("Calibration", onMouse);

    Mat curr_frame;
    Mat disp_frame;

    vector<Point3f> obj;
    for (int j = 0; j < numCornersVer; j++) {
        for (int k = 0; k < numCornersHor; k++) {
            obj.push_back(Point3f(sq_size * k, sq_size * j, 0.0f));
        }
    }

    //dry test to capture first frame
    video.read(disp_frame);
    waitKey(100);

    frame_sz = Size(video.get(CAP_PROP_FRAME_WIDTH), video.get(CAP_PROP_FRAME_HEIGHT));
    cam_fps = video.get(CAP_PROP_FPS);

    cv::VideoWriter writer;
    bool write_frame = false;

    if (!out_vid_path.empty()) {
        writer.open(out_vid_path, video.get(CAP_PROP_FOURCC), 30, frame_sz);

        if (!writer.isOpened()) {
            spdlog::critical("video writer can not open. file @ " + out_vid_path);
            writer.release();
        }
        else {
            write_frame = true;
            spdlog::info("writing video @ " + out_vid_path);
        }
    }

    //cout << "Btn size: " << capture_btn << endl;
    std::cout << "frame size: " << frame_sz << endl;
    int sk = 0;
    while (true) {
        video.read(disp_frame);

        if (disp_frame.empty())
            break;

        cvtColor(disp_frame, curr_frame, COLOR_BGR2GRAY);

        if (findChessboardCorners(curr_frame, board_sz, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS)) {
            if (sk % skip == 0) {
                //sub-pixel accurate location
                cornerSubPix(curr_frame, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER | TermCriteria::EPS, 30, 0.001));
                //drawChessboardCorners(image, board_sz, corners, found);

                image_points.push_back(corners);
                object_points.push_back(obj);
                cap_frame++;
                if (write_frame)
                    writer.write(disp_frame);
                spdlog::info("captured frame. Total: " + to_string(cap_frame));
            }
            sk++;
        }
        else {
            spdlog::info("Invalid frame");
        }

        imshow("Calibration", disp_frame);
        //Exit on press of esc key
        if (waitKey(30) == 27)
            break;
    }

	if (write_frame)
	{
        writer.release();
        spdlog::info("Finished writing video");
	}

    Mat R, T;
    if (cap_frame >= 10)
        cv::calibrateCamera(object_points, image_points, frame_sz, intrinsic, distCoeffs, R, T);
    else
        spdlog::critical("Not enough captured frames... (need at least 10) - " + to_string(cap_frame));

    //spdlog::info("Intrinsics: ",intrinsic);
    //spdlog::info("Distortion coefficents: ",distCoeffs);

    std::cout << "\nIntrinsic:\n"
         << intrinsic << endl;
    std::cout << "\nDist coeff: " << distCoeffs << endl;

    serialize_calib_params(config_file_path);
}

int main(int argc, char* argv[]) {
    // create options
    popl::OptionParser op("Allowed options for calibration with 9x6 checkerboard:");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto video_path = op.add<popl::Value<std::string>>("i", "video", "input video file path");
    auto sq_size = op.add<popl::Value<float>>("s", "size", "square size in cm", 0.026);
    auto skip = op.add<popl::Value<int>>("k", "skip", "skip k valid frames", 2);
    auto config_file_path = op.add<popl::Value<std::string>>("c", "config", "output camera config file path");
    auto out_vid_path = op.add<popl::Value<std::string>>("o", "vid", "output video file path","");
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
    if (!video_path->is_set() || !config_file_path->is_set()) {
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

    initiate_camera_calibration(video_path->value(), sq_size->value(), config_file_path->value(), skip->value(), out_vid_path->value());

    return EXIT_SUCCESS;
}