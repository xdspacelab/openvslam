#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/camera_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/data/map_database.h"
#include "openvslam/io/map_database_io.h"

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace openvslam {
namespace io {

map_database_io::map_database_io(data::camera_database* cam_db, data::map_database* map_db,
                                 data::bow_database* bow_db, data::bow_vocabulary* bow_vocab)
    : cam_db_(cam_db), map_db_(map_db), bow_db_(bow_db), bow_vocab_(bow_vocab) {}

void map_database_io::save_message_pack(const std::string& path) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    assert(cam_db_ && map_db_);
    const auto cameras = cam_db_->to_json();
    nlohmann::json keyfrms;
    nlohmann::json landmarks;
    map_db_->to_json(keyfrms, landmarks);

    nlohmann::json json{{"cameras", cameras},
                        {"keyframes", keyfrms},
                        {"landmarks", landmarks},
                        {"frame_next_id", static_cast<unsigned int>(data::frame::next_id_)},
                        {"keyframe_next_id", static_cast<unsigned int>(data::keyframe::next_id_)},
                        {"landmark_next_id", static_cast<unsigned int>(data::landmark::next_id_)}};

    std::ofstream ofs(path, std::ios::out | std::ios::binary);

    if (ofs.is_open()) {
        spdlog::info("save the MessagePack file of database to {}", path);
        const auto msgpack = nlohmann::json::to_msgpack(json);
        ofs.write(reinterpret_cast<const char*>(msgpack.data()), msgpack.size() * sizeof(uint8_t));
        ofs.close();
    }
    else {
        spdlog::critical("cannot create a file at {}", path);
    }
}

void map_database_io::load_message_pack(const std::string& path) {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    // 1. initialize database

    assert(cam_db_ && map_db_ && bow_db_ && bow_vocab_);
    map_db_->clear();
    bow_db_->clear();

    // 2. load binary bytes

    std::ifstream ifs(path, std::ios::in | std::ios::binary);
    if (!ifs.is_open()) {
        spdlog::critical("cannot load the file at {}", path);
        throw std::runtime_error("cannot load the file at " + path);
    }

    spdlog::info("load the MessagePack file of database from {}", path);
    std::vector<uint8_t> msgpack;
    while (true) {
        uint8_t buffer;
        ifs.read(reinterpret_cast<char*>(&buffer), sizeof(uint8_t));
        if (ifs.eof()) {
            break;
        }
        msgpack.push_back(buffer);
    }
    ifs.close();

    // 3. parse into JSON

    const auto json = nlohmann::json::from_msgpack(msgpack);

    // 4. load database

    // load static variables
    data::frame::next_id_ = json.at("frame_next_id").get<unsigned int>();
    data::keyframe::next_id_ = json.at("keyframe_next_id").get<unsigned int>();
    data::landmark::next_id_ = json.at("landmark_next_id").get<unsigned int>();
    // load database
    const auto json_cameras = json.at("cameras");
    cam_db_->from_json(json_cameras);
    const auto json_keyfrms = json.at("keyframes");
    const auto json_landmarks = json.at("landmarks");
    map_db_->from_json(cam_db_, bow_vocab_, bow_db_, json_keyfrms, json_landmarks);
    const auto keyfrms = map_db_->get_all_keyframes();
    for (const auto keyfrm : keyfrms) {
        bow_db_->add_keyframe(keyfrm);
    }
}

} // namespace io
} // namespace openvslam
