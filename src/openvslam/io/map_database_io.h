#ifndef OPENVSLAM_IO_MAP_DATABASE_IO_H
#define OPENVSLAM_IO_MAP_DATABASE_IO_H

#include "openvslam/data/bow_vocabulary.h"

#include <string>

namespace openvslam {

namespace data {
class camera_database;
class bow_database;
class map_database;
} // namespace data

namespace io {

class map_database_io {
public:
    /**
     * Constructor
     */
    map_database_io(data::camera_database* cam_db, data::map_database* map_db,
                    data::bow_database* bow_db, data::bow_vocabulary* bow_vocab);

    /**
     * Destructor
     */
    ~map_database_io() = default;

    /**
     * Save the map database as MessagePack
     */
    void save_message_pack(const std::string& path);

    /**
     * Load the map database from MessagePack
     */
    void load_message_pack(const std::string& path);

private:
    //! camera database
    data::camera_database* const cam_db_ = nullptr;
    //! map_database
    data::map_database* const map_db_ = nullptr;
    //! BoW database
    data::bow_database* const bow_db_ = nullptr;
    //! BoW vocabulary
    data::bow_vocabulary* const bow_vocab_ = nullptr;
};

} // namespace io
} // namespace openvslam

#endif // OPENVSLAM_IO_MAP_DATABASE_IO_H
