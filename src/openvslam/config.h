#ifndef OPENVSLAM_CONFIG_H
#define OPENVSLAM_CONFIG_H

#include "openvslam/camera/base.h"
#include "openvslam/feature/orb_params.h"

#include <yaml-cpp/yaml.h>

namespace openvslam {

class config {
public:
    //! Constructor
    explicit config(const std::string& config_file_path);
    explicit config(const YAML::Node& yaml_node, const std::string& config_file_path = "");

    //! Destructor
    ~config();

    friend std::ostream& operator<<(std::ostream& os, const config& cfg);

    //! path to config YAML file
    const std::string config_file_path_;

    //! YAML node
    const YAML::Node yaml_node_;

    //! Camera model
    camera::base* camera_ = nullptr;
};

} // namespace openvslam

#endif // OPENVSLAM_CONFIG_H
