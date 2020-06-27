#pragma once

#include <stdlib.h>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>
#include <sstream>
#include <chrono>
#include <spdlog/spdlog.h>

#include "time_sync.h"

using namespace std;

class gps_parser {
public:
    gps_parser() {}
    gps_parser(std::string path)
        : file_path(path), is_valid(false), terminate(false) {}
    void start_reading(time_sync& time_s);
    void terminate_process();

    std::string lat, lon, alt;
    bool is_valid;

private:
    std::string file_path;
    long long timestamp;
    bool terminate;
};

struct geo_location {
public:
    //double for 15 digits of decimal precision
    double latitude;
    double longitude;
    double altitude;

    geo_location() { this->latitude = this->longitude = this->altitude = 0.0; }

    geo_location(double lat, double lon, double alt)
        : latitude(lat), longitude(lon), altitude(alt) {}

    geo_location(std::string lat, std::string lon, std::string alt) {
        geo_location(stod(lat), stod(lon), stod(alt));
    }

    std::string value() {
        return to_string(this->latitude) + "," + to_string(this->longitude) + "," + to_string(this->altitude);
    }

    void update_value(std::string lat, std::string lon, std::string alt) {
        this->latitude = stod(lat);
        this->longitude = stod(lon);
        this->altitude = stod(alt);
    }

    void update_value(gps_parser& gps) {
        this->latitude = stod(gps.lat);
        this->longitude = stod(gps.lon);
        this->altitude = stod(gps.alt);
    }
};