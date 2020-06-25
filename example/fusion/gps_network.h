#pragma once

#include <iostream>
#include <SFML/Network.hpp>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>

class gps_network {
public:
    //latitude
    double lat;
    //longitude
    double lon;
    //altitude
    double alt;

    bool init_gps_server(std::string address, std::string port);
    void launch_update_loop(int freq_hz);
    static bool is_port_open(const std::string& address, int port) {
        return (sf::TcpSocket().connect(address, port) == sf::Socket::Done);
    }
    void close_connection();

private:
    void update_thread();
    sf::TcpSocket socket;
    sf::Socket::Status status;
    std::thread tcp_connection_th;
	//in Hz
    int update_freq;
    bool terminate_thread;
};
