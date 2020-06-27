#include "gps_network.h"

#include <math.h>
//#define PORT 20175

bool gps_network::init_gps_server(std::string address, std::string port) {
    this->status = this->socket.connect(address, std::stoi(port));
    if (this->status != sf::Socket::Done) {
        std::cout << "TCP: error initializing server";
        return false;
    }
    this->lat = this->lon = this->alt = 0.0;
    return true;
}

// frequence in hertz clampped between [1 - 30]
void gps_network::launch_update_loop(int freq_hz) {
    if (freq_hz < 1)
        freq_hz = 1;
    else if (freq_hz > 30)
        freq_hz = 30;

    this->update_freq = freq_hz;
    this->terminate_thread = false;

    this->tcp_connection_th = std::thread(&gps_network::update_thread, this);
}

void gps_network::close_connection() {
    this->terminate_thread = true;
    tcp_connection_th.join();
}

void gps_network::update_thread() {
    if (this->status != sf::Socket::Done) {
        std::cout << "\nTCP: error connecting";
        return;
    }

    char data[200];
    std::size_t received;
    std::chrono::duration<double, std::ratio<1>> freq((double)(1.0 / (double)this->update_freq));

    // TCP socket:
    while (true) {
        if (this->terminate_thread)
            break;

        if (this->socket.receive(data, 200, received) != sf::Socket::Done) {
            this->terminate_thread = true;
            std::cout << "\nTCP: error receiving data";
        }
        else {
            std::cout << "\nTCP: " << data << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        //std::cout << "\nSleep: " << 1.0/update_freq << std::endl;
        //std::this_thread::sleep_for(freq);
    }
}