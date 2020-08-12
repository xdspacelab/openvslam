#ifndef SOCKET_PUBLISHER_SOCKET_CLIENT_H
#define SOCKET_PUBLISHER_SOCKET_CLIENT_H

#include "openvslam/config.h"

#include <sioclient/sio_client.h>

namespace openvslam {
class config;
} // namespace openvslam

namespace socket_publisher {

class socket_client {
public:
    socket_client(const std::string& server_uri);

    void emit(const std::string tag, const std::string buffer) {
        socket_->emit(tag, buffer);
    }

    void set_signal_callback(std::function<void(std::string)> callback) {
        callback_ = callback;
    }

private:
    void on_close();
    void on_fail();
    void on_open();
    void on_receive(const sio::event& event);

    sio::client client_;
    sio::socket::ptr socket_;

    std::function<void(std::string)> callback_;
};

} // namespace socket_publisher

#endif // SOCKET_PUBLISHER_SOCKET_CLIENT_H
