#include "socket_publisher/socket_client.h"

#include <spdlog/spdlog.h>

namespace socket_publisher {

socket_client::socket_client(const std::string& server_uri)
    : client_(), callback_() {
    // register socket callbacks
    client_.set_open_listener(std::bind(&socket_client::on_open, this));
    client_.set_close_listener(std::bind(&socket_client::on_close, this));
    client_.set_fail_listener(std::bind(&socket_client::on_fail, this));

    // start connection
    client_.connect(server_uri);
    // get socket
    socket_ = client_.socket();

    socket_->on("signal", std::bind(&socket_client::on_receive, this, std::placeholders::_1));
}

void socket_client::on_close() {
    spdlog::info("connection closed correctly");
}

void socket_client::on_fail() {
    spdlog::info("connection closed incorrectly");
}

void socket_client::on_open() {
    spdlog::info("connected to server");
}

void socket_client::on_receive(const sio::event& event) {
    try {
        const std::string message = event.get_message()->get_string();
        if (callback_) {
            callback_(message);
        }
    }
    catch (std::exception& ex) {
        spdlog::error(ex.what());
    }
}

} // namespace socket_publisher
