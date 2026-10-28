// A non-blocking UDP socket: bound to a port to receive, or aimed at a host:port to forward.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

class Udp {
public:
    ~Udp();
    // Binds to `port` on every interface. False with a message on stderr if it cannot:
    // usually because something else (SimRacingStudio) holds the port.
    bool listen(int port);
    // Sends every packet to "host:port".
    bool aim(const std::string& spec);
    // The next datagram waiting, or -1 if none.
    long receive(uint8_t* buf, size_t cap);
    void send(const uint8_t* data, size_t size);

private:
    intptr_t sock_ = -1;
    uint8_t dest_[32] = {};   // a sockaddr_in
    bool aimed_ = false;
};
