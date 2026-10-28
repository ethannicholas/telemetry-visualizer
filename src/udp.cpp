#include "udp.h"

#include <cstdio>
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using SockT = SOCKET;
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
using SockT = int;
#endif

namespace {

bool open_socket(intptr_t& sock) {
#ifdef _WIN32
    static bool started;
    if (!started) {
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return false;
        started = true;
    }
    SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s == INVALID_SOCKET) return false;
    u_long nonblocking = 1;
    ioctlsocket(s, FIONBIO, &nonblocking);
    sock = (intptr_t)s;
#else
    int s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) return false;
    fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
    sock = s;
#endif
    return true;
}

}  // namespace

Udp::~Udp() {
    if (sock_ < 0) return;
#ifdef _WIN32
    closesocket((SOCKET)sock_);
#else
    close((int)sock_);
#endif
}

bool Udp::listen(int port) {
    if (!open_socket(sock_)) {
        fprintf(stderr, "cannot make a UDP socket\n");
        return false;
    }
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind((SockT)sock_, (const sockaddr*)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "cannot listen on UDP port %d: is something else (SimRacingStudio?) holding it?\n", port);
        return false;
    }
    return true;
}

bool Udp::aim(const std::string& spec) {
    const size_t colon = spec.rfind(':');
    if (colon == std::string::npos) {
        fprintf(stderr, "--forward wants host:port, not '%s'\n", spec.c_str());
        return false;
    }
    const std::string host = spec.substr(0, colon), port = spec.substr(colon + 1);
    if (!open_socket(sock_)) return false;
    addrinfo hints = {}, *res = nullptr;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res) != 0 || !res) {
        fprintf(stderr, "cannot resolve '%s'\n", spec.c_str());
        return false;
    }
    memcpy(dest_, res->ai_addr, sizeof(sockaddr_in));
    freeaddrinfo(res);
    aimed_ = true;
    return true;
}

long Udp::receive(uint8_t* buf, size_t cap) {
    if (sock_ < 0) return -1;
    const auto n = recv((SockT)sock_, (char*)buf, (int)cap, 0);
    return n < 0 ? -1 : (long)n;
}

void Udp::send(const uint8_t* data, size_t size) {
    if (!aimed_) return;
    sendto((SockT)sock_, (const char*)data, (int)size, 0, (const sockaddr*)dest_,
           sizeof(sockaddr_in));
}
