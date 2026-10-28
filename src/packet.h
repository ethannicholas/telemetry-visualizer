// SimRacingStudio's custom-game "API" packet, version 102, as gcn-recomp's runtime/telemetry.cpp
// sends it: Python struct "3sI50s50s50sfffifffffffffffIIII" in native layout, 236 bytes,
// little-endian, a pad byte after the "api" tag and two after the three strings.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

struct Packet {
    std::string game, vehicle, location;
    float speed_kmh = 0, rpm = 0, max_rpm = 0;
    int gear = 0;
    float pitch = 0;          // degrees, nose up positive
    float roll = 0;           // degrees, right side down positive
    float yaw = 0;            // degrees, right positive (the platform's own turn, not a heading)
    float sway_velocity = 0;  // m/s to the right
    float sway = 0;           // m/s^2 to the right
    float heave = 0;          // m/s^2 up, without gravity
    float surge = 0;          // m/s^2 forward
    float suspension[4] = {}; // FL, FR, RL, RR, metres
};

constexpr size_t kPacketSize = 236;

// False if `data` is not a version 102 "api" packet.
bool decode(const uint8_t* data, size_t size, Packet& out);
