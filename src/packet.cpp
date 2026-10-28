#include "packet.h"

#include <cstring>

namespace {

uint32_t u32(const uint8_t* p, size_t at) { uint32_t v; memcpy(&v, p + at, 4); return v; }
float f32(const uint8_t* p, size_t at) { float v; memcpy(&v, p + at, 4); return v; }
std::string str(const uint8_t* p, size_t at, size_t n) {
    size_t len = 0;
    while (len < n && p[at + len]) len++;
    return std::string((const char*)p + at, len);
}

}  // namespace

bool decode(const uint8_t* p, size_t size, Packet& out) {
    // The four trailing terrain words are optional to a reader; everything up to the
    // suspension is required.
    if (size < 220 || memcmp(p, "api", 3) != 0 || u32(p, 4) != 102) return false;
    out.game = str(p, 8, 50);
    out.vehicle = str(p, 58, 50);
    out.location = str(p, 108, 50);
    out.speed_kmh = f32(p, 160);
    out.rpm = f32(p, 164);
    out.max_rpm = f32(p, 168);
    out.gear = (int)u32(p, 172);
    out.pitch = f32(p, 176);
    out.roll = f32(p, 180);
    out.yaw = f32(p, 184);
    out.sway_velocity = f32(p, 188);
    out.sway = f32(p, 192);
    out.heave = f32(p, 196);
    out.surge = f32(p, 200);
    for (int w = 0; w < 4; w++) out.suspension[w] = f32(p, 204 + 4 * w);
    return true;
}
