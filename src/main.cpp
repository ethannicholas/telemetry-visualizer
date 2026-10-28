// A motion platform going through the motions a game sends it: listens for SimRacingStudio's
// "api" packets (version 102, as the recomp projects' --telemetry sends them), moves a six-legged
// platform with them, and charts every channel over the last ten seconds.
//
// What the platform does with the numbers is this program's guess at a rig, not
// SimRacingStudio's: pitch, roll and yaw are the platform's angles as sent (times
// --angle-scale), and surge, sway and heave move it along its axes, --travel metres for each
// m/s^2, up to --max-travel. The charts are the numbers themselves.

#include "packet.h"
#include "udp.h"

#include <SDL.h>
#include <SDL_opengl.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>

#define STB_EASY_FONT_IMPLEMENTATION
#include "stb_easy_font.h"

namespace {

// ---- Options ------------------------------------------------------------------------------

struct Options {
    int port = 33001;
    std::string forward;
    float angle_scale = 1.0f;
    float travel = 0.004f;      // metres of platform travel per m/s^2
    float max_travel = 0.12f;   // metres
    float history_s = 10.0f;
};

void usage() {
    printf(
        "telemetry-visualizer [options]\n"
        "  --port N            UDP port to listen on (default 33001, SimRacingStudio's)\n"
        "  --forward HOST:PORT also send every packet received on to HOST:PORT\n"
        "  --angle-scale X     platform angle per degree sent (default 1)\n"
        "  --travel X          platform travel in metres per m/s^2 (default 0.004)\n"
        "  --max-travel X      most the platform moves along an axis, metres (default 0.12)\n"
        "  --history S         seconds the charts show (default 10)\n"
        "\n"
        "Drag to orbit, scroll to zoom, R to reset the view, Space to freeze, Esc to quit.\n");
}

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        const bool has_value = i + 1 < argc;
        if ((a == "-h" || a == "--help")) { usage(); exit(0); }
        else if (a == "--port" && has_value) o.port = atoi(argv[++i]);
        else if (a == "--forward" && has_value) o.forward = argv[++i];
        else if (a == "--angle-scale" && has_value) o.angle_scale = (float)atof(argv[++i]);
        else if (a == "--travel" && has_value) o.travel = (float)atof(argv[++i]);
        else if (a == "--max-travel" && has_value) o.max_travel = (float)atof(argv[++i]);
        else if (a == "--history" && has_value) o.history_s = std::max(1.0f, (float)atof(argv[++i]));
        else { fprintf(stderr, "unknown option '%s'\n\n", a.c_str()); usage(); return false; }
    }
    return true;
}

// ---- Vectors ------------------------------------------------------------------------------

struct V3 { float x, y, z; };
V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
float length(V3 a) { return sqrtf(dot(a, a)); }
V3 normalize(V3 a) { const float l = length(a); return l > 0 ? a * (1.0f / l) : a; }

// Rows of a rotation.
struct M3 { V3 r[3]; };
V3 operator*(const M3& m, V3 v) { return {dot(m.r[0], v), dot(m.r[1], v), dot(m.r[2], v)}; }
M3 operator*(const M3& a, const M3& b) {
    M3 m;
    for (int i = 0; i < 3; i++) {
        const V3 c0 = {b.r[0].x, b.r[1].x, b.r[2].x}, c1 = {b.r[0].y, b.r[1].y, b.r[2].y},
                 c2 = {b.r[0].z, b.r[1].z, b.r[2].z};
        m.r[i] = {dot(a.r[i], c0), dot(a.r[i], c1), dot(a.r[i], c2)};
    }
    return m;
}
constexpr float kDeg = 3.14159265f / 180.0f;
M3 rot_x(float d) { const float c = cosf(d * kDeg), s = sinf(d * kDeg); return {{{1, 0, 0}, {0, c, -s}, {0, s, c}}}; }
M3 rot_y(float d) { const float c = cosf(d * kDeg), s = sinf(d * kDeg); return {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}}; }
M3 rot_z(float d) { const float c = cosf(d * kDeg), s = sinf(d * kDeg); return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}}; }

// ---- The rig ------------------------------------------------------------------------------
//
// World: y up, the rider facing -z, +x to their right. A Stewart platform: six actuators in
// pairs from a base ring to a smaller top ring, the pairs crossed over by 60 degrees.

constexpr float kBaseY = 0.08f;          // top of the base plate
constexpr float kBaseRadius = 0.75f;
constexpr float kTopRadius = 0.48f;
constexpr float kTopHeight = 0.62f;      // the platform's underside at rest
constexpr float kBodyLength = 0.42f;     // an actuator's cylinder; its rod covers the rest

struct Pose {
    M3 rot;
    V3 pos;   // the platform's origin, in the middle of its underside
};

Pose pose_for(const Packet& p, const Options& o) {
    auto travel = [&](float a) { return std::clamp(a * o.travel, -o.max_travel, o.max_travel); };
    Pose pose;
    // Nose up is +x rotation of a -z nose; right side down and a right turn are the
    // negative rotations about z and y.
    pose.rot = rot_y(-p.yaw * o.angle_scale) * rot_x(p.pitch * o.angle_scale) * rot_z(-p.roll * o.angle_scale);
    pose.pos = {travel(p.sway), kTopHeight + travel(p.heave), -travel(p.surge)};
    return pose;
}

V3 ring(float radius, float degrees, float y) {
    return {radius * cosf(degrees * kDeg), y, radius * sinf(degrees * kDeg)};
}
const float kBaseAngles[6] = {-12, 12, 108, 132, 228, 252};
const float kTopAngles[6] = {-48, 48, 72, 168, 192, 288};

void legs(const Pose& pose, V3 base[6], V3 top[6]) {
    for (int i = 0; i < 6; i++) {
        base[i] = ring(kBaseRadius, kBaseAngles[i] + 90, kBaseY);
        top[i] = pose.pos + pose.rot * ring(kTopRadius, kTopAngles[i] + 90, 0);
    }
}

// ---- Drawing ------------------------------------------------------------------------------

void color(float r, float g, float b, float a = 1) { glColor4f(r, g, b, a); }

void cylinder(V3 a, V3 b, float radius, bool caps = true) {
    const V3 axis = normalize(b - a);
    const V3 other = fabsf(axis.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0};
    const V3 u = normalize(cross(axis, other)), v = cross(axis, u);
    constexpr int kSides = 20;
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= kSides; i++) {
        const float t = 2 * 3.14159265f * i / kSides;
        const V3 n = u * cosf(t) + v * sinf(t);
        glNormal3f(n.x, n.y, n.z);
        const V3 p = a + n * radius, q = b + n * radius;
        glVertex3f(p.x, p.y, p.z);
        glVertex3f(q.x, q.y, q.z);
    }
    glEnd();
    if (!caps) return;
    for (int end = 0; end < 2; end++) {
        const V3 c = end ? b : a, n = end ? axis : axis * -1;
        glBegin(GL_TRIANGLE_FAN);
        glNormal3f(n.x, n.y, n.z);
        glVertex3f(c.x, c.y, c.z);
        for (int i = 0; i <= kSides; i++) {
            const float t = 2 * 3.14159265f * (end ? i : kSides - i) / kSides;
            const V3 p = c + (u * cosf(t) + v * sinf(t)) * radius;
            glVertex3f(p.x, p.y, p.z);
        }
        glEnd();
    }
}

// An axis-aligned box from its centre and half sizes, in whatever frame is current.
void box(V3 c, V3 h) {
    const V3 lo = c - h, hi = c + h;
    const float f[6][4][3] = {
        {{hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, hi.y, hi.z}, {hi.x, lo.y, hi.z}},
        {{lo.x, lo.y, hi.z}, {lo.x, hi.y, hi.z}, {lo.x, hi.y, lo.z}, {lo.x, lo.y, lo.z}},
        {{lo.x, hi.y, lo.z}, {lo.x, hi.y, hi.z}, {hi.x, hi.y, hi.z}, {hi.x, hi.y, lo.z}},
        {{lo.x, lo.y, hi.z}, {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, lo.y, hi.z}},
        {{lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}},
        {{lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, lo.y, lo.z}, {lo.x, lo.y, lo.z}},
    };
    const float n[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    glBegin(GL_QUADS);
    for (int i = 0; i < 6; i++) {
        glNormal3fv(n[i]);
        for (int k = 0; k < 4; k++) glVertex3fv(f[i][k]);
    }
    glEnd();
}

// A flat round plate from y0 to y1.
void disc(float radius, float y0, float y1) { cylinder({0, y0, 0}, {0, y1, 0}, radius); }

void arrow(V3 from, V3 to, float radius) {
    const V3 d = to - from;
    const float len = length(d);
    if (len < 0.01f) return;
    const V3 axis = d * (1.0f / len);
    const float head = std::min(0.08f, len * 0.4f);
    const V3 neck = to - axis * head;
    cylinder(from, neck, radius);
    const V3 other = fabsf(axis.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0};
    const V3 u = normalize(cross(axis, other)), v = cross(axis, u);
    glBegin(GL_TRIANGLE_FAN);
    glNormal3f(axis.x, axis.y, axis.z);
    glVertex3f(to.x, to.y, to.z);
    for (int i = 0; i <= 16; i++) {
        const float t = 2 * 3.14159265f * i / 16;
        const V3 r = u * cosf(t) + v * sinf(t);
        const V3 p = neck + r * (radius * 2.6f);
        glNormal3f(r.x, r.y, r.z);
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();
}

void mult(const Pose& pose) {
    const M3& r = pose.rot;
    const float m[16] = {r.r[0].x, r.r[1].x, r.r[2].x, 0, r.r[0].y, r.r[1].y, r.r[2].y, 0,
                         r.r[0].z, r.r[1].z, r.r[2].z, 0, pose.pos.x, pose.pos.y, pose.pos.z, 1};
    glMultMatrixf(m);
}

struct Camera {
    float yaw = 35, pitch = 20, distance = 3.9f;
};

void draw_rig(const Packet& p, const Options& o, const Camera& cam, int w, int h) {
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const float near_ = 0.05f, far_ = 50.0f, fy = near_ * tanf(35 * kDeg / 2);
    const float fx = fy * w / std::max(1, h);
    glFrustum(-fx, fx, -fy, fy, near_, far_);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0, 0, -cam.distance);
    glRotatef(cam.pitch, 1, 0, 0);
    glRotatef(-cam.yaw, 0, 1, 0);
    glTranslatef(0, -0.6f, 0);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    const float key[4] = {0.4f, 1.0f, 0.6f, 0}, fill[4] = {-0.6f, 0.3f, -0.5f, 0};
    const float key_c[4] = {0.85f, 0.85f, 0.8f, 1}, fill_c[4] = {0.25f, 0.28f, 0.35f, 1};
    const float ambient[4] = {0.22f, 0.22f, 0.25f, 1};
    glLightfv(GL_LIGHT0, GL_POSITION, key);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, key_c);
    glLightfv(GL_LIGHT1, GL_POSITION, fill);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, fill_c);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

    // The floor.
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    for (int i = -10; i <= 10; i++) {
        const float t = i * 0.25f;
        const float a = i == 0 ? 0.32f : 0.17f;
        color(a, a, a + 0.03f);
        glVertex3f(t, 0, -2.5f); glVertex3f(t, 0, 2.5f);
        glVertex3f(-2.5f, 0, t); glVertex3f(2.5f, 0, t);
    }
    glEnd();
    glEnable(GL_LIGHTING);

    const Pose pose = pose_for(p, o);
    const Pose rest = {rot_x(0), {0, kTopHeight, 0}};

    // The base.
    color(0.32f, 0.33f, 0.36f);
    disc(kBaseRadius + 0.12f, 0, kBaseY);

    // The actuators, coloured by how far each is extended from rest: blue short, red long.
    V3 base[6], top[6], rest_base[6], rest_top[6];
    legs(pose, base, top);
    legs(rest, rest_base, rest_top);
    for (int i = 0; i < 6; i++) {
        const V3 d = top[i] - base[i];
        const float len = length(d), rest_len = length(rest_top[i] - rest_base[i]);
        const V3 axis = d * (1.0f / len);
        const float e = std::clamp((len - rest_len) / 0.12f, -1.0f, 1.0f);
        color(0.18f, 0.18f, 0.2f);
        cylinder(base[i], base[i] + V3{0, 0.03f, 0}, 0.05f);   // the mount
        color(0.45f + 0.4f * std::max(e, 0.0f), 0.47f - 0.2f * fabsf(e), 0.5f + 0.4f * std::max(-e, 0.0f));
        cylinder(base[i], base[i] + axis * kBodyLength, 0.04f);
        color(0.82f, 0.83f, 0.85f);
        cylinder(base[i] + axis * (kBodyLength - 0.02f), top[i], 0.018f);
    }

    // The platform and what is on it, in its own frame.
    glPushMatrix();
    mult(pose);
    color(0.62f, 0.64f, 0.68f);
    disc(kTopRadius + 0.08f, 0, 0.04f);
    color(0.95f, 0.55f, 0.15f);                 // the front edge, so its way is plain
    box({0, 0.045f, -(kTopRadius + 0.02f)}, {0.12f, 0.008f, 0.05f});
    color(0.12f, 0.12f, 0.14f);
    box({0, 0.16f, 0.05f}, {0.17f, 0.04f, 0.2f});        // seat
    box({0, 0.08f, 0.05f}, {0.05f, 0.04f, 0.05f});       // seat post
    color(0.14f, 0.14f, 0.17f);
    box({0, 0.36f, 0.27f}, {0.16f, 0.17f, 0.03f});        // backrest
    color(0.3f, 0.31f, 0.34f);
    cylinder({0, 0.04f, -0.42f}, {0, 0.45f, -0.30f}, 0.025f);   // steering column
    cylinder({-0.24f, 0.47f, -0.30f}, {0.24f, 0.47f, -0.30f}, 0.018f);   // handlebar
    color(0.1f, 0.1f, 0.1f);
    cylinder({-0.24f, 0.47f, -0.30f}, {-0.15f, 0.47f, -0.30f}, 0.025f);
    cylinder({0.15f, 0.47f, -0.30f}, {0.24f, 0.47f, -0.30f}, 0.025f);
    color(0.3f, 0.31f, 0.34f);
    box({-0.2f, 0.05f, -0.25f}, {0.07f, 0.01f, 0.1f});    // footrests
    box({0.2f, 0.05f, -0.25f}, {0.07f, 0.01f, 0.1f});

    // The acceleration sent, in the vehicle's frame, from where the rider's chest would be:
    // 2 cm per m/s^2.
    const V3 accel = {p.sway, p.heave, -p.surge};
    const V3 origin = {0, 0.6f, 0.05f};
    color(0.3f, 0.85f, 0.95f);
    arrow(origin, origin + accel * 0.02f, 0.012f);
    glPopMatrix();

    // Where the platform is at rest.
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    color(1, 1, 1, 0.22f);
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 48; i++) {
        const V3 q = ring(kTopRadius + 0.08f, i * 7.5f, kTopHeight);
        glVertex3f(q.x, q.y, q.z);
    }
    glEnd();
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
}

// ---- The charts ---------------------------------------------------------------------------

struct Sample {
    double t;
    float v[9];
};

struct Channel {
    const char* name;
    const char* unit;
    float r, g, b;
    float min_span;   // the chart's scale never shrinks below this, either side of zero
    bool positive;    // 0 at the bottom (speed, revs) rather than in the middle
};

const Channel kChannels[9] = {
    {"speed", "km/h", 0.85f, 0.85f, 0.88f, 50, true},
    {"rpm", "", 0.85f, 0.85f, 0.88f, 1000, true},
    {"pitch", "deg", 1.0f, 0.62f, 0.25f, 10, false},
    {"roll", "deg", 1.0f, 0.62f, 0.25f, 10, false},
    {"yaw", "deg", 1.0f, 0.62f, 0.25f, 10, false},
    {"surge", "m/s2", 0.35f, 0.85f, 0.95f, 5, false},
    {"sway", "m/s2", 0.35f, 0.85f, 0.95f, 5, false},
    {"heave", "m/s2", 0.35f, 0.85f, 0.95f, 5, false},
    {"slide", "m/s", 0.7f, 0.9f, 0.5f, 2, false},
};

float channel_value(const Packet& p, int c) {
    switch (c) {
        case 0: return p.speed_kmh;
        case 1: return p.rpm;
        case 2: return p.pitch;
        case 3: return p.roll;
        case 4: return p.yaw;
        case 5: return p.surge;
        case 6: return p.sway;
        case 7: return p.heave;
        default: return p.sway_velocity;
    }
}

float g_text_scale = 2;

void text(float x, float y, const char* s, float r = 0.9f, float g = 0.9f, float b = 0.92f) {
    static char buf[60000];
    const int quads = stb_easy_font_print(0, 0, (char*)s, nullptr, buf, sizeof(buf));
    glPushMatrix();
    glTranslatef(x, y, 0);
    glScalef(g_text_scale, g_text_scale, 1);
    color(r, g, b);
    glEnableClientState(GL_VERTEX_ARRAY);
    glVertexPointer(2, GL_FLOAT, 16, buf);
    glDrawArrays(GL_QUADS, 0, quads * 4);
    glDisableClientState(GL_VERTEX_ARRAY);
    glPopMatrix();
}

float text_width(const char* s) { return stb_easy_font_width((char*)s) * g_text_scale; }

void rect(float x, float y, float w, float h) {
    glBegin(GL_QUADS);
    glVertex2f(x, y); glVertex2f(x + w, y); glVertex2f(x + w, y + h); glVertex2f(x, y + h);
    glEnd();
}

void draw_charts(const std::deque<Sample>& history, double now, const Options& o, float x0, float y0,
                 float w, float h) {
    const float gap = 6 * g_text_scale / 2, chart_h = (h - gap * 8) / 9;
    char label[96];
    for (int c = 0; c < 9; c++) {
        const Channel& ch = kChannels[c];
        const float y = y0 + c * (chart_h + gap);
        float lo = 0, hi = 0;
        for (const Sample& s : history) lo = std::min(lo, s.v[c]), hi = std::max(hi, s.v[c]);
        float bottom, top;
        if (ch.positive) {
            bottom = 0;
            top = std::max(ch.min_span, hi * 1.1f);
        } else {
            const float span = std::max(ch.min_span, std::max(-lo, hi) * 1.1f);
            bottom = -span, top = span;
        }
        auto ypos = [&](float v) { return y + chart_h - (v - bottom) / (top - bottom) * chart_h; };

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        color(1, 1, 1, 0.05f);
        rect(x0, y, w, chart_h);
        if (!ch.positive) {
            color(1, 1, 1, 0.18f);
            glBegin(GL_LINES);
            glVertex2f(x0, ypos(0)); glVertex2f(x0 + w, ypos(0));
            glEnd();
        }
        glDisable(GL_BLEND);

        color(ch.r, ch.g, ch.b);
        glBegin(GL_LINE_STRIP);
        for (const Sample& s : history) {
            const float t = (float)((s.t - (now - o.history_s)) / o.history_s);
            glVertex2f(x0 + t * w, ypos(s.v[c]));
        }
        glEnd();

        const float value = history.empty() ? 0 : history.back().v[c];
        snprintf(label, sizeof(label), "%s %s", ch.name, ch.unit);
        text(x0 + 6, y + 5, label, ch.r, ch.g, ch.b);
        snprintf(label, sizeof(label), c == 1 ? "%.0f" : "%+.2f", value);
        text(x0 + w - 6 - text_width(label), y + 5, label);
        snprintf(label, sizeof(label), ch.positive ? "0..%.0f" : "+-%.0f", top);
        text(x0 + 6, y + chart_h - 7 * g_text_scale, label, 0.5f, 0.5f, 0.55f);
    }
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) return 1;

    Udp in, out;
    if (!in.listen(o.port)) return 1;
    if (!o.forward.empty() && !out.aim(o.forward)) return 1;
    printf("listening for SimRacingStudio API packets on UDP %d", o.port);
    if (!o.forward.empty()) printf(", forwarding to %s", o.forward.c_str());
    printf("\n");

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    SDL_Window* window = SDL_CreateWindow("Telemetry Visualizer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                          1400, 860, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window) {
        // No multisampling to be had: try again without.
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
        window = SDL_CreateWindow("Telemetry Visualizer", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1400, 860,
                                  SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    }
    if (!window) {
        fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GLContext gl = SDL_GL_CreateContext(window);
    if (!gl) {
        fprintf(stderr, "SDL_GL_CreateContext: %s\n", SDL_GetError());
        return 1;
    }
    SDL_GL_SetSwapInterval(1);

    Packet packet;
    std::deque<Sample> history;
    std::deque<double> arrivals;   // the last second's, for the rate
    double last_packet = -1;
    long bad_packets = 0;
    bool frozen = false, dragging = false;
    Camera cam;
    const Uint64 freq = SDL_GetPerformanceFrequency(), start = SDL_GetPerformanceCounter();
    auto clock = [&] { return (double)(SDL_GetPerformanceCounter() - start) / freq; };
    double frozen_at = 0;

    for (bool running = true; running;) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            switch (e.type) {
                case SDL_QUIT: running = false; break;
                case SDL_KEYDOWN:
                    if (e.key.keysym.sym == SDLK_ESCAPE) running = false;
                    else if (e.key.keysym.sym == SDLK_r) cam = Camera();
                    else if (e.key.keysym.sym == SDLK_SPACE) frozen = !frozen, frozen_at = clock();
                    break;
                case SDL_MOUSEBUTTONDOWN: if (e.button.button == SDL_BUTTON_LEFT) dragging = true; break;
                case SDL_MOUSEBUTTONUP: if (e.button.button == SDL_BUTTON_LEFT) dragging = false; break;
                case SDL_MOUSEMOTION:
                    if (dragging) {
                        cam.yaw -= e.motion.xrel * 0.4f;
                        cam.pitch = std::clamp(cam.pitch + e.motion.yrel * 0.4f, -10.0f, 89.0f);
                    }
                    break;
                case SDL_MOUSEWHEEL:
                    cam.distance = std::clamp(cam.distance * powf(0.9f, (float)e.wheel.y), 1.0f, 12.0f);
                    break;
            }
        }

        const double now = clock();
        uint8_t buf[2048];
        for (long n; (n = in.receive(buf, sizeof(buf))) >= 0;) {
            out.send(buf, (size_t)n);
            Packet p;
            if (!decode(buf, (size_t)n, p)) {
                bad_packets++;
                continue;
            }
            last_packet = now;
            arrivals.push_back(now);
            if (frozen) continue;
            packet = p;
            Sample s;
            s.t = now;
            for (int c = 0; c < 9; c++) s.v[c] = channel_value(p, c);
            history.push_back(s);
        }
        while (!arrivals.empty() && arrivals.front() < now - 1) arrivals.pop_front();
        const double shown = frozen ? frozen_at : now;
        while (!history.empty() && history.front().t < shown - o.history_s) history.pop_front();

        int ww, wh, dw, dh;
        SDL_GetWindowSize(window, &ww, &wh);
        SDL_GL_GetDrawableSize(window, &dw, &dh);
        const float dpi = ww > 0 ? (float)dw / ww : 1;
        g_text_scale = 2 * dpi;

        glClearColor(0.07f, 0.075f, 0.09f, 1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const int panel = std::min(dw / 2, (int)(460 * dpi));
        draw_rig(packet, o, cam, dw - panel, dh);

        // The overlay, in drawable pixels with y down.
        glViewport(0, 0, dw, dh);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, dw, dh, 0, -1, 1);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glDisable(GL_LIGHTING);

        const float pad = 10 * dpi;
        draw_charts(history, shown, o, (float)(dw - panel) + pad, pad, panel - 2 * pad, dh - 2 * pad);

        char line[256];
        float y = pad;
        const float line_h = 9 * g_text_scale;
        snprintf(line, sizeof(line), "UDP %d%s%s", o.port, o.forward.empty() ? "" : " -> ", o.forward.c_str());
        text(pad, y, line), y += line_h;
        if (last_packet < 0) {
            text(pad, y, "waiting for packets", 0.95f, 0.75f, 0.3f), y += line_h;
        } else if (now - last_packet > 0.5) {
            snprintf(line, sizeof(line), "no packets for %.0f s", now - last_packet);
            text(pad, y, line, 0.95f, 0.75f, 0.3f), y += line_h;
        } else {
            snprintf(line, sizeof(line), "%s  %zu packets/s", packet.game.c_str(), arrivals.size());
            text(pad, y, line, 0.5f, 0.9f, 0.5f), y += line_h;
        }
        if (bad_packets) {
            snprintf(line, sizeof(line), "%ld packets not v102 \"api\"", bad_packets);
            text(pad, y, line, 0.95f, 0.4f, 0.35f), y += line_h;
        }
        if (frozen) text(pad, y, "FROZEN (space)", 0.95f, 0.75f, 0.3f), y += line_h;
        snprintf(line, sizeof(line), "angles x%.2g   travel %.3g m per m/s2, at most %.2g m", o.angle_scale, o.travel,
                 o.max_travel);
        text(pad, dh - pad - 7 * g_text_scale, line, 0.5f, 0.5f, 0.55f);

        SDL_GL_SwapWindow(window);
    }

    SDL_GL_DeleteContext(gl);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
