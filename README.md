# Telemetry Visualizer

Shows what a game's motion telemetry would do to a motion rig. It listens for
SimRacingStudio "api" packets (version 102), which is what the recomp projects'
`--telemetry` option sends. It then moves a six-actuator motion platform in a 3D view and
charts every channel over the last ten seconds.

## Build

Needs CMake and a C++17 compiler. SDL2 comes from the system if it is installed (Homebrew,
the SDL2 framework, vcpkg); if not, CMake downloads and builds it.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Use

```sh
build/telemetry-visualizer                 # listens on UDP 33001, SimRacingStudio's port
build/telemetry-visualizer --port 33011    # or anywhere else
```

Then run the game with `--telemetry` (or `--telemetry=127.0.0.1:33011`).

Only one program can listen on a port. If SimRacingStudio is running, it already holds
33001. To watch a real rig and the visualizer together, point the game at the visualizer
and have it pass every packet on:

```sh
build/telemetry-visualizer --port 33011 --forward 127.0.0.1:33001
```

`tools/send_test.py [host:port]` sends a made-up test pattern, so you can try the
visualizer without a game.

Drag to orbit the view, scroll to zoom, press **R** to reset the view, **Space** to freeze,
and **Esc** to quit.

## What the platform shows

The charts show the numbers exactly as they arrive. The platform's movement is an
illustration, not SimRacingStudio's mapping:

- **Angles:** pitch, roll and yaw are the platform's own angles in degrees, multiplied by
  `--angle-scale` (default 1).
- **Accelerations:** surge, sway and heave move the platform forward, right and up by
  `--travel` metres per m/s² (default 0.004), at most `--max-travel` metres (default 0.12).
- **Actuator colours:** each actuator turns red as it extends past its rest length and
  blue as it shortens.
- **Rest position:** the faint ring marks where the top of the platform sits at rest.
- **Acceleration arrow:** the cyan arrow is the acceleration sent, in the vehicle's frame.
- **Front:** the orange tab marks the front of the platform.

Sign conventions are the packet's: nose up, right side down and a turn to the right are all
positive. Forward, right and up are positive for the accelerations.
