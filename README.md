# Descent Into Madness

A drawing robot controlled by an ESP32-based sketch (`plotter.ino`). It renders vector drawings on a canvas and translates them to coordinated motor actions for a pair of stepper-driven axes and a pen mechanism.

## Features
- Dual-axis plotter (X and Y) using stepper motors
- Pen via a servo with adjustable pen-down angle
- Web-based user interface served by the ESP32
- Real-time drawing canvas with live preview and calibration tools
- WebSocket-based bidirectional control for remote input
- Simple teleoperation/joystick-like controls and keyboard shortcuts
- Configurable logging for canvas activity, motor commands, and general info
- Calibration workflow: set Top-Left and Bottom-Right bounds, then move between them
- Lightweight command protocol with a range of system controls (enable/disable motors and logs)

## Build and Run
This sketch targets an ESP32 board. It relies on several Arduino libraries.

### Prerequisites
- Arduino IDE or VSCode + PlatformIO
- ESP32 board core installed (via Board Manager; add ESP32 URL if needed)
- Libraries (via Library Manager or equivalent):
  - `AccelStepper` by Mike McCauley
  - `ESPAsyncWebServer`
  - `ESPAsyncTCP` or `AsyncTCP` (ESP32 compatible)
  - `AsyncTCP` (or the ESP32 equivalent)
  - `ESP32Servo`
  - `WiFi` (built into ESP32 core)

### Setup (Arduino IDE)
1. Install the ESP32 board core:
   - Preferences → Additional Board Manager URLs: add
     `https://dl.espressif.com/dl/package_esp32_index.json`
2. Tools → Board → ESP32 Dev Module (or your preferred ESP32 variant)
3. Install libraries via Sketch → Include Library → Library Manager:
   - `AccelStepper`
   - `ESPAsyncWebServer`
   - `ESPAsyncTCP` (if using the ESP32 variant that requires it)
4. Open `plotter.ino` and compile/upload to the ESP32.

### Wiring Overview
- X axis: step pin `X_STEP_PIN` = 32, dir pin `X_DIR_PIN` = 33
- Y axis: step pin `Y_STEP_PIN` = 25, dir pin `Y_DIR_PIN` = 26
- X limit switch: pin 23
- Y limit switch: pin 22
- Pen servo: pin 27
- Power and signal wiring should match the board's voltage/current capabilities and the stepper drivers in use.

### Initial Configuration
- WiFi network (in code): SSID `MAKERSPACE`, password `12345678`
- The device hosts a web UI at the root path and a WebSocket endpoint at `/ws`.
- After upload, monitor the serial console to view IP address and status.

### Web UI Overview
- Root URL serves a page with a drawing canvas and controls:
  - Clear, Submit, Settings, Telephone
- Settings panel includes:
  - Color, Stroke Width, Pen Angle
- Telephony/game mode (experimental) for interactive input and automated replay
- Calibration UI to set Top-Left and Bottom-Right bounds and to move to those bounds

### Commands and Interaction
- WebSocket path: `/ws`
- Serial/WebSocket commands (examples):
  - `cmd:status` prints device status
  - `cmd:enable_motors`, `cmd:disable_motors`
  - `cmd:enable_logs_all`, `cmd:disable_logs_all`
  - `cmd:get_pos` returns current X/Y positions
  - `cmd:pen_down`, `cmd:pen_up`, `cmd:pen_angle:<deg>`
- Canvas input maps to motor coordinates; calibration defines the drawing area.

## Calibration and Drawing Workflow
- Use TL/BR controls to set the working area by moving the axes to the respective corners.
- During drawing, coordinates from the canvas are translated to motor steps within the calibrated bounds.
- Pen up/down is controlled via the servo; ensure the servo can safely reach the required angles.

## Local Development and Testing
- Connect through the ESP32's IP address discovered in the serial console.
- Use the browser UI to draw, clear, and submit drawings.
- Use the “Settings” panel to adjust stroke characteristics and calibrate the plotting area.

## Troubleshooting
- If the UI does not load, verify WiFi credentials and board power.
- If the servo does not move, check the servo power supply and that the pin is correctly defined.
- If the motors do not move as expected, verify motor driver wiring and limit switch wiring.
- If the WebSocket connection drops, ensure the ESP32 network is stable and re-load the page.

## About the Name
- Descent Into Madness is the nickname for this drawing robot project. It emphasizes creative, perhaps chaotic, exploration of generative art through a hardware artist's toolchain.

