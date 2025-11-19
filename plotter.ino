#include <AccelStepper.h>
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <cstring>
#include <ezButton.h>

// Makerspace Wifi Credentials
const char *ssid = "MAKERSPACE";
const char *password = "12345678";
constexpr int WIFI_CONNECT_TIMEOUT_MS = 20000;

// Stepper pin assignments
constexpr int X_STEP_PIN = 32;
constexpr int X_DIR_PIN = 33;
constexpr int Y_STEP_PIN = 25;
constexpr int Y_DIR_PIN = 26;
constexpr int X_LIMIT_PIN = 23;
constexpr int Y_LIMIT_PIN = 22;

// Servo configuration
constexpr int SERVO_PIN = 27;
constexpr int PEN_UP_ANGLE = 90;
constexpr int PEN_DOWN_ANGLE = 45;

// Motion calibration (tune for your machine)
constexpr float STEPS_PER_PIXEL_X = 1.0f;
constexpr float STEPS_PER_PIXEL_Y = 1.0f;
constexpr int32_t X_OFFSET_STEPS = 0;
constexpr int32_t Y_OFFSET_STEPS = 0;
constexpr int32_t X_MIN_STEPS = 0;
constexpr int32_t X_MAX_STEPS = 1200;
constexpr int32_t Y_MIN_STEPS = 0;
constexpr int32_t Y_MAX_STEPS = 450;

// Motion dynamics
constexpr float MAX_SPEED_STEPS_PER_SEC = 500.0f;
constexpr float ACCEL_STEPS_PER_SEC2 = 250.0f;
constexpr int32_t MIN_STEP_DELTA = 1;
constexpr int32_t LIMIT_SWITCH_BUFFER_STEPS = 20;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

Servo penServo;
ezButton limitSwitchX(X_LIMIT_PIN);
ezButton limitSwitchY(Y_LIMIT_PIN);
bool penCurrentlyDown = false;

portMUX_TYPE targetMux = portMUX_INITIALIZER_UNLOCKED;

struct PlotCmd {
  int32_t xSteps;
  int32_t ySteps;
  bool penDown;
};

constexpr size_t QUEUE_CAPACITY = 500;
PlotCmd cmdQueue[QUEUE_CAPACITY];
volatile size_t qHead = 0;
volatile size_t qTail = 0;

// Control flags
bool motorsEnabled = true;
bool enableLogCanvasOutput = true;
bool enableLogMotorCommands = true;
bool enableLogGeneral = true;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
  <title>Plotter</title>
  <style>
    * { box-sizing: border-box; }
    html, body { margin: 0; height: 100%; background: #111; color: #f5f5f5; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; }
    #toolbar { position: fixed; top: 0; left: 0; right: 0; display: flex; gap: 12px; align-items: center; padding: 10px 16px; background: rgba(0, 0, 0, 0.35); backdrop-filter: blur(12px); z-index: 10; }
    button { background: #1f1f1f; border: 1px solid #333; border-radius: 8px; color: inherit; font-size: 16px; padding: 8px 14px; cursor: pointer; }
    button:active { background: #2e2e2e; }
    button:hover { background: #2a2a2a; }
    button:disabled { opacity: 0.5; cursor: not-allowed; }
    #status { margin-left: auto; font-size: 14px; opacity: 0.85; }
    #canvas { display: block; width: 100vw; height: 100vh; cursor: crosshair; touch-action: none; }
    #settingsPanel { position: fixed; top: 60px; right: 16px; background: rgba(0, 0, 0, 0.9); backdrop-filter: blur(12px); border: 1px solid #333; border-radius: 12px; padding: 20px; min-width: 240px; z-index: 20; display: none; box-shadow: 0 8px 32px rgba(0, 0, 0, 0.5); }
    #settingsPanel.open { display: block; }
    .settings-group { margin-bottom: 20px; }
    .settings-group:last-child { margin-bottom: 0; }
    .settings-label { display: block; font-size: 13px; margin-bottom: 8px; opacity: 0.9; font-weight: 500; }
    #colorPicker { width: 100%; height: 40px; border: 1px solid #444; border-radius: 8px; cursor: pointer; background: transparent; }
    #strokeWidthInput { width: 100%; height: 36px; background: #1f1f1f; border: 1px solid #444; border-radius: 8px; color: inherit; font-size: 14px; padding: 0 12px; }
    #strokeWidthInput:focus { outline: none; border-color: #666; }
    .stroke-width-display { display: flex; align-items: center; gap: 10px; }
    #strokeWidthSlider { flex: 1; height: 6px; background: #2a2a2a; border-radius: 3px; outline: none; -webkit-appearance: none; }
    #strokeWidthSlider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 16px; height: 16px; background: #ff5252; border-radius: 50%; cursor: pointer; }
    #strokeWidthSlider::-moz-range-thumb { width: 16px; height: 16px; background: #ff5252; border-radius: 50%; cursor: pointer; border: none; }
    #strokeWidthValue { min-width: 40px; text-align: right; font-size: 13px; opacity: 0.8; }
    .control-pad { display: flex; flex-direction: column; align-items: center; gap: 4px; margin-top: 10px; }
    .control-pad div { display: flex; gap: 4px; }
    .control-pad button { width: 40px; height: 40px; padding: 0; display: flex; align-items: center; justify-content: center; font-size: 20px; background: #2a2a2a; }
    .control-pad button:active { background: #ff5252; border-color: #ff5252; }
    .instruction { text-align: center; font-size: 12px; opacity: 0.6; margin-top: 8px; }
  </style>
</head>
<body>
  <div id="toolbar">
    <button id="clearBtn">Clear</button>
    <button id="submitBtn">Submit</button>
    <button id="settingsBtn">Settings</button>
    <div id="status">Offline</div>
  </div>
  <div id="settingsPanel">
    <div class="settings-group">
      <label class="settings-label" for="colorPicker">Color</label>
      <input type="color" id="colorPicker" value="#ff5252">
    </div>
    <div class="settings-group">
      <label class="settings-label" for="strokeWidthSlider">Stroke Width</label>
      <div class="stroke-width-display">
        <input type="range" id="strokeWidthSlider" min="1" max="50" value="2" step="1">
        <span id="strokeWidthValue">2</span>
      </div>
      <input type="number" id="strokeWidthInput" min="1" max="50" value="2" step="1">
    </div>
    <div class="settings-group">
      <label class="settings-label">Manual Control</label>
      <div class="control-pad">
        <button id="jogYMinus">▲</button>
        <div>
          <button id="jogXMinus">◀</button>
          <button id="jogXPlus">▶</button>
        </div>
        <button id="jogYPlus">▼</button>
      </div>
      <div class="instruction">Use Arrow Keys</div>
      <label class="settings-label" style="margin-top: 10px;">Jog Steps</label>
      <select id="jogStepSize" style="width: 100%; height: 30px; background: #1f1f1f; border: 1px solid #444; color: inherit; border-radius: 4px;">
          <option value="1">1 Step</option>
          <option value="10">10 Steps</option>
          <option value="50">50 Steps</option>
          <option value="100">100 Steps</option>
      </select>
    </div>
  </div>
  <canvas id="canvas"></canvas>
  <script>
    const canvas = document.getElementById('canvas');
    const ctx = canvas.getContext('2d');
    const statusEl = document.getElementById('status');
    const clearBtn = document.getElementById('clearBtn');
    const submitBtn = document.getElementById('submitBtn');
    const settingsBtn = document.getElementById('settingsBtn');
    const settingsPanel = document.getElementById('settingsPanel');
    const colorPicker = document.getElementById('colorPicker');
    const strokeWidthSlider = document.getElementById('strokeWidthSlider');
    const strokeWidthInput = document.getElementById('strokeWidthInput');
    const strokeWidthValue = document.getElementById('strokeWidthValue');
    const jogStepSelect = document.getElementById('jogStepSize');
    const gateway = `${window.location.protocol === 'https:' ? 'wss' : 'ws'}://${window.location.host}/ws`;
    
    let websocket;
    let drawing = false;
    let lastPoint = { x: 0, y: 0 };
    let pointsBuffer = [];
    let isSending = false;
    let ackResolver = null;
    
    let currentDpr = 1;
    let currentColor = '#2365ea';
    let currentStrokeWidth = 2;

    // Plotter physical limits (in steps)
    const PLOTTER_MAX_X = 1200;
    const PLOTTER_MAX_Y = 450;

    function applyCanvasSettings() {
      ctx.lineWidth = currentStrokeWidth;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.strokeStyle = currentColor;
    }

    function resizeCanvas() {
      currentDpr = Math.max(1, window.devicePixelRatio || 1);
      
      const windowW = window.innerWidth;
      const windowH = window.innerHeight;
      
      const plotterAspect = PLOTTER_MAX_X / PLOTTER_MAX_Y;
      const windowAspect = windowW / windowH;
      
      let w, h;
      if (windowAspect > plotterAspect) {
        // Window is wider -> limit by height
        h = windowH;
        w = h * plotterAspect;
      } else {
        // Window is taller -> limit by width
        w = windowW;
        h = w / plotterAspect;
      }

      canvas.style.width = w + 'px';
      canvas.style.height = h + 'px';
      canvas.style.position = 'absolute';
      canvas.style.left = (windowW - w) / 2 + 'px';
      canvas.style.top = (windowH - h) / 2 + 'px';

      canvas.width = Math.round(w * currentDpr);
      canvas.height = Math.round(h * currentDpr);
      
      ctx.setTransform(currentDpr, 0, 0, currentDpr, 0, 0);
      applyCanvasSettings();
      
      ctx.fillStyle = '#111';
      ctx.fillRect(0, 0, w, h);
      
      // Draw bounds
      ctx.save();
      ctx.strokeStyle = '#333';
      ctx.lineWidth = 2;
      ctx.strokeRect(0, 0, w, h);
      ctx.restore();
    }

    function updateStatus(text, ok = false) {
      statusEl.textContent = text;
      statusEl.style.color = ok ? '#8dff6a' : '#ff8d72';
    }

    function initWebSocket() {
      updateStatus('Connecting…', false);
      websocket = new WebSocket(gateway);
      websocket.onopen = () => updateStatus('Online', true);
      websocket.onclose = () => {
        updateStatus('Offline', false);
        setTimeout(initWebSocket, 1500);
      };
      websocket.onerror = () => websocket.close();
      
      websocket.onmessage = (event) => {
        if (event.data === "ACK") {
          if (ackResolver) {
            ackResolver();
            ackResolver = null;
          }
        }
      };
    }

    function recordPoint(x, y, pen) {
      const now = performance.now();
      if (pointsBuffer.length > 0) {
        const last = pointsBuffer[pointsBuffer.length - 1];
        if (pen === last.pen &&
            Math.abs(x - last.x) < 1 &&
            Math.abs(y - last.y) < 1 &&
            now - last.time < 10) {
          return;
        }
      }
      pointsBuffer.push({ x, y, pen, time: now });
    }

    function sendSinglePoint(pt) {
      if (!websocket || websocket.readyState !== WebSocket.OPEN) return;
      
      // Map canvas coordinates to plotter steps
      // We map the entire canvas width/height to the plotter's physical limits
      // This stretches/squashes the drawing to fit the plotter area.
      // Alternatively, we could maintain aspect ratio, but let's fill for now.
      
      // Canvas dimensions (logical pixels)
      const w = canvas.width / currentDpr;
      const h = canvas.height / currentDpr;
      
      // Simple linear mapping:
      // x_steps = (pt.x / canvas_width) * PLOTTER_MAX_X
      // y_steps = ((h - pt.y) / canvas_height) * PLOTTER_MAX_Y  <-- Inverted Y for bottom-left origin
      
      const targetX = Math.round((pt.x / w) * PLOTTER_MAX_X);
      // Invert Y: Canvas (0 at top) -> Plotter (Max at top)
      const targetY = Math.round(((h - pt.y) / h) * PLOTTER_MAX_Y);

      websocket.send(`${targetX}&${targetY}-${pt.pen}`);
    }
    
    async function submitDrawing() {
        if (isSending) return;
        if (pointsBuffer.length === 0) {
            updateStatus('Nothing to send', false);
            return;
        }
        
        isSending = true;
        submitBtn.disabled = true;
        clearBtn.disabled = true;
        updateStatus('Sending...', true);
        
        try {
            for (let pt of pointsBuffer) {
                sendSinglePoint(pt);
                await new Promise(resolve => {
                    ackResolver = resolve;
                    // Timeout fallback just in case
                    setTimeout(resolve, 2000);
                });
            }
            updateStatus('Sent!', true);
        } catch (e) {
            updateStatus('Error sending', false);
            console.error(e);
        } finally {
            pointsBuffer = []; // Clear buffer after successful send
            isSending = false;
            submitBtn.disabled = false;
            clearBtn.disabled = false;
            // Optional: Clear canvas here if desired? Probably not, user might want to see what they sent.
        }
    }

    function getCanvasPoint(evt) {
      const rect = canvas.getBoundingClientRect();
      const source = evt.touches ? evt.touches[0] : evt;
      return {
        x: source.clientX - rect.left,
        y: source.clientY - rect.top
      };
    }

    function startDraw(evt) {
      if (isSending) return;
      evt.preventDefault();
      drawing = true;
      lastPoint = getCanvasPoint(evt);
      applyCanvasSettings();
      ctx.beginPath();
      ctx.moveTo(lastPoint.x, lastPoint.y);
      recordPoint(lastPoint.x, lastPoint.y, 1);
    }

    function moveDraw(evt) {
      if (!drawing || isSending) return;
      evt.preventDefault();
      const point = getCanvasPoint(evt);
      ctx.lineTo(point.x, point.y);
      ctx.stroke();
      lastPoint = point;
      recordPoint(point.x, point.y, 1);
    }

    function endDraw(evt) {
      if (!drawing || isSending) return;
      evt.preventDefault();
      drawing = false;
      recordPoint(lastPoint.x, lastPoint.y, 0);
    }

    clearBtn.addEventListener('click', () => {
      if (isSending) return;
      
      const w = canvas.width / currentDpr;
      const h = canvas.height / currentDpr;

      ctx.fillStyle = '#111';
      ctx.fillRect(0, 0, w, h);
      
      ctx.save();
      ctx.strokeStyle = '#333';
      ctx.lineWidth = 2;
      ctx.strokeRect(0, 0, w, h);
      ctx.restore();

      ctx.beginPath();
      pointsBuffer = [];
    });
    
    submitBtn.addEventListener('click', submitDrawing);

    settingsBtn.addEventListener('click', () => {
      settingsPanel.classList.toggle('open');
    });

    colorPicker.addEventListener('input', (e) => {
      currentColor = e.target.value;
      applyCanvasSettings();
    });

    function updateStrokeWidth(value) {
      const numValue = Math.max(1, Math.min(50, parseInt(value) || 2));
      currentStrokeWidth = numValue;
      strokeWidthSlider.value = numValue;
      strokeWidthInput.value = numValue;
      strokeWidthValue.textContent = numValue;
      applyCanvasSettings();
    }

    strokeWidthSlider.addEventListener('input', (e) => {
      updateStrokeWidth(e.target.value);
    });

    strokeWidthInput.addEventListener('input', (e) => {
      updateStrokeWidth(e.target.value);
    });

    strokeWidthInput.addEventListener('blur', (e) => {
      updateStrokeWidth(e.target.value);
    });

    // Close settings panel when clicking outside
    document.addEventListener('click', (e) => {
      if (!settingsPanel.contains(e.target) && e.target !== settingsBtn) {
        settingsPanel.classList.remove('open');
      }
    });

    canvas.addEventListener('pointerdown', startDraw);
    canvas.addEventListener('pointermove', moveDraw);
    window.addEventListener('pointerup', endDraw);
    canvas.addEventListener('touchstart', startDraw, { passive: false });
    canvas.addEventListener('touchmove', moveDraw, { passive: false });
    window.addEventListener('touchend', endDraw, { passive: false });

    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();
    function getJogStep() {
      return parseInt(jogStepSelect.value) || 1;
    }

    function sendJog(dx, dy) {
      if (!websocket || websocket.readyState !== WebSocket.OPEN) return;
      const step = getJogStep();
      websocket.send(`cmd:jog:${dx * step},${dy * step}`);
    }

    document.getElementById('jogYMinus').addEventListener('click', () => sendJog(0, 1));
    document.getElementById('jogYPlus').addEventListener('click', () => sendJog(0, -1));
    document.getElementById('jogXMinus').addEventListener('click', () => sendJog(-1, 0));
    document.getElementById('jogXPlus').addEventListener('click', () => sendJog(1, 0));

    window.addEventListener('keydown', (e) => {
      if (!settingsPanel.classList.contains('open')) return;

      switch(e.key) {
        case 'ArrowUp': sendJog(0, 1); break;
        case 'ArrowDown': sendJog(0, -1); break;
        case 'ArrowLeft': sendJog(-1, 0); break;
        case 'ArrowRight': sendJog(1, 0); break;
        default: return;
      }
      e.preventDefault();
    });

    initWebSocket();
  </script>
</body>
</html>
)rawliteral";

template <typename T> T clampValue(T value, T minValue, T maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

int32_t roundToSteps(float value) {
  return static_cast<int32_t>((value >= 0.0f) ? (value + 0.5f)
                                              : (value - 0.5f));
}

int32_t pixelsToStepsX(int32_t pixels) {
  float base = pixels * STEPS_PER_PIXEL_X;
  int32_t steps = roundToSteps(base) + X_OFFSET_STEPS;
  if (steps < X_MIN_STEPS) {
    Serial.printf("[BOUNDS] X steps %ld exceeds minimum %ld (clamped)\n",
                  (long)steps, (long)X_MIN_STEPS);
  } else if (steps > X_MAX_STEPS) {
    Serial.printf("[BOUNDS] X steps %ld exceeds maximum %ld (clamped)\n",
                  (long)steps, (long)X_MAX_STEPS);
  }
  return clampValue<int32_t>(steps, X_MIN_STEPS, X_MAX_STEPS);
}

int32_t pixelsToStepsY(int32_t pixels) {
  float base = pixels * STEPS_PER_PIXEL_Y;
  int32_t steps = roundToSteps(base) + Y_OFFSET_STEPS;
  if (steps < Y_MIN_STEPS) {
    Serial.printf("[BOUNDS] Y steps %ld exceeds minimum %ld (clamped)\n",
                  (long)steps, (long)Y_MIN_STEPS);
  } else if (steps > Y_MAX_STEPS) {
    Serial.printf("[BOUNDS] Y steps %ld exceeds maximum %ld (clamped)\n",
                  (long)steps, (long)Y_MAX_STEPS);
  }
  return clampValue<int32_t>(steps, Y_MIN_STEPS, Y_MAX_STEPS);
}

void logCanvasOutput(int32_t xPixels, int32_t yPixels, bool penDown) {
  if (enableLogCanvasOutput) {
    Serial.printf("[CANVAS] x=%d y=%d pen=%s\n", xPixels, yPixels,
                  penDown ? "DOWN" : "UP");
  }
}

void logMotorCommands(int32_t xSteps, int32_t ySteps, bool penDown) {
  if (enableLogMotorCommands) {
    Serial.printf("[MOTOR] X=%d Y=%d pen=%s\n", xSteps, ySteps,
                  penDown ? "DOWN" : "UP");
  }
}

void logGeneral(const char *message) {
  if (enableLogGeneral) {
    Serial.printf("[INFO] %s\n", message);
  }
}

bool queueTargetSteps(int32_t xSteps, int32_t ySteps, bool penDown) {
  bool success = false;
  portENTER_CRITICAL(&targetMux);
  size_t nextHead = (qHead + 1) % QUEUE_CAPACITY;
  if (nextHead != qTail) {
    cmdQueue[qHead] = {xSteps, ySteps, penDown};
    qHead = nextHead;
    success = true;
  }
  portEXIT_CRITICAL(&targetMux);
  return success;
}

void handleWebSocketMessage(AsyncWebSocketClient *client, void *arg,
                            uint8_t *data, size_t len) {
  AwsFrameInfo *info = static_cast<AwsFrameInfo *>(arg);
  if (!(info->final && info->index == 0 && info->opcode == WS_TEXT)) {
    return;
  }

  if (len >= 128) {
    return; // ignore overly long messages
  }

  data[len] = 0;
  const char *payload = reinterpret_cast<char *>(data);

  // Delegate to shared payload processor (works for WebSocket and Serial
  // inputs) payload is null-terminated and within size limits Forward to the
  // common processor below. Note: keep this lightweight — heavy work happens in
  // the shared function. We'll implement processTextPayload(...) elsewhere in
  // this file.
  extern bool processTextPayload(const char *payload);
  if (processTextPayload(payload)) {
    // Send ACK for flow control (only if processed/enqueued)
    client->text("ACK");
  }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
  case WS_EVT_CONNECT: {
    char msg[64];
    snprintf(msg, sizeof(msg), "WebSocket client #%u connected", client->id());
    logGeneral(msg);
    break;
  }
  case WS_EVT_DISCONNECT: {
    char msg[64];
    snprintf(msg, sizeof(msg), "WebSocket client #%u disconnected",
             client->id());
    logGeneral(msg);
    break;
  }
  case WS_EVT_DATA:
    handleWebSocketMessage(client, arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

// Shared processor for text payloads coming from WebSocket or Serial.
// Accepts null-terminated payload strings (max ~127 chars).
bool processTextPayload(const char *payload) {
  if (!payload)
    return false;
  size_t len = strlen(payload);
  if (len == 0 || len >= 128) {
    return false;
  }

  // Check for command messages (start with "cmd:")
  if (strncmp(payload, "cmd:", 4) == 0) {
    const char *cmd = payload + 4;

    // Status command: print system metadata and commands
    if (strcmp(cmd, "status") == 0) {
      // implemented below as logStatus()
      extern void logStatus();
      logStatus();
      return true;
    }

    if (strcmp(cmd, "disable_motors") == 0) {
      motorsEnabled = false;
      logGeneral("Motors disabled");
      return true;
    } else if (strcmp(cmd, "enable_motors") == 0) {
      motorsEnabled = true;
      logGeneral("Motors enabled");
      return true;
    } else if (strcmp(cmd, "disable_logs_canvas") == 0) {
      enableLogCanvasOutput = false;
      logGeneral("Canvas logging disabled");
      return true;
    } else if (strcmp(cmd, "enable_logs_canvas") == 0) {
      enableLogCanvasOutput = true;
      logGeneral("Canvas logging enabled");
      return true;
    } else if (strcmp(cmd, "disable_logs_motor") == 0) {
      enableLogMotorCommands = false;
      logGeneral("Motor command logging disabled");
      return true;
    } else if (strcmp(cmd, "enable_logs_motor") == 0) {
      enableLogMotorCommands = true;
      logGeneral("Motor command logging enabled");
      return true;
    } else if (strcmp(cmd, "disable_logs_general") == 0) {
      enableLogGeneral = false;
      Serial.println("[INFO] General logging disabled");
      return true;
    } else if (strcmp(cmd, "enable_logs_general") == 0) {
      enableLogGeneral = true;
      Serial.println("[INFO] General logging enabled");
      return true;
    } else if (strcmp(cmd, "disable_logs_all") == 0) {
      enableLogCanvasOutput = false;
      enableLogMotorCommands = false;
      enableLogGeneral = false;
      Serial.println("[INFO] All logging disabled");
      return true;
    } else if (strcmp(cmd, "enable_logs_all") == 0) {
      enableLogCanvasOutput = true;
      enableLogMotorCommands = true;
      enableLogGeneral = true;
      Serial.println("[INFO] All logging enabled");
      return true;
    } else if (strncmp(cmd, "jog:", 4) == 0) {
      const char *args = cmd + 4;
      const char *comma = strchr(args, ',');
      if (comma) {
        int dx = atoi(args);
        int dy = atoi(comma + 1);

        int32_t targetX = stepperX.targetPosition() + dx;
        int32_t targetY = stepperY.targetPosition() + dy;

        // Clamp to bounds
        targetX = clampValue<int32_t>(targetX, X_MIN_STEPS, X_MAX_STEPS);
        targetY = clampValue<int32_t>(targetY, Y_MIN_STEPS, Y_MAX_STEPS);

        queueTargetSteps(targetX, targetY, penCurrentlyDown);
      }
      return true;
    }
    // Unknown command, ignore
    return false;
  }

  // Parse drawing coordinates (format: "x&y-pen")
  const char *amp = strchr(payload, '&');
  const char *dash = strchr(payload, '-');
  if (!amp || !dash || dash <= amp) {
    return false;
  }

  int32_t xPixels = atoi(payload);
  int32_t yPixels = atoi(amp + 1);
  int penValue = atoi(dash + 1);
  bool penDown = penValue > 0;

  // Log canvas output
  logCanvasOutput(xPixels, yPixels, penDown);

  // Direct mapping: inputs are already in steps from JS
  int32_t xSteps = xPixels;
  int32_t ySteps = yPixels;

  // Clamp just in case
  xSteps = clampValue<int32_t>(xSteps, X_MIN_STEPS, X_MAX_STEPS);
  ySteps = clampValue<int32_t>(ySteps, Y_MIN_STEPS, Y_MAX_STEPS);

  // Log motor commands
  logMotorCommands(xSteps, ySteps, penDown);

  return queueTargetSteps(xSteps, ySteps, penDown);
}

// Print a detailed status summary (metadata + available commands).
void logStatus() {
  Serial.println("=== PLOTTER STATUS ===");
  // Network
  Serial.printf("WiFi SSID: %s\n", ssid ? ssid : "(none)");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // Motor & motion
  Serial.printf("Motors enabled: %s\n", motorsEnabled ? "YES" : "NO");
  Serial.printf("Steps per pixel: X=%.2f Y=%.2f\n", STEPS_PER_PIXEL_X,
                STEPS_PER_PIXEL_Y);
  Serial.printf("X offset: %ld, Y offset: %ld\n", (long)X_OFFSET_STEPS,
                (long)Y_OFFSET_STEPS);
  Serial.printf("X range: [%ld, %ld], Y range: [%ld, %ld]\n", (long)X_MIN_STEPS,
                (long)X_MAX_STEPS, (long)Y_MIN_STEPS, (long)Y_MAX_STEPS);
  Serial.printf("Max speed (steps/s): %.2f, Accel (steps/s^2): %.2f\n",
                MAX_SPEED_STEPS_PER_SEC, ACCEL_STEPS_PER_SEC2);

  // Servo
  Serial.printf("Servo pin: %d, pen up angle: %d, pen down angle: %d\n",
                SERVO_PIN, PEN_UP_ANGLE, PEN_DOWN_ANGLE);

  // Logging flags
  Serial.printf("Logging - canvas: %s, motor: %s, general: %s\n",
                enableLogCanvasOutput ? "ON" : "OFF",
                enableLogMotorCommands ? "ON" : "OFF",
                enableLogGeneral ? "ON" : "OFF");

  // Available commands
  Serial.println("Available serial/websocket commands:");
  Serial.println("  cmd:status");
  Serial.println("  cmd:disable_motors");
  Serial.println("  cmd:enable_motors");
  Serial.println("  cmd:disable_logs_canvas");
  Serial.println("  cmd:enable_logs_canvas");
  Serial.println("  cmd:disable_logs_motor");
  Serial.println("  cmd:enable_logs_motor");
  Serial.println("  cmd:disable_logs_general");
  Serial.println("  cmd:enable_logs_general");
  Serial.println("  cmd:disable_logs_all");
  Serial.println("  cmd:enable_logs_all");
  Serial.println("");
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  Serial.printf("Connecting to WiFi \"%s\" ", ssid);
  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
    if (millis() - startAttempt > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println("\nWiFi connection timeout, restarting...");
      ESP.restart();
    }
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void configureSteppers() {
  // Invert X axis direction to match physical setup (Positive moves away from switch)
  stepperX.setPinsInverted(true, false, false);
  // Invert Y axis direction so Positive moves UP (away from switch at bottom)
  stepperY.setPinsInverted(true, false, false);

  // Speed and acceleration are in steps per second
  stepperX.setMaxSpeed(MAX_SPEED_STEPS_PER_SEC);
  stepperX.setAcceleration(ACCEL_STEPS_PER_SEC2);
  stepperY.setMaxSpeed(MAX_SPEED_STEPS_PER_SEC);
  stepperY.setAcceleration(ACCEL_STEPS_PER_SEC2);
}

void updatePen(bool penDown) {
  const int targetAngle = penDown ? PEN_DOWN_ANGLE : PEN_UP_ANGLE;
  penServo.write(targetAngle);
  penCurrentlyDown = penDown;
}

int32_t absDiff(int32_t a, int32_t b) { return (a > b) ? (a - b) : (b - a); }

void applyQueuedTarget() {
  if (stepperX.distanceToGo() != 0 || stepperY.distanceToGo() != 0) {
    return;
  }

  bool hasTarget = false;
  PlotCmd cmd;

  portENTER_CRITICAL(&targetMux);
  if (qHead != qTail) {
    hasTarget = true;
    cmd = cmdQueue[qTail];
    qTail = (qTail + 1) % QUEUE_CAPACITY;
  }
  portEXIT_CRITICAL(&targetMux);

  if (!hasTarget) {
    return;
  }

  int32_t targetX = cmd.xSteps;
  int32_t targetY = cmd.ySteps;
  bool targetPen = cmd.penDown;

  updatePen(targetPen);

  const int32_t currentX = stepperX.targetPosition();
  const int32_t currentY = stepperY.targetPosition();

  const bool needXMove = (absDiff(targetX, currentX) >= MIN_STEP_DELTA);
  const bool needYMove = (absDiff(targetY, currentY) >= MIN_STEP_DELTA);

  if (needXMove && motorsEnabled)
    stepperX.moveTo(targetX);
  if (needYMove && motorsEnabled)
    stepperY.moveTo(targetY);
}

void homeAxis(AccelStepper &stepper, ezButton &limitSwitch, int direction) {
  float originalMaxSpeed = stepper.maxSpeed();
  stepper.setMaxSpeed(100.0); // Slow speed for homing

  // Move indefinitely in the homing direction
  stepper.moveTo(direction * 100000);

  while (true) {
    limitSwitch.loop();
    if (limitSwitch.getState() == LOW) { // Touched
      stepper.stop();
      break;
    }
    stepper.run();
    if (stepper.distanceToGo() == 0) {
      break;
    }
  }

  // Back off from the limit switch
  stepper.setCurrentPosition(0); // Temporarily set 0 at switch
  stepper.move(-direction * LIMIT_SWITCH_BUFFER_STEPS); // Move away
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Set the new 0 position (home)
  stepper.setCurrentPosition(0);

  stepper.setMaxSpeed(originalMaxSpeed);
  stepper.moveTo(0);
}

void measureAxisLength(AccelStepper &stepper, ezButton &limitSwitch,
                       int homeDir, const char *axisName) {
  Serial.printf("Measuring %s axis length...\n", axisName);

  // 1. Home to the starting bound
  homeAxis(stepper, limitSwitch, homeDir);
  delay(500);

  // 2. Move away to clear the switch
  Serial.println("Backing off from home...");
  long backoffSteps = -homeDir * 100; // Move 100 steps away
  stepper.move(backoffSteps);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    limitSwitch.loop(); // Keep button updated
  }

  if (limitSwitch.getState() == LOW) {
    Serial.println("Warning: Switch still triggered after backoff!");
  }

  // 3. Move to the other bound
  Serial.println("Moving to opposite bound...");
  float originalMaxSpeed = stepper.maxSpeed();
  stepper.setMaxSpeed(200.0); // Slower for accuracy

  // Move effectively forever in opposite direction
  stepper.moveTo(-homeDir * 1000000);

  while (true) {
    stepper.run();

    // Print position every 10 steps to avoid flooding serial too much,
    // but frequently enough to capture the last position.
    if (abs(stepper.currentPosition()) % 10 == 0) {
      Serial.printf("Current Position: %ld\n", stepper.currentPosition());
    }

    if (stepper.distanceToGo() == 0) {
      Serial.println("Error: Reached max travel limit (virtual)");
      break;
    }
  }

  long totalSteps = abs(stepper.currentPosition());
  Serial.printf(">>> %s AXIS TOTAL STEPS: %ld <<<\n", axisName, totalSteps);

  // 4. Return to home
  stepper.setMaxSpeed(originalMaxSpeed);
  stepper.moveTo(0);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  Serial.println("Returned to home.");
}

void setup() {
  Serial.begin(115200);
  penServo.attach(SERVO_PIN);
  updatePen(false);
  Serial.println("Servo initialized - starting test cycle");

  configureSteppers();

  limitSwitchX.setDebounceTime(50);
  limitSwitchY.setDebounceTime(50);

  Serial.println("Homing X axis...");
  homeAxis(stepperX, limitSwitchX, -1);
  Serial.println("X axis homed");

  Serial.println("Homing Y axis...");
  homeAxis(stepperY, limitSwitchY, -1);
  Serial.println("Y axis homed");

  // // Calibrate axes (measures total steps)
  // measureAxisLength(stepperX, limitSwitchX, 1, "X");
  // measureAxisLength(stepperY, limitSwitchY, 1, "Y");

  initWiFi();

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });

  server.begin();
  Serial.println("Web server started");
  // Print initial status at startup
  logStatus();
}

void loop() {
  limitSwitchX.loop();
  limitSwitchY.loop();

  applyQueuedTarget();

  // Safety: Stop if hitting limit switches (Active)
  // Homing uses direction -1 for X, implying switches are at the negative limit (0).
  // If switch pressed, prevent negative movement (distanceToGo < 0).
  if (limitSwitchX.isPressed() && stepperX.distanceToGo() < 0) {
    Serial.println("X limit switch pressed, stopping");
    stepperX.moveTo(stepperX.currentPosition());
  }
  if (limitSwitchY.isPressed() && stepperY.distanceToGo() < 0) {
    Serial.println("Y limit switch pressed, stopping");
    stepperY.moveTo(stepperY.currentPosition());
  }

  stepperX.run();
  stepperY.run();
  // Read serial lines and process them as commands or drawing payloads.
  // Lines ending in '\n' or '\r' are considered complete.
  static char serialLineBuf[128];
  static size_t serialLineIdx = 0;
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c == -1)
      break;
    if (c == '\n' || c == '\r') {
      if (serialLineIdx == 0) {
        // ignore empty line
        continue;
      }
      serialLineBuf[serialLineIdx] = 0;
      processTextPayload(serialLineBuf);
      serialLineIdx = 0;
    } else {
      if (serialLineIdx < (sizeof(serialLineBuf) - 1)) {
        serialLineBuf[serialLineIdx++] = static_cast<char>(c);
      } else {
        // overflow - reset buffer
        serialLineIdx = 0;
      }
    }
  }

  ws.cleanupClients();
}
