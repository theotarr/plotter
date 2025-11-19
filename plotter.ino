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
int PEN_DOWN_ANGLE = 45; // Mutable for runtime adjustment

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
    html, body { margin: 0; height: 100%; background: #111; color: #f5f5f5; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; overflow: hidden; }
    #toolbar { position: fixed; top: 0; left: 0; right: 0; display: flex; gap: 12px; align-items: center; padding: 10px 16px; background: rgba(0, 0, 0, 0.35); backdrop-filter: blur(12px); z-index: 10; }
    button { background: #1f1f1f; border: 1px solid #333; border-radius: 8px; color: inherit; font-size: 16px; padding: 8px 14px; cursor: pointer; }
    button:active { background: #2e2e2e; }
    button:hover { background: #2a2a2a; }
    button:disabled { opacity: 0.5; cursor: not-allowed; }
    #status { margin-left: auto; font-size: 14px; opacity: 0.85; }
    #canvas { display: block; width: 100vw; height: 100vh; cursor: crosshair; touch-action: none; position: fixed; top: 0; left: 0; }
    #settingsPanel { position: fixed; top: 60px; right: 16px; background: rgba(0, 0, 0, 0.9); backdrop-filter: blur(12px); border: 1px solid #333; border-radius: 12px; padding: 20px; min-width: 240px; z-index: 20; display: none; box-shadow: 0 8px 32px rgba(0, 0, 0, 0.5); max-height: calc(100vh - 80px); overflow-y: auto; }
    #settingsPanel.open { display: block; }
    .settings-group { margin-bottom: 20px; padding-bottom: 15px; border-bottom: 1px solid #333; }
    .settings-group:last-child { margin-bottom: 0; border-bottom: none; }
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
    #gamePanel { position: fixed; bottom: 20px; left: 50%; transform: translateX(-50%); background: rgba(0, 0, 0, 0.85); backdrop-filter: blur(12px); border: 1px solid #333; border-radius: 12px; padding: 15px 20px; display: none; z-index: 15; text-align: center; min-width: 300px; }
    #gamePanel.visible { display: block; }
    .game-status { font-size: 18px; margin-bottom: 10px; font-weight: bold; color: #fff; }
    .game-controls { display: flex; gap: 10px; justify-content: center; }
    .game-controls button { flex: 1; font-size: 14px; padding: 10px; }
  </style>
</head>
<body>
  <div id="toolbar">
    <button id="clearBtn">Clear</button>
    <button id="submitBtn">Submit</button>
    <button id="settingsBtn">Settings</button>
    <button id="gameBtn">Telephone</button>
    <div id="status">Offline</div>
  </div>
  
  <div id="gamePanel">
    <div class="game-status" id="gameStatusText">Waiting for Input</div>
    <div class="game-controls">
      <button id="gameActionBtn">Start Round</button>
      <button id="gameResetBtn">Reset Game</button>
    </div>
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
      <label class="settings-label" for="penAngleSlider">Pen Down Angle</label>
      <div class="stroke-width-display">
        <input type="range" id="penAngleSlider" min="0" max="180" value="45" step="1">
        <span id="penAngleValue">45</span>
      </div>
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
    <div class="settings-group">
      <label class="settings-label">Calibration</label>
      <div style="display: flex; gap: 8px; margin-bottom: 8px;">
        <button id="setTLBtn" style="flex: 1; font-size: 12px;">Set Top-Left</button>
        <button id="setBRBtn" style="flex: 1; font-size: 12px;">Set Btm-Right</button>
      </div>
      <div style="display: flex; gap: 8px; margin-bottom: 8px;">
        <button id="goTLBtn" style="flex: 1; font-size: 12px;">Go Top-Left</button>
        <button id="goBRBtn" style="flex: 1; font-size: 12px;">Go Btm-Right</button>
      </div>
      <div style="font-size: 11px; opacity: 0.6; line-height: 1.4;">
        TL: <span id="tlCoord">Not set</span><br>
        BR: <span id="brCoord">Not set</span>
      </div>
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
    const gameBtn = document.getElementById('gameBtn');
    const settingsPanel = document.getElementById('settingsPanel');
    const gamePanel = document.getElementById('gamePanel');
    const gameStatusText = document.getElementById('gameStatusText');
    const gameActionBtn = document.getElementById('gameActionBtn');
    const gameResetBtn = document.getElementById('gameResetBtn');
    
    const colorPicker = document.getElementById('colorPicker');
    const strokeWidthSlider = document.getElementById('strokeWidthSlider');
    const strokeWidthInput = document.getElementById('strokeWidthInput');
    const strokeWidthValue = document.getElementById('strokeWidthValue');
    const penAngleSlider = document.getElementById('penAngleSlider');
    const penAngleValue = document.getElementById('penAngleValue');
    const jogStepSelect = document.getElementById('jogStepSize');
    const setTLBtn = document.getElementById('setTLBtn');
    const setBRBtn = document.getElementById('setBRBtn');
    const goTLBtn = document.getElementById('goTLBtn');
    const goBRBtn = document.getElementById('goBRBtn');
    const tlCoordEl = document.getElementById('tlCoord');
    const brCoordEl = document.getElementById('brCoord');
    
    // Plotter physical limits (in steps)
    const PLOTTER_MAX_X = 1200;
    const PLOTTER_MAX_Y = 450;

    // Calibration state
    let calMinX = 0;
    let calMinY = 0;
    let calMaxX = PLOTTER_MAX_X;
    let calMaxY = PLOTTER_MAX_Y;

    // Load saved calibration
    if (localStorage.getItem('plotter_cal')) {
      try {
        const saved = JSON.parse(localStorage.getItem('plotter_cal'));
        calMinX = saved.minX;
        calMinY = saved.minY;
        calMaxX = saved.maxX;
        calMaxY = saved.maxY;
        tlCoordEl.textContent = `${calMinX}, ${calMinY}`;
        brCoordEl.textContent = `${calMaxX}, ${calMaxY}`;
      } catch(e) { console.error(e); }
    }

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
        } else if (event.data.startsWith("pos:")) {
          const parts = event.data.substring(4).split(',');
          const x = parseInt(parts[0]);
          const y = parseInt(parts[1]);
          if (pendingCalType === 'TL') {
            calMinX = x;
            calMinY = y;
            tlCoordEl.textContent = `${x}, ${y}`;
          } else if (pendingCalType === 'BR') {
            calMaxX = x;
            calMaxY = y;
            brCoordEl.textContent = `${x}, ${y}`;
          }
          // Save calibration
          localStorage.setItem('plotter_cal', JSON.stringify({
            minX: calMinX, minY: calMinY, maxX: calMaxX, maxY: calMaxY
          }));
          pendingCalType = null;
        }
      };
    }

    let pendingCalType = null;

    // --- TELEPHONE GAME LOGIC ---
    let gameMode = false;
    let gameState = 'IDLE'; // IDLE, RECORDING_USER, REPLAYING
    let nextRoundBuffer = [];
    let isReplaying = false;

    gameBtn.addEventListener('click', () => {
        gameMode = !gameMode;
        gameBtn.style.background = gameMode ? '#2365ea' : '#1f1f1f';
        gamePanel.classList.toggle('visible', gameMode);
        if (gameMode) resetGame();
    });

    function resetGame() {
        gameState = 'IDLE';
        pointsBuffer = [];
        nextRoundBuffer = [];
        isReplaying = false;
        updateGameUI();
        // Clear canvas
        clearBtn.click();
    }

    function updateGameUI() {
        if (!gameMode) return;
        switch(gameState) {
            case 'IDLE':
                gameStatusText.textContent = "Draw something & Click Start";
                gameActionBtn.textContent = "Start Round 1";
                gameActionBtn.disabled = false;
                break;
            case 'RECORDING_USER':
                // Not really a state, transitioning immediately usually
                break;
            case 'REPLAYING':
                gameStatusText.textContent = "Robot Drawing...";
                gameActionBtn.textContent = "Please Wait";
                gameActionBtn.disabled = true;
                break;
            case 'ROUND_DONE':
                gameStatusText.textContent = "Round Complete. Next?";
                gameActionBtn.textContent = "Start Next Round";
                gameActionBtn.disabled = false;
                break;
        }
    }

    gameActionBtn.addEventListener('click', () => {
        if (gameState === 'IDLE') {
            // User has drawn something manually. 
            // Convert current pointsBuffer to the source for the robot
            if (pointsBuffer.length === 0) {
                alert("Draw something first!");
                return;
            }
            startRobotRound(pointsBuffer);
        } else if (gameState === 'ROUND_DONE') {
            // Use the buffer we captured during the last robot session
            if (nextRoundBuffer.length === 0) {
                alert("No input detected from last round!");
                return;
            }
            // Clear screen for clean drawing
            clearBtn.click();
            
            // Swap buffers: nextRoundBuffer becomes the source
            const sourcePoints = [...nextRoundBuffer];
            nextRoundBuffer = []; // Clear for new recording
            
            startRobotRound(sourcePoints);
        }
    });

    gameResetBtn.addEventListener('click', resetGame);

    async function startRobotRound(points) {
        gameState = 'REPLAYING';
        isReplaying = true;
        updateGameUI();
        
        // Send points to robot
        // Note: We use a modified submit logic here to allow simultaneous recording
        await sendPointsForGame(points);
    }

    // Modified sender that doesn't block UI interaction events (so we can record)
    async function sendPointsForGame(points) {
        if (isSending) return;
        isSending = true;
        updateStatus('Robot Drawing...', true);
        
        try {
            for (let pt of points) {
                sendSinglePoint(pt);
                await new Promise(resolve => {
                    ackResolver = resolve;
                    setTimeout(resolve, 2000);
                });
            }
            // Wait for final "Idle" or empty queue from firmware? 
            // For now, we rely on the ACK loop finishing.
            // Ideally, we wait for a "JobDone" message.
        } catch (e) {
            console.error(e);
        } finally {
            isSending = false;
            isReplaying = false;
            gameState = 'ROUND_DONE';
            updateGameUI();
            updateStatus('Round Done', true);
        }
    }

    
    setTLBtn.addEventListener('click', () => {
      pendingCalType = 'TL';
      websocket.send('cmd:get_pos');
    });
    
    setBRBtn.addEventListener('click', () => {
      pendingCalType = 'BR';
      websocket.send('cmd:get_pos');
    });
    
    goTLBtn.addEventListener('click', () => {
      if (!websocket || websocket.readyState !== WebSocket.OPEN) return;
      // Send command to move to TL coordinates with Pen UP
      websocket.send(`${calMinX}&${calMinY}-0`);
    });

    goBRBtn.addEventListener('click', () => {
      if (!websocket || websocket.readyState !== WebSocket.OPEN) return;
      // Send command to move to BR coordinates with Pen UP
      websocket.send(`${calMaxX}&${calMaxY}-0`);
    });

    function recordPoint(x, y, pen) {
      const now = performance.now();
      
      // Decide which buffer to use
      // If game mode is active AND we are in REPLAYING state, we record to nextRoundBuffer
      // Otherwise, we record to the standard pointsBuffer (user drawing manually)
      let targetBuffer = pointsBuffer;
      
      if (gameMode && isReplaying) {
          targetBuffer = nextRoundBuffer;
      }
      
      if (targetBuffer.length > 0) {
        const last = targetBuffer[targetBuffer.length - 1];
        if (pen === last.pen &&
            Math.abs(x - last.x) < 1 &&
            Math.abs(y - last.y) < 1 &&
            now - last.time < 10) {
          return;
        }
      }
      targetBuffer.push({ x, y, pen, time: now });
    }

    function sendSinglePoint(pt) {
      if (!websocket || websocket.readyState !== WebSocket.OPEN) return;
      
      // Map canvas coordinates to plotter steps using CALIBRATION BOUNDS
      
      // Canvas dimensions (logical pixels)
      const w = canvas.width / currentDpr;
      const h = canvas.height / currentDpr;
      
      // Calibration bounds range
      const rangeX = calMaxX - calMinX;
      const rangeY = calMaxY - calMinY; // Note: Inverted Y logic applies relative to these steps
      
      // Normalize input (0..1)
      const normX = pt.x / w;
      const normY = pt.y / h;
      
      // Map to step range + offset
      const targetX = Math.round(calMinX + (normX * rangeX));
      
      // For Y: 
      // Canvas Y is 0 at top, increasing downwards.
      // Plotter Y is 0 at switch (bottom), increasing upwards.
      // So we need to invert: 
      // normY=0 (top) -> targetY should be MAX (away from switch)
      // normY=1 (bottom) -> targetY should be MIN (at switch)
      const targetY = Math.round(calMinY + (normY * rangeY));

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
      // If sending normally (submit button), block. 
      // But if REPLAYING (game mode), we MUST allow events to pass to recordPoint
      if (isSending && !isReplaying) return;
      evt.preventDefault();
      drawing = true;
      lastPoint = getCanvasPoint(evt);
      // Only draw visually if user is manually drawing, OR if we want to see the feedback line?
      // Let's allow visual drawing always for feedback.
      applyCanvasSettings();
      ctx.beginPath();
      ctx.moveTo(lastPoint.x, lastPoint.y);
      recordPoint(lastPoint.x, lastPoint.y, 1);
    }

    function moveDraw(evt) {
      if (!drawing) return;
      // Block if sending regular job, allow if Replaying game
      if (isSending && !isReplaying) return;
      
      evt.preventDefault();
      const point = getCanvasPoint(evt);
      ctx.lineTo(point.x, point.y);
      ctx.stroke();
      lastPoint = point;
      recordPoint(point.x, point.y, 1);
    }

    function endDraw(evt) {
      if (!drawing) return;
       if (isSending && !isReplaying) return;
       
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
    penAngleSlider.addEventListener('input', (e) => {
      const val = e.target.value;
      penAngleValue.textContent = val;
      if (websocket && websocket.readyState === WebSocket.OPEN) {
        websocket.send(`cmd:pen_angle:${val}`);
      }
    });

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
    } else if (strncmp(cmd, "pen_angle:", 10) == 0) {
      int angle = atoi(cmd + 10);
      angle = clampValue(angle, 0, 180);
      PEN_DOWN_ANGLE = angle;
      if (penCurrentlyDown) {
        penServo.write(PEN_DOWN_ANGLE);
      }
      Serial.printf("Pen down angle set to %d\n", PEN_DOWN_ANGLE);
      return true;
    } else if (strcmp(cmd, "get_pos") == 0) {
       Serial.printf("pos:%ld,%ld\n", stepperX.currentPosition(), stepperY.currentPosition());
       // Also send to WebSocket
       char buf[64];
       snprintf(buf, sizeof(buf), "pos:%ld,%ld", stepperX.currentPosition(), stepperY.currentPosition());
       ws.textAll(buf);
       return true;
    }
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

  const int32_t currentX = stepperX.targetPosition();
  const int32_t currentY = stepperY.targetPosition();

  const bool needXMove = (absDiff(targetX, currentX) >= MIN_STEP_DELTA);
  const bool needYMove = (absDiff(targetY, currentY) >= MIN_STEP_DELTA);

  // Smart pen logic:
  // If pen is currently UP and target is DOWN: Move first, then put pen DOWN.
  // If pen is currently DOWN and target is UP: Put pen UP first, then move.
  // If pen state doesn't change: Just move.

  // Case 1: We are UP and need to go DOWN (Travel Move).
  if (!penCurrentlyDown && targetPen) {
      // We need to MOVE first, then DROP.
      if (needXMove || needYMove) {
          // Execute the move part now.
          if (needXMove && motorsEnabled) stepperX.moveTo(targetX);
          if (needYMove && motorsEnabled) stepperY.moveTo(targetY);

          // CRITICAL: We popped the command, but we only did HALF of it (Move).
          // The Pen Down part must happen AFTER arrival.
          // We can "unread" the command or push a new one to the front.
          // Since our queue is a ring buffer, pushing to front (qTail - 1) is valid 
          // as long as it's not full (which it isn't, we just popped one).
          
          portENTER_CRITICAL(&targetMux);
          // Decrement tail to "un-pop" essentially, OR push a modified command.
          // Let's push a new command: {targetX, targetY, true}
          // But wait, if we just "un-pop", next time we come back, 
          // penCurrentlyDown is still false, targetPen is true, needMove is FALSE (because we set targets).
          // So we will fall into the "else" block or a "no move" block?
          
          // Let's see what happens if we re-queue the SAME command:
          // Next loop: needXMove=false, needYMove=false.
          // It enters logic. 
          // !penCurrentlyDown && targetPen -> True.
          // needMove -> False.
          // So we need a branch here for "No Move Needed".
          
          // Let's push the command back to the HEAD (so it's next).
          // We need to manipulate qTail.
          size_t prevTail = (qTail == 0) ? (QUEUE_CAPACITY - 1) : (qTail - 1);
          cmdQueue[prevTail] = cmd; // Put it back
          qTail = prevTail;
          portEXIT_CRITICAL(&targetMux);
          
          // But wait, if we put it back, next loop will see the exact same state?
          // No, because we called stepper.moveTo(), so now `stepper.distanceToGo() != 0`.
          // So `applyQueuedTarget` returns immediately at the top check!
          // It waits until the move finishes.
          // When move finishes, we come back here.
          // CurrentPos == TargetPos. needMove = False.
          // We enter the block: !penCurrentlyDown && targetPen is True.
          // needMove is False.
          // So we fall through to... where?
      } else {
          // We are AT the location, and Pen is UP, but Target is DOWN.
          // Just drop the pen!
          updatePen(true);
      }
  } 
  // Case 2: We are DOWN and need to go UP (Lift Move).
  else if (penCurrentlyDown && !targetPen) {
      // Lift immediately, then move.
      updatePen(false); 
      // Delay slightly to allow servo to move? 
      // A small non-blocking delay is hard here. 
      // Usually servo is fast enough compared to accel ramp-up.
      
      if (needXMove && motorsEnabled) stepperX.moveTo(targetX);
      if (needYMove && motorsEnabled) stepperY.moveTo(targetY);
  } 
  // Case 3: State matches (Down->Down or Up->Up).
  else {
      // Just move.
      // updatePen not needed technically, but good for consistency.
      updatePen(targetPen); 
      if (needXMove && motorsEnabled) stepperX.moveTo(targetX);
      if (needYMove && motorsEnabled) stepperY.moveTo(targetY);
  }

    
  // Check if queue is empty after processing
  portENTER_CRITICAL(&targetMux);
  bool isEmpty = (qHead == qTail) && (stepperX.distanceToGo() == 0) && (stepperY.distanceToGo() == 0);
  portEXIT_CRITICAL(&targetMux);
  
  if (isEmpty) {
      // Simple debounce or state check could be added here if needed
      // For now, we just let the loop continue.
      // A more robust "IDLE" check is done in the main loop.
  }
}

// Track idle state to send notification
bool wasIdle = true;

void checkIdleStatus() {
    bool isIdle = (stepperX.distanceToGo() == 0) && (stepperY.distanceToGo() == 0);
    // Also check queue
    portENTER_CRITICAL(&targetMux);
    if (qHead != qTail) isIdle = false;
    portEXIT_CRITICAL(&targetMux);
    
    if (isIdle && !wasIdle) {
        // Transitioned to Idle
        ws.textAll("status:idle");
        wasIdle = true;
    } else if (!isIdle) {
        wasIdle = false;
    }
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
  
  checkIdleStatus();
  
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
