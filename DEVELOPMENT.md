# DDC264EVM UI - Development Guide

## System Architecture

### Overview
This is a PyQt5-based GUI application for controlling a Texas Instruments **DDC264EVM board** (X-ray detector with 256 channels). The system captures detector data, processes it, and reconstructs it into either:
- **2D images**: 16×16 pixel arrays (standard mode)
- **1D detector**: 128-stripe vertical arrays (new mode, added Feb 2026)

### Data Flow Pipeline

```
DDC264 Hardware (256 channels)
    ↓
[USB via DDC264EVM_IO.dll]
    ↓
FPGAControl.capture_data()
    ↓
Raw integer array (256 channels × 2 integrators × N samples)
    ↓
[Save to .txt file]
    ↓
load_file() — Reconstruct to both 1D AND 2D arrays
    ↓
build_image() — Choose mode and apply normalization
    ↓
Display in PyQtGraph ImageItem (with autoRange for dynamic scaling)
```

### Key Components

#### 1. **FPGAControl** (`tools/fpga_control.py`)
- Interfaces with `DDC264EVM_IO.dll` (Windows only)
- Configures ADC settings (range, bit rate, timing)
- `capture_data()` returns raw samples from all 256 channels
- Data stored as: `01A, 01B, 02A, 02B, ..., 256A, 256B` (A/B integrators)

#### 2. **Data File Format** (.txt files)
```
ChA, sample_idx, raw_value, 0, 0, bit_rate
01A, 0, 12345, 0, 0, 16
01A, 1, 12350, 0, 0, 16
...
256B, 511, 12200, 0, 0, 16
```

#### 3. **Decoder Matrices** (Channel → Pixel Mapping)
- **`decoder_matrix.txt`**: 16×16 (maps 256 channels to 16×16 pixel grid)
  - Loaded at startup into `self.decoder_matrix`
  - Used for 2D reconstruction
  
- **`decoder_matrix_sample1.txt`**: 128 channels (maps to 128 vertical stripes)
  - Loaded at startup into `self.decoder_matrix_1d`
  - Used for 1D reconstruction

#### 4. **Reconstruction Logic** (`load_file()` method)
**Key insight: Always reconstruct BOTH 1D and 2D** (regardless of current mode)
This allows mode switching without reloading files.

```python
# 2D Reconstruction (16×16)
for i in range(16):
    for j in range(16):
        channel = decoder_matrix[i, j]
        pixel_value = peaks[f"channel_keyA"][0]  # Signal
        
# Then rotate 270°: np.rot90(array, 3)

# 1D Reconstruction (128 stripes)
for i in range(128):
    channel = decoder_matrix_1d[i]
    stripe_value = peaks[f"channel_keyA"][0]
```

#### 5. **Image Processing** (`build_image()` method)
**Normalization Modes:**
1. **"none"** - Absolute charge (raw values scaled by pixel size and integration time)
2. **"full leakage/flat-field"** - (signal - baseline) / open_beam_signal
3. **"leakage/flat-field from open beam"** - Uses open beam's baseline for correction
4. **"flat-field only"** - Simple ratio of full means (no baseline subtraction)

**1D Display Special Handling:**
- 1D arrays (128,) are reshaped to (128, 1) then **tiled to (128, 128)**
- Each stripe stretches horizontally to fill square display
- Physical conversion applied: `value / (pixelX × pixelY × ConvLowInt) × 1e15`
- Pixel‑size fields are automatically set to **1.56 µm** when 1D mode is
  enabled (previous values are restored when returning to 2D).
- **Layout change:** the two image panes are stacked vertically when 1D
  mode is active (`horizontalLayout_26` direction switched to TopToBottom),
  allowing them to span the full window width and produce long horizontal
  stripes. Aspect locking is disabled in 1D so the images can stretch freely.
  The layout/orientation is adjusted **immediately when the checkbox is
  toggled**; the image is rebuilt automatically so no extra "Build image"
  click is required.

### Important Implementation Details

#### Aspect Ratio Locking
Both image views have `setAspectLocked(True)` to maintain 1:1 square pixels:
- 2D: 16×16 = square
- 1D: 128×128 (after tiling) = square

#### Dynamic View Scaling
After every `setImage()`, `autoRange()` is called unconditionally:
```python
self.img_item.setImage(left_image)
self.image_view.autoRange()  # ← ALWAYS called, not just for 1D
```
**Reason:** Prevents zoom/crop issues when window resizes or mode switches.

#### Data Storage Strategy
```python
# Always stored simultaneously
self.image_data       # 2D: (16, 16)
self.image_data_1d    # 1D: (128,)
self.dark_current_data
self.dark_current_data_1d
self.open_beam_data
self.open_beam_data_1d
self.open_beam_baseline
self.open_beam_baseline_1d
```
This dual-storage enables seamless mode switching without reloading files.

### Division-by-Zero Safety
To avoid NaN warnings when normalizing:
```python
denom = np.array(self.open_beam_data_1d, dtype=float)
denom[denom == 0] = np.nan  # Replace zeros with NaN before division
left_image = image_data / denom
```

---

## Recent Changes (February 2026)

### Added: 1D Detector Mode
**Checkbox:** "1D detector" (default: OFF) in image controls section

**What it does:**
- Uses `decoder_matrix_sample1.txt` (128 channels)
- Reconstructs data to 128-element vertical arrays
- Tiles to 128×128 for display (each stripe stretches horizontally)
- Applies all 4 normalization modes identically to 2D mode
- Supports both left and right image displays (image vs dark current, or image vs open beam)

**Key fixes during implementation:**
1. Always reconstruct both 1D and 2D (not just active mode)
2. Call `autoRange()` after every image update
3. Apply physical conversion to 1D "none" mode
4. Guard division by zero with NaN replacement

---

## Arduino Communication & Stepper Motor Control

### System Overview
The application communicates with an Arduino-based stepper motor controller via serial (USB) to enable:
- **Rotational scanning** - Rotate the detector by specified angles (CW/CCW)
- **Absolute positioning** - Move to specific detector angles
- **Home reference** - Define and return to zero position
- **Power cycling** - Toggle DDC264EVM board power via relay (on critical FPGA errors)

### Hardware Configuration

**Serial Connection:**
- **Baud rate:** 115,200
- **Default port:** COM6 (Windows) or COM4 (fallback)
- **Auto-detection:** Searches for Arduino by VID/PID matching
- **Reset behavior:** 2-second reset delay after serial connection established

**Motor & Stepper Driver Specifications:**
- **Microstepping:** 1/256 (configured via DIP switches SW5-SW8: 1110)
- **Full revolution:** 25,600 steps
- **Resolution:** 25,600 steps = 360°, so **0.0140625° per step**
- **Direction control:** CW (clockwise) = 0 / CCW (counter-clockwise) = 1
- **Pulse timing:** 100 microseconds between steps (defined as `DELAY_MICROS`)

**Pinout (Arduino - A6821 Stepper Driver):**
- Pin 5: PUL (Pulse) - triggers step
- Pin 6: DIR (Direction) - sets rotation direction (0=CW, 1=CCW)
- Pin 7: ENA (Enable) - motor enable (LOW=enabled, HIGH=disabled)
- Pin 8: RELAY_PIN - power cycle relay control

**Position Tracking:**
- Current position stored in `currentSteps` (volatile, 0 to 25,599)
- EEPROM address 0-1: Persistent position storage (survives power loss)
- Position wraps around at 25,600 (modular arithmetic for 360°)

### Communication Protocol

All commands use **JSON format** with newline termination:

**Command Format:**
```json
{
  "command": "go|stop|status|home|set_home|absolute_position|enable_motor|disable_motor|power_cycle|run_continuous|set_speed|position",
  "direction": "CW|CCW",         // required for "go" command (CW=0, CCW=1 internally)
  "steps": <integer>,             // optional, for "go" command (relative movement)
  "degrees": <float>,             // optional for "go" or "absolute_position", auto-converted to steps
  "delay": <microseconds>         // optional for "run_continuous" or "set_speed"
}
```

**Available Commands:**

1. **Rotate by steps (relative):**
   ```json
   {"command": "go", "direction": "CW", "steps": 1000}
   ```
   Rotates 1000 steps clockwise (~14.06°). Checks if motor busy; fails if already moving.
   Updates `currentSteps` via modular arithmetic and persists to EEPROM.

2. **Rotate by degrees (relative):**
   ```json
   {"command": "go", "direction": "CCW", "degrees": 45.0}
   ```
   Rotates 45° counter-clockwise (internally converted to steps: 45 × 71.111 ≈ 3200 steps).
   Direction: "CW" or "CCW" (string comparison on Arduino).

3. **Move to absolute position:**
   ```json
   {"command": "absolute_position", "degrees": 120.0}
   ```
   Moves detector to absolute 120° angle. Calculates shortest path (CW or CCW).
   Fails with error if motor already running. Returns current and target position.

4. **Go home (return to zero position):**
   ```json
   {"command": "home"}
   ```
   Returns motor to home (0°/0 steps). Calculates shortest path:
   - If `currentSteps ≤ STEPS_PER_REVOLUTION/2`: Move CCW (backwards)
   - Else: Move CW (forward, wrapping around)
   Resets `currentSteps = 0` and updates EEPROM.

5. **Set current position as home:**
   ```json
   {"command": "set_home"}
   ```
   Marks current position as zero reference without moving motor.
   Useful for calibration after manually rotating detector.

6. **Query current position:**
   ```json
   {"command": "position"}
   ```
   Returns two lines:
   - `Position: X degrees / Y steps` (volatile RAM, current session)
   - `Position (memory): X degrees / Y steps` (EEPROM, persistent)

7. **Query motor status:**
   ```json
   {"command": "status"}
   ```
   Returns:
   - `{"status":"OK","msg":"Ready"}` if motor idle
   - `{"status":"BUSY","msg":"Moving"}` if motor running

8. **Stop motor immediately:**
   ```json
   {"command": "stop"}
   ```
   Halts motor and clears `steps_left` counter.

9. **Enable/disable stepper driver:**
   ```json
   {"command": "enable_motor"}
   {"command": "disable_motor"}
   ```
   Toggle the ENA pin (LOW=enabled, HIGH=disabled).
   Useful for saving power or manual motor positioning.

10. **Continuous rotation:**
    ```json
    {"command": "run_continuous", "dir": 0, "delay": 100}
    ```
    Starts continuous rotation in direction (0=CW, 1=CCW) with custom step delay.
    Uses `start_continuous()` method; must use "stop" command to halt.

11. **Update rotation speed:**
    ```json
    {"command": "set_speed", "delay": 150}
    ```
    Modifies step delay (in microseconds) for ongoing continuous rotation.
    Default delay: 100μs (fastest). Larger values = slower rotation.

12. **Power cycle relay:**
    ```json
    {"command": "power_cycle"}
    ```
    Triggers relay on pin 8 for 2000ms to power-cycle DDC264EVM board.
    Sends confirmation when relay is activated and deactivated.

### Response Protocol

Arduino responds with JSON formatted lines. The motor state machine monitors for motion start/stop transitions and emits automatic status updates:

**Automatic Responses (no command input required):**

When motor starts moving:
```json
{"status":"BUSY","msg":"Move started"}
```

When motor finishes moving:
```json
{"status":"OK","msg":"Move complete"}
```

**Command Response Examples:**

*Home command successful:*
```json
{"status":"OK","msg":"current offset: 45 degrees / 3200 steps"}
```

*Position command:*
```json
{"status":"OK","msg":"Position: 120 degrees / 8533 steps"}
{"status":"OK","msg":"Position (memory): 120 degrees / 8533 steps"}
```

*Status command (motor idle):*
```json
{"status":"OK","msg":"Ready"}
```

*Status command (motor moving):*
```json
{"status":"BUSY","msg":"Moving"}
```

*Absolute position command (motor busy):*
```json
{"status":"ERROR","msg":"Motor is busy! Wait until end or stop with stop command"}
```

*Go command (motor busy):*
```json
{"status":"ERROR","msg":"Motor is busy!. To use command wait until end or stop with \"stop\" command!"}
```

*Invalid/malformed JSON:*
```json
{"status":"ERROR","msg":"Bad JSON"}
```

*Invalid command value:*
```json
{"status":"ERROR","msg":"Invalid command!"}
```

*Power cycle relay triggered:*
```json
{"status":"OK","msg":"Triggering power cycle relay"}
{"status":"OK","msg":"Power cycle relay triggered"}
```

*Absolute position already reached:*
```json
{"status":"OK","msg":"Already at target position: 120.0 degrees"}
```

*Absolute position move initiated:*
```json
{"status":"OK","msg":"Moving to 120.0 degrees (8533 steps)"}
```

### Timeout Behavior

The application implements stage-based timeouts to handle communication delays:

1. **Initial response timeout: 5 seconds**
   - Expected response: `{"status": "BUSY", "msg": "Move started"}`
   - If not received: Command assumed failed

2. **Motion completion timeout: 30 seconds**
   - Expected response: `{"status": "OK", "msg": "Move complete"}`
   - For status queries: Immediate response (no motion timeout)

3. **Fallback & Error Handling:**
   ```python
   if not response_received_within_timeout:
       status = "No response from Arduino"
       user_prompted = "Please verify Arduino connection and retry"
   ```

### Python Implementation

#### RotationalController Class (`rotational_control.py`)

The Python interface to Arduino stepper control:

```python
class RotationalController:
    def __init__(self, port="COM4", full_rev=25600):
        """Initialize Arduino connection
        
        Args:
            port: Serial port (e.g., "COM4", "/dev/ttyUSB0")
            full_rev: Steps per full revolution (default 25,600)
        """
        self.arduino = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(2)  # Reset delay
        self.full_rev = full_rev
        
    def rotate(self, angle_deg):
        """Rotate by angle in degrees (positive=CW, negative=CCW)
        
        Converts angle to steps and sends via JSON command.
        Blocks until Arduino reports motion complete.
        """
        steps = int(abs((angle_deg / 360) * self.full_rev))
        direction = "CW" if angle_deg >= 0 else "CCW"
        self._rotate(steps, direction)
        
    def _rotate(self, steps, direction="CW"):
        """Send rotation command and wait for completion
        
        Sends: {"command": "go", "direction": "CW|CCW", "steps": N}
        Waits for: {"status":"OK","msg":"Move complete"}
        """
        command = {
            "command": "go",
            "direction": direction,
            "steps": steps
        }
        self._send_command(command)
        
    def _send_command(self, command_dict):
        """Send JSON command, parse responses with timeouts"""
        cmd_json = json.dumps(command_dict) + "\n"
        self.arduino.write(cmd_json.encode("utf-8"))
        
        responses = []
        start = time.time()
        
        # Read responses until OK or ERROR
        while time.time() - start < 30.0:  # 30s total timeout
            if self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode('utf-8').strip()
                if line:
                    try:
                        resp = json.loads(line)
                        responses.append(resp)
                        if resp.get("status") in ["OK", "ERROR"]:
                            return responses
                    except json.JSONDecodeError:
                        pass  # Skip invalid JSON, wait for valid response
            time.sleep(0.02)
            
        raise TimeoutError("No response from Arduino")
    
    def _read_json_line(self, timeout=30):
        """Block until valid JSON line received
        
        Skips invalid JSON lines, continues reading until valid.
        Raises TimeoutError if timeout exceeded without valid JSON.
        """
        start = time.time()
        buffer = ""
        
        while time.time() - start < timeout:
            if self.arduino.in_waiting > 0:
                byte = self.arduino.read(1).decode('utf-8', errors='ignore')
                buffer += byte
                
                if byte == '\n':
                    try:
                        return json.loads(buffer.strip())
                    except json.JSONDecodeError:
                        buffer = ""  # Reset and continue
            time.sleep(0.01)
            
        raise TimeoutError(f"No valid JSON received within {timeout}s")
```

**Key Characteristics:**
- **Blocking I/O:** `rotate()` blocks until Arduino confirms motion complete
- **JSON protocol:** All commands and responses use JSON format with newline termination
- **Error handling:** Catches JSON decode errors, retries until valid response or timeout
- **Position tracking:** Arduino maintains position in EEPROM across power cycles
- **Automatic responses:** Motor start/stop emitted automatically (used to confirm motion initiation)

#### Main UI Integration (`mainwindow.py`)

The application provides high-level handlers for motor control. All methods use `_send_arduino_command()` to communicate with Arduino via RotationalController:

**Core Methods:**

1. **`_send_arduino_command(command_dict, timeout=5.0)`**
   - Generic command sender for all Arduino operations
   - Auto-detects Arduino port via `_find_arduino_port()` or falls back to stored port
   - Prefers persistent controller (`self._rotation_controller`) to avoid repeated opens
   - Sends command as JSON, reads responses until `status` is "OK" or "ERROR"
   - Logs all commands and responses to UI status bar
   - Returns: `(success: bool, responses: list, error_msg: str)`
   - **Timeout stages:** 5s for initial response (motion start/error), 30s for completion

2. **`_perform_rotation(angle_deg)`**
   - High-level rotation wrapper for batch capture
   - Uses `RotationalController.rotate(angle_deg)` internally
   - Handles port detection and local controller creation if needed
   - Adds 0.5s settling time after motion completes (motor inertia)
   - Logs rotation time to status bar
   - Used by rotation mode to rotate detector between captures

3. **`_on_home_clicked()`**
   - HOME button handler (red button, large UI button)
   - Sends `{"command": "home"}` with 10s timeout
   - Shows error dialog if Arduino response status is "ERROR"
   - Updates status bar with completion time
   - Used for single-click return to zero reference

4. **`_on_set_home_clicked()`**
   - Set HOME button handler (black button)
   - Shows confirmation dialog before proceeding
   - Only sends command if user confirms: "Set current position as HOME (zero)?"
   - Sends `{"command": "set_home"}` with 3s timeout
   - Shows info dialog on success: "Current position has been set as HOME (zero)."
   - Used for calibration after manual motor positioning

5. **`_on_custom_angle_clicked()`**
   - Custom angle input handler (float input field + button)
   - Reads angle from `self.customAngleInput.text()` and validates as float
   - Shows error dialog if input is invalid
   - Sends `{"command": "absolute_position", "degrees": <angle>}` with 30s timeout
   - Logs completion to status bar: "moved to X degrees"
   - Used for absolute positioning to specific angles

6. **`_find_arduino_port()`**
   - Auto-detects Arduino by VID/PID matching or COM port scanning
   - Searches for common Arduino VID:PID combinations
   - Falls back to default port (COM6 or COM4)
   - Called on every command to ensure proper port (supports hot-swap)

7. **`_ensure_rotation_controller()`**
   - Initializes or reuses persistent RotationalController
   - Stores in `self._rotation_controller` to avoid repeated port opens
   - Called before rotation operations to ensure port is open

**Error Handling in UI:**
- If Arduino not found: Falls back to manual control, logs error to status bar
- If timeout: Shows warning dialog and prompts user to:
  - Verify Arduino connection
  - Check motor power
  - Manually adjust position
  - Retry command
- If JSON parse error: Skipped automatically (motor continues responding)
- If motor busy: Shows error message "Motor is busy! Wait until end or stop with stop command"

### Rotation Mode (Multiple Angles)

When rotation mode is enabled, the application automatically:

1. **Batches angles:** Divides 360° by number of requested batches
   - Example: 4 batches = [0°, 90°, 180°, 270°]

2. **Captures at each angle:** Calls `_perform_rotation(angle)` for each batch

3. **Concatenates data:** All samples from each angle stored sequentially

**Example workflow (3 batches):**
```
Angle 1: 0°    → Capture N samples → Store in file
         ↓ Rotate 120°
Angle 2: 120°  → Capture N samples → Append to file
         ↓ Rotate 120°
Angle 3: 240°  → Capture N samples → Append to file
```

### Error Recovery

**Arduino Not Found:**
- If auto-detection fails, falls back to default port (COM6)
- User can manually set port via environment variable or config file
- UI shows error message but continues with manual control option

**Serial Communication Timeout:**
- First timeout (5s): Command assumed not received, retry suggested
- Second timeout (30s): Motion failure, user prompted to:
  - Check Arduino connection
  - Verify motor is powered
  - Manually adjust motor position
  - Reconnect and retry

**Corrupted JSON Response:**
- If Arduino sends invalid JSON: Line is skipped, waiting continues
- Invalid responses logged but don't block subsequent valid responses

### Power Cycling via Relay

**Hardware Setup:**
- Arduino pin 8 controls relay switch (LOW=off, HIGH=on)
- Relay holds HIGH for 2000ms when power-cycle triggered
- Relay used to interrupt power to DDC264EVM board (via external circuit)
- Allows board reset without manual intervention

**Trigger Condition (in FPGAControl):**
When FPGA error code indicates critical failure (undefined in current version):
```python
if error_code in [CRITICAL_ERROR_CODES]:
    self.request_power_cycle.emit()  # Signal to UI
```

**Arduino Command:**
```json
{"command": "power_cycle"}
```

**Arduino Implementation (main.cpp):**
```cpp
if (strcmp(command, "power cycle") == 0) {
    Serial.println("{\"status\":\"OK\",\"msg\":\"Triggering power cycle relay\"}");
    digitalWrite(RELAY_PIN, HIGH);
    delay(2000);  // Hold relay on for 2 seconds
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("{\"status\":\"OK\",\"msg\":\"Power cycle relay triggered\"}");
}
```

**Relay Timing:**
- Command received → Relay activates (2000ms hold)
- During 2s: Board power supply cut off
- After 2s: Relay deactivates, board power restores automatically
- Complete power-cycle: 2 seconds

**Current Implementation (Manual Flow):**
1. FPGAControl detects error and emits `request_power_cycle` signal
2. UI shows dialog: "Power-cycle the board? (Click OK after manually cycling power)"
3. User manually switches board power off/on (or uses Arduino relay)
4. User clicks "OK" in dialog
5. Python calls `notify_power_cycle_done()` to update worker's FPGA reference
6. Data capture resumes

**Python Interface:**
In `mainwindow.py`, power-cycle handling is managed by:
- `request_power_cycle` signal emitted from ReaderWorker
- `_powercycle_ack` flag tracks confirmation
- `_waiting_for_powercycle` flag tracks state
- `notify_power_cycle_done()` method called on user confirmation

---

## Testing Checklist

### Data Acquisition & Image Processing
- [ ] Load file in 2D mode → verify image correct
- [ ] Load file in 1D mode → verify 128 stripes displayed
- [ ] Switch 2D → 1D → 2D → verify no errors or shape mismatches
- [ ] Resize window in 1D mode → verify no cropping
- [ ] Test all 4 normalization modes in both 2D and 1D
- [ ] Test dark current + open beam display combinations
- [ ] Verify manual scale input doesn't interfere with auto-range

### Arduino Communication & Rotation

**Port Detection:**
- [ ] Arduino connected → port auto-detected on startup
- [ ] Arduino on default COM6 → recognized as fallback
- [ ] Arduino on non-standard port → can be manually set via config

**Motor Commands:**
- [ ] HOME button → motor moves to position 0
- [ ] Set HOME button → current position marked as zero (requires confirmation)
- [ ] Custom angle input → motor moves to specified absolute angle
- [ ] Rotation mode (3+ batches) → motor rotates through all angles without error
- [ ] Stop button during rotation → motion halts immediately

**Response Handling:**
- [ ] Valid JSON response → parsed and displayed in status bar
- [ ] Invalid JSON response → skipped without blocking
- [ ] Timeout (5s for motion start) → error message shown, user can retry
- [ ] Timeout (30s for motion completion) → error dialog prompts manual intervention

**Mode Switching:**
- [ ] 2D mode with rotation enabled → angle batches captured correctly
- [ ] 1D mode with rotation enabled → stripe batches captured correctly
- [ ] Switch modes mid-capture (if UI allows) → graceful fallback

### Integration & Edge Cases
- [ ] Arduino disconnected → fallback port tried, error shown if all fail
- [ ] Rapid button clicks (HOME, custom angle) → commands queued/handled gracefully
- [ ] Long rotation (360° in small steps) → timeout extended appropriately
- [ ] Motor at physical limit → Arduino returns status, UI handles gracefully

---

## File Structure

```
mainwindow.py
├── Ui class (main PyQt5 application)
├── ReaderWorker (background data capture thread)
├── load_file() — Reconstruct to both 1D and 2D
├── load_decoder_matrix() — Load 16×16 decoder
├── load_decoder_matrix_1d() — Load 128 decoder
├── build_image() — Display with normalization
└── change_scales() — Manual scale adjustment

tools/fpga_control.py
├── FPGAControl class
├── capture_data() — Read from DDC264
├── convert_adc() — Physical unit conversion
└── reconnect/reinit/reopen() — Recovery methods

decoder_matrix.txt — 16 lines × 16 channels
decoder_matrix_sample1.txt — 128 lines × 1 channel
```

---

## Future Work / Known Limitations

1. **Windows-only** (requires `DDC264EVM_IO.dll`)
2. **No Linux/macOS support** without alternative USB driver
3. Could optimize 1D tiling with numpy operations instead of pure Python reconstruction
4. Consider caching decoded images to speed up mode switching

---

## Recent changes

- **27 Feb 2026**: 1‑D detector mode now overrides the pixel‑size fields to
  1.56 µm during normalization and restores the user's previous values when
  switching back to 2‑D.  The toggle handler also rebuilds the image and
  updates layout immediately, so the new size is used without extra clicks.

## Quick Reference: Adding a New Normalization Mode

1. Add option to `useNormalization` ComboBox in `__init__`
2. Add `elif norm_mode == "your_mode":` block in `build_image()`
3. Handle both 1D and 2D:
   ```python
   if is_1d:
       # 1D logic: use self.image_data_1d, self.open_beam_data_1d, etc.
       temp = (...).reshape(-1, 1)
       left_image = np.tile(temp, (1, 128))
   else:
       # 2D logic: use self.image_data, self.open_beam_data, etc.
       left_image = ...
   ```
4. Call `self.img_item.setImage(left_image)` followed by `self.image_view.autoRange()`
5. Test mode switching and window resizing

