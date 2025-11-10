import serial, json, sys, time


class RotationalController:
    def __init__(self, port="COM4", full_rev=25600):
        # serial timeout controls read() blocking behavior; individual reads will use short polling
        self.arduino = serial.Serial(port=port, baudrate=115200, timeout=0.1)
        time.sleep(2)  # Wait for Arduino reset
        self.arduino.reset_input_buffer()  # Clear any startup messages
        self.arduino.reset_output_buffer()
        self.full_rev = full_rev

    def _read_json_line(self, timeout=5.0, poll=0.05):
        """Read lines from serial until a non-empty line is received and parsed as JSON.

        Returns tuple (parsed_json, raw_bytes). Raises TimeoutError if nothing valid received.
        """
        start = time.time()
        last_raw = b""
        while time.time() - start < timeout:
            try:
                raw = self.arduino.read_until(b"\n")
            except Exception as e:
                # pass through serial errors
                raise
            if not raw:
                time.sleep(poll)
                continue
            last_raw = raw
            text = raw.decode(errors="ignore").strip()
            if not text:
                time.sleep(poll)
                continue
            try:
                return json.loads(text), raw
            except json.JSONDecodeError:
                # Received data but not JSON yet; keep waiting (Arduino may send other lines)
                # small sleep to avoid busy loop
                time.sleep(poll)
                continue

        raise TimeoutError(f"Timeout waiting for JSON line. Last raw: {repr(last_raw)}")

    def _rotate(self, steps):
        # ensure steps is integer (Arduino likely expects integer steps)
        try:
            steps = int(round(steps))
        except Exception:
            raise ValueError(f"Invalid step count computed: {steps}")

        command = json.dumps({"command": "go", "direction": "CW" if steps > 0 else "CCW", "steps": abs(steps)})

        # clear buffers and send command
        try:
            self.arduino.flush()
        except Exception:
            pass
        try:
            self.arduino.reset_input_buffer()
        except Exception:
            pass

        self.arduino.write((command + "\r\n").encode("utf-8"))
        try:
            self.arduino.flush()
        except Exception:
            pass

        # wait for outgoing to drain
        timeout_start = time.time()
        while getattr(self.arduino, "out_waiting", 0) != 0 and time.time() - timeout_start < 2.0:
            time.sleep(0.01)

        # read first JSON response (expected BUSY / Move started)
        try:
            resp1, raw1 = self._read_json_line(timeout=5.0)
        except Exception as e:
            print("Exception in rotation controller (reading start):", e, "| Raw:", repr(getattr(e, 'args', [None])[0] if isinstance(e, TimeoutError) else None), file=sys.stderr)
            raise

        if resp1.get("status") != "BUSY" or resp1.get("msg") != "Move started":
            raise ConnectionError(f"Not expected output. Expected BUSY/Move started but got {resp1} (raw={repr(raw1)})")

        # read completion response
        try:
            resp2, raw2 = self._read_json_line(timeout=30.0)
        except Exception as e:
            print("Exception in rotation controller (reading completion):", e, file=sys.stderr)
            raise

        if resp2.get("status") != "OK" or resp2.get("msg") != "Move complete":
            raise ConnectionError(f"Not expected output. Expected OK/Move complete but got {resp2} (raw={repr(raw2)})")

    def rotate(self, angle_deg=360):
        steps = angle_deg / 360 * self.full_rev
        self._rotate(steps)
