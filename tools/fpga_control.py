import os
import ctypes
import subprocess
import shlex
import time


class FPGAControl:
    adc_ranges = {"12.5": (0, 0), "50.0": (0, 1), "100.0": (1, 0), "150.0": (1, 1)}
    bit_rates = {16: 0, 20: 1}

    int16_t = ctypes.c_int16
    int32_t = ctypes.c_int32
    uint32_t = ctypes.c_uint32
    BYTE = ctypes.c_ubyte
    WINAPI = ctypes.WINFUNCTYPE
    LONG = ctypes.c_long
    DOUBLE = ctypes.c_double
    INT = ctypes.c_int

    def __init__(
        self,
        CONV_LOW_INT,
        CONV_HIGH_INT,
        CONV_CONFIG,
        CLK_HIGH,
        CLK_LOW,
        DDC_CLK_CONFIG,
        CHANNEL_COUNT,
        NDVALID_IGNORE,
        NDVALID_READ,
        DCLK_HIGH,
        DCLK_LOW,
        DCLK_CONFIG,
        DCLKWait,
        HARDWARE_TRIGGER,
        CLK_CFG_HI,
        CLK_CFG_LO,
        ADC_RANGE,
        BIT_RATE,
    ):
        self.regsSize = 255
        self.CONV_LOW_INT = CONV_LOW_INT
        self.CONV_HIGH_INT = CONV_HIGH_INT
        self.CONV_CONFIG = CONV_CONFIG
        self.CLC_HIGH = CLK_HIGH
        self.CLC_LOW = CLK_LOW
        self.DDC_CLK_CONFIG = DDC_CLK_CONFIG
        self.CHANNEL_COUNT = CHANNEL_COUNT
        self.NDVALID_IGNORE = NDVALID_IGNORE
        self.NDVALID_READ = NDVALID_READ
        self.DCLK_HIGH = DCLK_HIGH
        self.DCLK_LOW = DCLK_LOW
        self.DCLK_CONFIG = DCLK_CONFIG
        self.DCLK_WAIT_MCLK = DCLKWait
        self.HADWARE_TRIGGER = HARDWARE_TRIGGER
        self.CLK_CFG_HI = CLK_CFG_HI
        self.CLK_CFG_LO = CLK_CFG_LO
        self.CONV_WAIT_LOW = 1550
        self.CONV_WAIT_HIGH = 1550
        self.CLKDELAY_AROUND_CONV = 0

        self.DDCbit13 = 0
        self.DDCbit10 = self.adc_ranges[ADC_RANGE][0]
        self.DDCbit9 = self.adc_ranges[ADC_RANGE][1]
        self.DDCbit8 = self.bit_rates[BIT_RATE]

        self.DDCbit7 = 0
        self.DDCbit4 = 0
        self.DDCbit0 = 0

        self.CFGLOW = (self.DDCbit7 << 7) + (self.DDCbit4 << 4) + self.DDCbit0
        self.CFGHIGH = (
            (self.DDCbit13 << 5)
            + (self.DDCbit10 << 2)
            + (self.DDCbit9 << 1)
            + self.DDCbit8
        )

        self.RegsIn = (self.INT * self.regsSize)()
        self.RegsOut = (self.INT * self.regsSize)()
        self.RegsEnable = (self.INT * self.regsSize)()

        dll_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), "DDC264EVM_IO.dll"
        )

        # store DLL path for potential reopen/reload operations
        self.dll_path = dll_path

        self.dll = ctypes.CDLL(self.dll_path)
        self.USBdev = self.INT(0)

        self.dll.dllID.argtypes = [ctypes.c_char_p, ctypes.c_int]
        self.dll.dllID.restype = None

        self.dll.dllCprght.argtypes = [ctypes.c_char_p, ctypes.c_int]
        self.dll.dllCprght.restype = None

        self.dll.EVM_RegDataOut.argtypes = [
            ctypes.POINTER(self.INT),
            ctypes.POINTER(self.INT),
            ctypes.POINTER(self.INT),
        ]
        self.dll.EVM_RegDataOut.restype = self.INT

        self.dll.EVM_ResetDDC.argtypes = [ctypes.POINTER(self.INT)]
        self.dll.EVM_ResetDDC.restype = ctypes.c_bool

        self.dll.EVM_ClearTriggers.argtypes = [ctypes.POINTER(self.INT)]
        self.dll.EVM_ClearTriggers.restype = ctypes.c_bool

        self.dll.EVM_DataSequence.argtypes = [
            ctypes.POINTER(self.INT),
            ctypes.POINTER(self.BYTE),
            ctypes.POINTER(self.BYTE),
        ]
        self.dll.EVM_DataSequence.restype = ctypes.c_bool

        self.dll.EVM_RegsTransfer.argtypes = [
            ctypes.POINTER(self.INT),
            ctypes.POINTER(self.INT),
            ctypes.POINTER(self.INT),
            ctypes.POINTER(self.INT),
        ]
        self.dll.EVM_RegsTransfer.restype = ctypes.c_long

        self.dll.EVM_RegNameTable.argtypes = [
            ctypes.c_int,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        self.dll.EVM_RegNameTable.restype = ctypes.c_int

        self.dll.EVM_DataCap.argtypes = [
            ctypes.POINTER(self.INT),
            self.INT,
            self.INT,
            ctypes.POINTER(self.INT),
            ctypes.POINTER(self.INT),
        ]
        self.dll.EVM_DataCap.restype = ctypes.c_long

        self.dll.EVM_WriteCFGFast.argtypes = [
            ctypes.POINTER(self.INT),
            ctypes.POINTER(self.BYTE),
            ctypes.POINTER(self.BYTE),
            ctypes.POINTER(self.INT),
        ]
        self.dll.EVM_WriteCFGFast.restype = ctypes.c_int

    def _bind_functions(self):
        """(Re)bind DLL function prototypes. Call after loading a new CDLL object."""
        # This mirrors the prototypes set in __init__ so they can be rebound after reload.
        try:
            self.dll.dllID.argtypes = [ctypes.c_char_p, ctypes.c_int]
            self.dll.dllID.restype = None

            self.dll.dllCprght.argtypes = [ctypes.c_char_p, ctypes.c_int]
            self.dll.dllCprght.restype = None

            self.dll.EVM_RegDataOut.argtypes = [
                ctypes.POINTER(self.INT),
                ctypes.POINTER(self.INT),
                ctypes.POINTER(self.INT),
            ]
            self.dll.EVM_RegDataOut.restype = self.INT

            self.dll.EVM_ResetDDC.argtypes = [ctypes.POINTER(self.INT)]
            self.dll.EVM_ResetDDC.restype = ctypes.c_bool

            self.dll.EVM_ClearTriggers.argtypes = [ctypes.POINTER(self.INT)]
            self.dll.EVM_ClearTriggers.restype = ctypes.c_bool

            self.dll.EVM_DataSequence.argtypes = [
                ctypes.POINTER(self.INT),
                ctypes.POINTER(self.BYTE),
                ctypes.POINTER(self.BYTE),
            ]
            self.dll.EVM_DataSequence.restype = ctypes.c_bool

            self.dll.EVM_RegsTransfer.argtypes = [
                ctypes.POINTER(self.INT),
                ctypes.POINTER(self.INT),
                ctypes.POINTER(self.INT),
                ctypes.POINTER(self.INT),
            ]
            self.dll.EVM_RegsTransfer.restype = ctypes.c_long

            self.dll.EVM_RegNameTable.argtypes = [
                ctypes.c_int,
                ctypes.c_char_p,
                ctypes.c_int,
            ]
            self.dll.EVM_RegNameTable.restype = ctypes.c_int

            self.dll.EVM_DataCap.argtypes = [
                ctypes.POINTER(self.INT),
                self.INT,
                self.INT,
                ctypes.POINTER(self.INT),
                ctypes.POINTER(self.INT),
            ]
            self.dll.EVM_DataCap.restype = ctypes.c_long

            self.dll.EVM_WriteCFGFast.argtypes = [
                ctypes.POINTER(self.INT),
                ctypes.POINTER(self.BYTE),
                ctypes.POINTER(self.BYTE),
                ctypes.POINTER(self.INT),
            ]
            self.dll.EVM_WriteCFGFast.restype = ctypes.c_int
        except Exception:
            # If rebind fails, do not raise here; callers will see failures when calling.
            pass

    def reset_regs(self):
        for i in range(self.regsSize):
            self.RegsEnable[i] = 0

    def set_reg_in(self, reg, val):
        self.RegsIn[reg] = val & 0xFF
        self.RegsEnable[reg] = 1

    def set_regs(self):
        if self.CHANNEL_COUNT <= 1:
            channel_value = 0
        elif self.CHANNEL_COUNT <= 2:
            channel_value = 1
        elif self.CHANNEL_COUNT <= 4:
            channel_value = 2
        elif self.CHANNEL_COUNT <= 8:
            channel_value = 3
        elif self.CHANNEL_COUNT <= 16:
            channel_value = 4
        elif self.CHANNEL_COUNT <= 32:
            channel_value = 5
        elif self.CHANNEL_COUNT <= 64:
            channel_value = 6
        elif self.CHANNEL_COUNT <= 128:
            channel_value = 7
        elif self.CHANNEL_COUNT <= 256:
            channel_value = 8
        else:
            raise ValueError("Invalid channel count")

        self.CHANNEL_COUNT = int(2**channel_value)

        self.set_reg_in(0x01, (self.CONV_LOW_INT - 1) >> 16)
        self.set_reg_in(0x02, (self.CONV_LOW_INT - 1) >> 8)
        self.set_reg_in(0x03, (self.CONV_LOW_INT - 1))
        self.set_reg_in(0x04, (self.CONV_HIGH_INT - 1) >> 16)
        self.set_reg_in(0x05, (self.CONV_HIGH_INT - 1) >> 8)
        self.set_reg_in(0x06, (self.CONV_HIGH_INT - 1))

        self.set_reg_in(0x07, (self.CLC_HIGH << 4) | (self.CLC_LOW & 0x0F))
        self.set_reg_in(0x08, self.DDC_CLK_CONFIG)

        self.FORMAT = self.CFGHIGH & 1

        self.set_reg_in(0x09, (self.FORMAT << 4) | (channel_value & 0x0F))

        self.set_reg_in(0x0A, (self.DCLK_HIGH << 4) | (self.DCLK_LOW & 0x0F))
        self.set_reg_in(0x0B, self.DCLK_CONFIG)

        self.set_reg_in(0x0C, self.NDVALID_IGNORE)
        self.set_reg_in(0x0D, self.NDVALID_READ)
        self.set_reg_in(0x0E, self.NDVALID_READ >> 8)
        self.set_reg_in(0x0F, self.NDVALID_READ >> 16)

        self.set_reg_in(0x13, self.DCLK_WAIT_MCLK >> 8)
        self.set_reg_in(0x14, self.DCLK_WAIT_MCLK)

        self.set_reg_in(0x1F, self.FORMAT)

        self.set_reg_in(0x20, (self.CLK_CFG_HI << 4) | (self.CLK_CFG_LO & 0x0F))

        self.set_reg_in(0x57, self.CONV_CONFIG)

        self.set_reg_in(0x51, self.CONV_WAIT_LOW >> 8)
        self.set_reg_in(0x52, self.CONV_WAIT_LOW)
        self.set_reg_in(0x53, self.CONV_WAIT_HIGH >> 8)
        self.set_reg_in(0x54, self.CONV_WAIT_HIGH)
        self.set_reg_in(0xEB, self.CLKDELAY_AROUND_CONV)

    def get_dll_version(self):
        buf = ctypes.create_string_buffer(128)
        self.dll.dllID(buf, 128)
        return buf.value.decode()

    def get_dll_copyright(self):
        buf = ctypes.create_string_buffer(128)
        self.dll.dllCprght(buf, 128)
        return buf.value.decode()

    def get_register_name(self, reg_num):
        buf = ctypes.create_string_buffer(64)
        length = self.dll.EVM_RegNameTable(reg_num, buf, 64)
        return buf.value.decode() if length > 0 else "UNKNOWN"

    def write_register(self, usb_id, reg, data):
        usb = self.INT(usb_id)
        reg = self.INT(reg)
        data = self.INT(data)
        return self.dll.EVM_RegDataOut(
            ctypes.byref(usb), ctypes.byref(reg), ctypes.byref(data)
        )

    def reset_ddc(self):
        return self.dll.EVM_ResetDDC(ctypes.byref(self.USBdev))

    def clear_triggers(self):
        return self.dll.EVM_ClearTriggers(ctypes.byref(self.USBdev))

    def configure_sequence(self, cfg_high, cfg_low):
        high = self.BYTE(cfg_high)
        low = self.BYTE(cfg_low)
        return self.dll.EVM_DataSequence(
            ctypes.byref(self.USBdev), ctypes.byref(high), ctypes.byref(low)
        )

    def write_cfg_fast(self, cfg_high, cfg_low):
        high = self.BYTE(cfg_high)
        low = self.BYTE(cfg_low)
        verify_results = (self.INT * 3)()

        cfg_result = self.dll.EVM_WriteCFGFast(
            ctypes.byref(self.USBdev),
            ctypes.byref(high),
            ctypes.byref(low),
            verify_results,
        )

        return cfg_result, verify_results[0], verify_results[1], verify_results[2]

    def transfer_registers(self, RegsIn, RegEnable):
        regs_in = (self.INT * 255)(*RegsIn[:255])
        regs_en = (self.INT * 255)(*RegEnable[:255])
        regs_out = (self.INT * 255)()
        rc = self.dll.EVM_RegsTransfer(
            ctypes.byref(self.USBdev), regs_in, regs_en, regs_out
        )
        return rc, list(regs_out)

    def capture_data(self, channels, reads, AorBfirst=0):
        total_samples = channels * reads
        data_arr = (self.INT * total_samples)()
        aorbfirst_c = self.INT(AorBfirst)
        rc = self.dll.EVM_DataCap(
            ctypes.byref(self.USBdev),
            self.INT(channels),
            self.INT(reads),
            data_arr,
            ctypes.byref(aorbfirst_c),
        )
        return rc, list(data_arr), aorbfirst_c.value

    def show_registers(self):
        try:
            rc, _ = self.transfer_registers(list(self.RegsIn), list(self.RegsEnable))
        except Exception as e:
            raise RuntimeError(f"Failed to transfer registers: {str(e)}")

        if rc != 0:
            raise RuntimeError(f"Register transfer failed with code: {rc}")

        for i in range(1, self.regsSize):
            _ = self.get_register_name(i)

        return True

    def prepare(self):
        res_out = self.reset_ddc()
        if not res_out:
            raise RuntimeError("Failed to reset DDC")

        res_out = self.clear_triggers()
        if not res_out:
            raise RuntimeError("Failed to clear triggers")

        cfg_result, _, _, _ = self.write_cfg_fast(self.CFGHIGH, self.CFGLOW)

        if cfg_result != 0:
            raise RuntimeError(f"Failed to write CFG fast, result: {cfg_result}")

        return True

    def refresh(self):
        self.reset_ddc()
        self.clear_triggers()
        if not self.show_registers():
            raise RuntimeError("Failed to read registers from FPGA")

        self.prepare()

        self.reset_regs()
        self.set_regs()

        rc, _ = self.transfer_registers(list(self.RegsIn), list(self.RegsEnable))
        if rc != 0:
            raise RuntimeError(f"Error in register transfer, result: {rc}")

        return True

    def get_data(self, file_path, file_index):
        filename = f"{file_path}_{file_index+1}.txt"

        channels = self.CHANNEL_COUNT
        reads = self.NDVALID_READ
        rc, all_data, all_data_aorbfirst = self.capture_data(channels, reads)
        if rc != 0:
            raise RuntimeError(f"Error in data capture: {rc}")

        os.makedirs(os.path.dirname(filename), exist_ok=True)

        with open(filename, "w") as dataFile:
            samples_per_channel = reads // 2

            bit_rate = next(k for k, v in self.bit_rates.items() if v == self.DDCbit8)

            # Write A integrator samples. The device may return A-first or B-first sequencing
            # indicated by all_data_aorbfirst (0 = A-first, 1 = B-first).
            for ch in range(channels - 1, -1, -1):
                prefix = "0" if (ch + 1) < 10 else ""

                for sample_idx in range(samples_per_channel):
                    if all_data_aorbfirst == 0:
                        # A block is first
                        data_idx_a = sample_idx * channels + ch
                    else:
                        # B block is first, so A samples are in the second half
                        data_idx_a = (sample_idx + samples_per_channel) * channels + ch

                    if data_idx_a < len(all_data):
                        dataFile.write(
                            f"{prefix}{ch+1}A, {sample_idx}, {all_data[data_idx_a]}, {0}, {0}, {bit_rate}\n"
                        )

            # Write B integrator samples, using complementary indexing
            for ch in range(channels - 1, -1, -1):
                prefix = "0" if (ch + 1) < 10 else ""

                for sample_idx in range(samples_per_channel):
                    if all_data_aorbfirst == 0:
                        # B block is second
                        data_idx_b = (sample_idx + samples_per_channel) * channels + ch
                    else:
                        # B block is first
                        data_idx_b = sample_idx * channels + ch

                    if data_idx_b < len(all_data):
                        dataFile.write(
                            f"{prefix}{ch+1}B, {sample_idx}, {all_data[data_idx_b]}, {0}, {0}, {bit_rate}\n"
                        )

            # Basic validation: ensure that we wrote the expected number of samples per channel/integrator
            # (useful as a sanity check; don't raise here to avoid breaking callers)
            try:
                expected_per = samples_per_channel
                # Rewind and count lines per key
                dataFile.flush()
                # file is closed after with-block; do a quick check by reopening
                written_counts = {}
                with open(filename, "r") as checkf:
                    for ln in checkf:
                        key = ln.split(",")[0].strip()
                        written_counts[key] = written_counts.get(key, 0) + 1
                # Optionally log a warning for missing / incomplete channels
                incomplete = [k for k, v in written_counts.items() if v != expected_per]
                if incomplete:
                    # Not raising, but return a helpful message
                    return f"File {filename} saved (but incomplete counts for some channels)"
            except Exception:
                # If validation fails, silently continue and return success string as before
                pass

        return f"File {filename} was saved successfully"

    def convert_adc(self, value):
        power = next(k for k, v in self.bit_rates.items() if v == self.DDCbit8)
        adc_range = 1e-12 * float(
            next(
                k
                for k, v in self.adc_ranges.items()
                if v == (self.DDCbit10, self.DDCbit9)
            )
        )
        return value / (2**power - 1) * adc_range

    # --- Recovery / reconnect helpers ---
    def reopen(self):
        """Attempt to reload the underlying DLL and rebind prototypes.

        This can help recover from low-level USB/DLL state corruption.
        Returns True on success, False on failure.
        """
        try:
            # reload CDLL (this may unblock some driver state)
            self.dll = ctypes.CDLL(self.dll_path)
            # rebind function prototypes
            self._bind_functions()
            return True
        except Exception:
            return False

    def reconnect(self):
        """Higher-level reconnect: attempt DDC reset, clear triggers and re-prepare device.

        Returns True if the sequence completed without exception, False otherwise.
        """
        try:
            try:
                self.reset_ddc()
            except Exception:
                # ignore and continue
                pass
            try:
                self.clear_triggers()
            except Exception:
                pass
            # attempt prepare (write cfg, check results)
            self.prepare()
            return True
        except Exception:
            return False

    def reinit(self):
        """Attempt a fuller reinitialization: refresh registers and transfer configured registers.

        Use this when a simple reset/clear doesn't help. Returns True on success.
        """
        try:
            self.refresh()
            return True
        except Exception:
            # as a fallback try reopen + refresh
            try:
                if self.reopen():
                    self.refresh()
                    return True
            except Exception:
                pass
        return False

    def usb_reset(self):
        """Alias for reopening the DLL / reloading driver state.

        Some recovery routines call this name; it performs the same action as reopen().
        """
        # Prefer a Windows USB re-enumeration if possible (uses VID/PID of device)
        try:
            # default VID/PID for DDC264EVM
            return self.usb_reenumerate(vid="04B4", pid="1004")
        except Exception:
            return self.reopen()

    def usb_reenumerate(self, vid="04B4", pid="1004", timeout=5.0, attempts=2):
        """Attempt to re-enumerate (disable/enable) the USB device using PowerShell.

        This uses PowerShell's Disable-PnpDevice / Enable-PnpDevice to disable and
        re-enable the device matching the supplied VID/PID. Requires administrator
        privileges; returns True if the device was successfully toggled.

        vid and pid should be hex strings (case-insensitive), e.g. '04B4' and '1004'.
        """
        # Build PowerShell command to find device instance IDs matching VID/PID
        # We'll disable then enable; use -Confirm:$false to avoid prompts.
        match = f"*VID_{vid.upper()}*PID_{pid.upper()}*"
        # PowerShell command to disable matching devices
        disable_cmd = (
            f"Get-PnpDevice -PresentOnly | Where-Object {{$_.InstanceId -like '{match}'}} | "
            "Disable-PnpDevice -Confirm:$false"
        )
        enable_cmd = (
            f"Get-PnpDevice -PresentOnly | Where-Object {{$_.InstanceId -like '{match}'}} | "
            "Enable-PnpDevice -Confirm:$false"
        )

        for attempt in range(attempts):
            try:
                # Disable
                p1 = subprocess.run([
                    "powershell",
                    "-NoProfile",
                    "-NonInteractive",
                    "-Command",
                    disable_cmd,
                ], capture_output=True, timeout=timeout)
                # Give the OS a moment
                time.sleep(0.5)
                # Enable
                p2 = subprocess.run([
                    "powershell",
                    "-NoProfile",
                    "-NonInteractive",
                    "-Command",
                    enable_cmd,
                ], capture_output=True, timeout=timeout)

                stdout = (p1.stdout or b"") + (p2.stdout or b"")
                stderr = (p1.stderr or b"") + (p2.stderr or b"")

                # If PowerShell returned non-zero, it may still have worked for some devices.
                if p1.returncode == 0 and p2.returncode == 0:
                    return True
                # If not zero, try again a limited number of times
                self._log_ps(cmds=(disable_cmd, enable_cmd), out=stdout, err=stderr)
            except subprocess.TimeoutExpired:
                # try again
                continue
            except Exception:
                continue

        return False

    def _log_ps(self, cmds=None, out=b"", err=b""):
        try:
            # Best-effort debug output; don't raise on failure
            msg = f"PowerShell cmds: {cmds}\nstdout: {out.decode(errors='ignore')}\nstderr: {err.decode(errors='ignore')}"
            # If the UI isn't available, just print
            try:
                print(msg)
            except Exception:
                pass
        except Exception:
            pass

    def restore_registers(self):
        """Rewrite register configuration to the device (helpful after reset).

        This runs through the same sequence as `refresh` which reads and writes
        registers and prepares the device. Returns True on success.
        """
        try:
            return self.refresh()
        except Exception:
            # try a safer manual restore path
            try:
                self.reset_regs()
                self.set_regs()
                rc, _ = self.transfer_registers(list(self.RegsIn), list(self.RegsEnable))
                return rc == 0
            except Exception:
                return False
