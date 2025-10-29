from PyQt5 import uic
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QThread, Qt, QTimer
from PyQt5.QtWidgets import QMainWindow, QFileDialog, QVBoxLayout, QMessageBox, QPushButton, QCheckBox, QDialog, QLabel, QGridLayout
from tools import FPGAControl
import pyqtgraph as pg
import numpy as np
import os
import time
import datetime
# Optional serial support for Arduino relay switching. Import is guarded so
# the app continues to work if pyserial is not installed in the environment.
try:
    import serial
    from serial.tools import list_ports as serial_list_ports
except Exception:
    serial = None
    serial_list_ports = None


class ReaderWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)
    status = pyqtSignal(str)
    # request the user to power-cycle the device (UI should prompt the user)
    request_power_cycle = pyqtSignal()

    def __init__(
        self, fpga, folder_path, file_name, numFiles, max_retries=1, poll_timeout=1.0
    ):
        super().__init__()
        self.fpga = fpga
        self.folder_path = folder_path
        self.file_name = file_name
        self.numFiles = numFiles
        self.readFilePath = None
        self.max_retries = max_retries
        self.poll_timeout = poll_timeout
        self._powercycle_ack = False
        self._waiting_for_powercycle = False
        self._stop_requested = False

    @pyqtSlot(object)
    def set_fpga(self, fpga):
        """Called by the UI after a manual power-cycle to update the worker's FPGA object.

        Setting this also acts as acknowledgement for the worker to continue.
        """
        try:
            self.fpga = fpga
            self._powercycle_ack = True
            self._waiting_for_powercycle = False
            self.status.emit("Worker: received new FPGA reference after power cycle")
        except Exception as e:
            self.status.emit(f"Worker: failed to update FPGA reference: {e}")

    @pyqtSlot()
    def stop(self):
        """Request the worker to stop after finishing the current file."""
        try:
            self._stop_requested = True
            self.status.emit("Worker: stop requested; will stop after current file")
        except Exception:
            pass

    @pyqtSlot()
    def run(self):
        # Ensure destination folder exists
        try:
            if not os.path.isdir(self.folder_path):
                os.makedirs(self.folder_path, exist_ok=True)
        except Exception as e:
            self.status.emit(f"Cannot create target folder: {e}")
            self.finished.emit()
            return

        # --- new: inspect available methods on the FPGA object and emit them ---
        try:
            fpga_methods = sorted(
                [
                    m
                    for m in dir(self.fpga)
                    if callable(getattr(self.fpga, m)) and not m.startswith("_")
                ]
            )
            # emit a short summary (trim if long)
            meth_list = ", ".join(fpga_methods[:20])
            #if len(fpga_methods) > 20:
            #    meth_list += ", ..."
            #self.status.emit(f"FPGA methods: {meth_list}")
        except Exception:
            # non-fatal; continue
            pass

        # helper to list numeric indices in folder
        def _list_indices():
            idxs = []
            try:
                for fname in os.listdir(self.folder_path):
                    if fname.startswith(f"{self.file_name}_") and fname.endswith(".txt"):
                        try:
                            idx = int(fname[len(f"{self.file_name}_") : -4])
                            idxs.append(idx)
                        except ValueError:
                            continue
            except Exception:
                pass
            return idxs

        # --- new: generic recovery routine trying many common method names ---
        def _attempt_recovery():
            # ordered list of candidate recovery method names (common variants)
            candidates = [
                # reload DLL / USB helpers
                #"usb_reset",
                "reopen",
                #"reopen_device",
                #"reconnect_device",
                # fallback register restore (rewrites registers after a reset)
                "restore_registers",
                # higher-level reconnects first
                "reconnect",
                "reinit",
                # basic device helpers
                "reset_ddc",
                "clear_triggers",
                "refresh",
                # generic names that some drivers may implement
                #"reopen",
               # "open",
                #"close",
                #"init",
                #"connect",
                #"disconnect",
            ]
            performed = []
            for name in candidates:
                if hasattr(self.fpga, name):
                    try:
                        fn = getattr(self.fpga, name)
                        # call without arguments; many recovery functions are no-arg
                        fn()
                        performed.append(name)
                        self.status.emit(f"Attempted FPGA.{name}()")
                        # small pause after a recovery call
                        time.sleep(0.1)
                        # some methods may be enough; continue trying others only if needed
                    except Exception as e:
                        # continue trying other methods
                        self.status.emit(f"FPGA.{name}() raised: {e}")
            return performed

        existing_indices = _list_indices()
        start_index = max(existing_indices) if existing_indices else 0
        last_saved_index = start_index

        try:
            for i in range(self.numFiles):
                # Recompute the highest existing index on disk before each
                # capture attempt. This makes the worker robust to manual
                # power-cycles or external files appearing while the worker
                # was paused so we don't accidentally skip or repeat numbers.
                try:
                    cur_idxs = _list_indices()
                    current_max = max(cur_idxs) if cur_idxs else 0
                except Exception:
                    current_max = start_index
                desired_index = current_max + 1
                success = False
                severe_error = False
                attempt = 0
                while attempt < self.max_retries and not success:
                    attempt += 1
                    before_set = set(_list_indices())
                    error_msg = None
                    ret = None
                    poll_deadline = time.time() + self.poll_timeout
                    try:
                        self.status.emit(
                            f"Starting transfer for file index {desired_index} (attempt {attempt})"
                        )
                        ret = self.fpga.get_data(
                            os.path.join(self.folder_path, self.file_name), desired_index - 1
                        )
                        # If get_data returns a negative code treat it as transient but log it.
                        if isinstance(ret, int) and ret < 0:
                            error_msg = f"get_data returned error code {ret}"
                            self.status.emit(error_msg)
                            # If the driver returned -5 or -4 consider this a
                            # severe error that requires manual power-cycle.
                            if ret in (-5, -4):
                                severe_error = True
                                self.status.emit(
                                    f"Error code {ret} requires manual power-cycle. Requesting manual power-cycle."
                                )
                                # emit a signal to ask the UI to prompt the user
                                self._powercycle_ack = False
                                self._waiting_for_powercycle = True
                                try:
                                    self.request_power_cycle.emit()
                                    # wait for UI to update FPGA reference (ack) up to a timeout
                                    wait_deadline = time.time() + 500.0
                                    while time.time() < wait_deadline and not self._powercycle_ack:
                                        time.sleep(0.5)
                                    if not self._powercycle_ack:
                                        raise RuntimeError(
                                            f"Power-cycle requested for error {ret} but no power-cycle acknowledged"
                                        )
                                    else:
                                        # reset attempts for this file and try again
                                        attempt = 0
                                        continue
                                finally:
                                    self._waiting_for_powercycle = False
                            # extend poll deadline to allow file to appear despite the code
                            poll_deadline = time.time() + (self.poll_timeout * 1.8)
                    except Exception as e:
                        error_text = str(e)
                        self.status.emit(f"Transfer attempt {attempt} raised: {error_text}")
                        # If the exception message indicates a severe error (-4/-5),
                        # treat it the same as a returned -4/-5: request manual power-cycle.
                        if "-5" in error_text or "-4" in error_text:
                            severe_error = True
                            self.status.emit(
                                f"Detected severe error in exception ({error_text}): requesting manual power-cycle."
                            )
                        # If a severe error was previously detected, don't try
                        # the generic recovery routines (they don't help for -4/-5).
                        if severe_error:
                            self.status.emit(
                                "Severe error detected previously: requesting manual power-cycle without attempting recovery methods."
                            )
                            # request manual power-cycle and wait for ack
                            self._powercycle_ack = False
                            self._waiting_for_powercycle = True
                            try:
                                self.request_power_cycle.emit()
                                wait_deadline = time.time() + 500.0
                                while time.time() < wait_deadline and not self._powercycle_ack:
                                    time.sleep(0.5)
                                if not self._powercycle_ack:
                                    raise RuntimeError(
                                        "Power-cycle requested but no power-cycle acknowledged"
                                    )
                                else:
                                    attempt = 0
                                    continue
                            finally:
                                self._waiting_for_powercycle = False
                        else:
                            # try broad recovery routines (this also tells you which methods exist)
                            recovered = _attempt_recovery()
                            if recovered:
                                self.status.emit(f"Recovery methods attempted: {', '.join(recovered)}")
                            else:
                                self.status.emit("No recovery methods available on FPGA instance")
                        # backoff before retry
                        time.sleep(0.1)
                        continue

                    # Poll for new file(s) by comparing before/after directory listings.
                    while time.time() < poll_deadline:
                        after_set = set(_list_indices())
                        new = sorted(list(after_set - before_set))
                        if new:
                            last_saved_index = new[0]
                            success = True
                            break
                        time.sleep(0.15)

                    if not success:
                        # No new file detected: attempt lightweight recovery and log available methods
                        self.status.emit(
                            f"No new file detected for index {desired_index} after attempt {attempt}"
                        )
                        if severe_error:
                            self.status.emit(
                                "Severe error detected: skipping recovery methods and requesting manual power-cycle."
                            )
                            self._powercycle_ack = False
                            self._waiting_for_powercycle = True
                            try:
                                self.request_power_cycle.emit()
                                wait_deadline = time.time() + 500.0
                                while time.time() < wait_deadline and not self._powercycle_ack:
                                    time.sleep(0.5)
                                if not self._powercycle_ack:
                                    raise RuntimeError(
                                        "Power-cycle requested but no power-cycle acknowledged"
                                    )
                                else:
                                    attempt = 0
                                    continue
                            finally:
                                self._waiting_for_powercycle = False
                        else:
                            recovered = _attempt_recovery()
                            if not recovered:
                                self.status.emit("No recovery methods available to call (post-poll)")
                        time.sleep(0.5)

                if not success:
                    self.numFiles += 1
                    # After all recovery attempts failed, request a manual power-cycle from the user
                    self.status.emit(
                        f"Failed to capture file index {desired_index} after {self.max_retries} attempts. Requesting manual power-cycle."
                    )
                    try:
                        # emit a signal to ask the UI to prompt the user
                        self._powercycle_ack = False
                        self._waiting_for_powercycle = True
                        self.request_power_cycle.emit()
                        # wait for UI to update FPGA reference (ack) up to a timeout
                        wait_deadline = time.time() + 500.0
                        while time.time() < wait_deadline and not self._powercycle_ack:
                            time.sleep(0.5)
                        if not self._powercycle_ack:
                            raise RuntimeError(
                                f"Failed to capture file index {desired_index} after {self.max_retries} attempts and no power-cycle performed"
                            )
                        else:
                            # reset attempts for this file and try again
                            attempt = 0
                            continue
                    finally:
                        self._waiting_for_powercycle = False

                # Confirmed saved
                saved_name = f"{self.file_name}_{last_saved_index}.txt"
                self.status.emit(
                    f"File {last_saved_index} saved successfully as {saved_name}"
                )
                self.progress.emit(i + 1)
                # if user requested stop, break after successfully saving this file
                if getattr(self, "_stop_requested", False):
                    self.status.emit("Worker: stopping after current file due to stop request")
                    break

            self.status.emit("Data read successfully")
            # final readFilePath: use last saved filename
            self.readFilePath = f"{self.file_name}_{last_saved_index}.txt"
        except Exception as e:
            self.status.emit(f"Error during data capture: {str(e)}")
            # try to set a useful readFilePath even on failure
            if last_saved_index > start_index:
                self.readFilePath = f"{self.file_name}_{last_saved_index}.txt"
            else:
                self.readFilePath = ""
        finally:
            self.finished.emit()


class Ui(QMainWindow):
    conv_config = {"Free run": 0, "Low": 2, "High": 3}
    ddc_clk_config = {"Running": 1, "Low": 0}
    dclk_config = {"Running": 1, "Low": 0}
    hardware_trigger = {"Disabled": 0, "Enabled": 1}

    def __init__(self):
        super().__init__()

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("DDC264EVM_UI")

        self.ConvLowInt.setText("320")
        self.ConvHighInt.setText("320")

        self.ConvConfig.addItem("Free run")
        self.ConvConfig.addItem("Low")
        self.ConvConfig.addItem("High")

        self.MCLKFreq.setText("80.0")
        self.MCLKFreq.setDisabled(True)

        self.CLKHigh.setText("3")
        self.CLKLow.setText("3")

        self.DDCCLKConfig.addItem("Running")
        self.DDCCLKConfig.addItem("Low")

        self.Format.addItem("20 bit")
        self.Format.addItem("16 bit")

        self.ChannelCount.addItem("0")
        self.ChannelCount.addItem("16")
        self.ChannelCount.addItem("32")
        self.ChannelCount.addItem("64")
        self.ChannelCount.addItem("128")
        self.ChannelCount.addItem("256")
        self.ChannelCount.addItem("512")
        self.ChannelCount.addItem("1024")
        self.ChannelCount.setCurrentText("256")

        self.nDVALIDIgnore.setText("255")
        self.nDVALIDRead.setText("1024")

        self.DCLKHigh.setText("0")
        self.DCLKLow.setText("0")

        self.DCLKConfig.addItem("Running")
        self.DCLKConfig.addItem("Low")

        self.DCLKWait.setText("1750")

        self.HardwareTrigger.addItem("Disabled")
        self.HardwareTrigger.addItem("Enabled")

        self.CLK_CFGHigh.setText("3")
        self.CLK_CFGLow.setText("3")

        self.ADCrange.addItem("12.5")
        self.ADCrange.addItem("50.0")
        self.ADCrange.addItem("150.0")
        self.ADCrange.addItem("100.0")
        
        
        

        self.edgeLeft.setText("156")
        self.edgeRight.setText("356")

        self.progressBar.setMinimum(0)
        self.progressBar.setValue(0)

        self.progressBar.hide()

        self.nFiles.setText("1")

        self.pixelX.setText("0.36")
        self.pixelY.setText("0.36")
        self.imageLowScale.setText("0")
        self.imageUpperScale.setText("1")
        self.mixLowScale.setText("0")
        self.mixUpperScale.setText("1")

        public_documents = os.path.join(
            os.environ.get("PUBLIC", r"C:\Users\Public"), "Documents"
        )
        self.save_path = public_documents
        if os.path.isdir(public_documents):
            self.saveFolderLabel.setText(public_documents)
        else:
            self.saveFolderLabel.setText(
                os.path.join(os.path.expanduser("~"), "Documents")
            )

        # name of the transfer log file stored inside the selected save folder
        self._transfer_log_name = "ddc_transfer.log"
        # ensure the default log file exists (safe to fail silently)
        try:
            os.makedirs(self.save_path, exist_ok=True)
            open(os.path.join(self.save_path, self._transfer_log_name), "a", encoding="utf-8").close()
        except Exception:
            pass

        self.traceNumber.addItem("--")
        self.traceNumber.addItem("Mean value")
        for letter in ["A", "B"]:
            for i in range(256):
                self.traceNumber.addItem(f"{i+1}{letter}")
        self.graphWidget = pg.PlotWidget()
        layout = QVBoxLayout(self.tracePlot)
        layout.addWidget(self.graphWidget)
        self.graphWidget.setLabel("left", "Charge", units="C")
        self.graphWidget.setLabel("bottom", "Time")
        self.graphWidget.setBackground(None)
        #self.graphWidget.setSymbolSize(15)
        plotItem = self.graphWidget.getPlotItem()
        for axis in ['left', 'bottom']:
            ax = plotItem.getAxis(axis)
            ax.setPen(pg.mkPen('k', width=1))
            ax.setTextPen(pg.mkPen('k'))

        self.imageWidget = pg.GraphicsLayoutWidget()
        # Try to make the GraphicsLayoutWidget background transparent so
        # the containing QWidget's background shows through.
        try:
            # preferred pyqtgraph API
            self.imageWidget.setBackground(None)
        except Exception:
            pass
        try:
            # also set stylesheet fallback for Qt widget transparency
            self.imageWidget.setStyleSheet("background: transparent;")
        except Exception:
            pass
        try:
            # request translucent background attribute (best-effort)
            self.imageWidget.setAttribute(Qt.WA_TranslucentBackground)
        except Exception:
            pass

        image_layout = QVBoxLayout(self.imagePlot)
        image_layout.addWidget(self.imageWidget)

        self.mixWidget = pg.GraphicsLayoutWidget()
        # make mix widget transparent too (same approach as imageWidget)
        try:
            self.mixWidget.setBackground(None)
        except Exception:
            pass
        try:
            self.mixWidget.setStyleSheet("background: transparent;")
        except Exception:
            pass
        try:
            self.mixWidget.setAttribute(Qt.WA_TranslucentBackground)
        except Exception:
            pass

        mix_layout = QVBoxLayout(self.mixPlot)
        mix_layout.addWidget(self.mixWidget)

        self.image_file = ""
        self.open_beam_file = ""
        self.decoderMatrixLabel.setText("decoder_matrix.txt")
        self.decoder_matrix = np.zeros((16, 16), dtype=int)
        self.load_decoder_matrix(self.decoderMatrixLabel.text())

        self.image_view = self.imageWidget.addViewBox()
        self.image_view.setAspectLocked(False)
        self.img_item = pg.ImageItem(np.zeros((16, 16)))
        self.image_view.addItem(self.img_item)
        self.image_view.setRange(self.img_item.boundingRect(), padding=0)
        self.image_view.setMouseEnabled(x=False, y=False)

        # Try to make the underlying ViewBox background transparent as well.
        try:
            # pyqtgraph sometimes provides setBackground on GraphicsLayoutWidget
            # and ViewBox; try both approaches.
            self.image_view.setBackground(None)
        except Exception:
            try:
                self.image_view.setBackgroundColor(None)
            except Exception:
                pass

        cmap = pg.colormap.get("viridis")
        lut = cmap.getLookupTable(0.0, 1.0, 256)
        self.img_item.setLookupTable(lut)

        self.color_bar = pg.ColorBarItem(
            values=(0, 1), colorMap=cmap, interactive=False
        )
        self.color_bar.setImageItem(self.img_item)
        self.imageWidget.addItem(self.color_bar)
        # Ensure colorbar tick/label color is black for readability
        try:
            self.color_bar.axis.setPen(pg.mkPen('k'))
            self.color_bar.axis.setTextPen(pg.mkPen('k'))
        except Exception:
            try:
                # fallback attribute name used by some versions
                self.color_bar.axisItem.setPen(pg.mkPen('k'))
                self.color_bar.axisItem.setTextPen(pg.mkPen('k'))
            except Exception:
                # last-resort: attempt to find any child with setTextPen
                try:
                    for child in getattr(self.color_bar, 'items', lambda: [])():
                        try:
                            child.setTextPen(pg.mkPen('k'))
                        except Exception:
                            pass
                except Exception:
                    pass
        #self.imageWidget.getViewBox().setBackgroundColor(None)
        

        self.mix_view = self.mixWidget.addViewBox()
        self.mix_view.setAspectLocked(False)
        self.mix_img_item = pg.ImageItem(np.zeros((16, 16)))
        self.mix_view.addItem(self.mix_img_item)
        self.mix_view.setRange(self.mix_img_item.boundingRect(), padding=0)
        self.mix_view.setMouseEnabled(x=False, y=False)
        self.mix_img_item.setLookupTable(lut)
        self.mix_color_bar = pg.ColorBarItem(
            values=(0, 1), colorMap=cmap, interactive=False
        )
        self.mix_color_bar.setImageItem(self.mix_img_item)
        self.mixWidget.addItem(self.mix_color_bar)
        try:
            self.mix_color_bar.axis.setPen(pg.mkPen('k'))
            self.mix_color_bar.axis.setTextPen(pg.mkPen('k'))
        except Exception:
            try:
                self.mix_color_bar.axisItem.setPen(pg.mkPen('k'))
                self.mix_color_bar.axisItem.setTextPen(pg.mkPen('k'))
            except Exception:
                try:
                    for child in getattr(self.mix_color_bar, 'items', lambda: [])():
                        try:
                            child.setTextPen(pg.mkPen('k'))
                        except Exception:
                            pass
                except Exception:
                    pass
        try:
            self.mix_view.setBackground(None)
        except Exception:
            try:
                self.mix_view.setBackgroundColor(None)
            except Exception:
                pass

        self.file_data = {}
        self.image_data = np.zeros((16, 16))
        self.dark_current_data = np.zeros((16, 16))
        self.open_beam_data = np.zeros((16, 16))
        self.readFilePath.setText("")

        self.openBeam.setChecked(False)
        self.darkCurrent.setChecked(True)

        self.update_registers(is_startup=True)
        self.refresh_registers(is_startup=True)

        self.getData.clicked.connect(self.record_data)
        self.ConvLowInt.textChanged.connect(self.update_time)
        self.ConvHighInt.textChanged.connect(self.update_time)
        self.readFileButton.clicked.connect(self.load_trace_file)
        self.traceNumber.currentTextChanged.connect(self.plot_trace)
        self.writeRegisters.clicked.connect(self.update_registers)
        self.hardReset.clicked.connect(self.hard_reset)
        self.refresh.clicked.connect(self.refresh_registers)
        self.saveFolder.clicked.connect(lambda: self.save_folder_path())
        self.imageFile.clicked.connect(
            lambda: self.load_file(
                "image_file", self.imageFileLabel, "image_data", True
            )
        )
        self.openBeamFile.clicked.connect(
            lambda: self.load_file(
                "open_beam_file", self.openBeamFileLabel, "open_beam_data", False
            )
        )
        self.decoderMatrix.clicked.connect(self.load_decoder_matrix)
        self.buildImage.clicked.connect(self.build_image)
        self.imageUpperScale.textChanged.connect(self.change_scales)
        self.imageLowScale.textChanged.connect(self.change_scales)
        self.mixUpperScale.textChanged.connect(self.change_scales)
        self.mixLowScale.textChanged.connect(self.change_scales)
        self.darkCurrent.toggled.connect(self.build_image)
        self.openBeam.toggled.connect(self.build_image)
        self.openBeam.clicked.connect(self.build_image)
        self.darkCurrent.clicked.connect(self.build_image)

        self.show()

        # Create Stop button and Auto recover checkbox and insert them into
        # the same layout as the Get Data button so they behave like other
        # controls when the window is resized.
        try:
            self.stopRecording = QPushButton("Stop", self)
            self.stopRecording.clicked.connect(self._on_stop_clicked)
            self.autoRecover = QCheckBox("Auto recover", self)
            self.autoRecover.setChecked(True)

            # Try to find the layout that holds getData and insert both
            parent = None
            try:
                parent = self.getData.parentWidget() or self.getData.parent()
            except Exception:
                parent = None

            layout = None
            try:
                if parent is not None:
                    layout = parent.layout()
            except Exception:
                layout = None

            placed_stop = False
            placed_auto = False

            if layout is not None:
                try:
                    if isinstance(layout, QGridLayout):
                        # find getData position
                        gd_pos = None
                        for r in range(layout.rowCount()):
                            for c in range(layout.columnCount()):
                                item = layout.itemAtPosition(r, c)
                                if item and item.widget() is self.getData:
                                    gd_pos = (r, c)
                                    break
                            if gd_pos:
                                break
                        if gd_pos:
                            r, c = gd_pos
                            # Place Stop to the right if possible
                            try:
                                if layout.itemAtPosition(r, c + 1) is None:
                                    layout.addWidget(self.stopRecording, r, c + 1)
                                else:
                                    # try next column
                                    layout.addWidget(self.stopRecording, r, c + 2)
                                placed_stop = True
                            except Exception:
                                try:
                                    layout.addWidget(self.stopRecording)
                                    placed_stop = True
                                except Exception:
                                    placed_stop = False

                            # Prefer to place autoRecover ABOVE getData
                            try:
                                placed = False
                                if r > 0 and layout.itemAtPosition(r - 1, c) is None:
                                    layout.addWidget(self.autoRecover, r - 1, c)
                                    placed = True
                                elif layout.itemAtPosition(r, c - 1) is None:
                                    layout.addWidget(self.autoRecover, r, c - 1)
                                    placed = True
                                else:
                                    # fallback: try below or next to stop
                                    if layout.itemAtPosition(r + 1, c) is None:
                                        layout.addWidget(self.autoRecover, r + 1, c)
                                        placed = True
                                    elif layout.itemAtPosition(r, c + 2) is None:
                                        layout.addWidget(self.autoRecover, r, c + 2)
                                        placed = True
                                if placed:
                                    placed_auto = True
                                else:
                                    layout.addWidget(self.autoRecover)
                                    placed_auto = True
                            except Exception:
                                try:
                                    layout.addWidget(self.autoRecover)
                                    placed_auto = True
                                except Exception:
                                    placed_auto = False
                    else:
                        # Box/layouts: insert autoRecover BEFORE getData (above)
                        try:
                            idx = layout.indexOf(self.getData)
                            if idx != -1:
                                layout.insertWidget(idx + 1, self.stopRecording)
                                # insert autoRecover before getData so it appears above
                                layout.insertWidget(idx, self.autoRecover)
                                placed_stop = placed_auto = True
                            else:
                                layout.addWidget(self.autoRecover)
                                layout.addWidget(self.stopRecording)
                                placed_stop = placed_auto = True
                        except Exception:
                            try:
                                layout.addWidget(self.autoRecover)
                                layout.addWidget(self.stopRecording)
                                placed_stop = placed_auto = True
                            except Exception:
                                placed_stop = placed_auto = False
                except Exception:
                    # layout insert failed; will fallback to main_layout
                    placed_stop = placed_auto = False

            if not placed_stop or not placed_auto:
                # try adding to the central widget layout
                try:
                    main_layout = self.centralWidget().layout()
                    if main_layout is not None:
                        if not placed_stop:
                            main_layout.addWidget(self.stopRecording)
                            placed_stop = True
                        if not placed_auto:
                            main_layout.addWidget(self.autoRecover)
                            placed_auto = True
                except Exception:
                    pass

            # As last resort, show them so they're usable even without layout
            if not placed_stop:
                try:
                    self.stopRecording.show()
                except Exception:
                    pass
            if not placed_auto:
                try:
                    self.autoRecover.show()
                except Exception:
                    pass

        except Exception:
            # Non-fatal; UI will still operate albeit without these controls
            pass

    def _position_auto_recover(self):
        """Position the autoRecover checkbox at the bottom-right corner.

        The checkbox will match the size of the `getData` button if present,
        otherwise `stopRecording`, otherwise a sensible default.
        """
        try:
            if not getattr(self, 'autoRecover', None):
                return

            # Reference button to copy size from
            ref = getattr(self, 'getData', None) 
            if ref and hasattr(ref, 'width'):
                w = ref.width()
                h = ref.height()
            else:
                w = 120
                h = 28

            margin = 10
            # place inside the main window (self)
            total_w = self.width()
            total_h = self.height()
            x = max(margin, total_w - w - margin)
            y = max(margin, total_h - h - margin)
            # avoid overlapping the Build image button if present
            try:
                build_btn = getattr(self, 'buildImage', None)
                if build_btn is not None:
                    try:
                        bgeo = build_btn.geometry()
                        bx, by, bw, bh = bgeo.x(), bgeo.y(), bgeo.width(), bgeo.height()
                        # check if the default position would overlap the build button
                        intersects = (x < bx + bw and x + w > bx and y < by + bh and y + h > by)
                        if intersects:
                            # Preferred: place BELOW the build button if space allows
                            new_y = by + bh + margin
                            if new_y + h <= total_h - margin:
                                y = new_y
                                # keep x aligned to build button's left where possible
                                if bx + bw - w >= margin:
                                    x = bx + bw - w
                            else:
                                # Try to place to the RIGHT of the build button
                                new_x = bx + bw + margin
                                if new_x + w <= total_w - margin:
                                    x = new_x
                                    # keep y near build button top if possible
                                    if by + h <= total_h - margin:
                                        y = by
                                else:
                                    # fallback: place to the LEFT of the build button
                                    new_x = bx - w - margin
                                    if new_x >= margin:
                                        x = new_x
                                        y = by
                                    else:
                                        # last resort: place above the build button
                                        new_y2 = by - h - margin
                                        if new_y2 >= margin:
                                            y = new_y2
                                        else:
                                            # if nothing fits, nudge to keep inside window
                                            x = max(margin, min(x, total_w - w - margin))
                                            y = max(margin, min(y, total_h - h - margin))
                    except Exception:
                        pass
            except Exception:
                pass
            try:
                self.autoRecover.setGeometry(x, y, w, h)
            except Exception:
                self.autoRecover.move(x, y)
        except Exception:
            pass

    def resizeEvent(self, event):
        # Let the layout manager handle positions/sizes for widgets we
        # inserted into layouts (stopRecording and autoRecover). Just call
        # the base implementation to allow normal resizing behaviour.
        try:
            super().resizeEvent(event)
        except Exception:
            try:
                QMainWindow.resizeEvent(self, event)
            except Exception:
                pass

    def save_folder_path(self, max_length=40):
        path = QFileDialog.getExistingDirectory(self, "Select Folder")
        if len(path) > max_length:
            half = max_length // 2
            start = path[:half]
            end = path[-half:]
            display_text = f"{start}...{end}"
        else:
            display_text = path
        self.saveFolderLabel.setText(display_text)
        self.saveFolderLabel.setToolTip(path)
        self.save_path = path if path else self.save_path
        # create/touch the transfer log file in the newly selected folder
        try:
            os.makedirs(self.save_path, exist_ok=True)
            open(os.path.join(self.save_path, self._transfer_log_name), "a", encoding="utf-8").close()
        except Exception:
            pass

    def _handle_worker_status(self, msg: str):
        """Show message in status bar, print to console and append to log file in save_path."""
        try:
            # GUI status bar
            self.statusBar().showMessage(msg)
            # also print for console debugging
            print(msg)
            # append to log file under current save_path
            if getattr(self, "save_path", None):
                logfile = os.path.join(self.save_path, self._transfer_log_name)
                try:
                    with open(logfile, "a", encoding="utf-8") as f:
                        f.write(f"{datetime.datetime.now().isoformat()} - {msg}\n")
                except Exception:
                    # non-fatal: don't crash UI if logging fails
                    pass
        except Exception:
            # ensure no exception escapes the slot
            pass

    def _prompt_power_cycle(self):
        """Prompt the user to power-cycle the board and then refresh registers.

        This is invoked when the worker determines a soft recovery failed and a
        physical power-cycle is required. After the user confirms they performed
        the power-cycle, the UI attempts to refresh/registers and passes the new
        FPGA instance back to the worker.
        """
        try:
            # Block until the user clicks the explicit 'Device powered' button
            # or uses the optional 'Relay switch' to trigger a physical relay
            # connected to an Arduino. The Relay switch button will attempt
            # to send the ASCII character "1" to a detected serial device.
            while True:
                msg = QMessageBox(self)
                msg.setWindowTitle("Power-cycle required")
                msg.setIcon(QMessageBox.Warning)
                msg.setText(
                    "The device appears to be in a stuck state.\n\n"
                    "Please power-cycle the DDC264EVM board now (remove and reapply power).\n\n"
                    "After you have power-cycled the board, click 'Device powered' to continue."
                )
                # Only provide a single explicit button the user must press
                msg.setStandardButtons(QMessageBox.NoButton)
                btn = msg.addButton("Device powered", QMessageBox.AcceptRole)
                # Add Relay switch button which will attempt to send '1' to
                # a connected Arduino (via USB serial) to toggle a relay.
                relay_btn = msg.addButton("Relay switch", QMessageBox.ActionRole)
                msg.exec_()

                # If the user pressed the Relay switch button, attempt to
                # find the Arduino and send the command, then loop again so
                # the user can confirm 'Device powered' when ready.
                if msg.clickedButton() is relay_btn:
                    try:
                        if serial is None or serial_list_ports is None:
                            self._handle_worker_status(
                                "pyserial not available: cannot drive relay"
                            )
                        else:
                            port = None
                            try:
                                ports = list(serial_list_ports.comports())
                            except Exception:
                                ports = []
                            # heuristics: pick port with 'arduino' in description/hwid
                            for p in ports:
                                desc = (p.description or "").lower()
                                hwid = (p.hwid or "").lower()
                                if "arduino" in desc or "arduino" in hwid:
                                    port = p.device
                                    break
                            # fallback: if only one port is present, use it
                            if port is None and len(ports) == 1:
                                port = ports[0].device

                            if port is None:
                                self._handle_worker_status(
                                    "No Arduino-like serial device found to drive relay"
                                )
                            else:
                                try:
                                    baud = 9600
                                    timeout = 1
                                    s = serial.Serial(port, baudrate=baud, timeout=timeout)
                                    time.sleep(2)
                                    # send ASCII '1' — device on the other side
                                    s.write(b'1')
                                    s.flush()
                                    time.sleep(0.15)
                                    s.close()
                                    self._handle_worker_status(
                                        f"Sent relay command to {port}"
                                    )
                                except Exception as e:
                                    self._handle_worker_status(
                                        f"Failed to send relay command to {port}: {e}"
                                    )
                    except Exception as e:
                        # ensure any serial-related errors are logged but don't
                        # break the dialog loop
                        self._handle_worker_status(f"Relay switch error: {e}")
                    # loop again so the user can either retry relay switch or
                    # press the Device powered button when they have cycled power
                    continue

                # If the user pressed the explicit 'Device powered' button, continue.
                if msg.clickedButton() is btn:
                    # attempt to refresh registers and recreate FPGAControl instance
                    try:
                        self._handle_worker_status("User confirmed power-cycle; refreshing registers...")
                        # refresh_registers will call self.fpga.refresh() and update internal state
                        self.refresh_registers(is_startup=False)
                    except Exception as e:
                        self._handle_worker_status(f"Error during refresh after power-cycle: {e}")
                        # Loop again so the user can retry power-cycle if needed
                        continue

                    # if a worker exists, update its fpga reference so it can continue
                    try:
                        if hasattr(self, "worker") and self.worker is not None:
                            try:
                                self.worker.set_fpga(self.fpga)
                                self._handle_worker_status("Updated worker with new FPGA instance after power-cycle")
                            except Exception as e:
                                self._handle_worker_status(f"Failed to update worker FPGA: {e}")
                    except Exception:
                        pass

                    break
                else:
                    # If the dialog was closed (user closed window) without pressing the button,
                    # stop the recording: emit worker.finished so the UI stops the thread/cleanup.
                    self._handle_worker_status("Power-cycle dialog closed without confirmation: stopping recording")
                    try:
                        if hasattr(self, "worker") and self.worker is not None:
                            # set cancel flags on worker if present (best-effort)
                            try:
                                self.worker._powercycle_ack = False
                                self.worker._waiting_for_powercycle = False
                            except Exception:
                                pass
                            try:
                                # emit finished to trigger UI cleanup and stop the recording flow
                                self.worker.finished.emit()
                            except Exception:
                                pass
                    except Exception:
                        pass
                    return
        except Exception:
            # don't let UI errors stop worker
            pass

    def _handle_request_power_cycle(self):
        """Handle a worker power-cycle request: either auto-recover via Arduino
        (send '1' and refresh) or fall back to the interactive dialog.
        """
        try:
            if getattr(self, 'autoRecover', None) and self.autoRecover.isChecked():
                # attempt automatic relay toggle via serial
                try:
                    if serial is None or serial_list_ports is None:
                        self._handle_worker_status("pyserial not available: cannot auto-recover; opening dialog")
                        self._prompt_power_cycle()
                        return
                    port = None
                    try:
                        ports = list(serial_list_ports.comports())
                    except Exception:
                        ports = []
                    for p in ports:
                        desc = (p.description or "").lower()
                        hwid = (p.hwid or "").lower()
                        if "arduino" in desc or "arduino" in hwid:
                            port = p.device
                            break
                    if port is None and len(ports) == 1:
                        port = ports[0].device

                    if port is None:
                        self._handle_worker_status("No Arduino-like serial device found to drive relay; opening dialog")
                        self._prompt_power_cycle()
                        return

                    try:
                        # Many Arduinos auto-reset when the serial port is opened.
                        # Wait a short time after opening so the sketch can start.
                        baud = 9600
                        timeout = 1
                        s = serial.Serial(port, baudrate=baud, timeout=timeout)
                        time.sleep(2.0)
                        # send ASCII '1' — device on the other side
                        try:
                            s.write(b"1")
                        except TypeError:
                            s.write("1".encode('ascii'))
                        s.flush()
                        s.close()
                        self._handle_worker_status(f"Sent auto-recover relay command to {port}; waiting 5s")
                        try:
                            self._show_notification("Auto recover", f"Sent relay command to {port}; waiting 5s", timeout=3000)
                        except Exception:
                            pass
                    except Exception as e:
                        self._handle_worker_status(f"Failed to send auto-recover to {port}: {e}")
                        try:
                            self._show_notification("Auto recover failed", f"Failed to send to {port}: {e}", timeout=5000)
                        except Exception:
                            pass
                        self._prompt_power_cycle()
                        return

                    # wait 5 seconds for device to power-cycle and re-enumerate
                    time.sleep(5)

                    # attempt to refresh registers and recreate FPGAControl instance
                    try:
                        self._handle_worker_status("Refreshing registers after auto-recover...")
                        self.refresh_registers(is_startup=False)
                        try:
                            self._show_notification("Auto recover", "Refresh completed after auto-recover", timeout=3000)
                        except Exception:
                            pass
                    except Exception as e:
                        self._handle_worker_status(f"Refresh after auto-recover failed: {e}")
                        try:
                            self._show_notification("Auto recover failed", f"Refresh failed: {e}", timeout=5000)
                        except Exception:
                            pass
                        # fall back to interactive dialog
                        self._prompt_power_cycle()
                        return

                    # update worker with new FPGA instance
                    try:
                        if hasattr(self, "worker") and self.worker is not None:
                            try:
                                self.worker.set_fpga(self.fpga)
                                self._handle_worker_status("Updated worker with new FPGA instance after auto-recover")
                                try:
                                    self._show_notification("Auto recover", "Auto-recover succeeded; worker updated", timeout=3000)
                                except Exception:
                                    pass
                            except Exception as e:
                                self._handle_worker_status(f"Failed to update worker FPGA after auto-recover: {e}")
                    except Exception:
                        pass

                    return
                except Exception as e:
                    self._handle_worker_status(f"Error in auto-recover flow: {e}")
                    try:
                        self._prompt_power_cycle()
                    except Exception:
                        pass
                    return
            else:
                # interactive path
                self._prompt_power_cycle()
        except Exception as e:
            # ensure no exception escapes
            self._handle_worker_status(f"Error handling power-cycle request: {e}")
            try:
                self._prompt_power_cycle()
            except Exception:
                pass

    def _show_notification(self, title: str, message: str, timeout: int = 3000):
        """Show a small, non-modal popup notification independent of the main window.

        The popup auto-closes after `timeout` milliseconds.
        """
        try:
            dlg = QDialog(self)
            # Keep it as a tool window so it doesn't take focus from main window
            dlg.setWindowFlags(Qt.Tool | Qt.WindowStaysOnTopHint)
            dlg.setWindowTitle(title)
            layout = QVBoxLayout(dlg)
            lbl = QLabel(message, dlg)
            lbl.setWordWrap(True)
            layout.addWidget(lbl)
            dlg.setLayout(layout)
            dlg.setModal(False)
            dlg.show()
            # auto-close after timeout
            QTimer.singleShot(timeout, dlg.close)
        except Exception:
            # fallback: log status
            try:
                self._handle_worker_status(f"{title}: {message}")
            except Exception:
                pass

    def update_registers(self, is_startup=False):
        try:
            if (
                5 * int(self.ConvLowInt.text()) < 1600
                or 5 * int(self.ConvHighInt.text()) < 1600
            ):
                raise ValueError

            self.fpga = FPGAControl(
                5 * int(self.ConvLowInt.text()),
                5 * int(self.ConvHighInt.text()),
                self.conv_config[self.ConvConfig.currentText()],
                int(self.CLKHigh.text()),
                int(self.CLKLow.text()),
                self.ddc_clk_config[self.DDCCLKConfig.currentText()],
                int(self.ChannelCount.currentText()),
                int(self.nDVALIDIgnore.text()),
                int(self.nDVALIDRead.text()),
                int(self.DCLKHigh.text()),
                int(self.DCLKLow.text()),
                self.dclk_config[self.DCLKConfig.currentText()],
                int(self.DCLKWait.text()),
                self.hardware_trigger[self.HardwareTrigger.currentText()],
                int(self.CLK_CFGHigh.text()),
                int(self.CLK_CFGLow.text()),
                self.ADCrange.currentText(),
                int(self.Format.currentText()[:-4]),
            )
            if not is_startup:
                self.statusBar().showMessage("Registers updated successfully")
        except ValueError:
            self.statusBar().showMessage("Invalid input")

    def refresh_registers(self, is_startup=False):
        try:
            self.fpga.refresh()
            self.update_registers()
            if not is_startup:
                self.statusBar().showMessage(
                    "Registers refreshed and updated successfully"
                )
        except Exception as e:
            self.statusBar().showMessage(f"Error refreshing registers: {str(e)}")

    def hard_reset(self):
        try:
            reset_result = self.fpga.reset_ddc()
            clear_result = self.fpga.clear_triggers()

            if reset_result and clear_result:
                self.statusBar().showMessage("Hard reset completed successfully")
            else:
                self.statusBar().showMessage("Hard reset failed")
        except Exception as e:
            self.statusBar().showMessage(f"Error during hard reset: {str(e)}")

    def update_time(self):
        try:
            if self.ConvHighInt.text():
                self.conv_high_int_text.setText(
                    f"us = {int(self.ConvHighInt.text())*5}"
                )
            if self.ConvLowInt.text():
                self.conv_low_int_text.setText(f"us = {int(self.ConvLowInt.text())*5}")
        except ValueError:
            self.statusBar().showMessage("Invalid input")

    def record_data(self):
        if self.fpga:
            try:
                # perform an extra refresh before measurement to ensure device state
               # try:
                #    self._handle_worker_status("Refreshing FPGA before measurement...")
                 #   self.fpga.refresh()
                #except Exception as e:
                 #   self._handle_worker_status(f"FPGA refresh before measurement failed: {e}")
                  #  self.statusBar().showMessage(f"FPGA refresh failed: {e}")
                   # return

                numFiles = int(self.nFiles.text())
                if numFiles <= 0:
                    raise ValueError

                folder_path = self.save_path
                if not folder_path:
                    self.statusBar().showMessage("Please select a save folder")
                    return

                self.progressBar.setMaximum(numFiles)
                self.progressBar.setValue(0)
                self.progressBar.show()
                file_name = self.saveFileName.text() or "file"
                self.thread = QThread()
                # pass a few extra robustness parameters (retries, polling timeout)
                self.worker = ReaderWorker(
                    self.fpga, folder_path, file_name, numFiles, max_retries=2, poll_timeout=8.0
                )
                self.worker.moveToThread(self.thread)

                # clear any previous stop request when starting a new recording
                try:
                    self.worker._stop_requested = False
                except Exception:
                    pass

                self.thread.started.connect(self.worker.run)
                self.worker.progress.connect(self.progressBar.setValue)
                # route worker status messages to the combined handler (status bar + console + file)
                self.worker.status.connect(self._handle_worker_status)
                # if worker asks for power-cycle, handle either auto-recover or prompt the user
                self.worker.request_power_cycle.connect(self._handle_request_power_cycle)
                # allow UI to push back an updated FPGA instance to the worker
                self.worker.set_fpga = self.worker.set_fpga
                self.worker.finished.connect(self.thread.quit)
                self.worker.finished.connect(self.worker.deleteLater)
                self.worker.finished.connect(
                    lambda: self.readFilePath.setText(self.worker.readFilePath)
                )
                self.worker.finished.connect(
                    lambda: self.load_trace_file(
                        f"{folder_path}/{self.readFilePath.text()}"
                    )
                )

                self.worker.finished.connect(
                    lambda: self.load_file(
                        "image_file",
                        self.imageFileLabel,
                        "image_data",
                        True,
                        f"{folder_path}/{self.readFilePath.text()}",
                    )
                )
                self.worker.finished.connect(self.build_image)

                self.worker.finished.connect(self.build_image)
                self.thread.finished.connect(self.thread.deleteLater)
                self.thread.finished.connect(self.progressBar.hide)
                self.thread.start()

            except ValueError:
                self.statusBar().showMessage("Invalid number of files")
        else:
            self.statusBar().showMessage("Please update registers first")

    def _on_stop_clicked(self):
        """Handler for Stop button: request worker to stop after current file."""
        try:
            if hasattr(self, "worker") and self.worker is not None:
                try:
                    self.worker.stop()
                    self._handle_worker_status("User requested stop: will stop after current file")
                except Exception as e:
                    self._handle_worker_status(f"Failed to request stop: {e}")
            else:
                self._handle_worker_status("No recording in progress to stop")
        except Exception:
            pass

    def load_trace_file(self, file_path=None):
        self.file_data = {}
        if not file_path:
            options = QFileDialog.Options()
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                "Select File",
                "",
                "Text Files (*.txt);;All Files (*)",
                options=options,
            )
            if not file_path:
                return

        try:
            self.readFilePath.setText(file_path.split("/")[-1])
            with open(file_path) as f:
                lines = f.readlines()
                for line in lines:
                    if line.split(",")[0] not in self.file_data:
                        self.file_data[line.split(",")[0]] = [float(line.split(",")[2])]
                    else:
                        self.file_data[line.split(",")[0]].append(
                            float(line.split(",")[2])
                        )
            if self.traceNumber.currentText() == "--":
                self.traceNumber.setCurrentText("Mean value")
            self.plot_trace()
        except ValueError:
            self.statusBar().showMessage("Invalid file")

    def plot_trace(self):
        if not len(self.file_data) == 0:
            self.graphWidget.clear()
            trace = self.traceNumber.currentText()
            if not (trace == "--"):

                if trace == "Mean value":
                    self.graphWidget.setLabel("bottom", "Channel")
                    x = list(range(512))
                    sorted_keys = sorted(
                        self.file_data.keys(), key=lambda x: (x[-1], int(x[:-1]))
                    )
                    y = [
                        self.fpga.convert_adc(np.mean(self.file_data[key]))
                        for key in sorted_keys
                    ]
                    color = "r"
                else:
                    self.graphWidget.setLabel("bottom", "Time")
                    prefix = "0" if int(trace[:-1]) < 10 else ""
                    x = list(range(512))
                    y = self.fpga.convert_adc(
                        np.array(self.file_data[f"{prefix}{trace}"])
                    )
                    color = "b"

                self.graphWidget.plot(
                    x,
                    y,
                    pen=pg.mkPen(color, width=1),
                    symbol="o",
                    symbolSize=5,
                    symbolBrush=color,
                )

    def load_file(
        self, file_name_attr, label, data_attr, update_dark=False, file_path=None
    ):
        if not file_path:
            options = QFileDialog.Options()
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                "Select File",
                "",
                "Text Files (*.txt);;All Files (*)",
                options=options,
            )
            if not file_path:
                return
        try:
            setattr(self, file_name_attr, file_path)
            with open(file_path) as f:
                peaks = {}
                for i, line in enumerate(f.readlines()):
                    if line.split(",")[0] not in peaks:
                        peaks[line.split(",")[0]] = [
                            self.fpga.convert_adc(float(line.split(",")[2]))
                        ]
                    else:
                        peaks[line.split(",")[0]].append(
                            self.fpga.convert_adc(float(line.split(",")[2]))
                        )
                try:
                    if (
                        int(self.edgeLeft.text()) < 0
                        or int(self.edgeRight.text()) > 512
                        or int(self.edgeLeft.text()) >= int(self.edgeRight.text())
                    ):
                        raise ValueError
                    for key, value in peaks.items():
                        right_mean = np.mean(value[int(self.edgeRight.text()) :])
                        left_mean = np.mean(value[: int(self.edgeLeft.text())])
                        peaks[key] = (right_mean - left_mean, left_mean)
                except ValueError:
                    self.statusBar().showMessage("Invalid edge values")
                    return np.zeros((16, 16))

                array_image = np.zeros((16, 16))
                array_dark = np.zeros((16, 16))
                for i in range(16):
                    for j in range(16):
                        if self.decoder_matrix[i, j] >= 10:
                            array_image[i, j] = peaks[f"{self.decoder_matrix[i,j]}A"][0]
                            array_dark[i, j] = peaks[f"{self.decoder_matrix[i,j]}A"][1]
                        else:
                            array_image[i, j] = peaks[f"0{self.decoder_matrix[i,j]}A"][
                                0
                            ]
                            array_dark[i, j] = peaks[f"0{self.decoder_matrix[i,j]}A"][1]
                array_image = np.rot90(array_image, 3)
                array_dark = np.rot90(array_dark, 3)

                setattr(self, data_attr, array_image)
                if update_dark:
                    setattr(self, "dark_current_data", array_dark)

            label.setText(file_path.split("/")[-1])
        except ValueError:
            self.statusBar().showMessage("Invalid file")

    def load_decoder_matrix(self, file_path=None):
        if not file_path:
            options = QFileDialog.Options()
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                "Select File",
                "",
                "Text Files (*.txt);;All Files (*)",
                options=options,
            )
            if not file_path:
                return
        try:
            with open(file_path) as f:
                lines = f.readlines()
                for i, line in enumerate(lines):
                    self.decoder_matrix[i, :] = list(map(int, line.split()))
            self.decoderMatrixLabel.setText(file_path.split("/")[-1])
        except ValueError:
            self.statusBar().showMessage("Invalid file")

    def build_image(self):
        if self.useNormalization.isChecked():
            if (not self.image_file) or (not self.open_beam_file):
                self.statusBar().showMessage(
                    "Please select both image and open beam files"
                )
            else:
                left_image = self.image_data / self.open_beam_data

                if self.useThreshold.isChecked():
                    left_image[left_image > 1] = 1

                self.img_item.setImage(left_image)
                if np.isnan(left_image.min()) or np.isnan(left_image.max()):
                    self.img_item.setLevels((0, 1))
                    self.color_bar.setLevels((0, 1))
                    self.imageUpperScale.setText("1")
                    self.imageLowScale.setText("0")
                else:
                    self.img_item.setLevels((left_image.min(), left_image.max()))
                    self.color_bar.setLevels((left_image.min(), left_image.max()))
                self.imageUpperScale.setText(f"{left_image.max()}")
                self.imageLowScale.setText(f"{left_image.min()}")
        else:
            if not self.image_file:
                self.statusBar().showMessage("Please select image file")
            else:
                left_image = (
                    self.image_data
                    / float(self.pixelX.text())
                    / float(self.pixelY.text())
                    / float(self.ConvLowInt.text())
                    * 1e15
                )
                self.img_item.setImage(left_image)
                if np.isnan(left_image.min()) or np.isnan(left_image.max()):
                    self.img_item.setLevels((0, 1))
                    self.color_bar.setLevels((0, 1))
                    self.imageUpperScale.setText("1")
                    self.imageLowScale.setText("0")
                else:
                    self.img_item.setLevels((left_image.min(), left_image.max()))
                    self.color_bar.setLevels((left_image.min(), left_image.max()))
                    self.imageUpperScale.setText(f"{left_image.max()}")
                    self.imageLowScale.setText(f"{left_image.min()}")

        if self.darkCurrent.isChecked():
            if not self.image_file:
                self.statusBar().showMessage("Please select image file")
            else:
                right_image = (
                    self.dark_current_data
                    / float(self.pixelX.text())
                    / float(self.pixelY.text())
                    / float(self.ConvLowInt.text())
                    * 1e15
                )
                self.mix_img_item.setImage(right_image)
                if np.isnan(right_image.min()) or np.isnan(right_image.max()):
                    self.mix_img_item.setLevels((0, 1))
                    self.mix_color_bar.setLevels((0, 1))
                    self.mixUpperScale.setText("1")
                    self.mixLowScale.setText("0")
                else:
                    self.mix_img_item.setLevels((right_image.min(), right_image.max()))
                    self.mix_color_bar.setLevels((right_image.min(), right_image.max()))
                    self.mixUpperScale.setText(f"{right_image.max()}")
                    self.mixLowScale.setText(f"{right_image.min()}")

        if self.openBeam.isChecked():
            if not self.open_beam_file:
                self.statusBar().showMessage("Please select open beam file")
            else:
                right_image = (
                    self.open_beam_data
                    / float(self.pixelX.text())
                    / float(self.pixelY.text())
                    / float(self.ConvLowInt.text())
                    * 1e15
                )
                self.mix_img_item.setImage(right_image)
                if np.isnan(right_image.min()) or np.isnan(right_image.max()):
                    self.mix_img_item.setLevels((0, 1))
                    self.mix_color_bar.setLevels((0, 1))
                    self.mixUpperScale.setText("1")
                    self.mixLowScale.setText("0")
                else:
                    self.mix_img_item.setLevels((right_image.min(), right_image.max()))
                    self.mix_color_bar.setLevels((right_image.min(), right_image.max()))
                    self.mixUpperScale.setText(f"{right_image.max()}")
                    self.mixLowScale.setText(f"{right_image.min()}")

    def change_scales(self):
        if self.imageUpperScale.text() and self.imageLowScale.text():
            try:
                self.img_item.setLevels(
                    (
                        float(self.imageLowScale.text()),
                        float(self.imageUpperScale.text()),
                    )
                )
                self.color_bar.setLevels(
                    (
                        float(self.imageLowScale.text()),
                        float(self.imageUpperScale.text()),
                    )
                )
            except ValueError:
                self.statusBar().showMessage("Invalid left scale values")

        if self.mixUpperScale.text() and self.mixLowScale.text():
            try:
                self.mix_img_item.setLevels(
                    (float(self.mixLowScale.text()), float(self.mixUpperScale.text()))
                )
                self.mix_color_bar.setLevels(
                    (float(self.mixLowScale.text()), float(self.mixUpperScale.text()))
                )
            except ValueError:
                self.statusBar().showMessage("Invalid right scale values")
