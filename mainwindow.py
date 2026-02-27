from PyQt5 import uic
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QThread, Qt, QTimer
from PyQt5.QtWidgets import QMainWindow, QFileDialog, QVBoxLayout, QHBoxLayout, QMessageBox, QPushButton, QCheckBox, QDialog, QLabel, QGridLayout, QLineEdit, QWidget, QDoubleSpinBox, QComboBox, QSizePolicy
from tools import FPGAControl
from rotational_control import RotationalController
import pyqtgraph as pg
import numpy as np
import os
import sys
import time
import datetime
import json
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
    # signal emitted after each file is saved (for live plot updates)
    file_saved = pyqtSignal(str)  # emits the file path

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
                    try:
                        self.status.emit(
                            f"Starting transfer for file index {desired_index} (attempt {attempt})"
                        )
                        ret = self.fpga.get_data(
                            os.path.join(self.folder_path, self.file_name), desired_index - 1
                        )
                        # Set poll deadline AFTER get_data completes, so long acquisitions don't timeout
                        poll_deadline = time.time() + self.poll_timeout
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
                        if new and desired_index in new:
                            last_saved_index = desired_index
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
                
                # Emit file_saved signal for live plot updates
                saved_path = os.path.join(self.folder_path, saved_name)
                self.file_saved.emit(saved_path)
                
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
        
        # Initialize normalization dropdown with 4 modes
        self.useNormalization.addItem("none")
        self.useNormalization.addItem("full leakage/flat-field")
        self.useNormalization.addItem("leakage/flat-field from open beam")
        self.useNormalization.addItem("flat-field only")
        # persistent rotation controller (open once per measurement batch)
        self._rotation_controller = None
        self._rotation_port = None
        # Open persistent rotation controller for the program lifetime
        try:
            self._ensure_rotation_controller()
        except Exception:
            pass
        try:
            # Update status label if present
            try:
                self._update_arduino_status_label()
            except Exception:
                pass
        except Exception:
            pass
        self.useNormalization.setCurrentText("none")
        
        

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
        
        # Track whether user has manually edited scale fields
        self.imageLowScale_manual = False
        self.imageUpperScale_manual = False
        self.mixLowScale_manual = False
        self.mixUpperScale_manual = False

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
        
        # Add Auto update checkbox for live trace plot updates
        self.autoUpdateTrace = QCheckBox("Auto update", self)
        self.autoUpdateTrace.setChecked(False)
        try:
            # Add it to horizontalLayout_23 (same line as File Path button and traceNumber)
            if hasattr(self, 'horizontalLayout_23'):
                # Remove the spacer to make room for the checkbox
                spacer_to_remove = None
                for i in range(self.horizontalLayout_23.count()):
                    item = self.horizontalLayout_23.itemAt(i)
                    if item and item.spacerItem():
                        spacer_to_remove = item
                        break
                if spacer_to_remove:
                    self.horizontalLayout_23.removeItem(spacer_to_remove)
                
                # Add stretch to push checkbox to the right
                self.horizontalLayout_23.addStretch()
                # Add Auto update checkbox aligned to the right
                self.horizontalLayout_23.addWidget(self.autoUpdateTrace, alignment=Qt.AlignRight)
        except Exception:
            pass

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
        
        # Load 1D detector decoder matrix
        self.decoder_matrix_1d = np.zeros(128, dtype=int)
        self.load_decoder_matrix_1d("decoder_matrix_sample1.txt")

        self.image_view = self.imageWidget.addViewBox()
        self.image_view.setAspectLocked(True)
        self.img_item = pg.ImageItem(np.zeros((16, 16)))
        self.image_view.addItem(self.img_item)
        self.image_view.autoRange()
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
        self.mix_view.setAspectLocked(True)
        self.mix_img_item = pg.ImageItem(np.zeros((16, 16)))
        self.mix_view.addItem(self.mix_img_item)
        self.mix_view.autoRange()
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
        self.open_beam_baseline = np.zeros((16, 16))  # Store open beam baseline for mode 3
        
        # 1D detector arrays
        self.image_data_1d = np.zeros(128)
        self.dark_current_data_1d = np.zeros(128)
        self.open_beam_data_1d = np.zeros(128)
        self.open_beam_baseline_1d = np.zeros(128)
        
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
        
        # Add Auto update checkbox for live image updates (shared for both images)
        # Place it in the same row as the Build image button (before it)
        self.autoUpdateImages = QCheckBox("Auto update images", self)
        self.autoUpdateImages.setChecked(False)
        try:
            # Access the horizontalLayout_21 that contains the buildImage button
            if hasattr(self, 'horizontalLayout_21'):
                # Find the index of buildImage button
                build_index = -1
                for i in range(self.horizontalLayout_21.count()):
                    item = self.horizontalLayout_21.itemAt(i)
                    if item and item.widget() is self.buildImage:
                        build_index = i
                        break
                
                # Insert checkbox before buildImage button
                if build_index >= 0:
                    self.horizontalLayout_21.insertWidget(build_index, self.autoUpdateImages)
                else:
                    # Fallback: add at end
                    self.horizontalLayout_21.addWidget(self.autoUpdateImages)
            else:
                # Fallback: try to get parent layout
                build_layout = self.buildImage.parent().layout()
                if build_layout:
                    build_layout.addWidget(self.autoUpdateImages)
        except Exception as e:
            pass
        
        self.imageUpperScale.textChanged.connect(self.change_scales)
        self.imageLowScale.textChanged.connect(self.change_scales)
        self.mixUpperScale.textChanged.connect(self.change_scales)
        self.mixLowScale.textChanged.connect(self.change_scales)
        
        # Track manual edits to scale fields
        self.imageUpperScale.textEdited.connect(lambda: setattr(self, 'imageUpperScale_manual', True))
        self.imageLowScale.textEdited.connect(lambda: setattr(self, 'imageLowScale_manual', True))
        self.mixUpperScale.textEdited.connect(lambda: setattr(self, 'mixUpperScale_manual', True))
        self.mixLowScale.textEdited.connect(lambda: setattr(self, 'mixLowScale_manual', True))
        
        self.darkCurrent.toggled.connect(self.build_image)
        self.openBeam.toggled.connect(self.build_image)
        self.openBeam.clicked.connect(self.build_image)
        self.darkCurrent.clicked.connect(self.build_image)
        
        # Add 1D detector checkbox
        self.detector1D = QCheckBox("1D detector", self)
        self.detector1D.setChecked(False)
        try:
            if hasattr(self, 'horizontalLayout_21'):
                build_index = -1
                for i in range(self.horizontalLayout_21.count()):
                    item = self.horizontalLayout_21.itemAt(i)
                    if item and item.widget() is self.buildImage:
                        build_index = i
                        break
                if build_index >= 0:
                    self.horizontalLayout_21.insertWidget(build_index, self.detector1D)
                else:
                    self.horizontalLayout_21.addWidget(self.detector1D)
        except Exception:
            pass
        
        # toggle handler does both layout update and image rebuild
        self.detector1D.toggled.connect(self._on_1d_toggled)

        self.show()

        # Style the Get Data button (green) and Stop button (red)
        try:
            self.getData.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        except Exception:
            pass

        # Create Stop button and Auto recover checkbox and insert them into
        # the same layout as the Get Data button so they behave like other
        # controls when the window is resized.
        try:
            self.stopRecording = QPushButton("Stop", self)
            self.stopRecording.clicked.connect(self._on_stop_clicked)
            self.stopRecording.setStyleSheet("background-color: #DC143C; color: white; font-weight: bold;")
            self.autoRecover = QCheckBox("Auto recover", self)
            self.autoRecover.setChecked(True)

            # Rotation controls: checkbox + number of angles + HOME button
            self.rotationCheckbox = QCheckBox("Rotation", self)
            self.rotationCheckbox.setChecked(False)
            # numeric input for number of angles (per full rotation)
            self.angleNum = QLineEdit(self)
            self.angleNum.setText("4")
            self.angleNum.setFixedWidth(48)
            # HOME button to send home command to Arduino
            self.homeButton = QPushButton("HOME", self)
            self.homeButton.clicked.connect(self._on_home_clicked)
            self.homeButton.setStyleSheet("background-color: red; color: white; font-weight: bold;")
            self.homeButton.setFixedWidth(60)
            # Custom angle button and input to go to absolute position
            self.customAngleButton = QPushButton("Custom angle", self)
            self.customAngleButton.clicked.connect(self._on_custom_angle_clicked)
            self.customAngleButton.setStyleSheet("background-color: blue; color: white; font-weight: bold;")
            self.customAngleButton.setFixedWidth(100)
            self.customAngleInput = QLineEdit(self)
            self.customAngleInput.setText("0")
            self.customAngleInput.setFixedWidth(48)
            # Set HOME button to set current position as home (zero)
            self.setHomeButton = QPushButton("Set HOME", self)
            self.setHomeButton.clicked.connect(self._on_set_home_clicked)
            self.setHomeButton.setStyleSheet("background-color: black; color: white; font-weight: bold;")
            self.setHomeButton.setFixedWidth(80)

            # Arduino connect UI: status label and connect button
            self.arduinoStatusLabel = QLabel("Disconnected", self)
            self.arduinoStatusLabel.setStyleSheet("color: red;")
            self.arduinoStatusLabel.setFixedWidth(140)
            self.connectArduinoButton = QPushButton("Connect Arduino", self)
            self.connectArduinoButton.setFixedWidth(120)
            self.connectArduinoButton.clicked.connect(self._on_connect_arduino_clicked)
            
            # REV controls: time per revolution, computed delay, start/stop
            self.stepperRevWidget = QWidget(self)
            # keep horizontal expansion minimal so main window width does not grow
            try:
                self.stepperRevWidget.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
                # Prevent REV row from forcing the main window to widen: cap its max width
                try:
                    cw = self.centralWidget().width() if getattr(self, 'centralWidget', None) and self.centralWidget() is not None else 800
                    self.stepperRevWidget.setMaximumWidth(max(400, cw - 40))
                except Exception:
                    # fallback to a reasonable maximum
                    try:
                        self.stepperRevWidget.setMaximumWidth(800)
                    except Exception:
                        pass
            except Exception:
                pass
            self.stepperRevLayout = QHBoxLayout(self.stepperRevWidget)
            self.stepperRevLayout.setContentsMargins(0, 0, 0, 0)
            self.stepperRevLayout.setSpacing(6)

            self.revTimeLabel = QLabel("REV time (ms):", self)
            self.revTimeSpin = QDoubleSpinBox(self)
            # time per revolution in milliseconds; allow sub-ms values
            self.revTimeSpin.setRange(0.001, 36000000.0)
            self.revTimeSpin.setSingleStep(0.1)
            self.revTimeSpin.setDecimals(3)
            self.revTimeSpin.setValue(1000.0)
            self.revTimeSpin.setFixedWidth(100)

            self.revDirCombo = QComboBox(self)
            self.revDirCombo.addItems(["CW", "CCW"])
            self.revDirCombo.setFixedWidth(80)

            self.revDelayLabel = QLabel("Delay: - µs", self)
            self.revDelayLabel.setFixedWidth(140)

            self.revStartButton = QPushButton("REV start", self)
            self.revStopButton = QPushButton("REV stop", self)
            self.revStartButton.setFixedWidth(90)
            self.revStopButton.setFixedWidth(90)

            # push REV controls to the right within their row
            self.stepperRevLayout.addStretch()
            self.stepperRevLayout.addWidget(self.revTimeLabel)
            self.stepperRevLayout.addWidget(self.revTimeSpin)
            self.stepperRevLayout.addWidget(self.revDirCombo)
            self.stepperRevLayout.addWidget(self.revDelayLabel)
            self.stepperRevLayout.addWidget(self.revStartButton)
            self.stepperRevLayout.addWidget(self.revStopButton)

            # Wire handlers
            try:
                self.revTimeSpin.valueChanged.connect(self._update_rev_delay_display)
            except Exception:
                pass
            try:
                self.revStartButton.clicked.connect(self._on_rev_start_clicked)
                self.revStopButton.clicked.connect(self._on_rev_stop_clicked)
            except Exception:
                pass
            try:
                # initialize display
                self._update_rev_delay_display()
            except Exception:
                pass

            # Create a horizontal layout container for all stepper motor controls
            self.stepperControlWidget = QWidget(self)
            self.stepperControlLayout = QHBoxLayout(self.stepperControlWidget)
            self.stepperControlLayout.setContentsMargins(0, 0, 0, 0)
            self.stepperControlLayout.setSpacing(5)
            # Add widgets in order: Rotation checkbox, angleNum, HOME, Custom angle, customAngleInput, Set HOME
            self.stepperControlLayout.addWidget(self.rotationCheckbox)
            self.stepperControlLayout.addWidget(self.angleNum)
            self.stepperControlLayout.addWidget(self.homeButton)
            self.stepperControlLayout.addWidget(self.customAngleButton)
            self.stepperControlLayout.addWidget(self.customAngleInput)
            self.stepperControlLayout.addStretch()  # Add stretch to push Set HOME to the right
            self.stepperControlLayout.addWidget(self.setHomeButton, alignment=Qt.AlignRight)
            # Add Arduino connection controls to the stepper control row
            try:
                self.stepperControlLayout.addWidget(self.arduinoStatusLabel)
                self.stepperControlLayout.addWidget(self.connectArduinoButton)
            except Exception:
                pass
            try:
                # Ensure the status label reflects any controller opened earlier
                self._update_arduino_status_label()
            except Exception:
                pass

            # Combine stepper control row and REV row into a single widget (vertical)
            try:
                self.stepperCombinedWidget = QWidget(self)
                combinedLayout = QVBoxLayout(self.stepperCombinedWidget)
                combinedLayout.setContentsMargins(0, 0, 0, 0)
                combinedLayout.setSpacing(2)
                combinedLayout.addWidget(self.stepperControlWidget)
                combinedLayout.addWidget(self.stepperRevWidget)
            except Exception:
                self.stepperCombinedWidget = None

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

            # Place Auto recover checkbox next to Filename field (in horizontalLayout_25)
            try:
                # Access horizontalLayout_25 directly by name
                if hasattr(self, 'horizontalLayout_25'):
                    # Remove the spacer to make room for the checkbox
                    spacer_to_remove = None
                    for i in range(self.horizontalLayout_25.count()):
                        item = self.horizontalLayout_25.itemAt(i)
                        if item and item.spacerItem():
                            spacer_to_remove = item
                            break
                    if spacer_to_remove:
                        self.horizontalLayout_25.removeItem(spacer_to_remove)
                    
                    # Add stretch first to push checkbox to the right
                    self.horizontalLayout_25.addStretch()
                    # Add Auto recover checkbox aligned to the right
                    self.horizontalLayout_25.addWidget(self.autoRecover, alignment=Qt.AlignRight)
                    placed_auto = True
            except Exception as e:
                pass

            if layout is not None:
                try:
                    if isinstance(layout, QGridLayout):
                        # find getData position in the grid
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
                            
                            # Place Stop button to the right of getData
                            try:
                                if layout.itemAtPosition(r, c + 1) is None:
                                    layout.addWidget(self.stopRecording, r, c + 1)
                                else:
                                    layout.addWidget(self.stopRecording, r, c + 2)
                                placed_stop = True
                            except Exception:
                                try:
                                    layout.addWidget(self.stopRecording)
                                    placed_stop = True
                                except Exception:
                                    placed_stop = False
                    else:
                        # Box/layouts: We need to find which layout item contains getData
                        # and insert autoRecover before that item
                        try:
                            # getData is inside horizontalLayout_17, we need to find that layout in the parent
                            getData_parent_layout = self.getData.parent().layout() if self.getData.parent() else None
                            if getData_parent_layout and layout:
                                # Find the index of the layout that contains getData
                                idx = -1
                                for i in range(layout.count()):
                                    item = layout.itemAt(i)
                                    if item and item.layout() == getData_parent_layout:
                                        idx = i
                                        break
                                
                                if idx != -1:
                                    # Insert autoRecover in a horizontal layout before getData's row
                                    autoRecoverLayout = QHBoxLayout()
                                    autoRecoverLayout.addWidget(self.autoRecover)
                                    autoRecoverLayout.addStretch()
                                    layout.insertLayout(idx, autoRecoverLayout)
                                    placed_auto = True
                                    
                                    # Add Stop button to the same row as getData
                                    if getData_parent_layout:
                                        getData_parent_layout.addWidget(self.stopRecording)
                                        placed_stop = True
                                else:
                                    # Fallback
                                    layout.addWidget(self.autoRecover)
                                    layout.addWidget(self.stopRecording)
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
                        # Add the stepper control + REV combined container as a single block
                        try:
                            if getattr(self, 'stepperCombinedWidget', None) is not None:
                                main_layout.addWidget(self.stepperCombinedWidget)
                            else:
                                main_layout.addWidget(self.stepperControlWidget)
                                try:
                                    main_layout.addWidget(self.stepperRevWidget, alignment=Qt.AlignRight)
                                except Exception:
                                    pass
                        except Exception:
                            pass
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
            # ensure rotation controls are visible if placement failed
            try:
                if getattr(self, 'stepperCombinedWidget', None) is not None:
                    try:
                        self.stepperCombinedWidget.show()
                    except Exception:
                        pass
                else:
                    if not getattr(self, 'stepperControlWidget', None):
                        pass
                    else:
                        self.stepperControlWidget.show()
            except Exception:
                pass

            # Ensure stepper controls and REV row are present in the central layout
            try:
                main_layout = self.centralWidget().layout()
                if main_layout is not None:
                    # add stepperControlWidget if not already present
                    found = False
                    for i in range(main_layout.count()):
                        item = main_layout.itemAt(i)
                        try:
                            if item and item.widget() is self.stepperControlWidget:
                                found = True
                                break
                        except Exception:
                            continue
                    if not found:
                        try:
                            if getattr(self, 'stepperCombinedWidget', None) is not None:
                                main_layout.addWidget(self.stepperCombinedWidget, alignment=Qt.AlignRight)
                            else:
                                main_layout.addWidget(self.stepperControlWidget)
                        except Exception:
                            pass
            except Exception:
                pass

        except Exception:
            # Non-fatal; UI will still operate albeit without these controls
            pass

        # If Rotation checkbox is ON at startup, enable the motor on the Arduino
        try:
            if getattr(self, 'rotationCheckbox', None) and self.rotationCheckbox.isChecked():
                try:
                    self._handle_worker_status("Enabling motor at startup...")
                    success, responses, error = self._send_arduino_command({"command": "enable_motor"}, timeout=3.0)
                    if success:
                        self._handle_worker_status("Motor enabled successfully at startup")
                    else:
                        self._handle_worker_status(f"Failed to enable motor at startup: {error}")
                except Exception as e:
                    self._handle_worker_status(f"Failed to enable rotation motor at startup: {e}")
        except Exception:
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

    def _find_arduino_port(self):
        """Return a serial device path for the Arduino if found.

        Detection order:
        1) Match VID=0x2341 and PID=0x0058 (preferred)
        2) Match "arduino" in description or hwid (existing heuristic)
        3) If only one serial port present, return it as a last resort
        Returns device string (e.g. 'COM3') or None.
        """
        try:
            if serial is None or serial_list_ports is None:
                return None
            try:
                ports = list(serial_list_ports.comports())
            except Exception:
                ports = []

            # 1) VID/PID match for official Arduino variants
            # Prefer Nano (VID 0x2341, PID 0x0058) and Mega2560 (VID 0x2341, PID 0x0042)
            for p in ports:
                try:
                    vid = getattr(p, 'vid', None)
                    pid = getattr(p, 'pid', None)
                    if vid == 0x2341 and pid in (0x0058, 0x0042):
                        return p.device
                except Exception:
                    pass

            # 2) existing heuristic: look for 'arduino' in description/hwid
            for p in ports:
                try:
                    desc = (p.description or "").lower()
                    hwid = (p.hwid or "").lower()
                    if 'arduino' in desc or 'arduino' in hwid:
                        return p.device
                except Exception:
                    pass

            # 3) fallback: if exactly one candidate USB-serial port is present
            if len(ports) == 1:
                return ports[0].device

            return None
        except Exception:
            return None

    def _ensure_rotation_controller(self):
        """Create and store a persistent `RotationalController` if not present.

        Returns the controller instance or None on failure.
        """
        if getattr(self, '_rotation_controller', None) is not None:
            return self._rotation_controller

        port = self._find_arduino_port() or getattr(self, '_rotation_port', None) or "COM6"
        try:
            rc = RotationalController(port=port)
            self._rotation_controller = rc
            self._rotation_port = port
            try:
                self._handle_worker_status(f"Rotation controller opened on {port}")
            except Exception:
                pass
            return rc
        except Exception as e:
            try:
                self._handle_worker_status(f"Failed to open rotation controller on {port}: {e}")
            except Exception:
                pass
            self._rotation_controller = None
            return None

    def _update_arduino_status_label(self):
        """Update the on-screen Arduino connection status label and color."""
        try:
            lbl = getattr(self, 'arduinoStatusLabel', None)
            if lbl is None:
                return
            rc = getattr(self, '_rotation_controller', None)
            if rc is not None and getattr(rc, 'arduino', None):
                port = getattr(self, '_rotation_port', None) or getattr(rc.arduino, 'port', None) or 'unknown'
                lbl.setText(f"Connected: {port}")
                lbl.setStyleSheet("color: green;")
            else:
                lbl.setText("Disconnected")
                lbl.setStyleSheet("color: red;")
        except Exception:
            pass

    def _on_connect_arduino_clicked(self):
        """Manual connect button handler: try to open (or reopen) the rotation controller."""
        try:
            # Close any existing to force re-open
            try:
                self._close_rotation_controller()
            except Exception:
                pass
            rc = self._ensure_rotation_controller()
            if rc is not None:
                try:
                    self._handle_worker_status(f"Arduino connected on {self._rotation_port}")
                except Exception:
                    pass
                try:
                    QMessageBox.information(self, "Arduino", f"Connected to {self._rotation_port}")
                except Exception:
                    pass
            else:
                try:
                    QMessageBox.warning(self, "Arduino", "Failed to connect to Arduino")
                except Exception:
                    pass
        finally:
            try:
                self._update_arduino_status_label()
            except Exception:
                pass

    def _update_rev_delay_display(self):
        """Recalculate and show the pulse delay (microseconds) for the requested revolution time in milliseconds."""
        try:
            # total microsteps per revolution on the hardware (matches Arduino config)
            steps_per_rev = 25600.0
            t = float(self.revTimeSpin.value())
            if t <= 0:
                self.revDelayLabel.setText("Delay: - µs")
                return
            # Each step period ~= 2 * DELAY_MICROS (pulse high + low). Compute DELAY_MICROS
            # rev time is in milliseconds -> convert to microseconds: t*1e3
            delay_us = int((t * 1e3) / (2.0 * steps_per_rev))
            self.revDelayLabel.setText(f"Delay: {delay_us} µs")
        except Exception:
            try:
                self.revDelayLabel.setText("Delay: - µs")
            except Exception:
                pass

    def _on_rev_start_clicked(self):
        """Start continuous rotation using Arduino JSON command.

        Computes delay from revTimeSpin (ms) and sends: {"command":"run continuous","dir":0|1,"delay":<us>}.
        """
        try:
            # compute delay
            steps_per_rev = 25600.0
            t = float(self.revTimeSpin.value())
            if t <= 0:
                QMessageBox.warning(self, "REV start", "Invalid revolution time")
                return
            # rev time is in milliseconds -> convert to microseconds: t*1e3
            delay_us = int((t * 1e3) / (2.0 * steps_per_rev))
            dir_txt = self.revDirCombo.currentText()
            dir_val = 0 if dir_txt == "CW" else 1
            cmd = {"command": "run continuous", "dir": dir_val, "delay": int(delay_us)}
            success, responses, error = self._send_arduino_command(cmd, timeout=3.0)
            if success:
                try:
                    self._handle_worker_status(f"Started continuous rotation (delay {delay_us} µs)")
                except Exception:
                    pass
            else:
                QMessageBox.warning(self, "REV start", f"Failed to start rotation: {error}")
        except Exception as e:
            QMessageBox.warning(self, "REV start", f"Error: {e}")

    def _on_rev_stop_clicked(self):
        """Stop continuous rotation via Arduino JSON command {"command":"stop"}."""
        try:
            cmd = {"command": "stop"}
            success, responses, error = self._send_arduino_command(cmd, timeout=2.0)
            if success:
                try:
                    self._handle_worker_status("Stopped continuous rotation")
                except Exception:
                    pass
            else:
                QMessageBox.warning(self, "REV stop", f"Failed to stop rotation: {error}")
        except Exception as e:
            QMessageBox.warning(self, "REV stop", f"Error: {e}")

    def _close_rotation_controller(self):
        """Close and clear the persistent rotation controller if present."""
        rc = getattr(self, '_rotation_controller', None)
        if rc:
            try:
                if getattr(rc, 'arduino', None):
                    try:
                        rc.arduino.close()
                    except Exception:
                        pass
            except Exception:
                pass
        self._rotation_controller = None

    def _send_power_cycle_command(self):
        """Send power cycle command to Arduino using JSON protocol with robust port detection.
        
        Waits for Arduino to complete the power cycle and send confirmation response.
        
        Returns:
            Tuple of (success: bool, error_msg: str or None)
        """
        try:
            if serial is None or serial_list_ports is None:
                return (False, "pyserial not available")
            
            # Use the robust Arduino port detection
            port = self._find_arduino_port()
            
            if port is None:
                return (False, "No Arduino-like serial device found")
            
            try:
                # Prefer reusing persistent controller if available to avoid reset delays
                rc = getattr(self, '_rotation_controller', None)
                created_locally = False
                if rc is None:
                    # open a temporary serial for power-cycle
                    baud = 115200
                    timeout = 5
                    s = serial.Serial(port, baudrate=baud, timeout=timeout)
                    time.sleep(2.0)
                    created_locally = True
                else:
                    s = rc.arduino

                # Send JSON command for power cycle
                command = {"command": "power cycle"}
                cmd_json = json.dumps(command) + "\n"
                self._handle_worker_status(f"→ Arduino ({port}): {cmd_json.strip()}")

                s.write(cmd_json.encode('utf-8'))
                s.flush()
                
                # Wait for both responses from Arduino:
                # 1. "Triggering power cycle relay"
                # 2. "Power cycle relay triggered" (completion confirmation)
                responses_received = 0
                completion_confirmed = False
                start_time = time.time()
                max_wait = 5.0  # Maximum 5 seconds to wait for completion
                
                while time.time() - start_time < max_wait and not completion_confirmed:
                    try:
                        if s.in_waiting > 0:
                            response = s.readline().decode('utf-8', errors='ignore').strip()
                            if response:
                                self._handle_worker_status(f"← Arduino: {response}")
                                responses_received += 1
                                
                                # Check if this is the completion message
                                try:
                                    resp_json = json.loads(response)
                                    if resp_json.get("msg") == "Power cycle relay triggered":
                                        completion_confirmed = True
                                        break
                                except json.JSONDecodeError:
                                    pass
                        else:
                            time.sleep(0.1)  # Small delay between checks
                    except Exception as e:
                        self._handle_worker_status(f"Error reading response: {e}")
                        break
                
                if created_locally:
                    try:
                        s.close()
                    except Exception:
                        pass
                
                if completion_confirmed:
                    return (True, None)
                elif responses_received > 0:
                    return (False, "Power cycle started but completion not confirmed")
                else:
                    return (False, "No response from Arduino")
                
            except Exception as e:
                return (False, f"Failed to communicate with {port}: {e}")
                
        except Exception as e:
            return (False, f"Power cycle command error: {e}")

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

    def closeEvent(self, event):
        """Ensure rotation controller is closed when the main window is closed."""
        try:
            try:
                self._close_rotation_controller()
            except Exception:
                pass
        except Exception:
            pass
        try:
            super().closeEvent(event)
        except Exception:
            try:
                QMainWindow.closeEvent(self, event)
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
                        success, error = self._send_power_cycle_command()
                        if success:
                            self._handle_worker_status("Sent power cycle command to Arduino")
                        else:
                            self._handle_worker_status(f"Failed to send power cycle command: {error}")
                    except Exception as e:
                        # ensure any serial-related errors are logged but don't
                        # break the dialog loop
                        self._handle_worker_status(f"Power cycle command error: {e}")
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
        (send JSON power cycle command) or fall back to the interactive dialog.
        """
        try:
            if getattr(self, 'autoRecover', None) and self.autoRecover.isChecked():
                # attempt automatic power cycle via Arduino
                try:
                    success, error = self._send_power_cycle_command()
                    
                    if not success:
                        self._handle_worker_status(f"Auto-recover failed: {error}; opening dialog")
                        self._prompt_power_cycle()
                        return
                    
                    self._handle_worker_status("Sent auto-recover power cycle command; waiting 5s")
                    try:
                        self._show_notification("Auto recover", "Sent power cycle command; waiting 5s", timeout=3000)
                    except Exception:
                        pass

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
        if not self.fpga:
            self.statusBar().showMessage("Please update registers first")
            return

        try:
            numFiles = int(self.nFiles.text())
            if numFiles <= 0:
                raise ValueError
        except Exception:
            self.statusBar().showMessage("Invalid number of files")
            return

        folder_path = self.save_path
        if not folder_path:
            self.statusBar().showMessage("Please select a save folder")
            return

        file_name = self.saveFileName.text() or "file"

        # If rotation is not enabled, keep previous behaviour (single worker)
        if not getattr(self, 'rotationCheckbox', None) or not self.rotationCheckbox.isChecked():
            # original single-run behaviour
            self.progressBar.setMaximum(numFiles)
            self.progressBar.setValue(0)
            self.progressBar.show()
            self.thread = QThread()
            self.worker = ReaderWorker(self.fpga, folder_path, file_name, numFiles, max_retries=2, poll_timeout=8.0)
            self.worker.moveToThread(self.thread)
            try:
                self.worker._stop_requested = False
            except Exception:
                pass
            self.thread.started.connect(self.worker.run)
            self.worker.progress.connect(self.progressBar.setValue)
            self.worker.status.connect(self._handle_worker_status)
            self.worker.request_power_cycle.connect(self._handle_request_power_cycle)
            self.worker.file_saved.connect(self._on_file_saved)  # Connect live trace update signal
            self.worker.file_saved.connect(self._on_file_saved_images)  # Connect live image update signal
            self.worker.set_fpga = self.worker.set_fpga
            self.worker.finished.connect(self.thread.quit)
            self.worker.finished.connect(self.worker.deleteLater)
            self.worker.finished.connect(lambda: self.readFilePath.setText(self.worker.readFilePath))
            self.worker.finished.connect(lambda: self.load_trace_file(f"{folder_path}/{self.readFilePath.text()}"))
            self.worker.finished.connect(lambda: self.load_file("image_file", self.imageFileLabel, "image_data", True, f"{folder_path}/{self.readFilePath.text()}"))
            self.worker.finished.connect(self.build_image)
            self.thread.finished.connect(self.thread.deleteLater)
            self.thread.finished.connect(self.progressBar.hide)
            self.thread.start()
            return

        # Rotation mode: run numFiles per angle and rotate between batches
        try:
            angleNum = int(self.angleNum.text())
            if angleNum <= 0:
                raise ValueError
        except Exception:
            self.statusBar().showMessage("Invalid number of angles")
            return

        # compute discrete rotation step
        angle_step = round(360 / angleNum)

        # prepare cumulative progress
        total_files = numFiles * angleNum
        self.progressBar.setMaximum(total_files)
        self.progressBar.setValue(0)
        self.progressBar.show()

        # Try to create rotational controller on demand
        try:
            # If a persistent controller already exists (opened at startup), reuse it
            if getattr(self, '_rotation_controller', None) is not None:
                try:
                    existing_port = getattr(self, '_rotation_port', None) or getattr(self._rotation_controller.arduino, 'port', None)
                    self._handle_worker_status(f"Using existing rotation controller on {existing_port}")
                    self._rotation_port = existing_port
                except Exception:
                    pass
            else:
                # Try to auto-detect a port, fall back to COM6 then COM4
                port = self._find_arduino_port() or "COM6"
                # validate port can be opened then closed (don't keep it open)
                try:
                    rc = RotationalController(port=port)
                    try:
                        rc.arduino.close()
                    except Exception:
                        pass
                    self._handle_worker_status(f"Rotation controller port {port} is available")
                except Exception as e:
                    # try fallback COM6 explicitly if not already
                    if port != "COM6":
                        try:
                            rc = RotationalController(port="COM6")
                            try:
                                rc.arduino.close()
                            except Exception:
                                pass
                            self._handle_worker_status("Rotation controller connected to COM6 (fallback)")
                            port = "COM6"
                        except Exception as e2:
                            self._handle_worker_status(f"Failed to open rotation controller: {e}; {e2}")
                            QMessageBox.warning(self, "Rotation error", f"Cannot open rotation controller: {e}\n{e2}")
                            return
                    else:
                        self._handle_worker_status(f"Failed to open rotation controller on {port}: {e}")
                        QMessageBox.warning(self, "Rotation error", f"Cannot open rotation controller: {e}")
                        return
                # store chosen port for future rotate operations
                self._rotation_port = port
        except Exception:
            QMessageBox.warning(self, "Rotation error", "Cannot find a serial port for rotation controller")
            return

        # internal state for batch-run
        self._record_batch_idx = 0
        self._record_batch_total = angleNum
        self._record_batch_numFiles = numFiles
        self._record_folder = folder_path
        self._record_fname = file_name
        self._rotation_stop_requested = False  # Flag to abort rotation sequence

        # start first batch
        self._start_next_rotation_batch()

    def _on_stop_clicked(self):
        """Handler for Stop button: request worker to stop and abort all remaining batches."""
        try:
            # Set flag to abort rotation sequence
            self._rotation_stop_requested = True
            
            if hasattr(self, "worker") and self.worker is not None:
                try:
                    self.worker.stop()
                    # Worker will emit its own status message
                except Exception as e:
                    self._handle_worker_status(f"Failed to request stop: {e}")
            else:
                self._handle_worker_status("No recording in progress to stop")
        except Exception:
            pass

    def _on_home_clicked(self):
        """Handler for HOME button: send home command to Arduino and close port."""
        try:
            self._handle_worker_status("Sending HOME command to Arduino...")
            success, responses, error = self._send_arduino_command({"command": "home"}, timeout=10.0)
            
            if success:
                self._handle_worker_status("HOME command completed successfully")
                # Check if any response indicates an error
                for resp in responses or []:
                    if isinstance(resp, dict) and resp.get("status") == "ERROR":
                        QMessageBox.warning(self, "HOME Error", f"Arduino error: {resp.get('msg', 'Unknown error')}")
                        return
            else:
                self._handle_worker_status(f"HOME command failed: {error}")
                QMessageBox.warning(self, "HOME Error", f"Failed to send HOME command: {error}")
        except Exception as e:
            self._handle_worker_status(f"Error in HOME button handler: {e}")
            QMessageBox.warning(self, "HOME Error", f"Error: {e}")

    def _on_set_home_clicked(self):
        """Handler for Set HOME button: confirm and send set home command to Arduino."""
        try:
            # Show confirmation dialog
            reply = QMessageBox.question(
                self,
                "Confirm Set HOME",
                "This will set the current motor position as the HOME (zero) position.\n\nAre you sure you want to continue?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No
            )
            
            if reply != QMessageBox.Yes:
                self._handle_worker_status("Set HOME cancelled by user")
                return
            
            # User confirmed, send the command
            self._handle_worker_status("Sending Set HOME command to Arduino...")
            success, responses, error = self._send_arduino_command({"command": "set home"}, timeout=3.0)
            
            if success:
                self._handle_worker_status("Set HOME command completed successfully")
                # Check if any response indicates an error
                for resp in responses or []:
                    if isinstance(resp, dict) and resp.get("status") == "ERROR":
                        QMessageBox.warning(self, "Set HOME Error", f"Arduino error: {resp.get('msg', 'Unknown error')}")
                        return
                QMessageBox.information(self, "Set HOME", "Current position has been set as HOME (zero).")
            else:
                self._handle_worker_status(f"Set HOME command failed: {error}")
                QMessageBox.warning(self, "Set HOME Error", f"Failed to send Set HOME command: {error}")
        except Exception as e:
            self._handle_worker_status(f"Error in Set HOME button handler: {e}")
            QMessageBox.warning(self, "Set HOME Error", f"Error: {e}")

    def _on_custom_angle_clicked(self):
        """Handler for Custom angle button: send absolute position command to Arduino."""
        try:
            # Get the angle from the input field
            try:
                absolute_angle = float(self.customAngleInput.text())
            except ValueError:
                QMessageBox.warning(self, "Invalid Input", "Please enter a valid number for the angle.")
                return
            
            # Send the command
            self._handle_worker_status(f"Sending command to move to {absolute_angle} degrees...")
            success, responses, error = self._send_arduino_command(
                {"command": "absolute position", "degrees": absolute_angle}, 
                timeout=30.0
            )
            
            if success:
                self._handle_worker_status(f"Custom angle command completed: moved to {absolute_angle} degrees")
                # Check if any response indicates an error
                for resp in responses or []:
                    if isinstance(resp, dict) and resp.get("status") == "ERROR":
                        QMessageBox.warning(self, "Custom Angle Error", f"Arduino error: {resp.get('msg', 'Unknown error')}")
                        return
            else:
                self._handle_worker_status(f"Custom angle command failed: {error}")
                QMessageBox.warning(self, "Custom Angle Error", f"Failed to send custom angle command: {error}")
        except Exception as e:
            self._handle_worker_status(f"Error in Custom angle button handler: {e}")
            QMessageBox.warning(self, "Custom Angle Error", f"Error: {e}")

    def _start_next_rotation_batch(self):
        """Start the next batch for rotation mode."""
        # Check if user requested stop
        if getattr(self, '_rotation_stop_requested', False):
            self._handle_worker_status("Stop requested: not starting next rotation batch")
            # Force to end state
            self._record_batch_idx = self._record_batch_total
            try:
                self.progressBar.hide()
            except Exception:
                pass
            return
        
        if self._record_batch_idx >= self._record_batch_total:
            # all done
            try:
                # final UI updates: set last read file and load images
                if getattr(self, 'worker', None) and getattr(self.worker, 'readFilePath', None):
                    self.readFilePath.setText(self.worker.readFilePath)
                    try:
                        self.load_trace_file(f"{self._record_folder}/{self.readFilePath.text()}")
                    except Exception:
                        pass
                    try:
                        self.load_file("image_file", self.imageFileLabel, "image_data", True, f"{self._record_folder}/{self.readFilePath.text()}")
                    except Exception:
                        pass
                self.build_image()
            finally:
                try:
                    self.progressBar.hide()
                except Exception:
                    pass
            return

        numFiles = self._record_batch_numFiles
        folder_path = self._record_folder
        # create per-angle base filename when in rotation mode: format base_a{angleIndex}
        angle_index = self._record_batch_idx  # 0-based
        file_name = f"{self._record_fname}_a{angle_index+1}"

        # create and start worker for this batch
        self.thread = QThread()
        self.worker = ReaderWorker(self.fpga, folder_path, file_name, numFiles, max_retries=2, poll_timeout=8.0)
        self.worker.moveToThread(self.thread)
        try:
            self.worker._stop_requested = False
        except Exception:
            pass
        self.thread.started.connect(self.worker.run)
        self.worker.progress.connect(self._on_batch_progress)
        self.worker.status.connect(self._handle_worker_status)
        self.worker.request_power_cycle.connect(self._handle_request_power_cycle)
        self.worker.file_saved.connect(self._on_file_saved)  # Connect live trace update signal
        self.worker.file_saved.connect(self._on_file_saved_images)  # Connect live image update signal
        # ensure worker can receive updated fpga instance
        self.worker.set_fpga = self.worker.set_fpga
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.worker.finished.connect(self._on_batch_finished)
        self.thread.finished.connect(self.thread.deleteLater)
        # Start
        self.thread.start()

    def _on_batch_progress(self, value):
        """Update cumulative progress bar from current batch progress value."""
        try:
            base = self._record_batch_idx * self._record_batch_numFiles
            self.progressBar.setValue(base + int(value))
        except Exception:
            try:
                self.progressBar.setValue(int(value))
            except Exception:
                pass

    def _on_file_saved(self, file_path):
        """Handler for live trace plot updates when a file is saved.
        
        Only updates if Auto update checkbox is checked.
        """
        try:
            # Check if auto update is enabled
            if not getattr(self, 'autoUpdateTrace', None) or not self.autoUpdateTrace.isChecked():
                return
            
            # Load and plot the trace from the saved file
            self.load_trace_file(file_path)
        except Exception as e:
            # Don't let plot update errors stop recording
            pass

    def _on_file_saved_images(self, file_path):
        """Handler for live 2D image updates when a file is saved.
        
        Only updates if Auto update checkbox is checked.
        """
        try:
            # Check if auto update is enabled
            if not getattr(self, 'autoUpdateImages', None) or not self.autoUpdateImages.isChecked():
                return
            
            # Load the file and update image data
            self.load_file("image_file", self.imageFileLabel, "image_data", True, file_path)
            # Rebuild images to update both left and right displays
            self.build_image()
        except Exception as e:
            # Don't let image update errors stop recording
            pass

    def _perform_rotation(self, angle_deg):
        """Perform a single rotation by opening the rotation controller, rotating, then closing the port.

        Uses robust port detection to find Arduino.
        This avoids leaving the COM port open between runs.
        """
        # Try to reuse a persistent controller if available
        created_locally = False
        rc = getattr(self, '_rotation_controller', None)
        if rc is None:
            rc = self._ensure_rotation_controller()
        if rc is None:
            # fallback: create a temporary controller for this rotation
            port = self._find_arduino_port() or getattr(self, '_rotation_port', None) or "COM6"
            try:
                rc = RotationalController(port=port)
                created_locally = True
            except Exception as e:
                self._handle_worker_status(f"Failed to open rotation controller for rotation: {e}")
                return

        t0 = time.time()
        try:
            self._handle_worker_status(f"Rotating {angle_deg}° (port {getattr(rc, 'arduino', None) and getattr(rc.arduino, 'port', getattr(self, '_rotation_port', 'unknown'))})")
            rc.rotate(angle_deg)
            # Add small settling time after motor completes rotation (can be reduced)
            time.sleep(0.5)
            try:
                self._handle_worker_status(f"Rotation complete in {time.time()-t0:.3f}s")
            except Exception:
                pass
        finally:
            if created_locally:
                try:
                    if getattr(rc, 'arduino', None):
                        rc.arduino.close()
                except Exception:
                    pass

    def _update_1d_layout(self, is_1d: bool):
        """Adjust the image pane layout and aspect locking based on mode.

        Called both from the 1D checkbox handler and at the start of
        :meth:`build_image`. When ``is_1d`` is true the horizontal container
        is switched to a vertical stack and aspect locking disabled.
        """
        try:
            container = self.findChild(QHBoxLayout, "horizontalLayout_26")
            if container is not None:
                from PyQt5.QtWidgets import QBoxLayout
                container.setDirection(
                    QBoxLayout.TopToBottom if is_1d else QBoxLayout.LeftToRight
                )
                # ensure the containing widget updates its layout
                try:
                    w = container.parentWidget()
                    if w is not None:
                        w.updateGeometry()
                        w.repaint()
                except Exception:
                    pass
        except Exception:
            pass
        try:
            self.image_view.setAspectLocked(not is_1d)
            self.mix_view.setAspectLocked(not is_1d)
        except Exception:
            pass

    def _on_1d_toggled(self, checked: bool):
        """Slot for detector1D checkbox toggle.

        Updates the layout immediately and rebuilds the image so the change
        is visible without needing to press "Build image" again.

        The 1‑D detector uses a different effective pixel size (1.56) than the
        default 2‑D mode (0.36).  When the mode is enabled we overwrite the
        pixel size fields and save the previous values so they can be restored
        when the mode is turned off.  The new entry is then picked up by the
        normal `build_image` calculations which always read the QLineEdit
        contents.
        """
        # adjust pixel size fields before rebuilding
        try:
            if checked:
                # save the user's previous values only once
                if not hasattr(self, '_backup_pixel_sizes'):
                    self._backup_pixel_sizes = (
                        self.pixelX.text(),
                        self.pixelY.text(),
                    )
                self.pixelX.setText("1.56")
                self.pixelY.setText("1.56")
            else:
                if hasattr(self, '_backup_pixel_sizes'):
                    self.pixelX.setText(self._backup_pixel_sizes[0])
                    self.pixelY.setText(self._backup_pixel_sizes[1])
                    delattr(self, '_backup_pixel_sizes')
        except Exception:
            pass

        self._update_1d_layout(checked)
        self.build_image()
        # Try to reuse a persistent controller if available
        created_locally = False
        rc = getattr(self, '_rotation_controller', None)
        if rc is None:
            rc = self._ensure_rotation_controller()
        if rc is None:
            # fallback: create a temporary controller for this rotation
            port = self._find_arduino_port() or getattr(self, '_rotation_port', None) or "COM6"
            try:
                rc = RotationalController(port=port)
                created_locally = True
            except Exception as e:
                self._handle_worker_status(f"Failed to open rotation controller for rotation: {e}")
                return

        t0 = time.time()
        try:
            self._handle_worker_status(f"Rotating {angle_deg}° (port {getattr(rc, 'arduino', None) and getattr(rc.arduino, 'port', getattr(self, '_rotation_port', 'unknown'))})")
            rc.rotate(angle_deg)
            # Add small settling time after motor completes rotation (can be reduced)
            time.sleep(0.5)
            try:
                self._handle_worker_status(f"Rotation complete in {time.time()-t0:.3f}s")
            except Exception:
                pass
        finally:
            if created_locally:
                try:
                    if getattr(rc, 'arduino', None):
                        rc.arduino.close()
                except Exception:
                    pass

    def _send_arduino_command(self, command_dict, timeout=5.0):
        """Send a command to Arduino, read and display the response.
        
        Uses robust port detection to find Arduino.
        
        Args:
            command_dict: Dictionary with command (e.g., {"command": "home"})
            timeout: How long to wait for response (seconds)
            
        Returns:
            Tuple of (success: bool, response_dict: dict or None, error_msg: str or None)
        """
        # Use robust Arduino port detection
        port = self._find_arduino_port()
        if port is None:
            # Fallback to stored port or default
            port = getattr(self, '_rotation_port', None) or "COM6"
        
        # Prefer using persistent controller to avoid repeated opens/resets
        created_locally = False
        rc = getattr(self, '_rotation_controller', None)
        if rc is None:
            rc = self._ensure_rotation_controller()
        if rc is None:
            # fallback: create temporary controller
            try:
                rc = RotationalController(port=port)
                created_locally = True
            except Exception as e:
                error_msg = f"Arduino communication open error: {e}"
                self._handle_worker_status(error_msg)
                return (False, None, error_msg)

        try:
            # Send command
            cmd_json = json.dumps(command_dict)
            try:
                self._handle_worker_status(f"→ Arduino ({port}): {cmd_json}")
            except Exception:
                pass

            t0 = time.time()
            rc.arduino.write((cmd_json + "\n").encode("utf-8"))
            rc.arduino.flush()
            try:
                self._handle_worker_status(f"Wrote command in {time.time()-t0:.3f}s")
            except Exception:
                pass
            time.sleep(0.05)

            # Read response(s) - Arduino may send multiple lines
            responses = []
            start_time = time.time()
            while time.time() - start_time < timeout:
                try:
                    if rc.arduino.in_waiting > 0:
                        line = rc.arduino.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            try:
                                self._handle_worker_status(f"← Arduino: {line}")
                            except Exception:
                                pass
                            try:
                                resp_json = json.loads(line)
                                responses.append(resp_json)
                                if resp_json.get("status") in ["OK", "ERROR"]:
                                    return (True, responses, None)
                            except json.JSONDecodeError:
                                pass
                    time.sleep(0.02)
                except Exception:
                    break

            if responses:
                return (True, responses, None)
            else:
                return (False, None, "No response from Arduino")

        except Exception as e:
            error_msg = f"Arduino communication error: {e}"
            try:
                self._handle_worker_status(error_msg)
            except Exception:
                pass
            return (False, None, error_msg)
        finally:
            if created_locally:
                try:
                    if getattr(rc, 'arduino', None):
                        rc.arduino.close()
                except Exception:
                    pass

    def _on_batch_finished(self):
        """Handle a finished batch: rotate if more batches remain, otherwise finalize."""
        try:
            # Check if user requested stop
            if getattr(self, '_rotation_stop_requested', False):
                self._handle_worker_status("Stop requested: aborting remaining rotation batches")
                # Force to end state
                self._record_batch_idx = self._record_batch_total
                # Clean up and hide progress bar
                try:
                    self.progressBar.hide()
                except Exception:
                    pass
                return
            
            # increment finished batch count
            self._record_batch_idx += 1
            # If we still have batches remaining, rotate and start next
            # compute step angle
            angle_step = 360 / self._record_batch_total
            if self._record_batch_idx < self._record_batch_total:
                # perform rotation and then start next batch
                try:
                    self._handle_worker_status(f"Rotating by {angle_step} degrees for next batch ({self._record_batch_idx + 1}/{self._record_batch_total})")
                    # perform rotation (open/rotate/close)
                    try:
                        self._perform_rotation(angle_step)
                    except Exception as e:
                        self._handle_worker_status(f"Rotation failed: {e}")
                        QMessageBox.warning(self, "Rotation error", f"Rotation failed: {e}")
                        # abort further batches
                        self._record_batch_idx = self._record_batch_total
                        self._start_next_rotation_batch()
                        return
                finally:
                    self._start_next_rotation_batch()
                    return
            else:
                # we've finished all batches; perform one final rotation to return to origin
                try:
                    self._handle_worker_status("Performing final rotation to return motor to original position")
                    try:
                        self._perform_rotation(360 - (angle_step * (self._record_batch_total - 1)))
                    except Exception as e:
                        self._handle_worker_status(f"Final rotation failed: {e}")
                        QMessageBox.warning(self, "Rotation error", f"Final rotation failed: {e}")
                finally:
                    self._start_next_rotation_batch()
        except Exception as e:
            self._handle_worker_status(f"Batch finished handler exception: {e}")

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
                    
                    # Determine processing mode based on normalization dropdown
                    norm_mode = self.useNormalization.currentText()
                    
                    for key, value in peaks.items():
                        if norm_mode == "flat-field only":
                            # Use full trace mean (no baseline subtraction)
                            full_mean = np.mean(value)
                            peaks[key] = (full_mean, 0)  # Signal = full mean, dark current = 0
                        elif norm_mode == "leakage/flat-field from open beam":
                            # Calculate right_mean, but baseline correction will use open beam baseline
                            right_mean = np.mean(value[int(self.edgeRight.text()) :])
                            left_mean = np.mean(value[: int(self.edgeLeft.text())])
                            # Store right_mean as signal (will be corrected in build_image with open beam baseline)
                            peaks[key] = (right_mean, left_mean)  # Store both for later processing
                        else:
                            # Original behavior: use edge-based baseline subtraction
                            right_mean = np.mean(value[int(self.edgeRight.text()) :])
                            left_mean = np.mean(value[: int(self.edgeLeft.text())])
                            peaks[key] = (right_mean - left_mean, left_mean)
                except ValueError:
                    self.statusBar().showMessage("Invalid edge values")
                    return np.zeros((16, 16))

                # Always reconstruct BOTH 1D and 2D, regardless of current mode
                # 1D reconstruction
                array_image_1d = np.zeros(128)
                array_dark_1d = np.zeros(128)
                for i in range(128):
                    ch = int(self.decoder_matrix_1d[i])
                    if ch >= 10:
                        key = f"{ch}A"
                    else:
                        key = f"0{ch}A"
                    if key in peaks:
                        array_image_1d[i] = peaks[key][0]
                        array_dark_1d[i] = peaks[key][1]
                
                # 2D reconstruction
                array_image_2d = np.zeros((16, 16))
                array_dark_2d = np.zeros((16, 16))
                for i in range(16):
                    for j in range(16):
                        if self.decoder_matrix[i, j] >= 10:
                            array_image_2d[i, j] = peaks[f"{self.decoder_matrix[i,j]}A"][0]
                            array_dark_2d[i, j] = peaks[f"{self.decoder_matrix[i,j]}A"][1]
                        else:
                            array_image_2d[i, j] = peaks[f"0{self.decoder_matrix[i,j]}A"][0]
                            array_dark_2d[i, j] = peaks[f"0{self.decoder_matrix[i,j]}A"][1]
                array_image_2d = np.rot90(array_image_2d, 3)
                array_dark_2d = np.rot90(array_dark_2d, 3)

                # Store BOTH 1D and 2D data always, regardless of current mode
                # This allows switching modes without reloading files
                if data_attr == "image_data":
                    self.image_data_1d = np.array(array_image_1d, dtype=float)
                    self.image_data = np.array(array_image_2d, dtype=float)
                elif data_attr == "open_beam_data":
                    self.open_beam_data_1d = np.array(array_image_1d, dtype=float)
                    self.open_beam_data = np.array(array_image_2d, dtype=float)

                if update_dark:
                    self.dark_current_data_1d = np.array(array_dark_1d, dtype=float)
                    self.dark_current_data = np.array(array_dark_2d, dtype=float)

                # If loading open beam file, also store its baseline for mode 3
                if data_attr == "open_beam_data":
                    self.open_beam_baseline_1d = np.array(array_dark_1d, dtype=float)
                    self.open_beam_baseline = np.array(array_dark_2d, dtype=float)

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

    def load_decoder_matrix_1d(self, file_path):
        """Load 1D detector decoder matrix (128 channels)."""
        try:
            with open(file_path) as f:
                lines = f.readlines()
                for i, line in enumerate(lines):
                    if i < 128:
                        self.decoder_matrix_1d[i] = int(line.strip())
        except Exception as e:
            self.statusBar().showMessage(f"Failed to load 1D decoder matrix: {e}")

    def build_image(self):
        norm_mode = self.useNormalization.currentText()
        is_1d = self.detector1D.isChecked()
        # ensure layout orientation matches current mode
        self._update_1d_layout(is_1d)
        
        # make sure layout/orientation matches current mode
        self._update_1d_layout(is_1d)
        
        # Helper function to set image levels respecting manual scale input
        def set_image_levels(img_item, color_bar, image_data, low_field, upper_field, low_manual_flag, upper_manual_flag):
            """Set image and colorbar levels, respecting manual user input if set."""
            # For 1D arrays, reshape to 2D for display (128,) -> (128, 1) for vertical orientation
            if image_data.ndim == 1:
                display_data = image_data.reshape(-1, 1)
            else:
                display_data = image_data
            
            # Check if user has manually set the scales
            if low_manual_flag or upper_manual_flag:
                # Use manual values if they exist
                try:
                    low_val = float(low_field.text()) if low_manual_flag else display_data.min()
                    upper_val = float(upper_field.text()) if upper_manual_flag else display_data.max()
                    img_item.setLevels((low_val, upper_val))
                    color_bar.setLevels((low_val, upper_val))
                    # Don't update text fields - keep user's manual values
                except ValueError:
                    # If manual value is invalid, fall back to auto-detection
                    if np.isnan(display_data.min()) or np.isnan(display_data.max()):
                        img_item.setLevels((0, 1))
                        color_bar.setLevels((0, 1))
                    else:
                        img_item.setLevels((display_data.min(), display_data.max()))
                        color_bar.setLevels((display_data.min(), display_data.max()))
            else:
                # Auto-detection (original behavior)
                if np.isnan(display_data.min()) or np.isnan(display_data.max()):
                    img_item.setLevels((0, 1))
                    color_bar.setLevels((0, 1))
                    upper_field.setText("1")
                    low_field.setText("0")
                else:
                    img_item.setLevels((display_data.min(), display_data.max()))
                    color_bar.setLevels((display_data.min(), display_data.max()))
                    upper_field.setText(f"{display_data.max()}")
                    low_field.setText(f"{display_data.min()}")
        
        # Process left image based on normalization mode
        if norm_mode == "none":
            # Original absolute charge mode (no normalization)
            if not self.image_file:
                self.statusBar().showMessage("Please select image file")
            else:
                if is_1d:
                    # Apply same physical conversion as 2D to 1D stripes, then tile to 128x128
                    temp = (
                        self.image_data_1d
                        / float(self.pixelX.text())
                        / float(self.pixelY.text())
                        / float(self.ConvLowInt.text())
                        * 1e15
                    ).reshape(-1, 1)
                    left_image = np.tile(temp, (1, 128))
                else:
                    left_image = (
                        self.image_data
                        / float(self.pixelX.text())
                        / float(self.pixelY.text())
                        / float(self.ConvLowInt.text())
                        * 1e15
                    )
                self.img_item.setImage(left_image)
                self.image_view.autoRange()
                set_image_levels(
                    self.img_item, self.color_bar, left_image,
                    self.imageLowScale, self.imageUpperScale,
                    self.imageLowScale_manual, self.imageUpperScale_manual
                )
        
        elif norm_mode == "full leakage/flat-field":
            # Original normalization mode (baseline-subtracted signals divided)
            if (not self.image_file) or (not self.open_beam_file):
                self.statusBar().showMessage(
                    "Please select both image and open beam files"
                )
            else:
                if is_1d:
                    # avoid division by zero, tile to 128x128
                    denom = np.array(self.open_beam_data_1d, dtype=float)
                    denom[denom == 0] = np.nan
                    temp = (self.image_data_1d / denom).reshape(-1, 1)
                    left_image = np.tile(temp, (1, 128))

                else:
                    denom2 = np.array(self.open_beam_data, dtype=float)
                    denom2[denom2 == 0] = np.nan
                    left_image = self.image_data / denom2

                if self.useThreshold.isChecked():
                    left_image[left_image > 1] = 1

                self.img_item.setImage(left_image)
                self.image_view.autoRange()
                set_image_levels(
                    self.img_item, self.color_bar, left_image,
                    self.imageLowScale, self.imageUpperScale,
                    self.imageLowScale_manual, self.imageUpperScale_manual
                )
        
        elif norm_mode == "leakage/flat-field from open beam":
            # Image signal: right_mean from image - left_mean (baseline) from open beam
            # Then normalize by open beam signal
            if (not self.image_file) or (not self.open_beam_file):
                self.statusBar().showMessage(
                    "Please select both image and open beam files"
                )
            else:
                if is_1d:
                    # Calculate baseline-corrected image signal using open beam's baseline, tile to 128x128
                    image_signal = self.image_data_1d - self.open_beam_baseline_1d
                    denom = np.array(self.open_beam_data_1d, dtype=float)
                    denom[denom == 0] = np.nan
                    temp = (image_signal / denom).reshape(-1, 1)
                    left_image = np.tile(temp, (1, 128))
                else:
                    # self.image_data contains right_mean from image file (from load_file mode 3)
                    # self.open_beam_baseline contains left_mean from open beam file
                    # self.open_beam_data contains (right_mean - left_mean) from open beam file
                    
                    # Calculate baseline-corrected image signal using open beam's baseline
                    image_signal = self.image_data - self.open_beam_baseline
                    denom2 = np.array(self.open_beam_data, dtype=float)
                    denom2[denom2 == 0] = np.nan
                    left_image = image_signal / denom2

                if self.useThreshold.isChecked():
                    left_image[left_image > 1] = 1

                self.img_item.setImage(left_image)
                self.image_view.autoRange()
                set_image_levels(
                    self.img_item, self.color_bar, left_image,
                    self.imageLowScale, self.imageUpperScale,
                    self.imageLowScale_manual, self.imageUpperScale_manual
                )
        
        elif norm_mode == "flat-field only":
            # Use full mean for both files, no baseline subtraction
            if (not self.image_file) or (not self.open_beam_file):
                self.statusBar().showMessage(
                    "Please select both image and open beam files"
                )
            else:
                if is_1d:
                    denom = np.array(self.open_beam_data_1d, dtype=float)
                    denom[denom == 0] = np.nan
                    temp = (self.image_data_1d / denom).reshape(-1, 1)
                    left_image = np.tile(temp, (1, 128))
                else:
                    # Both files have full mean values (no baseline subtraction)
                    denom2 = np.array(self.open_beam_data, dtype=float)
                    denom2[denom2 == 0] = np.nan
                    left_image = self.image_data / denom2

                if self.useThreshold.isChecked():
                    left_image[left_image > 1] = 1

                self.img_item.setImage(left_image)
                self.image_view.autoRange()
                set_image_levels(
                    self.img_item, self.color_bar, left_image,
                    self.imageLowScale, self.imageUpperScale,
                    self.imageLowScale_manual, self.imageUpperScale_manual
                )

        if self.darkCurrent.isChecked():
            if not self.image_file:
                self.statusBar().showMessage("Please select image file")
            else:
                if is_1d:
                    temp = (
                        self.dark_current_data_1d
                        / float(self.pixelX.text())
                        / float(self.pixelY.text())
                        / float(self.ConvLowInt.text())
                        * 1e15
                    ).reshape(-1, 1)
                    right_image = np.tile(temp, (1, 128))
                else:
                    right_image = (
                        self.dark_current_data
                        / float(self.pixelX.text())
                        / float(self.pixelY.text())
                        / float(self.ConvLowInt.text())
                        * 1e15
                    )
                self.mix_img_item.setImage(right_image)
                self.mix_view.autoRange()
                set_image_levels(
                    self.mix_img_item, self.mix_color_bar, right_image,
                    self.mixLowScale, self.mixUpperScale,
                    self.mixLowScale_manual, self.mixUpperScale_manual
                )

        if self.openBeam.isChecked():
            if not self.open_beam_file:
                self.statusBar().showMessage("Please select open beam file")
            else:
                if is_1d:
                    temp = (
                        self.open_beam_data_1d
                        / float(self.pixelX.text())
                        / float(self.pixelY.text())
                        / float(self.ConvLowInt.text())
                        * 1e15
                    ).reshape(-1, 1)
                    right_image = np.tile(temp, (1, 128))
                else:
                    right_image = (
                        self.open_beam_data
                        / float(self.pixelX.text())
                        / float(self.pixelY.text())
                        / float(self.ConvLowInt.text())
                        * 1e15
                    )
                self.mix_img_item.setImage(right_image)
                self.mix_view.autoRange()
                set_image_levels(
                    self.mix_img_item, self.mix_color_bar, right_image,
                    self.mixLowScale, self.mixUpperScale,
                    self.mixLowScale_manual, self.mixUpperScale_manual
                )

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
