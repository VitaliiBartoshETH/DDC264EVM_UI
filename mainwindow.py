from PyQt5 import uic
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot, QThread
from PyQt5.QtWidgets import QMainWindow, QFileDialog, QVBoxLayout
from tools import FPGAControl
import pyqtgraph as pg
import numpy as np
import os


class ReaderWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)
    status = pyqtSignal(str)

    def __init__(self, fpga, folder_path, file_name, numFiles):
        super().__init__()
        self.fpga = fpga
        self.folder_path = folder_path
        self.file_name = file_name
        self.numFiles = numFiles
        self.readFilePath = None

    @pyqtSlot()
    def run(self):
        existing_indices = []
        if os.path.isdir(self.folder_path):
            for fname in os.listdir(self.folder_path):
                if fname.startswith(f"{self.file_name}_") and fname.endswith(".txt"):
                    try:
                        idx = int(fname[len(f"{self.file_name}_") : -4])
                        existing_indices.append(idx)
                    except ValueError:
                        continue
        start_index = max(existing_indices) if existing_indices else 0

        try:
            for i in range(self.numFiles):
                file_index = start_index + i + 1
                result = self.fpga.get_data(
                    f"{self.folder_path}\\{self.file_name}", file_index - 1
                )
                self.status.emit(f"File {file_index} saved successfully")
                self.progress.emit(i + 1)
            self.status.emit("Data read successfully")
        except Exception as e:
            self.status.emit(f"Error during data capture: {str(e)}")
        finally:
            self.finished.emit()
        self.readFilePath = f"{self.file_name}_{start_index + self.numFiles}.txt"


class Ui(QMainWindow):
    conv_config = {"Free run": 0, "Low": 2, "High": 3}
    ddc_clk_config = {"Running": 1, "Low": 0}
    dclk_config = {"Running": 1, "Low": 0}
    hardware_trigger = {"Disabled": 0, "Enabled": 1}

    def __init__(self):
        super().__init__()

        uic.loadUi("mainwindow.ui", self)

        self.setWindowTitle("DDC264EVM_UI")

        self.ConvLowInt.setText("10000")
        self.ConvHighInt.setText("10000")

        self.ConvConfig.addItem("Free run")
        self.ConvConfig.addItem("Low")
        self.ConvConfig.addItem("High")

        self.MCLKFreq.setText("80.0")
        self.MCLKFreq.setDisabled(True)

        self.CLKHigh.setText("7")
        self.CLKLow.setText("7")

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

        self.DCLKWait.setText("13000")

        self.HardwareTrigger.addItem("Disabled")
        self.HardwareTrigger.addItem("Enabled")

        self.CLK_CFGHigh.setText("3")
        self.CLK_CFGLow.setText("3")

        self.ADCrange.addItem("150.0")
        self.ADCrange.addItem("100.0")
        self.ADCrange.addItem("50.0")
        self.ADCrange.addItem("12.5")
        
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
        if os.path.isdir(public_documents):
            self.saveFolderLabel.setText(public_documents)
        else:
            self.saveFolderLabel.setText(
                os.path.join(os.path.expanduser("~"), "Documents")
            )

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
        self.graphWidget.setMouseEnabled(x=False, y=False)

        self.imageWidget = pg.GraphicsLayoutWidget()
        image_layout = QVBoxLayout(self.imagePlot)
        image_layout.addWidget(self.imageWidget)

        self.mixWidget = pg.GraphicsLayoutWidget()
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

        cmap = pg.colormap.get("viridis")
        lut = cmap.getLookupTable(0.0, 1.0, 256)
        self.img_item.setLookupTable(lut)

        self.color_bar = pg.ColorBarItem(
            values=(0, 1), colorMap=cmap, interactive=False
        )
        self.color_bar.setImageItem(self.img_item)
        self.imageWidget.addItem(self.color_bar)

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

        self.file_data = {}
        self.image_data = np.zeros((16, 16))
        self.dark_current_data = np.zeros((16, 16))
        self.open_beam_data = np.zeros((16, 16))
        self.readFilePath.setText("")

        self.openBeam.setChecked(False)
        self.darkCurrent.setChecked(True)

        self.update_registers(is_startup=True)

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

    def refresh_registers(self):
        try:
            self.fpga.refresh()
            self.update_registers()
            self.statusBar().showMessage("Registers refreshed and updated successfully")
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
                numFiles = int(self.nFiles.text())
                if numFiles <= 0:
                    raise ValueError
                folder_path = self.saveFolderLabel.text()
                if folder_path:
                    self.progressBar.setMaximum(numFiles)
                    self.progressBar.setValue(0)
                    self.progressBar.show()
                    file_name = self.saveFileName.text() or "file"
                    self.thread = QThread()
                    self.worker = ReaderWorker(
                        self.fpga, folder_path, file_name, numFiles
                    )
                    self.worker.moveToThread(self.thread)

                    self.thread.started.connect(self.worker.run)
                    self.worker.progress.connect(self.progressBar.setValue)
                    self.worker.status.connect(self.statusBar().showMessage)
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
                    symbolSize=10,
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
