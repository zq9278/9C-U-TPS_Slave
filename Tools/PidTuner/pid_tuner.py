# -*- coding: utf-8 -*-
import sys
import time

import serial
import serial.tools.list_ports

try:
    from PyQt5.QtCore import QThread, pyqtSignal, Qt
    from PyQt5.QtGui import QTextCursor
    from PyQt5.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QSlider,
        QSpinBox,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    raise SystemExit("Please install dependencies first: python -m pip install -r requirements.txt") from exc


DEFAULT_BAUDRATE = 115200
SERIAL_READ_INTERVAL_S = 0.02

PID_SCALE = 1000
KP_MIN = 0.0
KP_MAX = 500.0
KI_MIN = 0.0
KI_MAX = 50.0
KD_MIN = 0.0
KD_MAX = 500.0


class SerialWorker(QThread):
    received = pyqtSignal(str)
    connected = pyqtSignal(str)
    disconnected = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, port_name: str, baudrate: int):
        super().__init__()
        self.port_name = port_name
        self.baudrate = baudrate
        self._serial = None
        self._running = True

    def run(self):
        try:
            self._serial = serial.Serial(self.port_name, self.baudrate, timeout=0.05)
            self.connected.emit(f"Connected {self.port_name} @ {self.baudrate}")
        except serial.SerialException as exc:
            self.error.emit(f"Open serial failed: {exc}")
            return

        while self._running:
            try:
                data = self._serial.read(512)
                if data:
                    self.received.emit(data.decode("utf-8", errors="replace"))
            except serial.SerialException as exc:
                self.error.emit(f"Serial read failed: {exc}")
                break
            time.sleep(SERIAL_READ_INTERVAL_S)

        if self._serial is not None:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
        self.disconnected.emit("Disconnected")

    def stop(self):
        self._running = False

    def send_line(self, line: str):
        if (self._serial is None) or (not self._serial.is_open):
            self.error.emit("Serial is not connected")
            return
        try:
            self._serial.write((line.strip() + "\r\n").encode("ascii"))
        except serial.SerialException as exc:
            self.error.emit(f"Serial write failed: {exc}")


class PidControlRow(QWidget):
    changed_by_slider = pyqtSignal()

    def __init__(self, name: str, value: float, minimum: float, maximum: float):
        super().__init__()
        self.minimum = minimum
        self.maximum = maximum
        self._syncing = False

        self.label = QLabel(name)
        self.spin = QDoubleSpinBox()
        self.spin.setRange(minimum, maximum)
        self.spin.setDecimals(4)
        self.spin.setSingleStep(0.1)
        self.spin.setValue(value)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(self._to_slider(minimum), self._to_slider(maximum))
        self.slider.setValue(self._to_slider(value))

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin)

        self.spin.valueChanged.connect(self._spin_changed)
        self.slider.valueChanged.connect(self._slider_changed)
        self.slider.sliderReleased.connect(self.changed_by_slider.emit)

    def value(self) -> float:
        return self.spin.value()

    def _to_slider(self, value: float) -> int:
        return int(round(value * PID_SCALE))

    def _from_slider(self, value: int) -> float:
        return value / PID_SCALE

    def _spin_changed(self, value: float):
        if self._syncing:
            return
        self._syncing = True
        self.slider.setValue(self._to_slider(value))
        self._syncing = False

    def _slider_changed(self, value: int):
        if self._syncing:
            return
        self._syncing = True
        self.spin.setValue(self._from_slider(value))
        self._syncing = False


class PidStageBox(QGroupBox):
    send_requested = pyqtSignal(str, float, float, float)

    def __init__(self, title: str, command: str, kp: float, ki: float, kd: float):
        super().__init__(title)
        self.command = command

        self.kp = PidControlRow("Kp", kp, KP_MIN, KP_MAX)
        self.ki = PidControlRow("Ki", ki, KI_MIN, KI_MAX)
        self.kd = PidControlRow("Kd", kd, KD_MIN, KD_MAX)
        self.auto_send = QCheckBox("Send on slider release")
        self.auto_send.setChecked(True)
        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self._emit_send)

        layout = QGridLayout(self)
        layout.addWidget(self.kp, 0, 0, 1, 2)
        layout.addWidget(self.ki, 1, 0, 1, 2)
        layout.addWidget(self.kd, 2, 0, 1, 2)
        layout.addWidget(self.auto_send, 3, 0)
        layout.addWidget(self.send_btn, 3, 1)

        self.kp.changed_by_slider.connect(self._emit_send_if_auto)
        self.ki.changed_by_slider.connect(self._emit_send_if_auto)
        self.kd.changed_by_slider.connect(self._emit_send_if_auto)

    def _emit_send_if_auto(self):
        if self.auto_send.isChecked():
            self._emit_send()

    def _emit_send(self):
        self.send_requested.emit(
            self.command,
            self.kp.value(),
            self.ki.value(),
            self.kd.value(),
        )


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Pressure PID Tuner")
        self.resize(980, 640)
        self.worker = None

        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("Refresh")
        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(1200, 3000000)
        self.baud_spin.setValue(DEFAULT_BAUDRATE)
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)

        self.rise_box = PidStageBox("RISE", "R", 80.0, 0.0, 0.0)
        self.hold_box = PidStageBox("HOLD", "H", 140.0, 8.0, 0.0)
        self.pulse_box = PidStageBox("PULSE", "P", 220.0, 0.0, 0.0)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.clear_log_btn = QPushButton("Clear")

        self._build_ui()
        self._connect_signals()
        self.refresh_ports()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)

        top = QHBoxLayout()
        top.addWidget(QLabel("Port"))
        top.addWidget(self.port_combo, 2)
        top.addWidget(self.refresh_btn)
        top.addWidget(QLabel("Baud"))
        top.addWidget(self.baud_spin)
        top.addWidget(self.connect_btn)
        top.addWidget(self.disconnect_btn)

        pid_layout = QHBoxLayout()
        pid_layout.addWidget(self.rise_box)
        pid_layout.addWidget(self.hold_box)
        pid_layout.addWidget(self.pulse_box)

        log_top = QHBoxLayout()
        log_top.addWidget(QLabel("Log"))
        log_top.addStretch(1)
        log_top.addWidget(self.clear_log_btn)

        layout = QVBoxLayout(root)
        layout.addLayout(top)
        layout.addLayout(pid_layout)
        layout.addLayout(log_top)
        layout.addWidget(self.log, 1)

    def _connect_signals(self):
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.clear_log_btn.clicked.connect(self.log.clear)

        self.rise_box.send_requested.connect(self.send_pid)
        self.hold_box.send_requested.connect(self.send_pid)
        self.pulse_box.send_requested.connect(self.send_pid)

    def refresh_ports(self):
        current_port = self.port_combo.currentData()
        self.port_combo.clear()
        for port in serial.tools.list_ports.comports():
            label = f"{port.device}  {port.description}"
            self.port_combo.addItem(label, port.device)

        if current_port:
            for index in range(self.port_combo.count()):
                if self.port_combo.itemData(index) == current_port:
                    self.port_combo.setCurrentIndex(index)
                    break

    def connect_serial(self):
        port_name = self.port_combo.currentData()
        if not port_name:
            QMessageBox.warning(self, "Serial", "No serial port available")
            return

        self.worker = SerialWorker(port_name, self.baud_spin.value())
        self.worker.received.connect(self.append_log)
        self.worker.connected.connect(self.on_connected)
        self.worker.disconnected.connect(self.on_disconnected)
        self.worker.error.connect(self.on_error)
        self.worker.start()

    def disconnect_serial(self):
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(1000)

    def on_connected(self, text: str):
        self.append_log(f"[PC] {text}\n")
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)

    def on_disconnected(self, text: str):
        self.append_log(f"[PC] {text}\n")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.worker = None

    def on_error(self, text: str):
        self.append_log(f"[PC] {text}\n")

    def append_log(self, text: str):
        self.log.moveCursor(QTextCursor.End)
        self.log.insertPlainText(text)
        self.log.moveCursor(QTextCursor.End)

    def send_pid(self, command: str, kp: float, ki: float, kd: float):
        line = f"{command} {kp:.4f} {ki:.4f} {kd:.4f}"
        self.append_log(f"[PC->MCU] {line}\n")
        if self.worker is None:
            self.on_error("Serial is not connected")
            return
        self.worker.send_line(line)

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
