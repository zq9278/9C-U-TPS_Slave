# -*- coding: utf-8 -*-
import re
import struct
import sys
import time
from collections import deque

import serial
import serial.tools.list_ports

try:
    import pyqtgraph as pg
    from PyQt5.QtCore import QThread, QTimer, pyqtSignal, Qt
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
        QLineEdit,
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
SERIAL_READ_INTERVAL_S = 0.01
PID_SCALE = 1000
KP_MIN = 0.0
KP_MAX = 500.0
KI_MIN = 0.0
KI_MAX = 50.0
KD_MIN = 0.0
KD_MAX = 500.0
MAX_POINTS = 1500

PID_FRAME_MAGIC = b"\xC5\x5C"
PID_FRAME_LEN = 40
PID_FRAME = struct.Struct("<2sBBI7fH2s")
PID_KEYS = ["p", "i", "d", "error", "output", "setpoint", "feedback"]
PID_TEXT_RE = re.compile(
    r"PIDDBG\s+target=(?P<target>\w+)\s+t=(?P<t>\d+)\s+"
    r"p=(?P<p>[-+]?\d+(?:\.\d+)?)\s+i=(?P<i>[-+]?\d+(?:\.\d+)?)\s+"
    r"d=(?P<d>[-+]?\d+(?:\.\d+)?)\s+err=(?P<error>[-+]?\d+(?:\.\d+)?)\s+"
    r"out=(?P<output>[-+]?\d+(?:\.\d+)?)\s+sp=(?P<setpoint>[-+]?\d+(?:\.\d+)?)\s+"
    r"fb=(?P<feedback>[-+]?\d+(?:\.\d+)?)"
)


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def parse_pid_text(line: str):
    match = PID_TEXT_RE.search(line)
    if not match:
        return None
    sample = {"target": match.group("target"), "tick_ms": int(match.group("t"))}
    for key in PID_KEYS:
        sample[key] = float(match.group(key))
    return sample


class MixedPidParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        text_chunks = []
        samples = []

        while self.buffer:
            magic_index = self.buffer.find(PID_FRAME_MAGIC)
            if magic_index < 0:
                text_chunks.append(bytes(self.buffer))
                self.buffer.clear()
                break

            if magic_index > 0:
                text_chunks.append(bytes(self.buffer[:magic_index]))
                del self.buffer[:magic_index]

            if len(self.buffer) < PID_FRAME_LEN:
                break

            frame = bytes(self.buffer[:PID_FRAME_LEN])
            if frame[-2:] != b"\r\n":
                text_chunks.append(bytes(self.buffer[:1]))
                del self.buffer[:1]
                continue

            crc_recv = struct.unpack_from("<H", frame, PID_FRAME_LEN - 4)[0]
            crc_calc = crc16_modbus(frame[2:PID_FRAME_LEN - 4])
            if crc_recv != crc_calc:
                text_chunks.append(bytes(self.buffer[:1]))
                del self.buffer[:1]
                continue

            unpacked = PID_FRAME.unpack(frame)
            sample = {
                "target": str(unpacked[2]),
                "tick_ms": unpacked[3],
            }
            for key, value in zip(PID_KEYS, unpacked[4:11]):
                sample[key] = value
            samples.append(sample)
            del self.buffer[:PID_FRAME_LEN]

        text = b"".join(text_chunks).decode("utf-8", errors="replace")
        for line in text.splitlines():
            sample = parse_pid_text(line)
            if sample:
                samples.append(sample)
        return text, samples


class SerialWorker(QThread):
    received_text = pyqtSignal(str)
    received_sample = pyqtSignal(dict)
    connected = pyqtSignal(str)
    disconnected = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self, port_name: str, baudrate: int):
        super().__init__()
        self.port_name = port_name
        self.baudrate = baudrate
        self._serial = None
        self._running = True
        self.parser = MixedPidParser()

    def run(self):
        try:
            self._serial = serial.Serial(self.port_name, self.baudrate, timeout=0.03)
            self.connected.emit(f"Connected {self.port_name} @ {self.baudrate}")
        except serial.SerialException as exc:
            self.error.emit(f"Open serial failed: {exc}")
            return

        while self._running:
            try:
                data = self._serial.read(512)
                if data:
                    text, samples = self.parser.feed(data)
                    if text:
                        self.received_text.emit(text)
                    for sample in samples:
                        self.received_sample.emit(sample)
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
        self._syncing = False
        self.label = QLabel(name)
        self.spin = QDoubleSpinBox()
        self.spin.setRange(minimum, maximum)
        self.spin.setDecimals(4)
        self.spin.setSingleStep(0.1)
        self.spin.setValue(value)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(round(minimum * PID_SCALE)), int(round(maximum * PID_SCALE)))
        self.slider.setValue(int(round(value * PID_SCALE)))

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

    def _spin_changed(self, value: float):
        if self._syncing:
            return
        self._syncing = True
        self.slider.setValue(int(round(value * PID_SCALE)))
        self._syncing = False

    def _slider_changed(self, value: int):
        if self._syncing:
            return
        self._syncing = True
        self.spin.setValue(value / PID_SCALE)
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
        self.send_requested.emit(self.command, self.kp.value(), self.ki.value(), self.kd.value())


class CurveRow(QWidget):
    def __init__(self, name: str):
        super().__init__()
        self.name = name
        self.visible = QCheckBox(name)
        self.visible.setChecked(True)
        self.scale = QDoubleSpinBox()
        self.scale.setRange(-1000000.0, 1000000.0)
        self.scale.setDecimals(4)
        self.scale.setValue(1.0)
        self.offset = QDoubleSpinBox()
        self.offset.setRange(-1000000.0, 1000000.0)
        self.offset.setDecimals(4)
        self.offset.setValue(0.0)
        self.auto_range = QCheckBox("Y")
        self.auto_range.setChecked(True)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.visible)
        layout.addWidget(QLabel("scale"))
        layout.addWidget(self.scale)
        layout.addWidget(QLabel("offset"))
        layout.addWidget(self.offset)
        layout.addWidget(QLabel("auto"))
        layout.addWidget(self.auto_range)


class PidCurveBox(QGroupBox):
    def __init__(self):
        super().__init__("PID Debug Curve")
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.addLegend()
        colors = ["#2E86DE", "#E74C3C", "#27AE60", "#8E44AD", "#F39C12", "#16A085", "#34495E"]
        self.items = {}
        self.rows = {}

        rows = QGridLayout()
        for index, key in enumerate(PID_KEYS):
            self.items[key] = self.plot.plot([], [], pen=pg.mkPen(colors[index], width=2), name=key)
            self.rows[key] = CurveRow(key)
            rows.addWidget(self.rows[key], index // 2, index % 2)

        layout = QVBoxLayout(self)
        layout.addWidget(self.plot, 1)
        layout.addLayout(rows)

    def update_data(self, x_values, buffers):
        x = list(x_values)
        for key in PID_KEYS:
            row = self.rows[key]
            item = self.items[key]
            item.setVisible(row.visible.isChecked())
            y = [(value * row.scale.value()) + row.offset.value() for value in buffers[key]]
            item.setData(x, y)
        self.plot.enableAutoRange(axis="y", enable=any(row.auto_range.isChecked() for row in self.rows.values()))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Single PID Tuner")
        self.resize(1180, 760)
        self.worker = None
        self.x_values = deque(maxlen=MAX_POINTS)
        self.buffers = {key: deque(maxlen=MAX_POINTS) for key in PID_KEYS}

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

        self.curve = PidCurveBox()
        self.target_label = QLineEdit()
        self.target_label.setReadOnly(True)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.clear_log_btn = QPushButton("Clear Log")
        self.clear_curve_btn = QPushButton("Clear Curve")

        self.plot_timer = QTimer(self)
        self.plot_timer.setInterval(50)
        self.plot_timer.timeout.connect(self.refresh_curve)
        self.plot_timer.start()

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
        log_top.addWidget(QLabel("Target"))
        log_top.addWidget(self.target_label)
        log_top.addStretch(1)
        log_top.addWidget(self.clear_curve_btn)
        log_top.addWidget(self.clear_log_btn)

        layout = QVBoxLayout(root)
        layout.addLayout(top)
        layout.addLayout(pid_layout)
        layout.addWidget(self.curve, 3)
        layout.addLayout(log_top)
        layout.addWidget(self.log, 1)

    def _connect_signals(self):
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.clear_log_btn.clicked.connect(self.log.clear)
        self.clear_curve_btn.clicked.connect(self.clear_curve)
        self.rise_box.send_requested.connect(self.send_pid)
        self.hold_box.send_requested.connect(self.send_pid)
        self.pulse_box.send_requested.connect(self.send_pid)

    def refresh_ports(self):
        current_port = self.port_combo.currentData()
        self.port_combo.clear()
        for port in serial.tools.list_ports.comports():
            self.port_combo.addItem(f"{port.device}  {port.description}", port.device)
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
        self.worker.received_text.connect(self.append_log)
        self.worker.received_sample.connect(self.append_sample)
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

    def append_sample(self, sample: dict):
        self.target_label.setText(str(sample["target"]))
        self.x_values.append(sample["tick_ms"] / 1000.0)
        for key in PID_KEYS:
            self.buffers[key].append(sample[key])

    def refresh_curve(self):
        self.curve.update_data(self.x_values, self.buffers)

    def clear_curve(self):
        self.x_values.clear()
        for buffer in self.buffers.values():
            buffer.clear()
        self.refresh_curve()

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
    pg.setConfigOptions(antialias=True)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
