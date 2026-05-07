# -*- coding: utf-8 -*-
import logging
import os
import struct
import sys
import time
import traceback
from collections import deque

import serial
import serial.tools.list_ports

try:
    import pyqtgraph as pg
    from PyQt5.QtCore import QThread, QTimer, pyqtSignal, Qt
    from PyQt5.QtGui import QFont, QTextCursor
    from PyQt5.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFormLayout,
        QFrame,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QPlainTextEdit,
        QScrollArea,
        QSlider,
        QSpinBox,
        QSplitter,
        QVBoxLayout,
        QWidget,
    )
except ImportError as exc:
    raise SystemExit("Please install dependencies first: python -m pip install -r requirements.txt") from exc


DEFAULT_BAUDRATE = 115200
SERIAL_READ_INTERVAL_S = 0.01
PID_SCALE = 10000
MAX_POINTS = 1200
MAX_LOG_BLOCKS = 800
DEBUG_LOG_NAME = "pid_tuner_debug.log"
FRAME_MAGIC = b"\xC5\x5C"
FRAME_LEN = 56
FRAME_STRUCT = struct.Struct("<2sBBI11fH2s")
CURVE_KEYS = ["p", "i", "d", "error", "output", "mapped_output", "setpoint", "measurement"]

PID_CMD_SET_GAINS = 0x01
PID_CMD_STREAM_ENABLE = 0x02

TARGETS = [
    {"id": 0, "code": "PR", "name": "Pressure Rise"},
    {"id": 1, "code": "PH", "name": "Pressure Hold"},
    {"id": 2, "code": "PP", "name": "Pressure Pulse"},
    {"id": 3, "code": "HL", "name": "Heat Left"},
    {"id": 4, "code": "HR", "name": "Heat Right"},
]
TARGET_BY_ID = {item["id"]: item for item in TARGETS}
TARGET_BY_CODE = {item["code"]: item for item in TARGETS}

DEFAULT_GAINS = {
    "PR": (0.0800, 0.0000, 0.0000),
    "PH": (0.1400, 4.0000, 0.0000),
    "PP": (0.2200, 0.0000, 0.0000),
    "HL": (0.0010, 0.0000, 0.0000),
    "HR": (0.0010, 0.0000, 0.0000),
}

RANGES = {
    "PR": ((0.0, 2.0), (0.0, 20.0), (0.0, 2.0)),
    "PH": ((0.0, 2.0), (0.0, 20.0), (0.0, 2.0)),
    "PP": ((0.0, 2.0), (0.0, 20.0), (0.0, 2.0)),
    "HL": ((0.0, 0.05), (0.0, 10.0), (0.0, 0.05)),
    "HR": ((0.0, 0.05), (0.0, 10.0), (0.0, 0.05)),
}

CURVE_COLORS = {
    "p": "#4f8cff",
    "i": "#ff6b6b",
    "d": "#2ecc71",
    "error": "#a66cff",
    "output": "#ff9f43",
    "mapped_output": "#f1c40f",
    "setpoint": "#00b894",
    "measurement": "#95a5a6",
}

CURVE_LABELS = {
    "p": "p",
    "i": "i",
    "d": "d",
    "error": "error",
    "output": "output",
    "mapped_output": "pwm_actual",
    "setpoint": "setpoint",
    "measurement": "measurement",
}

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEBUG_LOG_PATH = os.path.join(BASE_DIR, DEBUG_LOG_NAME)


def setup_debug_logger():
    logger = logging.getLogger("pid_tuner")
    if logger.handlers:
        return logger

    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler(DEBUG_LOG_PATH, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logger.addHandler(handler)
    logger.propagate = False
    logger.info("logger start")
    return logger


LOGGER = setup_debug_logger()


def excepthook(exc_type, exc_value, exc_tb):
    LOGGER.error("uncaught exception\n%s", "".join(traceback.format_exception(exc_type, exc_value, exc_tb)))
    sys.__excepthook__(exc_type, exc_value, exc_tb)


sys.excepthook = excepthook


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


class MixedPidParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        text_chunks = []
        samples = []

        while self.buffer:
            magic_index = self.buffer.find(FRAME_MAGIC)
            if magic_index < 0:
                text_chunks.append(bytes(self.buffer))
                self.buffer.clear()
                break

            if magic_index > 0:
                text_chunks.append(bytes(self.buffer[:magic_index]))
                del self.buffer[:magic_index]

            if len(self.buffer) < FRAME_LEN:
                break

            frame = bytes(self.buffer[:FRAME_LEN])
            if frame[-2:] != b"\r\n":
                text_chunks.append(bytes(self.buffer[:1]))
                del self.buffer[:1]
                continue

            crc_recv = struct.unpack_from("<H", frame, FRAME_LEN - 4)[0]
            crc_calc = crc16_modbus(frame[2:FRAME_LEN - 4])
            if crc_recv != crc_calc:
                text_chunks.append(bytes(self.buffer[:1]))
                del self.buffer[:1]
                continue

            unpacked = FRAME_STRUCT.unpack(frame)
            target = TARGET_BY_ID.get(unpacked[2])
            if target is not None:
                samples.append(
                    {
                        "target_id": target["id"],
                        "target": target["code"],
                        "target_name": target["name"],
                        "tick_ms": unpacked[3],
                        "kp": unpacked[4],
                        "ki": unpacked[5],
                        "kd": unpacked[6],
                        "p": unpacked[7],
                        "i": unpacked[8],
                        "d": unpacked[9],
                        "error": unpacked[10],
                        "output": unpacked[11],
                        "mapped_output": unpacked[12],
                        "setpoint": unpacked[13],
                        "measurement": unpacked[14],
                    }
                )
            del self.buffer[:FRAME_LEN]

        return b"".join(text_chunks).decode("utf-8", errors="replace"), samples


def build_pid_set_frame(target_id: int, kp: float, ki: float, kd: float) -> bytes:
    payload = struct.pack("<Bfff", target_id, kp, ki, kd)
    body = struct.pack("<BBB", 0x01, PID_CMD_SET_GAINS, len(payload)) + payload
    crc = crc16_modbus(body)
    return FRAME_MAGIC + body + struct.pack("<H", crc)


def build_pid_stream_frame(enabled: bool) -> bytes:
    payload = struct.pack("<B", 1 if enabled else 0)
    body = struct.pack("<BBB", 0x01, PID_CMD_STREAM_ENABLE, len(payload)) + payload
    crc = crc16_modbus(body)
    return FRAME_MAGIC + body + struct.pack("<H", crc)


class SerialWorker(QThread):
    received_text = pyqtSignal(str)
    received_samples = pyqtSignal(list)
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
            LOGGER.info("serial connected port=%s baud=%s", self.port_name, self.baudrate)
            self.connected.emit(f"Connected {self.port_name} @ {self.baudrate}")
        except serial.SerialException as exc:
            LOGGER.exception("serial open failed")
            self.error.emit(f"Open serial failed: {exc}")
            return

        while self._running:
            try:
                data = self._serial.read(512)
                if data:
                    text, samples = self.parser.feed(data)
                    LOGGER.debug("serial read bytes=%d text_len=%d samples=%d", len(data), len(text), len(samples))
                    if text:
                        self.received_text.emit(text)
                    if samples:
                        self.received_samples.emit(samples)
            except serial.SerialException as exc:
                LOGGER.exception("serial read failed")
                self.error.emit(f"Serial read failed: {exc}")
                break
            except Exception:
                LOGGER.exception("worker loop exception")
                self.error.emit("Worker exception, see pid_tuner_debug.log")
                break
            time.sleep(SERIAL_READ_INTERVAL_S)

        if self._serial is not None:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
        LOGGER.info("serial disconnected")
        self.disconnected.emit("Disconnected")

    def stop(self):
        self._running = False

    def send_bytes(self, data: bytes):
        if (self._serial is None) or (not self._serial.is_open):
            LOGGER.warning("serial write rejected not connected bytes=%d", len(data) if data is not None else 0)
            self.error.emit("Serial is not connected")
            return
        try:
            LOGGER.info("serial write bytes=%d", len(data))
            self._serial.write(data)
        except serial.SerialException as exc:
            LOGGER.exception("serial write failed")
            self.error.emit(f"Serial write failed: {exc}")


class GainRow(QWidget):
    changed_by_slider = pyqtSignal()

    def __init__(self, name: str, value: float, minimum: float, maximum: float):
        super().__init__()
        self._syncing = False
        self.label = QLabel(name)
        self.label.setFixedWidth(28)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(round(minimum * PID_SCALE)), int(round(maximum * PID_SCALE)))
        self.spin = QDoubleSpinBox()
        self.spin.setRange(minimum, maximum)
        self.spin.setDecimals(4)
        self.spin.setSingleStep(max((maximum - minimum) / 200.0, 0.0001))

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)
        layout.addWidget(self.label)
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.spin)

        self.set_value(value)
        self.spin.valueChanged.connect(self._spin_changed)
        self.slider.valueChanged.connect(self._slider_changed)
        self.slider.sliderReleased.connect(self.changed_by_slider.emit)

    def value(self) -> float:
        return self.spin.value()

    def set_value(self, value: float):
        self._syncing = True
        self.spin.setValue(value)
        self.slider.setValue(int(round(value * PID_SCALE)))
        self._syncing = False

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


class PidTuneCard(QFrame):
    send_requested = pyqtSignal(str, float, float, float)

    def __init__(self, code: str, title: str):
        super().__init__()
        self.code = code
        self.setObjectName("pidCard")
        self.title = QLabel(title)
        self.title.setObjectName("cardTitle")
        self.auto_send = QCheckBox("Auto send on slider release")
        self.auto_send.setChecked(True)
        self.send_btn = QPushButton("Send")
        self.send_btn.clicked.connect(self._emit_send)

        kp, ki, kd = DEFAULT_GAINS[code]
        kp_range, ki_range, kd_range = RANGES[code]
        self.rows = {
            "kp": GainRow("Kp", kp, kp_range[0], kp_range[1]),
            "ki": GainRow("Ki", ki, ki_range[0], ki_range[1]),
            "kd": GainRow("Kd", kd, kd_range[0], kd_range[1]),
        }

        body = QVBoxLayout(self)
        body.setContentsMargins(14, 14, 14, 14)
        body.setSpacing(10)
        body.addWidget(self.title)
        for row in self.rows.values():
            body.addWidget(row)
            row.changed_by_slider.connect(self._emit_send_if_auto)

        footer = QHBoxLayout()
        footer.addWidget(self.auto_send)
        footer.addStretch(1)
        footer.addWidget(self.send_btn)
        body.addLayout(footer)

    def values(self):
        return self.rows["kp"].value(), self.rows["ki"].value(), self.rows["kd"].value()

    def set_values(self, kp: float, ki: float, kd: float):
        self.rows["kp"].set_value(kp)
        self.rows["ki"].set_value(ki)
        self.rows["kd"].set_value(kd)

    def _emit_send_if_auto(self):
        if self.auto_send.isChecked():
            self._emit_send()

    def _emit_send(self):
        kp, ki, kd = self.values()
        self.send_requested.emit(self.code, kp, ki, kd)


class CurveControlBar(QFrame):
    def __init__(self):
        super().__init__()
        self.setObjectName("curveBar")
        self.controls = {}
        layout = QGridLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setHorizontalSpacing(16)
        layout.setVerticalSpacing(8)

        for index, key in enumerate(CURVE_KEYS):
            box = QCheckBox(CURVE_LABELS[key])
            box.setChecked(True)
            scale = QDoubleSpinBox()
            scale.setRange(-1000000.0, 1000000.0)
            scale.setDecimals(4)
            scale.setValue(1.0)
            offset = QDoubleSpinBox()
            offset.setRange(-1000000.0, 1000000.0)
            offset.setDecimals(4)
            offset.setValue(0.0)
            self.controls[key] = {"visible": box, "scale": scale, "offset": offset}

            row = index // 4
            col = (index % 4) * 4
            layout.addWidget(box, row, col)
            layout.addWidget(QLabel("scale"), row, col + 1)
            layout.addWidget(scale, row, col + 2)
            layout.addWidget(offset, row, col + 3)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID Tuner")
        self.resize(1460, 900)
        self.worker = None
        self.stream_enabled = False
        self.current_target = "PR"

        self.series_x = {item["code"]: deque(maxlen=MAX_POINTS) for item in TARGETS}
        self.series_y = {
            item["code"]: {key: deque(maxlen=MAX_POINTS) for key in CURVE_KEYS}
            for item in TARGETS
        }
        self.last_samples = {item["code"]: None for item in TARGETS}
        self._sample_batches = 0
        self._sample_count = 0
        self._max_batch = 0
        self._max_refresh_ms = 0.0
        self._last_diag_ts = time.time()

        self.port_combo = QComboBox()
        self.refresh_btn = QPushButton("Refresh")
        self.baud_spin = QSpinBox()
        self.baud_spin.setRange(1200, 3000000)
        self.baud_spin.setValue(DEFAULT_BAUDRATE)
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)
        self.stream_btn = QPushButton("Enable PID Stream")

        self.tune_cards = {
            item["code"]: PidTuneCard(item["code"], item["name"])
            for item in TARGETS
        }

        self.target_combo = QComboBox()
        for item in TARGETS:
            self.target_combo.addItem(f'{item["code"]}  {item["name"]}', item["code"])

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.addLegend()
        self.plot.setBackground("#111315")
        self.plot_items = {
            key: self.plot.plot([], [], pen=pg.mkPen(CURVE_COLORS[key], width=2), name=CURVE_LABELS[key])
            for key in CURVE_KEYS
        }
        self.curve_bar = CurveControlBar()
        self.clear_curve_btn = QPushButton("Clear Curve")

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.clear_log_btn = QPushButton("Clear Log")

        self.plot_timer = QTimer(self)
        self.plot_timer.setInterval(80)
        self.plot_timer.timeout.connect(self.refresh_visuals)
        self.plot_timer.start()

        self._build_ui()
        self._apply_style()
        self._connect_signals()
        self.refresh_ports()
        LOGGER.info("mainwindow init")

    def _build_ui(self):
        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(14, 14, 14, 14)
        root_layout.setSpacing(12)
        self.setCentralWidget(root)

        top = QHBoxLayout()
        top.setSpacing(10)
        top.addWidget(QLabel("Port"))
        top.addWidget(self.port_combo, 2)
        top.addWidget(self.refresh_btn)
        top.addWidget(QLabel("Baud"))
        top.addWidget(self.baud_spin)
        top.addWidget(self.connect_btn)
        top.addWidget(self.disconnect_btn)
        top.addWidget(self.stream_btn)
        root_layout.addLayout(top)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        root_layout.addWidget(splitter, 1)

        left_panel = self._build_left_panel()
        right_panel = self._build_right_panel()
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([480, 980])

    def _build_left_panel(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        title = QLabel("PID Tuning")
        title.setObjectName("sectionTitle")
        layout.addWidget(title)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll_body = QWidget()
        body_layout = QVBoxLayout(scroll_body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(12)

        pressure_group = QGroupBox("Pressure")
        pressure_layout = QVBoxLayout(pressure_group)
        pressure_layout.setSpacing(10)
        for code in ["PR", "PH", "PP"]:
            pressure_layout.addWidget(self.tune_cards[code])

        heat_group = QGroupBox("Heat")
        heat_layout = QVBoxLayout(heat_group)
        heat_layout.setSpacing(10)
        for code in ["HL", "HR"]:
            heat_layout.addWidget(self.tune_cards[code])

        body_layout.addWidget(pressure_group)
        body_layout.addWidget(heat_group)
        body_layout.addStretch(1)
        scroll.setWidget(scroll_body)
        layout.addWidget(scroll, 1)
        return container

    def _build_right_panel(self):
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        header = QHBoxLayout()
        header.addWidget(QLabel("Curve Target"))
        header.addWidget(self.target_combo)
        header.addStretch(1)
        header.addWidget(self.clear_curve_btn)
        layout.addLayout(header)

        layout.addWidget(self.plot, 8)
        layout.addWidget(self.curve_bar)

        log_header = QHBoxLayout()
        log_title = QLabel("Log")
        log_title.setObjectName("sectionTitle")
        log_header.addWidget(log_title)
        log_header.addStretch(1)
        log_header.addWidget(self.clear_log_btn)
        layout.addLayout(log_header)
        layout.addWidget(self.log, 1)
        return container

    def _apply_style(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #f5f7fa; color: #1f2937; }
            QGroupBox {
                font-weight: 600;
                border: 1px solid #d7dde5;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
                background: #fbfcfe;
            }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
            QFrame#pidCard, QFrame#curveBar {
                border: 1px solid #d7dde5;
                border-radius: 10px;
                background: white;
            }
            QLabel#cardTitle, QLabel#sectionTitle {
                font-size: 15px;
                font-weight: 700;
            }
            QPushButton {
                background: white;
                border: 1px solid #cfd8e3;
                border-radius: 8px;
                padding: 6px 12px;
                min-height: 28px;
            }
            QPushButton:hover { border-color: #94a3b8; }
            QPushButton:pressed { background: #eef2f7; }
            QPlainTextEdit {
                background: #0f172a;
                color: #e5e7eb;
                border: 1px solid #cfd8e3;
                border-radius: 10px;
            }
            """
        )

    def _connect_signals(self):
        self.refresh_btn.clicked.connect(self.refresh_ports)
        self.connect_btn.clicked.connect(self.connect_serial)
        self.disconnect_btn.clicked.connect(self.disconnect_serial)
        self.stream_btn.clicked.connect(self.toggle_stream)
        self.clear_curve_btn.clicked.connect(self.clear_curve)
        self.clear_log_btn.clicked.connect(self.log.clear)
        self.target_combo.currentIndexChanged.connect(self._target_changed)
        for card in self.tune_cards.values():
            card.send_requested.connect(self.send_pid)

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
        LOGGER.info("connect requested port=%s baud=%s", port_name, self.baud_spin.value())
        self.worker = SerialWorker(port_name, self.baud_spin.value())
        self.worker.received_text.connect(self.append_log)
        self.worker.received_samples.connect(self.append_samples)
        self.worker.connected.connect(self.on_connected)
        self.worker.disconnected.connect(self.on_disconnected)
        self.worker.error.connect(self.on_error)
        self.worker.start()

    def disconnect_serial(self):
        LOGGER.info("disconnect requested stream=%s", self.stream_enabled)
        if self.worker is not None:
            if self.stream_enabled:
                self.send_bytes("PID STREAM 0", build_pid_stream_frame(False))
            self.worker.stop()
            self.worker.wait(1000)

    def on_connected(self, text: str):
        LOGGER.info("ui connected text=%s", text)
        self.append_log(f"[PC] {text}\n")
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.stream_enabled = False
        self._update_stream_button()

    def on_disconnected(self, text: str):
        LOGGER.info("ui disconnected text=%s", text)
        self.append_log(f"[PC] {text}\n")
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.stream_enabled = False
        self._update_stream_button()
        self.worker = None

    def on_error(self, text: str):
        LOGGER.error("ui error text=%s", text)
        self.append_log(f"[PC] {text}\n")

    def append_log(self, text: str):
        self.log.moveCursor(QTextCursor.End)
        self.log.insertPlainText(text)
        self.log.moveCursor(QTextCursor.End)
        if self.log.document().blockCount() > MAX_LOG_BLOCKS:
            cursor = self.log.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()
        if "[PC]" in text:
            LOGGER.debug("ui log text=%s", text.strip())

    def append_samples(self, samples: list):
        self._sample_batches += 1
        self._sample_count += len(samples)
        self._max_batch = max(self._max_batch, len(samples))
        dirty_current = False
        for sample in samples:
            code = sample["target"]
            self.last_samples[code] = sample
            self.series_x[code].append(sample["tick_ms"] / 1000.0)
            for key in CURVE_KEYS:
                self.series_y[code][key].append(sample[key])
            if code == self.current_target:
                dirty_current = True
        self._emit_periodic_diag()

    def refresh_visuals(self):
        start_ts = time.perf_counter()
        x_values = list(self.series_x[self.current_target])
        controls = self.curve_bar.controls
        for key in CURVE_KEYS:
            control = controls[key]
            y_values = [
                (value * control["scale"].value()) + control["offset"].value()
                for value in self.series_y[self.current_target][key]
            ]
            self.plot_items[key].setVisible(control["visible"].isChecked())
            self.plot_items[key].setData(x_values, y_values)
        elapsed_ms = (time.perf_counter() - start_ts) * 1000.0
        self._max_refresh_ms = max(self._max_refresh_ms, elapsed_ms)
        if elapsed_ms > 80.0:
            LOGGER.warning("slow refresh current_target=%s elapsed_ms=%.2f points=%d",
                           self.current_target,
                           elapsed_ms,
                           len(x_values))
        self._emit_periodic_diag()

    def clear_curve(self):
        LOGGER.info("clear curve")
        for x_values in self.series_x.values():
            x_values.clear()
        for group in self.series_y.values():
            for values in group.values():
                values.clear()
        self.last_samples = {item["code"]: None for item in TARGETS}
        self.refresh_visuals()

    def send_bytes(self, label: str, data: bytes):
        self.append_log(f"[PC->MCU] {label}  {data.hex(' ').upper()}\n")
        if self.worker is None:
            self.on_error("Serial is not connected")
            return
        self.worker.send_bytes(data)

    def send_pid(self, code: str, kp: float, ki: float, kd: float):
        LOGGER.info("send pid code=%s kp=%.4f ki=%.4f kd=%.4f", code, kp, ki, kd)
        target = TARGET_BY_CODE[code]
        frame = build_pid_set_frame(target["id"], kp, ki, kd)
        self.send_bytes(f"PID SET {code}", frame)

    def toggle_stream(self):
        self.stream_enabled = not self.stream_enabled
        LOGGER.info("toggle stream enabled=%s", self.stream_enabled)
        self._update_stream_button()
        frame = build_pid_stream_frame(self.stream_enabled)
        self.send_bytes(f"PID STREAM {1 if self.stream_enabled else 0}", frame)

    def _update_stream_button(self):
        self.stream_btn.setText("Disable PID Stream" if self.stream_enabled else "Enable PID Stream")

    def _target_changed(self):
        code = self.target_combo.currentData()
        if not code:
            return
        LOGGER.info("target changed code=%s", code)
        self.current_target = code

    def _emit_periodic_diag(self):
        now_ts = time.time()
        if (now_ts - self._last_diag_ts) < 2.0:
            return

        current_points = len(self.series_x[self.current_target])
        last_sample = self.last_samples.get(self.current_target)
        last_tick = last_sample["tick_ms"] if last_sample is not None else -1
        LOGGER.info(
            "diag target=%s points=%d batches=%d samples=%d max_batch=%d max_refresh_ms=%.2f log_blocks=%d last_tick=%s",
            self.current_target,
            current_points,
            self._sample_batches,
            self._sample_count,
            self._max_batch,
            self._max_refresh_ms,
            self.log.document().blockCount(),
            last_tick,
        )
        self._sample_batches = 0
        self._sample_count = 0
        self._max_batch = 0
        self._max_refresh_ms = 0.0
        self._last_diag_ts = now_ts

    def closeEvent(self, event):
        self.disconnect_serial()
        event.accept()


def main():
    pg.setConfigOptions(antialias=True)
    LOGGER.info("app start log_path=%s", DEBUG_LOG_PATH)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.append_log(f"[PC] debug log: {DEBUG_LOG_PATH}\n")
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
