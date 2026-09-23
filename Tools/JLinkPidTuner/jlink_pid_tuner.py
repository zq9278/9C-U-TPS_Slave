"""Pressure PID tuner over J-Link SWD background memory access."""
from collections import deque
import csv
import json
import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
import queue
import sys
import time

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

from jlink_protocol import (
    FirmwareSymbols, MailboxClient, PHASES, PROFILES, STAGES, validate_gains, connect_background,
)

ROOT = Path(__file__).resolve().parents[2]
LOGGER = logging.getLogger("jlink_pid_tuner")


class JLinkWorker(QtCore.QThread):
    sample_ready = QtCore.pyqtSignal(object)
    message = QtCore.pyqtSignal(str)
    connected = QtCore.pyqtSignal()
    applied = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, elf, device, serial, speed, dll):
        super().__init__()
        self.options = elf, device, serial, speed, dll
        self.commands = queue.Queue(maxsize=1)

    def run(self):
        link = None
        try:
            import pylink
            from pylink.library import Library

            elf, device, serial, speed, dll = self.options
            symbols = FirmwareSymbols.from_elf(elf)
            library = Library(dllpath=dll) if dll else Library()
            self.message.emit("正在打开探针…")
            link = pylink.JLink(lib=library, log=LOGGER.info, error=LOGGER.error, warn=LOGGER.warning)
            link.open(serial_no=int(serial) if serial else None)
            self.message.emit("正在通过 Cortex-M0 内核附加（跳过 STM32 复位初始化）…")
            connect_background(link, device, speed)
            client = MailboxClient(link, symbols)
            client.verify()
            self.message.emit(f"邮箱 0x{symbols.mailbox:08X}；构建标识已核对；20ms 轮询")
            self.connected.emit()
            last_sequence = None
            last_tick = None
            last_fresh = time.monotonic()
            while not self.isInterruptionRequested():
                started = time.monotonic()
                sample = client.read_sample()
                if sample is not None and sample.sequence != last_sequence:
                    if last_tick is not None and ((sample.tick_ms - last_tick) & 0xFFFFFFFF) > 0x7FFFFFFF:
                        raise RuntimeError("检测到 MCU 重启，请重新连接并重新读取参数")
                    last_sequence = sample.sequence
                    last_tick = sample.tick_ms
                    last_fresh = started
                    self.sample_ready.emit(sample)
                if started - last_fresh > 2:
                    raise TimeoutError("设备数据超过 2 秒未更新，请检查 MCU 运行状态、供电和 SWD 连接")
                result = client.poll_ack(sample)
                if result is not None:
                    self.applied.emit(result)
                if client.pending is None:
                    try:
                        command = self.commands.get_nowait()
                    except queue.Empty:
                        pass
                    else:
                        client.send_gains(*command)
                remaining = 0.02 - (time.monotonic() - started)
                if remaining > 0:
                    self.msleep(max(1, int(remaining * 1000)))
        except Exception as exc:
            LOGGER.exception("J-Link connection/telemetry failure")
            self.failed.emit(str(exc))
        finally:
            if link is not None:
                try:
                    link.close()
                except Exception:
                    pass


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("压力 PID 调试台 · J-Link / SWD")
        self.resize(1440, 900)
        self.worker = None
        self.latest = None
        self.pending_row = None
        self.closing = False
        self.elapsed_ms = 0
        self.last_tick = None
        self.samples = deque(maxlen=30000)  # ten minutes at 50 Hz
        self.record_file = None
        self.record_writer = None
        self.rows = []
        self.settings = QtCore.QSettings("9C-TPS", "JLinkPidTuner")
        self._build_ui()
        self._set_connected(False)
        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_plot)
        self.refresh_timer.start(100)

    def _build_ui(self):
        widget = QtWidgets.QWidget()
        self.setCentralWidget(widget)
        layout = QtWidgets.QVBoxLayout(widget)
        title = QtWidgets.QLabel("压力 PID 调试台")
        title.setStyleSheet("font-size:24px;font-weight:600;padding:8px")
        layout.addWidget(title)
        connection = QtWidgets.QGroupBox("J-Link 连接")
        grid = QtWidgets.QGridLayout(connection)
        self.elf = QtWidgets.QLineEdit(self.settings.value("elf", str(ROOT / "build/Debug/9C-TPS100-Slave.elf")))
        browse = QtWidgets.QPushButton("选择 ELF")
        browse.clicked.connect(self.browse_elf)
        grid.addWidget(QtWidgets.QLabel("已烧录固件 ELF"), 0, 0)
        grid.addWidget(self.elf, 0, 1, 1, 5)
        grid.addWidget(browse, 0, 6)
        self.device = QtWidgets.QLineEdit(self.settings.value("device", "STM32G070RBTx"))
        self.serial = QtWidgets.QLineEdit(self.settings.value("serial", ""))
        self.serial.setPlaceholderText("留空自动选择")
        self.speed = QtWidgets.QSpinBox()
        self.speed.setRange(100, 12000)
        self.speed.setValue(int(self.settings.value("speed", 1000)))
        self.speed.setSuffix(" kHz")
        grid.addWidget(QtWidgets.QLabel("芯片"), 1, 0)
        grid.addWidget(self.device, 1, 1)
        grid.addWidget(QtWidgets.QLabel("探针序列号"), 1, 2)
        grid.addWidget(self.serial, 1, 3)
        grid.addWidget(QtWidgets.QLabel("SWD 时钟"), 1, 4)
        grid.addWidget(self.speed, 1, 5)
        self.connect_button = QtWidgets.QPushButton("连接")
        self.connect_button.clicked.connect(self.connect_device)
        grid.addWidget(self.connect_button, 1, 6)
        self.dll = QtWidgets.QLineEdit(self.settings.value("dll", ""))
        self.dll.setPlaceholderText("留空自动查找 SEGGER J-Link DLL；也可填 JLink_x64.dll 完整路径")
        grid.addWidget(QtWidgets.QLabel("J-Link DLL"), 2, 0)
        grid.addWidget(self.dll, 2, 1, 1, 5)
        self.disconnect_button = QtWidgets.QPushButton("断开")
        self.disconnect_button.clicked.connect(self.disconnect_device)
        grid.addWidget(self.disconnect_button, 2, 6)
        self.connection_inputs = [self.elf, browse, self.device, self.serial, self.speed, self.dll]
        layout.addWidget(connection)
        self.values_label = QtWidgets.QLabel("目标 —  |  左压力 —  |  右压力 —  |  PWM —")
        self.values_label.setStyleSheet("font-size:18px;padding:10px;background:#e8f0f8;border-radius:6px")
        layout.addWidget(self.values_label)

        splitter = QtWidgets.QSplitter()
        layout.addWidget(splitter, 1)
        tuning = QtWidgets.QWidget()
        tuning_layout = QtWidgets.QVBoxLayout(tuning)
        note = QtWidgets.QLabel("参数按单眼 / 双眼独立保存于 RAM。\n修改后点击对应行“应用”；复位后恢复固件默认值。")
        note.setWordWrap(True)
        tuning_layout.addWidget(note)
        self.table = QtWidgets.QTableWidget(6, 6)
        self.table.setHorizontalHeaderLabels(["模式 / 阶段", "Kp", "Ki", "Kd", "写入", "设备回读 Kp / Ki / Kd"])
        self.table.verticalHeader().hide()
        self.table.setMinimumHeight(260)
        self.table.setMaximumHeight(290)
        self.table.verticalHeader().setDefaultSectionSize(36)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Fixed)
        for column, width in enumerate((105, 95, 95, 95, 60)):
            self.table.setColumnWidth(column, width)
        self.table.horizontalHeader().setSectionResizeMode(5, QtWidgets.QHeaderView.Stretch)
        for profile in range(2):
            for stage in range(3):
                row = profile * 3 + stage
                label = QtWidgets.QTableWidgetItem(f"{PROFILES[profile]} / {STAGES[stage]}")
                label.setFlags(QtCore.Qt.ItemIsEnabled)
                self.table.setItem(row, 0, label)
                fields = []
                for column in range(1, 4):
                    spin = QtWidgets.QDoubleSpinBox()
                    spin.setRange(0, 1000)
                    spin.setDecimals(6)
                    spin.setSingleStep(0.01)
                    spin.setKeyboardTracking(False)
                    spin.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
                    self.table.setCellWidget(row, column, spin)
                    fields.append(spin)
                button = QtWidgets.QPushButton("应用")
                button.clicked.connect(lambda checked=False, r=row: self.apply_row(r))
                self.table.setCellWidget(row, 4, button)
                readback = QtWidgets.QTableWidgetItem("—")
                readback.setFlags(QtCore.Qt.ItemIsEnabled)
                self.table.setItem(row, 5, readback)
                self.rows.append((fields, button))
        tuning_layout.addWidget(self.table)
        buttons = QtWidgets.QHBoxLayout()
        self.read_button = QtWidgets.QPushButton("设备参数 → 编辑框")
        self.read_button.clicked.connect(self.read_gains)
        save = QtWidgets.QPushButton("保存编辑参数")
        save.clicked.connect(self.save_gains)
        load = QtWidgets.QPushButton("加载参数文件")
        load.clicked.connect(self.load_gains)
        buttons.addWidget(self.read_button)
        buttons.addWidget(save)
        buttons.addWidget(load)
        tuning_layout.addLayout(buttons)
        self.details = QtWidgets.QLabel("阶段：—\nPID 原始输出：—\nP / I / D：—\n有效 PWM：20～100 ‰；关闭为 0")
        self.details.setStyleSheet("padding:10px;font-size:14px")
        tuning_layout.addWidget(self.details)
        tuning_layout.addStretch()
        splitter.addWidget(tuning)

        plots = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout(plots)
        self.pressure_plot = pg.PlotWidget(title="压力曲线")
        self.pressure_plot.setLabel("left", "压力", units="kPa")
        self.pressure_plot.setLabel("bottom", "连接后时间", units="s")
        self.pressure_plot.addLegend()
        self.pressure_plot.showGrid(x=True, y=True, alpha=0.2)
        self.curves = [self.pressure_plot.plot(name=name, pen=pg.mkPen(color, width=2))
                       for name, color in [("目标", "#d28e17"), ("左压力", "#2176ae"),
                                           ("右压力", "#b55398"), ("控制反馈", "#269c78")]]
        plot_layout.addWidget(self.pressure_plot, 3)
        self.pwm_plot = pg.PlotWidget(title="电机 PWM")
        self.pwm_plot.setLabel("left", "PWM", units="‰")
        self.pwm_plot.setLabel("bottom", "连接后时间", units="s")
        self.pwm_plot.setXLink(self.pressure_plot)
        self.pwm_plot.setYRange(0, 110)
        self.pwm_curve = self.pwm_plot.plot(pen=pg.mkPen("#2176ae", width=2))
        plot_layout.addWidget(self.pwm_plot, 1)
        options = QtWidgets.QHBoxLayout()
        self.follow = QtWidgets.QCheckBox("跟随最近 30 秒")
        self.follow.setChecked(True)
        clear = QtWidgets.QPushButton("清空曲线")
        clear.clicked.connect(self.clear_plot)
        export = QtWidgets.QPushButton("导出曲线 CSV")
        export.clicked.connect(self.export_csv)
        self.record_button = QtWidgets.QPushButton("开始连续记录")
        self.record_button.clicked.connect(self.toggle_recording)
        for item in [self.follow, clear, export, self.record_button]:
            options.addWidget(item)
        plot_layout.addLayout(options)
        splitter.addWidget(plots)
        splitter.setSizes([700, 720])
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(300)
        self.log.setMaximumHeight(100)
        layout.addWidget(self.log)
        self.statusBar().showMessage("未连接 · 治疗启动/停止仍由原设备界面控制")

    def log_message(self, message):
        LOGGER.info(message)
        self.log.appendPlainText(time.strftime("%H:%M:%S ") + message)

    def _set_connected(self, connected):
        self.connect_button.setEnabled(self.worker is None)
        self.disconnect_button.setEnabled(self.worker is not None)
        for widget in self.connection_inputs:
            widget.setEnabled(self.worker is None)
        for fields, button in self.rows:
            button.setEnabled(connected and self.latest is not None and self.pending_row is None)
        self.read_button.setEnabled(connected and self.latest is not None)

    def browse_elf(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "选择已烧录固件对应的 ELF", self.elf.text(), "ELF (*.elf)")
        if path:
            self.elf.setText(path)

    def connect_device(self):
        if self.worker is not None:
            return
        serial = self.serial.text().strip()
        if serial and not serial.isdigit():
            self.show_error("J-Link 序列号应为数字")
            return
        self.latest = None
        self.pending_row = None
        self.clear_plot()
        options = self.elf.text().strip(), self.device.text().strip(), serial, self.speed.value(), self.dll.text().strip()
        for key, value in zip(("elf", "device", "serial", "speed", "dll"), options):
            self.settings.setValue(key, value)
        self.worker = JLinkWorker(*options)
        self.worker.sample_ready.connect(self.on_sample)
        self.worker.message.connect(self.log_message)
        self.worker.connected.connect(lambda: self.statusBar().showMessage("已连接 · 等待第一帧压力数据"))
        self.worker.applied.connect(self.on_applied)
        self.worker.failed.connect(self.show_error)
        self.worker.finished.connect(self.on_finished)
        self._set_connected(False)
        self.statusBar().showMessage("正在连接 J-Link…")
        self.worker.start()

    def disconnect_device(self):
        if self.worker:
            self.worker.requestInterruption()
            self.disconnect_button.setEnabled(False)
            self._set_connected(False)
            self.statusBar().showMessage("正在断开…（不会停止设备治疗）")

    def on_finished(self):
        if self.worker:
            self.worker.deleteLater()
        self.worker = None
        self.latest = None
        self.pending_row = None
        self._set_connected(False)
        self.stop_recording()
        self.values_label.setText("已断开 · 当前压力未知（保留曲线供查看）")
        self.statusBar().showMessage("未连接 · 详见日志；断开不会停止设备治疗")
        if self.closing:
            self.close()

    def show_error(self, message):
        self.log_message("错误：" + message)
        self.statusBar().showMessage("错误：" + message)

    def on_sample(self, sample):
        first = self.latest is None
        self.latest = sample
        if self.last_tick is not None:
            self.elapsed_ms += (sample.tick_ms - self.last_tick) & 0xFFFFFFFF
        self.last_tick = sample.tick_ms
        seconds = self.elapsed_ms / 1000.0
        self.samples.append((seconds, sample))
        if self.record_writer is not None:
            try:
                self.record_writer.writerow(self.csv_row(seconds, sample))
                self.record_file.flush()
            except OSError as exc:
                self.show_error(f"CSV 记录失败：{exc}")
                self.stop_recording()
        for row in range(6):
            values = sample.row_gains(row // 3, row % 3)
            self.table.item(row, 5).setText(" / ".join(f"{v:.6g}" for v in values))
        if first:
            self.read_gains()
        self._set_connected(self.worker is not None and not self.worker.isInterruptionRequested())
        self.values_label.setText(
            f"目标 {sample.target:.2f} kPa   |   左 {sample.left:.2f} kPa   |   "
            f"右 {sample.right:.2f} kPa   |   PWM {sample.pwm} ‰ ({sample.pwm / 10:.1f}%)")
        phase = PHASES[sample.phase] if sample.phase < len(PHASES) else str(sample.phase)
        profile = PROFILES[sample.profile] if sample.profile < len(PROFILES) else str(sample.profile)
        self.details.setText(f"当前：{profile} / {phase}　反馈 {sample.feedback:.3f} kPa\n"
                             f"PID 原始输出：{sample.output:.4f} / 25\n"
                             f"P {sample.p:.4f}　I {sample.i:.4f}　D {sample.d:.4f}\n"
                             f"有效 PWM：{sample.pwm_min}～{sample.pwm_max} ‰；关闭为 0")
        self.statusBar().showMessage(f"数据更新 · {phase} · 设备时间 {sample.tick_ms / 1000:.2f}s · "
                                     f"{'急停' if sample.flags & 4 else '暂停' if sample.flags & 2 else '治疗中' if sample.flags & 1 else '空闲'}")

    def read_gains(self):
        if self.latest is None:
            return
        for row, (fields, _) in enumerate(self.rows):
            for field, value in zip(fields, self.latest.row_gains(row // 3, row % 3)):
                field.setValue(value)

    def apply_row(self, row):
        if self.worker is None or self.latest is None or self.pending_row is not None:
            return
        values = tuple(field.value() for field in self.rows[row][0])
        validate_gains(row // 3, row % 3, values)
        self.worker.commands.put_nowait((row // 3, row % 3, values))
        self.pending_row = row
        self._set_connected(True)
        self.log_message(f"正在写入 {PROFILES[row // 3]} / {STAGES[row % 3]}：{values}")

    def on_applied(self, result):
        profile, stage, gains = result
        self.pending_row = None
        self._set_connected(True)
        self.log_message(f"固件已确认，回读一致：{PROFILES[profile]} / {STAGES[stage]} {gains}")

    def refresh_plot(self):
        if not self.samples:
            return
        data = list(self.samples)
        # Limit drawing work; full ten-minute buffer is still available for CSV.
        if self.follow.isChecked():
            cutoff = data[-1][0] - 30
            data = [entry for entry in data if entry[0] >= cutoff]
            self.pressure_plot.setXRange(max(0, data[-1][0] - 30), max(30, data[-1][0]), padding=0)
        xs = [entry[0] for entry in data]
        for curve, field in zip(self.curves, ("target", "left", "right", "feedback")):
            curve.setData(xs, [getattr(entry[1], field) for entry in data])
        self.pwm_curve.setData(xs, [entry[1].pwm for entry in data])

    def clear_plot(self):
        self.samples.clear()
        # Keep the recording time axis monotonic within a connected session.
        if self.worker is None:
            self.elapsed_ms = 0
            self.last_tick = None
        for curve in self.curves + [self.pwm_curve]:
            curve.setData([], [])

    @staticmethod
    def csv_header():
        return ["time_s", "device_tick_ms", "phase", "profile", "flags", "target_kpa", "left_kpa",
                "right_kpa", "feedback_kpa", "pid_output", "p", "i", "d", "pwm_permille"] + [
                    f"{profile}_{stage}_{gain}" for profile in ("single", "dual")
                    for stage in ("rise", "hold", "pulse") for gain in ("kp", "ki", "kd")]

    @staticmethod
    def csv_row(seconds, sample):
        return [seconds, sample.tick_ms, sample.phase, sample.profile, sample.flags, sample.target,
                sample.left, sample.right, sample.feedback, sample.output, sample.p, sample.i,
                sample.d, sample.pwm, *sample.gains]

    def export_csv(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出最近最多 10 分钟采样", "pressure.csv", "CSV (*.csv)")
        if path:
            try:
                with open(path, "w", newline="", encoding="utf-8-sig") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(self.csv_header())
                    writer.writerows(self.csv_row(t, s) for t, s in self.samples)
                self.log_message(f"已导出 {len(self.samples)} 个采样：{path}")
            except OSError as exc:
                self.show_error(str(exc))

    def toggle_recording(self):
        if self.record_file is not None:
            self.stop_recording()
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "连续记录压力和 PID 数据", "pressure_record.csv", "CSV (*.csv)")
        if path:
            try:
                self.record_file = open(path, "w", newline="", encoding="utf-8-sig")
                self.record_writer = csv.writer(self.record_file)
                self.record_writer.writerow(self.csv_header())
                self.record_button.setText("停止记录")
                self.log_message(f"开始记录：{path}")
            except OSError as exc:
                self.show_error(str(exc))
                self.stop_recording()

    def stop_recording(self):
        if self.record_file is not None:
            try:
                self.record_file.close()
            except OSError as exc:
                self.show_error(str(exc))
            self.record_file = None
            self.record_writer = None
            self.record_button.setText("开始连续记录")

    def save_gains(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "保存编辑框参数（不是固件默认值）", "pid_gains.json", "JSON (*.json)")
        if path:
            try:
                rows = [[field.value() for field in fields] for fields, _ in self.rows]
                Path(path).write_text(json.dumps({"version": 1, "rows": rows}, indent=2), encoding="utf-8")
                self.log_message(f"已保存编辑参数：{path}")
            except OSError as exc:
                self.show_error(str(exc))

    def load_gains(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "加载参数到编辑框", "", "JSON (*.json)")
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            rows = data["rows"]
            if data["version"] != 1 or len(rows) != 6:
                raise ValueError("参数文件格式错误")
            for row, values in enumerate(rows):
                validate_gains(row // 3, row % 3, values)
            for (fields, _), values in zip(self.rows, rows):
                for field, value in zip(fields, values):
                    field.setValue(value)
            self.log_message("参数已加载至编辑框，点击各行“应用”后才会写入设备")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.show_error(str(exc))

    def closeEvent(self, event):
        if self.worker is not None:
            self.closing = True
            self.disconnect_device()
            event.ignore()  # Never destroy or terminate a thread inside the SDK.
            return
        self.stop_recording()
        event.accept()


def main():
    handler = RotatingFileHandler(Path(__file__).with_name("jlink_pid_tuner.log"),
                                  maxBytes=1000000, backupCount=2, encoding="utf-8")
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    LOGGER.addHandler(handler)
    LOGGER.setLevel(logging.INFO)
    pg.setConfigOptions(background="w", foreground="#263445", antialias=True)
    app = QtWidgets.QApplication(sys.argv)
    app.setFont(QtGui.QFont("Microsoft YaHei UI", 9))
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
