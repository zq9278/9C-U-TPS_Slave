import os
import math
from dataclasses import replace
from pathlib import Path
import sys
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PyQt5 import QtWidgets, QtGui
import pyqtgraph as pg
from jlink_pid_tuner import MainWindow
from test_protocol import FakeLink
from jlink_protocol import MailboxClient


class GuiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        font_path = Path("C:/Windows/Fonts/msyh.ttc")
        if font_path.exists():
            QtGui.QFontDatabase.addApplicationFont(str(font_path))
        cls.app.setFont(QtGui.QFont("Microsoft YaHei", 9))
        cls.app.setStyle("Fusion")
        pg.setConfigOptions(background="w", foreground="#263445", antialias=True)

    def test_plot_readback_edit_preservation_and_csv(self):
        window = MainWindow()
        try:
            link = FakeLink()
            sample = MailboxClient(link, link.symbols).read_sample()
            window.show()
            window.on_sample(sample)
            self.assertAlmostEqual(window.rows[0][0][0].value(), 0.1, places=5)
            window.rows[0][0][0].setValue(3.5)
            window.on_sample(sample)
            self.assertEqual(window.rows[0][0][0].value(), 3.5)
            window.refresh_plot()
            self.assertEqual(len(window.curves[0].getData()[0]), 2)
            self.assertEqual(len(window.csv_header()), len(window.csv_row(0, sample)))
            window.read_gains()
            self.assertAlmostEqual(window.rows[0][0][0].value(), 0.1, places=5)
            for index in range(1, 501):
                t = index / 25
                target = min(10, t)
                left = max(0, target - 0.7 + 0.3 * math.sin(t * 2))
                right = max(0, target - 0.9 + 0.2 * math.sin(t * 2 + 1))
                window.on_sample(replace(sample, sequence=2 + index * 2, tick_ms=100 + index * 40,
                                         phase=1 if t < 10 else 2, target=target,
                                         left=left, right=right, feedback=max(left, right),
                                         pwm=int(35 + 10 * math.sin(t))))
            window.refresh_plot()
            window.statusBar().showMessage("模拟数据 · 界面验证（未连接硬件）")
            self.app.processEvents()
            output = Path(__file__).resolve().parents[3] / "build" / "jlink_tuner_preview.png"
            output.parent.mkdir(exist_ok=True)
            self.assertTrue(window.grab().save(str(output)))
        finally:
            window.close()


if __name__ == "__main__":
    unittest.main()
