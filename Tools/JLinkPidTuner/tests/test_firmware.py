"""Execute the actual Debug ARM ELF; hardware functions are intercepted."""
from pathlib import Path
import struct
import sys
import unittest

from elftools.elf.elffile import ELFFile
try:
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
    from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC
    HAS_UNICORN = True
except ImportError:
    HAS_UNICORN = False

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from jlink_protocol import FirmwareSymbols, MailboxClient

ELF_PATH = Path(__file__).resolve().parents[3] / "build/Debug/9C-TPS100-Slave.elf"


def float_bits(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


@unittest.skipUnless(HAS_UNICORN and ELF_PATH.exists(), "Requires unicorn and built Debug ELF")
class FirmwareTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with ELF_PATH.open("rb") as stream:
            elf = ELFFile(stream)
            cls.symbols = {s.name: s["st_value"] for s in elf.get_section_by_name(".symtab").iter_symbols()}
            cls.segments = [(s["p_vaddr"], s.data()) for s in elf.iter_segments() if s["p_type"] == "PT_LOAD"]
            names = {b"TreatmentAppController", b"TreatmentAppRuntime", b"TreatmentPressurePlan",
                     b"control_config_t", b"sensor_data_t", b"PidController", b"PidControllerDebug"}
            cls.layouts = {}
            for cu in elf.get_dwarf_info().iter_CUs():
                for die in cu.iter_DIEs():
                    name = die.attributes.get("DW_AT_name")
                    if die.tag == "DW_TAG_typedef" and name and name.value in names:
                        target = die.get_DIE_from_attribute("DW_AT_type")
                        cls.layouts[name.value.decode()] = {
                            child.attributes["DW_AT_name"].value.decode(): child.attributes["DW_AT_data_member_location"].value
                            for child in target.iter_children() if child.tag == "DW_TAG_member"}
        cls.firmware_symbols = FirmwareSymbols.from_elf(ELF_PATH)

    def setUp(self):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.mem_map(0x08000000, 0x20000)
        # Upper SRAM region is simulator-only test fixtures and call stack.
        self.uc.mem_map(0x20000000, 0x20000)
        for address, data in self.segments:
            self.uc.mem_write(address, data)
        self.controller = 0x20010000
        self.sensor = 0x20010400
        self.plan = 0x20010500
        self.runtime = 0x20010600
        self.pwm = None
        self.wave = None
        hooks = {self.symbols[name] & ~1: name for name in
                 ("BspPwm_SetDutyPermille", "ValveControl_SetWave", "ValveControl_ApplyTreatmentRoute", "xTaskGetTickCount")}

        def intercept(uc, address, size, data):
            name = hooks.get(address)
            if name is None:
                return
            if name == "BspPwm_SetDutyPermille":
                self.pwm = uc.reg_read(UC_ARM_REG_R1)
            elif name == "ValveControl_SetWave":
                self.wave = uc.reg_read(UC_ARM_REG_R0)
            elif name == "xTaskGetTickCount":
                uc.reg_write(UC_ARM_REG_R0, 1000)
            uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))
        self.uc.hook_add(UC_HOOK_CODE, intercept)
        self.set_field(self.controller, "control_config_t", "press_enable_L", "B", 1)
        self.set_field(self.controller, "control_config_t", "press_enable_R", "B", 1)
        self.call("TreatmentPressureControl_InitPid", self.controller)
        self.call("TreatmentPressureControl_GetPwmLimits", 0x20010700, 0x20010704)
        self.pwm_min, self.pwm_max = struct.unpack("<2I", self.uc.mem_read(0x20010700, 8))

    def expected_pwm(self, raw_output):
        return int(self.pwm_min + raw_output * (self.pwm_max - self.pwm_min) / 25)

    def call(self, name, *args):
        stack = 0x2001F000
        self.uc.reg_write(UC_ARM_REG_SP, stack)
        self.uc.reg_write(UC_ARM_REG_LR, 0x0801FFF1)
        for register, value in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
            self.uc.reg_write(register, value)
        for index, value in enumerate(args[4:]):
            self.uc.mem_write(stack + index * 4, struct.pack("<I", value))
        self.uc.emu_start(self.symbols[name] | 1, 0x0801FFF0, count=500000)
        self.assertEqual(self.uc.reg_read(UC_ARM_REG_PC), 0x0801FFF0, f"{name} did not return")
        return self.uc.reg_read(UC_ARM_REG_R0)

    def set_field(self, base, layout, name, fmt, value):
        self.uc.mem_write(base + self.layouts[layout][name], struct.pack("<" + fmt, value))

    def get_field(self, base, layout, name, fmt):
        return struct.unpack("<" + fmt, self.uc.mem_read(base + self.layouts[layout][name], struct.calcsize(fmt)))[0]

    def set_gains(self, profile, stage, kp, ki=0, kd=0):
        return self.call("TreatmentPressureControl_SetProfileGains", profile, stage,
                         float_bits(kp), float_bits(ki), float_bits(kd))

    def apply(self, phase, stage, target=10, left=5, right=4):
        for name, value in (("phase", phase), ("pid_stage", stage), ("inflating", int(phase != 4))):
            self.set_field(self.plan, "TreatmentPressurePlan", name, "B", value)
        self.set_field(self.plan, "TreatmentPressurePlan", "target_pressure_kpa", "f", target)
        self.set_field(self.sensor, "sensor_data_t", "pressL", "f", left)
        self.set_field(self.sensor, "sensor_data_t", "pressR", "f", right)
        self.call("TreatmentPressureControl_ApplyPlan", self.controller, self.sensor,
                  float_bits(0.002), self.plan, self.runtime)
        return self.get_field(self.runtime, "TreatmentAppRuntime", "pump_pwm", "H")

    def test_all_stages_use_pid_not_fixed_power(self):
        for stage in range(3):
            for kp, expected in ((1, self.expected_pwm(5)), (2, self.expected_pwm(10)), (100, self.pwm_max)):
                with self.subTest(stage=stage, kp=kp):
                    self.set_gains(1, stage, kp)
                    self.assertEqual(self.apply(stage + 1, stage), expected)
                    self.assertEqual(self.pwm, expected)
                    self.assertEqual(self.wave, 1)

    def test_zero_and_minimum_and_saturation(self):
        for kp, left, expected in ((0, 5, 0), (0.001, 5, self.pwm_min), (1, 11, 0), (1000, 0, self.pwm_max)):
            with self.subTest(kp=kp, left=left):
                self.set_gains(1, 0, kp)
                self.assertEqual(self.apply(1, 0, left=left), expected)
                self.assertEqual(self.pwm, expected)

    def test_pulse_off_stops_and_vents(self):
        self.set_gains(1, 2, 100)
        self.assertEqual(self.apply(3, 2), 100)
        self.assertEqual(self.apply(4, 2, target=0), 0)
        self.assertEqual((self.pwm, self.wave), (0, 0))

    def configure_cycle(self, rise=10, hold=10, pulse=40):
        for name, value in (("t1_rise_s", rise), ("t2_hold_s", hold), ("t3_pulse_s", pulse),
                            ("press_target_max", 10), ("pulse_on_ms", 2500), ("pulse_off_ms", 500)):
            self.set_field(self.controller, "control_config_t", name, "f", value)

    def build_plan(self, tick):
        self.call("TreatmentPressureControl_BuildPlan", self.controller, tick, self.plan)
        return self.get_field(self.plan, "TreatmentPressurePlan", "phase", "B")

    def execute_plan(self, left=0, right=0):
        self.set_field(self.sensor, "sensor_data_t", "pressL", "f", left)
        self.set_field(self.sensor, "sensor_data_t", "pressR", "f", right)
        self.call("TreatmentPressureControl_ApplyPlan", self.controller, self.sensor,
                  float_bits(0.002), self.plan, self.runtime)

    def test_final_two_seconds_replace_pulse_tail(self):
        self.configure_cycle()
        for tick, phase in ((0, 1), (9999, 1), (10000, 2), (19999, 2),
                            (20000, 3), (57999, 3), (58000, 6), (59000, 6), (59999, 6),
                            (60000, 1), (118000, 6), (119999, 6), (120000, 1)):
            with self.subTest(tick=tick):
                self.assertEqual(self.build_plan(tick), phase)
                if phase == 6:
                    self.assertEqual(self.get_field(self.plan, "TreatmentPressurePlan", "inflating", "B"), 0)
                    self.assertEqual(self.get_field(self.plan, "TreatmentPressurePlan", "target_pressure_kpa", "f"), 0)

    def test_vent_lasts_full_window_and_no_next_cycle_delay(self):
        self.configure_cycle()
        self.call("PidJlink_Init")
        self.build_plan(57999)
        self.execute_plan()
        self.assertGreater(self.pwm, 0)
        for tick, pressure in ((58000, 10), (58500, 0), (59000, 5), (59999, 0)):
            self.assertEqual(self.build_plan(tick), 6)
            self.execute_plan(pressure, pressure)
            self.assertEqual((self.pwm, self.wave), (0, 0))
            self.call("PidJlink_Publish", self.controller, self.sensor, self.runtime, tick)
            sample = MailboxClient(self, self.firmware_symbols).read_sample()
            self.assertEqual((sample.phase, sample.target, sample.output, sample.pwm), (6, 0, 0, 0))
        self.assertEqual(self.build_plan(60000), 1)
        self.execute_plan()
        self.assertEqual(self.wave, 1)
        self.build_plan(60002)
        self.execute_plan()
        self.assertGreater(self.pwm, 0)
        self.assertAlmostEqual(self.get_field(self.runtime, "TreatmentAppRuntime", "target_pressure_kpa", "f"), .002)

    def test_final_vent_has_priority_when_pulse_is_too_short(self):
        for config in ((30, 30, 0), (60, 0, 0), (0, 0, 0), (15, 15, 30)):
            self.configure_cycle(*config)
            for tick in (58000, 59999):
                self.assertEqual(self.build_plan(tick), 6)
                self.execute_plan()
                self.assertEqual((self.pwm, self.wave), (0, 0))

    def get_display_pressure(self, measured=7.25, peak=10):
        bits = self.call("Rk3576Telemetry_GetDisplayPressure", self.runtime, float_bits(measured), float_bits(peak))
        return struct.unpack("<f", struct.pack("<I", bits))[0]

    def test_rk3576_rise_simulation_both_profiles_preserves_real_control(self):
        self.configure_cycle()
        self.call("PidJlink_Init")
        for left_enabled, right_enabled in ((1, 0), (0, 1), (1, 1)):
            self.set_field(self.controller, "control_config_t", "press_enable_L", "B", left_enabled)
            self.set_field(self.controller, "control_config_t", "press_enable_R", "B", right_enabled)
            for tick, expected in ((0, 0), (2500, 2.5), (5000, 5), (9999, 9.999)):
                with self.subTest(left=left_enabled, right=right_enabled, tick=tick):
                    self.assertEqual(self.build_plan(tick), 1)
                    self.execute_plan(left=1.25, right=2.5)
                    before_controller = bytes(self.uc.mem_read(self.controller, 512))
                    before_sensor = bytes(self.uc.mem_read(self.sensor, 24))
                    before_runtime = bytes(self.uc.mem_read(self.runtime, 32))
                    displayed = self.get_display_pressure()
                    self.assertLessEqual(abs(displayed - expected), 0.35 + 0.00001)
                    self.assertTrue(0 <= displayed <= 10)
                    self.assertEqual(bytes(self.uc.mem_read(self.controller, 512)), before_controller)
                    self.assertEqual(bytes(self.uc.mem_read(self.sensor, 24)), before_sensor)
                    self.assertEqual(bytes(self.uc.mem_read(self.runtime, 32)), before_runtime)
                    self.call("PidJlink_Publish", self.controller, self.sensor, self.runtime, tick)
                    sample = MailboxClient(self, self.firmware_symbols).read_sample()
                    self.assertEqual((sample.left, sample.right), (1.25, 2.5))
                    self.assertEqual(sample.feedback, 2.5 if right_enabled else 1.25)
                    pid = self.controller + self.layouts["TreatmentAppController"]["pressure_pid"]
                    debug = pid + self.layouts["PidController"]["debug"]
                    self.assertEqual(self.get_field(debug, "PidControllerDebug", "measurement", "f"), sample.feedback)

    def test_rk3576_nonrise_and_inactive_use_measured_pressure(self):
        self.configure_cycle()
        for tick in (10000, 20000, 22500, 58000, 59999):
            self.build_plan(tick)
            self.execute_plan()
            self.assertEqual(self.get_display_pressure(), 7.25)
        for phase, active, outputs in ((0, 0, 0), (5, 1, 0), (1, 0, 1), (1, 1, 0)):
            self.set_field(self.runtime, "TreatmentAppRuntime", "phase", "B", phase)
            self.set_field(self.runtime, "TreatmentAppRuntime", "session_active", "B", active)
            self.set_field(self.runtime, "TreatmentAppRuntime", "running_outputs", "B", outputs)
            self.assertEqual(self.get_display_pressure(), 7.25)

    def test_rk3576_simulation_follows_configured_rise_duration(self):
        self.configure_cycle(rise=15, hold=15, pulse=30)
        for tick, expected in ((0, 0), (7500, 5), (14999, 10 * 14999 / 15000), (60000, 0)):
            self.build_plan(tick)
            self.execute_plan()
            self.assertLessEqual(abs(self.get_display_pressure() - expected), 0.35 + 0.00001)

    def test_rk3576_random_curve_is_smooth_bounded_and_changes_each_cycle(self):
        self.configure_cycle()
        curves = []
        for cycle in range(2):
            curve = []
            for tick in range(0, 10000, 10):
                self.build_plan(cycle * 60000 + tick)
                self.execute_plan()
                value = self.get_display_pressure()
                target = tick / 1000
                progress = target / 10
                self.assertTrue(0 <= value <= 10)
                self.assertLessEqual(abs(value - target), .35 * 4 * progress * (1 - progress) + 0.00001)
                if curve:
                    self.assertLess(abs(value - curve[-1]), .035)
                curve.append(value)
            self.assertEqual(curve[0], 0)
            self.assertGreater(max(abs(v - n / 100) for n, v in enumerate(curve)), .05)
            curves.append(curve)
        self.assertNotEqual(curves[0], curves[1])
        self.set_field(self.runtime, "TreatmentAppRuntime", "target_pressure_kpa", "f", 10)
        self.assertEqual(self.get_display_pressure(), 10)
        self.assertEqual(self.get_display_pressure(peak=0), 0)

    def test_single_right_and_dual_max_feedback(self):
        self.set_gains(0, 0, 1)
        self.set_gains(1, 0, 1)
        self.set_field(self.controller, "control_config_t", "press_enable_L", "B", 0)
        self.assertEqual(self.apply(1, 0, left=9, right=5), self.expected_pwm(5))
        self.set_field(self.controller, "control_config_t", "press_enable_L", "B", 1)
        self.assertEqual(self.apply(1, 0, left=9, right=5), self.expected_pwm(1))

    def test_stop_output_and_invalid_gain_rejection(self):
        self.call("TreatmentPressureControl_SetIdleOutputs")
        self.assertEqual((self.pwm, self.wave), (0, 0))
        for value in (float("nan"), float("inf"), -1, 1001):
            self.assertEqual(self.set_gains(1, 0, value), 0)
        self.assertEqual(self.set_gains(2, 0, 1), 0)
        self.assertEqual(self.set_gains(0, 3, 1), 0)

    def test_hardware_output_clamp_and_nonfinite_pid(self):
        for value, expected in ((0, 0), (1, self.pwm_min), (self.pwm_min - 1, self.pwm_min),
                                (self.pwm_min, self.pwm_min), (self.pwm_max, self.pwm_max),
                                (self.pwm_max + 25, self.pwm_max)):
            self.call("TreatmentPressureControl_SetPumpPwmOutput", value)
            self.assertEqual(self.pwm, expected)
        for value in (float("nan"), float("inf"), -1, 0):
            self.assertEqual(self.call("TreatmentPressureControl_ClampPwm", float_bits(value), 100), 0)

    def memory_read32(self, address, count):
        return list(struct.unpack(f"<{count}I", self.uc.mem_read(address, count * 4)))

    def memory_write32(self, address, words):
        self.uc.mem_write(address, struct.pack(f"<{len(words)}I", *words))

    def test_real_mailbox_end_to_end(self):
        self.call("PidJlink_Init")
        client = MailboxClient(self, self.firmware_symbols)
        client.verify()
        client.send_gains(1, 1, (2, 0, 0))
        self.call("PidJlink_ProcessCommand")
        self.assertEqual(self.apply(2, 1), self.expected_pwm(10))
        self.call("PidJlink_Publish", self.controller, self.sensor, self.runtime, 100)
        sample = client.read_sample()
        self.assertEqual((sample.left, sample.right, sample.pwm), (5, 4, self.expected_pwm(10)))
        self.assertEqual((sample.pwm_min, sample.pwm_max), (self.pwm_min, self.pwm_max))
        self.assertEqual(client.poll_ack(sample), (1, 1, (2, 0, 0)))
        # The command updated only dual-eye HOLD.
        self.assertNotEqual(sample.row_gains(0, 1), (2, 0, 0))


if __name__ == "__main__":
    unittest.main()
