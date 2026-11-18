"""Execute the actual Debug ARM ELF; hardware functions are intercepted."""
from pathlib import Path
import struct
import unittest

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC


ELF_PATH = Path(__file__).resolve().parents[2] / "build/delivery/Debug/9C-TPS100-Slave.elf"


def float_bits(value):
    return struct.unpack("<I", struct.pack("<f", value))[0]


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
        self.pulse_ticks = 0
        self.wave = None
        hooks = {self.symbols[name] & ~1: name for name in
                 ("BspPwm_SetDutyPermille", "BspPwm_GetPeriodTicks", "BspPwm_GetPulse",
                  "BspPwm_SetPulse", "ValveControl_SetWave", "ValveControl_ApplyTreatmentRoute", "xTaskGetTickCount")}

        def intercept(uc, address, size, data):
            name = hooks.get(address)
            if name is None:
                return
            if name == "BspPwm_SetDutyPermille":
                self.pwm = uc.reg_read(UC_ARM_REG_R1)
                self.pulse_ticks = 3200 * self.pwm // 1000
            elif name == "BspPwm_GetPeriodTicks":
                uc.reg_write(UC_ARM_REG_R0, 3200)
            elif name == "BspPwm_GetPulse":
                uc.reg_write(UC_ARM_REG_R0, self.pulse_ticks)
            elif name == "BspPwm_SetPulse":
                self.pulse_ticks = uc.reg_read(UC_ARM_REG_R1)
                self.pwm = 1000 * self.pulse_ticks // 3200
            elif name == "ValveControl_SetWave":
                self.wave = uc.reg_read(UC_ARM_REG_R0)
            elif name == "xTaskGetTickCount":
                uc.reg_write(UC_ARM_REG_R0, 1000)
            uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR))
        self.uc.hook_add(UC_HOOK_CODE, intercept)
        self.set_field(self.controller, "control_config_t", "press_enable_L", "B", 1)
        self.set_field(self.controller, "control_config_t", "press_enable_R", "B", 1)
        self.call("TreatmentPressureControl_InitPid", self.controller)
        self.pwm_min, self.pwm_max = 30, 50


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
                    if stage == 1:
                        target_ticks = 96 + int(min(kp * 5 / 25, 1) * 64)
                        expected = 1000 * target_ticks // 3200
                        for _ in range(180):
                            self.apply(2, 1)
                    self.assertEqual(self.apply(stage + 1, stage), expected)
                    self.assertEqual(self.pwm, expected)
                    self.assertEqual(self.wave, 1)


    def test_zero_and_minimum_and_saturation(self):
        for kp, left, expected in ((0, 5, 0), (0.001, 5, self.pwm_min), (1, 11, 0), (1000, 0, self.pwm_max)):
            with self.subTest(kp=kp, left=left):
                self.set_gains(1, 0, kp)
                self.assertEqual(self.apply(1, 0, left=left), expected)
                self.assertEqual(self.pwm, expected)


    def test_hold_slews_up_one_compare_tick_and_stops_immediately(self):
        self.set_gains(1, 1, 100)
        self.apply(2, 1)
        self.assertEqual(self.pulse_ticks, 96)
        self.apply(2, 1)
        self.assertEqual(self.pulse_ticks, 96)
        self.apply(2, 1)
        self.assertEqual(self.pulse_ticks, 97)
        self.apply(2, 1, left=11)
        self.assertEqual(self.pulse_ticks, 0)

    def test_pulse_off_stops_and_vents(self):
        self.set_gains(1, 2, 100)
        self.assertEqual(self.apply(3, 2), self.pwm_max)
        self.assertEqual(self.apply(4, 2, target=0), 0)
        self.assertEqual((self.pwm, self.wave), (0, 0))


    def configure_cycle(self, rise=10, hold=20, pulse=30):
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


    def test_end_of_cycle_vents_for_full_two_seconds_and_next_cycle_restarts(self):
        self.configure_cycle()
        self.assertEqual(self.build_plan(9999),1)
        self.assertEqual(self.build_plan(10000),2)
        self.assertEqual(self.build_plan(29999),2)
        self.assertIn(self.build_plan(30000),(3,4))
        self.assertNotEqual(self.build_plan(57999),6)
        for tick,pressure in [(58000,10),(58500,0),(59000,5),(59999,0)]:
            self.assertEqual(self.build_plan(tick),6)
            self.execute_plan(pressure,pressure)
            self.assertEqual((self.pwm,self.wave),(0,0))
            self.assertEqual(self.get_field(self.runtime,'TreatmentAppRuntime','target_pressure_kpa','f'),0)
        self.assertEqual(self.build_plan(60000),1)
        self.execute_plan()
        self.assertEqual(self.wave,1)
        self.assertEqual(self.build_plan(60002),1)

    def test_vent_priority_even_if_rise_and_hold_fill_entire_cycle(self):
        self.configure_cycle(rise=40,hold=20,pulse=0)
        self.assertEqual(self.build_plan(58000),6)
        self.execute_plan(10,10)
        self.assertEqual((self.pwm,self.wave),(0,0))

    def test_rtt_profile_wrapper_retains_all_six_groups_and_defaults(self):
        address=0x20010700
        defaults=[[(.50,0,0),(.60,.50,0),(.30,0,0)],[(.50,0,0),(.60,.50,0),(.30,0,0)]]
        for profile in range(2):
            for stage in range(3):
                self.uc.mem_write(address,b'\0'*12)
                self.call('TreatmentPressureControl_ProfileGains',profile,stage,address,address+4,address+8,0)
                for value,expected in zip(struct.unpack('<3f',self.uc.mem_read(address,12)),defaults[profile][stage]):self.assertAlmostEqual(value,expected,places=6)
                gains=(1+profile+stage,.25,.125)
                self.uc.mem_write(address,struct.pack('<3f',*gains))
                self.call('TreatmentPressureControl_ProfileGains',profile,stage,address,address+4,address+8,1)
                self.uc.mem_write(address,b'\0'*12)
                self.call('TreatmentPressureControl_ProfileGains',profile,stage,address,address+4,address+8,0)
                self.assertEqual(struct.unpack('<3f',self.uc.mem_read(address,12)),gains)

if __name__=="__main__": unittest.main()
