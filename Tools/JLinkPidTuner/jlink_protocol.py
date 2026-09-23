"""J-Link RAM mailbox transport. No halt/reset/flash operations are used."""
from dataclasses import dataclass
import math
from pathlib import Path
import struct
import time

MAGIC = 0x5049444A
VERSION = 1
MAILBOX_SIZE = 180
REQUEST_OFFSET = 12
ACK_OFFSET = 16
PAYLOAD_OFFSET = 24
SAMPLE_OFFSET = 44
SAMPLE_STRUCT = struct.Struct("<8I26f")
PHASES = ("空闲", "升压", "保压", "脉冲加压", "脉冲放气", "暂停", "周期末泄气")
PROFILES = ("单眼", "双眼")
STAGES = ("升压", "保压", "脉冲")


def connect_background(link, device, speed):
    """Attach via the core, bypassing STM32 device-specific reset/init hooks.

    This tuner only accesses the STM32G070RB mailbox, never the flash loader.
    Selecting STM32G070RBTx in the tested J-Link DLL resets and halts the target.
    The generic Cortex-M0 connection works for this M0+ core without that hook.
    """
    import pylink

    if device.strip().upper() not in ("STM32G070RBTX", "STM32G070RB", "CORTEX-M0"):
        raise ValueError("本调试台仅适配 STM32G070RB，请填写 STM32G070RBTx")
    link.exec_command("InhibitConnectRetries = 1")
    link.exec_command("SetRestartOnClose = 0")
    link.set_tif(pylink.enums.JLinkInterfaces.SWD)
    link.connect("Cortex-M0", speed=speed, verbose=False)
    if link.halted():
        raise RuntimeError("MCU 已暂停；请在调试器中恢复运行并释放 J-Link 后重新连接。调试台不会自动复位或运行 CPU")


def validate_gains(profile, stage, gains):
    if profile not in (0, 1) or stage not in (0, 1, 2):
        raise ValueError("参数组或阶段无效")
    if len(gains) != 3 or any(not math.isfinite(x) or not 0 <= x <= 1000 for x in gains):
        raise ValueError("Kp / Ki / Kd 必须为 0～1000 的有限数值")


@dataclass(frozen=True)
class FirmwareSymbols:
    mailbox: int
    build_address: int
    build_id: bytes

    @classmethod
    def from_elf(cls, path):
        from elftools.elf.elffile import ELFFile

        with Path(path).open("rb") as stream:
            elf = ELFFile(stream)
            if elf.elfclass != 32 or not elf.little_endian or elf["e_machine"] != "EM_ARM":
                raise ValueError("请选择本工程的 32 位 ARM ELF 文件")
            table = elf.get_section_by_name(".symtab")
            if table is None:
                raise ValueError("ELF 缺少符号表，请勿使用 strip 后的文件")

            def symbol(name):
                found = table.get_symbol_by_name(name)
                if not found:
                    raise ValueError(f"ELF 缺少 {name}，请先编译包含 J-Link 接口的新固件")
                return found[0]

            mailbox = symbol("g_pid_jlink")
            marker = symbol("g_pid_jlink_build_id")
            if mailbox["st_size"] != MAILBOX_SIZE or marker["st_size"] != 32:
                raise ValueError("ELF 调试接口版本不匹配")
            address = mailbox["st_value"]
            if address % 4 or not 0x20000000 <= address <= 0x20009000 - MAILBOX_SIZE:
                raise ValueError("调试邮箱不在 STM32G070RB SRAM 范围")
            section = elf.get_section(marker["st_shndx"])
            offset = marker["st_value"] - section["sh_addr"]
            build_id = section.data()[offset:offset + 32]
            if len(build_id) != 32 or not 0x08000000 <= marker["st_value"] < 0x08020000 - 31:
                raise ValueError("固件标识地址无效")
            return cls(address, marker["st_value"], build_id)


@dataclass(frozen=True)
class Sample:
    sequence: int
    tick_ms: int
    phase: int
    profile: int
    flags: int
    pwm: int
    pwm_min: int
    pwm_max: int
    target: float
    left: float
    right: float
    feedback: float
    output: float
    p: float
    i: float
    d: float
    gains: tuple

    @classmethod
    def decode(cls, data):
        fields = SAMPLE_STRUCT.unpack(data)
        if any(not math.isfinite(x) for x in fields[8:]):
            raise ValueError("设备返回了非有限压力或 PID 数值")
        return cls(*fields[:16], tuple(fields[16:]))

    def row_gains(self, profile, stage):
        index = (profile * 3 + stage) * 3
        return self.gains[index:index + 3]


class MailboxClient:
    """Single-threaded access only; publish payload before the sequence commit."""

    def __init__(self, link, symbols):
        self.link = link
        self.symbols = symbols
        self.base = symbols.mailbox
        self.pending = None

    def read_bytes(self, address, size):
        words = self.link.memory_read32(address, size // 4)
        if len(words) != size // 4:
            raise IOError("J-Link 内存读取不完整")
        return struct.pack(f"<{len(words)}I", *words)

    def verify(self):
        if self.read_bytes(self.symbols.build_address, 32) != self.symbols.build_id:
            raise ValueError("板上固件与所选 ELF 的调试构建标识不同，请烧录对应固件")
        if struct.unpack("<3I", self.read_bytes(self.base, 12)) != (MAGIC, VERSION, MAILBOX_SIZE):
            raise ValueError("未找到已初始化的 PID 邮箱：请确认固件正在运行、ELF 对应当前固件")
        request, ack = struct.unpack("<2I", self.read_bytes(self.base + REQUEST_OFFSET, 8))
        if request != ack:
            raise RuntimeError("设备仍有未处理参数命令，请确认 MCU 正在运行后重新连接")

    def read_sample(self):
        for _ in range(5):
            data = self.read_bytes(self.base + SAMPLE_OFFSET, SAMPLE_STRUCT.size)
            sequence = struct.unpack_from("<I", data)[0]
            after = self.link.memory_read32(self.base + SAMPLE_OFFSET, 1)[0]
            if sequence != 0 and sequence == after and sequence % 2 == 0:
                return Sample.decode(data)
        return None  # firmware was publishing; retry on next polling interval

    def send_gains(self, profile, stage, gains):
        validate_gains(profile, stage, gains)
        if self.pending is not None:
            raise RuntimeError("上一次参数写入尚未确认")
        request, ack = struct.unpack("<2I", self.read_bytes(self.base + REQUEST_OFFSET, 8))
        if request != ack:
            raise RuntimeError("检测到其他未完成命令，请重新连接")
        sequence = (request + 1) & 0xFFFFFFFF
        # Zero is reserved for reset/uninitialised state, so skip on wrap.
        if sequence == 0:
            sequence = 1
        payload = struct.pack("<2I3f", profile, stage, *gains)
        expected = struct.unpack_from("<3f", payload, 8)
        self.link.memory_write32(self.base + PAYLOAD_OFFSET, list(struct.unpack("<5I", payload)))
        if self.read_bytes(self.base + PAYLOAD_OFFSET, 20) != payload:
            raise IOError("参数负载回读不一致，未提交命令")
        self.link.memory_write32(self.base + REQUEST_OFFSET, [sequence])
        self.pending = (sequence, profile, stage, expected, time.monotonic())

    def poll_ack(self, sample):
        if self.pending is None:
            return None
        sequence, profile, stage, expected, started = self.pending
        ack, status = struct.unpack("<2I", self.read_bytes(self.base + ACK_OFFSET, 8))
        if ack == sequence:
            if status != 1:
                self.pending = None
                raise ValueError("固件拒绝了 PID 参数")
            # ACK can precede the next 20 ms snapshot; wait for true readback.
            if sample is not None and sample.row_gains(profile, stage) == expected:
                self.pending = None
                return profile, stage, expected
        if time.monotonic() - started > 2:
            raise TimeoutError("参数确认超时，结果未知；请重新连接并读取设备参数，不要自动重发")
        return None
