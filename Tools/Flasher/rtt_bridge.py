"""JSON-lines IPC for background J-Link RTT. No halt/reset/flash operations."""
import codecs
import json
import os
import queue
import struct
import sys
import threading
import time


class RttMemory:
    START, END = 0x20000000, 0x20008FE0

    def __init__(self, link):
        self.link = link
        self.base = None
        data = bytes(link.memory_read8(self.START, self.END - self.START))
        for offset in range(0, len(data) - 24, 4):
            if data[offset:offset+16] != b'SEGGER RTT' + b'\0' * 6:
                continue
            up, down = struct.unpack_from('<II', data, offset + 16)
            if not (1 <= up <= 16 and 1 <= down <= 16) or offset + 24 + 24*(up+down) > len(data):
                continue
            self.base = self.START + offset
            self.up_count, self.down_count = up, down
            try:
                self.descriptor(0, True)
                self.descriptor(0, False)
                return
            except RuntimeError:
                self.base = None
        raise RuntimeError('未找到有效 RTT；请确认设备已运行带 RTT 的 TPS 固件')

    def descriptor(self, index, up):
        count = self.up_count if up else self.down_count
        if not isinstance(index, int) or index < 0 or index >= count:
            raise RuntimeError('RTT channel out of range')
        # Revalidate after resets; never write through stale or corrupted pointers.
        if bytes(self.link.memory_read8(self.base, 16)) != b'SEGGER RTT' + b'\0'*6:
            raise RuntimeError('RTT 控制块已改变，请重新连接')
        address = self.base + 24 + 24 * (index if up else self.up_count + index)
        _, buffer, size, wr, rd, flags = self.link.memory_read32(address, 6)
        if not (self.START <= buffer < self.END and 2 <= size <= self.END-buffer and wr < size and rd < size
                and (buffer+size <= self.base or buffer >= self.base + 24 + 24*(self.up_count+self.down_count))):
            raise RuntimeError('RTT ring pointer invalid; stopped without writing target data')
        return address, buffer, size, wr, rd

    def read(self, limit=2048):
        address, buffer, size, wr, rd = self.descriptor(0, True)
        count = min(limit, (wr-rd) % size)
        if not count:
            return b''
        first = min(count, size-rd)
        data = bytes(self.link.memory_read8(buffer+rd, first))
        if first < count:
            data += bytes(self.link.memory_read8(buffer, count-first))
        # Only the RTT ring's host-owned read index is changed.
        self.link.memory_write32(address+16, [(rd+count) % size])
        return data

    def write(self, data):
        address, buffer, size, wr, rd = self.descriptor(0, False)
        count = min(len(data), (rd-wr-1) % size)
        if not count:
            return 0
        first = min(count, size-wr)
        self.link.memory_write8(buffer+wr, list(data[:first]))
        if first < count:
            self.link.memory_write8(buffer, list(data[first:count]))
        self.link.memory_write32(address+12, [(wr+count) % size])
        return count


class Bridge:
    def __init__(self):
        self.link = None
        self.decoder = codecs.getincrementaldecoder('utf-8')('replace')

    def connect(self, options):
        if self.link:
            raise RuntimeError('RTT 已连接')
        serial = str(options.get('serial', ''))
        if (serial and (not serial.isdigit() or len(serial)>12)):
            raise ValueError('Invalid probe serial')
        import pylink
        dll = os.environ.get('JLINK_DLL')
        if not dll:
            raise RuntimeError('JLINK_DLL is not configured')
        self.link = pylink.JLink(lib=pylink.library.Library(dllpath=dll))
        try:
            self.link.open(serial_no=int(serial) if serial else None)
            self.link.set_tif(pylink.enums.JLinkInterfaces.SWD)
            self.link.exec_command('InhibitConnectRetries = 1')
            # Generic core attach avoids device-specific work-RAM initialization.
            # The separate programmer selects the exact STM32G070RB for flashing.
            self.link.connect('Cortex-M0', speed=1000)
            if self.link.halted():
                raise RuntimeError('目标已暂停；请先在调试器恢复运行，本工具不会自动运行 CPU')
            self.transport = RttMemory(self.link)
            return {'control_block': hex(self.transport.base), 'speed_khz': 1000, 'mode': 'background RTT'}
        except Exception:
            self.close()
            raise

    def close(self):
        if self.link:
            try:
                self.link.close()
            finally:
                self.link = None
        return True

    def request(self, method, args):
        if method == 'connect':
            return self.connect(args[0] if args else {})
        if method == 'disconnect':
            return self.close()
        if not self.link:
            raise RuntimeError('RTT 未连接')
        if method != 'command' or len(args)!=1 or args[0] not in ('help', 'info', 'status'):
            raise ValueError('Only read-only help/info/status commands are supported')
        data = (args[0] + '\n').encode('ascii')
        offset, deadline = 0, time.monotonic()+1
        while offset < len(data) and time.monotonic() < deadline:
            offset += self.transport.write(data[offset:])
            if offset < len(data):
                time.sleep(.01)
        if offset != len(data):
            raise RuntimeError('RTT 下行缓冲已满，命令未完整发送')
        # Transport success isn't a firmware acknowledgement; replies appear in log.
        return {'queued': True, 'command': args[0]}

    def logs(self):
        data = self.transport.read()
        if data:
            print(json.dumps({'event':'log', 'text':self.decoder.decode(data)}, ensure_ascii=True), flush=True)


def main():
    bridge = Bridge()
    commands = queue.Queue()
    def reader():
        for line in sys.stdin:
            try:
                commands.put(json.loads(line))
            except ValueError:
                pass
        commands.put(None)
    threading.Thread(target=reader, daemon=True).start()
    try:
        while True:
            try:
                cmd = commands.get(timeout=.03)
            except queue.Empty:
                if bridge.link:
                    try:
                        bridge.logs()
                    except Exception as exc:
                        bridge.close()
                        print(json.dumps({'event':'disconnected','error':str(exc)},ensure_ascii=True),flush=True)
                continue
            if cmd is None:
                break
            try:
                result = bridge.request(cmd['method'],cmd.get('args',[]))
                reply = {'id':cmd['id'], 'result':result}
            except Exception as exc:
                reply = {'id':cmd.get('id'), 'error':str(exc)}
            print(json.dumps(reply,ensure_ascii=True),flush=True)
    finally:
        bridge.close()


if __name__ == '__main__':
    main()
