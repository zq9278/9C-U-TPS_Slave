import importlib.util
from pathlib import Path
import struct
import unittest

spec=importlib.util.spec_from_file_location('rtt_bridge',Path(__file__).with_name('rtt_bridge.py'))
rtt=importlib.util.module_from_spec(spec);spec.loader.exec_module(rtt)


class Memory:
    def __init__(self):
        self.bytes=bytearray(0x8FE0)
        self.writes=[]
        self.bytes[0x100:0x110]=b'SEGGER RTT'+b'\0'*6
        struct.pack_into('<II',self.bytes,0x110,1,1)
        struct.pack_into('<6I',self.bytes,0x118,0,0x20000500,16,4,12,0)
        struct.pack_into('<6I',self.bytes,0x130,0,0x20000600,16,12,8,0)
        self.bytes[0x50C:0x510]=b'abcd';self.bytes[0x500:0x504]=b'efgh'

    def memory_read8(self,address,count):
        return list(self.bytes[address-0x20000000:address-0x20000000+count])

    def memory_read32(self,address,count):
        return list(struct.unpack_from('<'+'I'*count,self.bytes,address-0x20000000))

    def memory_write32(self,address,data):
        self.writes.append(address);struct.pack_into('<'+'I'*len(data),self.bytes,address-0x20000000,*data)

    def memory_write8(self,address,data):
        self.writes.append(address);self.bytes[address-0x20000000:address-0x20000000+len(data)]=bytes(data)


class RingTests(unittest.TestCase):
    def test_wrap_and_only_ring_owned_writes(self):
        memory=Memory();transport=rtt.RttMemory(memory)
        self.assertEqual(transport.read(),b'abcdefgh')
        self.assertEqual(memory.writes,[0x20000128])
        self.assertEqual(transport.write(b'0123456789'),10)
        self.assertEqual(memory.bytes[0x60C:0x610],b'0123')
        self.assertEqual(memory.bytes[0x600:0x606],b'456789')
        self.assertEqual(memory.writes[-1],0x2000013C)

    def test_bad_pointer_and_reset_never_write(self):
        memory=Memory();transport=rtt.RttMemory(memory)
        struct.pack_into('<I',memory.bytes,0x11C,0x20008FE0)
        with self.assertRaises(RuntimeError):transport.read()
        self.assertEqual(memory.writes,[])
        memory=Memory();transport=rtt.RttMemory(memory);memory.bytes[0x100]=0
        with self.assertRaises(RuntimeError):transport.write(b'help\n')
        self.assertEqual(memory.writes,[])

    def test_commands_whitelisted(self):
        bridge=rtt.Bridge();bridge.link=object();bridge.transport=rtt.RttMemory(Memory())
        with self.assertRaises(ValueError):bridge.request('command',['reset'])
        with self.assertRaises(ValueError):bridge.request('flash',[])
        self.assertTrue(bridge.request('command',['info'])['queued'])


if __name__=='__main__':unittest.main()
