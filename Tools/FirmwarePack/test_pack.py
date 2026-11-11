import struct
import unittest
import pack_firmware as p


class PackTests(unittest.TestCase):
    def setUp(self):
        self.image = struct.pack('<II', p.RAM_END, p.APP_BASE + 9) + b'hello world'

    def test_package_and_factory(self):
        header = p.make_header(self.image, p.version_code('1.2.3'))
        self.assertEqual(p.validate_package(header + self.image), (header, self.image))
        boot = struct.pack('<II', p.RAM_END, 0x08000009) + b'boot'
        factory = p.factory_image(boot, header + self.image)
        self.assertEqual(len(factory), 131072)
        self.assertEqual(factory[0x4000:0x4000+len(self.image)], self.image)
        self.assertEqual(factory[0x1F000:0x1F040], factory[0x1F800:0x1F840])
        self.assertEqual(factory[0x1F038:0x1F040], b'TPSVALID')

    def test_crc_and_rejections(self):
        self.assertEqual(p.crc32(b'123456789'), 0xCBF43926)
        package = p.make_header(self.image, 0) + self.image
        for corrupt in (package[:-1], package+b'x', package[:70]+bytes([package[70]^1])+package[71:]):
            with self.assertRaises(ValueError): p.validate_package(corrupt)
        with self.assertRaises(ValueError): p.make_header(b'\0'*16, 0)
        with self.assertRaises(ValueError): p.make_header(self.image, 0, 2, 1)
        with self.assertRaises(ValueError): p.version_code('1.256.0')

    def test_compiled_identity_version_is_authoritative(self):
        identity = bytearray(64)
        struct.pack_into('<8sIHHIIII',identity,0,b'TPSID001',0x10100,3,0,p.TARGET,p.APP_BASE,0x10000,1)
        identity[32:38] = b'0.1.1\0'
        parsed = p.inspect_identity(self.image + identity)
        self.assertEqual(parsed['version_code'],p.version_code('0.1.1'))
        self.assertEqual(parsed['hardware'],3)
        identity[32:38] = b'0.2.1\0'
        with self.assertRaises(ValueError): p.inspect_identity(self.image + identity)
        with self.assertRaises(ValueError): p.inspect_identity(self.image)


if __name__ == '__main__':
    unittest.main()
