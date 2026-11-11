"""Build a TPSFW001 package and optional full SWD factory image (stdlib only)."""
import argparse
from pathlib import Path
import struct
import zlib

APP_BASE = 0x08004000
APP_CAPACITY = 108 * 1024
TARGET = 0x9C01
RAM_END = 0x20008FE0


def crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF


def version_code(text):
    parts = [int(x) for x in text.split('.')]
    if len(parts) not in (3, 4) or any(x < 0 or x > 255 for x in parts):
        raise ValueError('version must be major.minor.patch[.build], each 0..255')
    parts += [0] * (4 - len(parts))
    return int.from_bytes(bytes(parts), 'big')


def check_vectors(image):
    if not 8 <= len(image) <= APP_CAPACITY:
        raise ValueError('App size must be 8..110592 bytes')
    sp, pc = struct.unpack_from('<II', image)
    if not (0x20000000 < sp <= RAM_END and sp % 8 == 0 and pc & 1
            and APP_BASE <= (pc & ~1) < APP_BASE + len(image)):
        raise ValueError('App vectors do not match the relocated Flash/RAM layout')


def inspect_identity(image):
    """Read the identity compiled into the App, not a manually supplied label."""
    offsets = [i for i in range(len(image)) if image.startswith(b'TPSID001', i)]
    if len(offsets) != 1 or offsets[0] + 64 > len(image):
        raise ValueError('exactly one compiled TPSID001 identity is required')
    offset = offsets[0]
    version, hardware, reserved, target, base, min_boot, rtt = struct.unpack_from('<IHHIIII', image, offset + 8)
    raw = image[offset + 32:offset + 64]
    text = raw.split(b'\0', 1)[0].decode('ascii')
    if (reserved or target != TARGET or base != APP_BASE or rtt not in (0, 1)
            or b'\0' not in raw or version_code(text) != version):
        raise ValueError('invalid compiled firmware identity')
    return dict(version=text, version_code=version, hardware=hardware, target=target,
                app_base=base, min_boot_version=min_boot, rtt=bool(rtt))


def make_header(image, version, hw_min=1, hw_max=1):
    check_vectors(image)
    if not 0 <= hw_min <= hw_max <= 65535:
        raise ValueError('invalid hardware revision range')
    h = bytearray(64)
    struct.pack_into('<8sHHIHHIIIII', h, 0, b'TPSFW001', 64, 1, TARGET,
                     hw_min, hw_max, APP_BASE, len(image), crc32(image), version, 0x10000)
    struct.pack_into('<I', h, 60, crc32(h[:60]))
    return bytes(h)


def validate_package(package):
    if len(package) < 72:
        raise ValueError('package too short')
    h, image = package[:64], package[64:]
    if (h[:8] != b'TPSFW001' or struct.unpack_from('<HHI', h, 8) != (64, 1, TARGET)
            or struct.unpack_from('<I', h, 20)[0] != APP_BASE
            or any(h[40:60]) or crc32(h[:60]) != struct.unpack_from('<I', h, 60)[0]
            or len(image) != struct.unpack_from('<I', h, 24)[0]
            or crc32(image) != struct.unpack_from('<I', h, 28)[0]):
        raise ValueError('invalid header, image length or CRC')
    check_vectors(image)
    hw_min, hw_max = struct.unpack_from('<HH', h, 16)
    if hw_min > hw_max:
        raise ValueError('invalid hardware range')
    return h, image


def make_metadata(h):
    r = bytearray(64)
    struct.pack_into('<4sHHI', r, 0, b'TPSM', 1, 64, 1)
    r[12:16] = h[12:16]
    r[16:32] = h[20:36]
    r[32:36] = h[16:20]
    struct.pack_into('<I', r, 52, crc32(r[:52]))
    r[56:64] = b'TPSVALID'
    return bytes(r)


def factory_image(boot, package):
    h, app = validate_package(package)
    if not 8 <= len(boot) <= 16384:
        raise ValueError('Boot size must be 8..16384 bytes')
    sp, pc = struct.unpack_from('<II', boot)
    if not (0x20000000 < sp <= RAM_END and sp % 8 == 0 and pc & 1
            and 0x08000000 <= (pc & ~1) < 0x08000000 + len(boot)):
        raise ValueError('invalid Boot vectors')
    out = bytearray(b'\xff' * (128 * 1024))
    out[:len(boot)] = boot
    out[0x4000:0x4000 + len(app)] = app
    metadata = make_metadata(h)
    out[0x1F000:0x1F040] = metadata
    out[0x1F800:0x1F840] = metadata
    return bytes(out)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('app', type=Path)
    p.add_argument('--version', help='optional assertion; must equal compiled App version')
    p.add_argument('--hw-min', type=int)
    p.add_argument('--hw-max', type=int)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--boot', type=Path)
    p.add_argument('--factory-output', type=Path)
    args = p.parse_args()
    if bool(args.boot) != bool(args.factory_output):
        p.error('--boot and --factory-output must be supplied together')
    try:
        image = args.app.read_bytes()
        identity = inspect_identity(image)
        if args.version and version_code(args.version) != identity['version_code']:
            raise ValueError('--version differs from the actual App identity')
        hw_min = identity['hardware'] if args.hw_min is None else args.hw_min
        hw_max = identity['hardware'] if args.hw_max is None else args.hw_max
        if not hw_min <= identity['hardware'] <= hw_max:
            raise ValueError('hardware range excludes compiled board revision')
        header = bytearray(make_header(image, identity['version_code'], hw_min, hw_max))
        struct.pack_into('<I', header, 36, identity['min_boot_version'])
        struct.pack_into('<I', header, 60, crc32(header[:60]))
        package = bytes(header) + image
        factory = factory_image(args.boot.read_bytes(), package) if args.boot else None
        args.output.write_bytes(package)
        if factory is not None:
            args.factory_output.write_bytes(factory)
        print(f'{args.output}: App {len(image)} bytes, CRC32={crc32(image):08X}')
    except (OSError, ValueError) as exc:
        p.exit(1, f'error: {exc}\n')


if __name__ == '__main__':
    main()
