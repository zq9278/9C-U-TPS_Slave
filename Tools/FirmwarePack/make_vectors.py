"""Generate deterministic wire examples for the Qt/C codec interop tests."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import pack_firmware as p


def wire(cmd, seq, session, payload):
    body = struct.pack('<BBHIH', 1, cmd, seq, session, len(payload)) + payload
    return b'TPSU' + body + struct.pack('<I', p.crc32(body))


def vectors():
    image = bytearray((i & 255) for i in range(521))
    struct.pack_into('<II', image, 0, p.RAM_END, p.APP_BASE + 9)
    header = p.make_header(image, p.version_code('1.2.3'))
    session = 0x12345678
    hello_info = struct.pack('<HBBIIHHIIIB3xI', 0, 1, 0, 0, p.TARGET,
                             1, 256, 0x10000, p.APP_BASE, p.APP_CAPACITY, 0, 0)
    rows = [
        ('hello_request', 1, 1, 0, struct.pack('<I', session)),
        ('hello_response_no_app', 0x81, 1, session, hello_info),
        ('begin_request', 2, 2, session, header),
        ('begin_response', 0x82, 2, session, struct.pack('<HBBI', 0, 2, 0, 0)),
        ('data_first_request', 3, 3, session, struct.pack('<I', 0) + image[:256]),
        ('data_first_response', 0x83, 3, session, struct.pack('<HBBI', 0, 2, 0, 256)),
        ('data_last_request', 3, 5, session, struct.pack('<I', 512) + image[512:]),
        ('end_request_empty', 4, 6, session, b''),
        ('end_response', 0x84, 6, session, struct.pack('<HBBI', 0, 3, 0, len(image))),
        ('run_request_empty', 5, 7, session, b''),
        ('bad_offset_response', 0x83, 4, session, struct.pack('<HBBI', 8, 2, 0, 256)),
    ]
    return {'crc32_check_123456789': 'CBF43926', 'fixture_image_hex': image.hex(),
            'fixture_header_hex': header.hex(), 'frames': [
                {'name': name, 'cmd': cmd, 'seq': seq, 'session': sess,
                 'payload_hex': bytes(payload).hex(), 'wire_hex': wire(cmd, seq, sess, payload).hex()}
                for name, cmd, seq, sess, payload in rows]}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-binary', help='native test_boot executable to check C codec against these vectors')
    args = parser.parse_args()
    data = vectors()
    path = Path(__file__).with_name('protocol_vectors.json')
    if args.check_binary:
        subprocess.run([args.check_binary, '--vectors'], check=True, text=True,
                       input='\n'.join(row['wire_hex'] for row in data['frames']) + '\n')
    else:
        path.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')
        print(path)
