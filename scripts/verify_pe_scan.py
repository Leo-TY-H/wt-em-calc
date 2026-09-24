"""Synthetic PE regressions for range attribution and instruction references."""
import struct
import tempfile
from pathlib import Path
import unittest

from pe_scan import PE, references


BASE = 0x140000000


def fixture():
    data = bytearray(0x800)
    def pack(fmt, offset, *values):
        struct.pack_into(fmt, data, offset, *values)
    data[:2] = b'MZ'
    pack('<I', 0x3c, 0x80)
    data[0x80:0x84] = b'PE\0\0'
    pack('<HHIIIHH', 0x84, 0x8664, 3, 0, 0, 0, 240, 0)
    opt = 0x98
    pack('<H', opt, 0x20b)
    pack('<Q', opt+24, BASE)
    pack('<II', opt+56, 0x4000, 0x200)
    pack('<I', opt+108, 16)
    pack('<II', opt+112+24, 0x3000, 24)
    for i, (name, rva, offset, flags) in enumerate([
        (b'.text', 0x1000, 0x200, 0x60000020),
        (b'.rdata', 0x2000, 0x400, 0x40000040),
        (b'.pdata', 0x3000, 0x600, 0x40000040),
    ]):
        row = opt+240+i*40
        pack('<8sIIII', row, name, 0x200, rva, 0x200, offset)
        pack('<I', row+36, flags)
    # One real string reference and direct call, then a leaf gap and function 2.
    data[0x200:0x207] = b'\x48\x8d\x05'+struct.pack('<i', 0x2000-0x1007)
    data[0x207:0x20c] = b'\xe8'+struct.pack('<i', 0x1020-0x100c)
    data[0x20c:0x210] = b'\x90\x90\x90\xc3'
    data[0x210:0x217] = b'\x48\x8d\x05'+struct.pack('<i', 0x2000-0x1017)
    data[0x220] = 0xc3
    data[0x400:0x40b] = b'Instructor\0'
    pack('<III', 0x600, 0x1000, 0x1010, 0x3050)
    pack('<III', 0x60c, 0x1020, 0x1021, 0x3054)
    return data


class PEChecks(unittest.TestCase):
    def load(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'fixture.exe'
            path.write_bytes(data)
            return PE(path)

    def test_ranges_and_file_backing(self):
        binary = self.load(fixture())
        self.assertEqual(binary.function(BASE+0x100f)[:2], (BASE+0x1000, BASE+0x1010))
        self.assertIsNone(binary.function(BASE+0xfff))
        self.assertIsNone(binary.function(BASE+0x1010))
        self.assertIsNone(binary.function(BASE+0x101f))
        self.assertIsNotNone(binary.function(BASE+0x1020))
        self.assertIsNone(binary.function(BASE+0x1021))
        self.assertEqual(binary.read(BASE+0x2000, 11), b'Instructor\0')
        with self.assertRaises(ValueError):
            binary.read(BASE+0x21ff, 2)
        with self.assertRaises(ValueError):
            binary.read(BASE+0x2000, -1)

    def test_references_exclude_unattributed_gap(self):
        binary = self.load(fixture())
        rows = references(binary, {BASE+0x2000: 'Instructor'})
        self.assertEqual([row['address'] for row in rows], [hex(BASE+0x1000)])
        calls = references(binary, {BASE+0x1020: 'callee'}, calls=True)
        self.assertEqual([row['address'] for row in calls], [hex(BASE+0x1007)])

    def test_overlapping_false_candidate_does_not_hide_real_call(self):
        data = fixture()
        # mov al, 0xe8; call ... -- the immediate must not consume the call.
        data[0x200:0x207] = b'\xb0\xe8\xe8'+struct.pack('<i', 0x1020-0x1007)
        data[0x207:0x210] = b'\x90'*8+b'\xc3'
        rows = references(self.load(data), {BASE+0x1020: 'callee'}, calls=True)
        self.assertEqual([row['address'] for row in rows], [hex(BASE+0x1002)])

    def test_malformed_images(self):
        variants = [b'', b'MZ', fixture()[:0x610]]
        for offset, fmt, value in [(0x84, '<H', 0x14c), (0x98, '<H', 0x10b),
                                   (0x98+112+28, '<I', 23), (0x604, '<I', 0x1000)]:
            data = fixture()
            struct.pack_into(fmt, data, offset, value)
            variants.append(data)
        for index, data in enumerate(variants):
            with self.subTest(index=index), self.assertRaises(ValueError):
                self.load(data)


if __name__ == '__main__':
    unittest.main()
