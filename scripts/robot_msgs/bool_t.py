"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO

import struct


class bool_t(object):
    __slots__ = ["utime", "data"]

    __typenames__ = ["int64_t", "boolean"]

    __dimensions__ = [None, None]

    def __init__(self):
        self.utime = 0
        self.data = False

    def encode(self):
        buf = BytesIO()
        buf.write(bool_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qb", self.utime, self.data))

    def decode(data):
        if hasattr(data, "read"):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != bool_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return bool_t._decode_one(buf)

    decode = staticmethod(decode)

    def _decode_one(buf):
        self = bool_t()
        self.utime = struct.unpack(">q", buf.read(8))[0]
        self.data = bool(struct.unpack("b", buf.read(1))[0])
        return self

    _decode_one = staticmethod(_decode_one)

    _hash = None

    def _get_hash_recursive(parents):
        if bool_t in parents:
            return 0
        tmphash = (0xB140141AA43A8D03) & 0xFFFFFFFFFFFFFFFF
        tmphash = (
            ((tmphash << 1) & 0xFFFFFFFFFFFFFFFF) + (tmphash >> 63)
        ) & 0xFFFFFFFFFFFFFFFF
        return tmphash

    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if bool_t._packed_fingerprint is None:
            bool_t._packed_fingerprint = struct.pack(
                ">Q", bool_t._get_hash_recursive([])
            )
        return bool_t._packed_fingerprint

    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)
