import struct

AA, MSG_SETPOINT, MSG_TELEMETRY, MSG_PID = 0xAA, 1, 2, 3


def frame(t, payload: bytes) -> bytes:
    c = t ^ len(payload)
    for b in payload:
        c ^= b
    return bytes([AA, 0x55, t, len(payload)]) + payload + bytes([c])


def setpoint(armed=False, depth_hold=False, heading_hold=False,
             surge=0.0, sway=0.0, heave=0.0, yaw_rate=0.0,
             target_heading=0.0, target_depth=0.0,
             target_pitch=0.0, target_roll=0.0) -> bytes:
    p = struct.pack("<BBB8f", 1 if armed else 0, 1 if depth_hold else 0,
                    1 if heading_hold else 0, surge, sway, heave,
                    yaw_rate, target_heading, target_depth,
                    target_pitch, target_roll)
    return frame(MSG_SETPOINT, p)


def pid(dkp=0.8, dki=0.05, dkd=0.2, hkp=0.01, hki=0.0, hkd=0.0,
        pkp=0.0, pki=0.0, pkd=0.0, rkp=0.0, rki=0.0, rkd=0.0) -> bytes:
    return frame(MSG_PID, struct.pack("<12f", dkp, dki, dkd, hkp, hki, hkd,
                                      pkp, pki, pkd, rkp, rki, rkd))


def parse_telemetry(p: bytes) -> dict:
    vals = struct.unpack("<5f2B", p[:22])
    return {
        "heading": vals[0], "pitch": vals[1], "roll": vals[2],
        "depth": vals[3], "temp": vals[4],
        "armed": vals[5], "fault": vals[6],
        "thr": struct.unpack("<8f", p[22:]),
    }


def parse_pid(p: bytes) -> dict:
    (dkp, dki, dkd, hkp, hki, hkd,
     pkp, pki, pkd, rkp, rki, rkd) = struct.unpack("<12f", p)
    return {"depth_kp": dkp, "depth_ki": dki, "depth_kd": dkd,
            "head_kp": hkp, "head_ki": hki, "head_kd": hkd,
            "pitch_kp": pkp, "pitch_ki": pki, "pitch_kd": pkd,
            "roll_kp": rkp, "roll_ki": rki, "roll_kd": rkd}


class Link:
    """Chunk-safe RX: feed() accepts any byte split, returns whole frames."""

    def __init__(self):
        self._buf = b""

    def feed(self, chunk: bytes):
        out = []
        self._buf += chunk
        while True:
            i = self._buf.find(b"\xAA")
            if i < 0:
                self._buf = b""
                break
            if i:
                self._buf = self._buf[i:]
            if len(self._buf) < 4:
                break
            if self._buf[1] != 0x55:
                self._buf = self._buf[1:]
                continue
            t, n = self._buf[2], self._buf[3]
            if len(self._buf) < 4 + n + 1:
                break
            payload = self._buf[4:4 + n]
            ok = t ^ n
            for b in payload:
                ok ^= b
            if ok == self._buf[4 + n]:
                if t == MSG_TELEMETRY and n == 54:
                    out.append(("telemetry", parse_telemetry(payload)))
                elif t == MSG_PID and n == 48:
                    out.append(("pid", parse_pid(payload)))
                self._buf = self._buf[4 + n + 1:]
            else:
                self._buf = self._buf[1:]
        return out
