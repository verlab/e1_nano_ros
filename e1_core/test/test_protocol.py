import math

from e1_core import protocol as P


def test_roundtrip_all_messages():
    msgs = [P.State((1, 0, 0, 0), (0.1, 0.2, 0.3), (0, 0, 9.8), (0.5, 0.5, 0.6, 0.6), 0.55, 4.01, P.AUTO, True, P.F_LANDED, 7, 1234),
            P.Pose((1, 2, 3), (1, 0, 0, 0), (0.1, 0.2, 0.3), 3, 99),
            P.CmdAttitude((0.9, 0.1, 0, 0), (0, 0, 0.5), 0.6, P.IGNORE_RATES, 1, 2),
            P.CmdManual(0.1, -0.2, 0.3, 0.4, math.nan, 5, 6),
            P.CmdMotors((0.1, 0.2, 0.3, 0.4), 1, 1),
            P.CmdArm(True, 1, 1), P.CmdMode(P.STAB, 1, 1), P.CmdParam('CTL_TRIM_ROLL', 0.01, 1, 1)]
    for m in msgs:
        out = type(m).decode(m.encode())
        for f in m.__dataclass_fields__:
            a, b = getattr(m, f), getattr(out, f)
            if isinstance(a, float) and math.isnan(a):
                assert math.isnan(b)
            elif isinstance(a, tuple):
                assert all(abs(x - y) < 1e-6 for x, y in zip(a, b)), f
            elif isinstance(a, float):
                assert abs(a - b) < 1e-6, f
            else:
                assert a == b, f


def test_sizes_match_firmware_header():
    # firmware/src_e1/e1_protocol.h static_asserts the same numbers
    assert P.SIZES['STATE'] == 76 and P.SIZES['POSE'] == 48
    assert P.SIZES['CMD_ATTITUDE'] == 41 and P.SIZES['CMD_PARAM'] == 28


def test_keys():
    assert P.key(3, 'cmd', 'attitude') == 'e1/3/cmd/attitude'
    assert P.parse_id('e1/12/state') == 12 and P.parse_id('e1/all/cmd/arm') == 'all' and P.parse_id('x/1') is None


def test_rejects_wrong_type_and_version():
    buf = P.CmdArm(True).encode()
    try:
        P.CmdMode.decode(buf)
        assert False
    except ValueError:
        pass
    bad = bytes([9]) + buf[1:]
    try:
        P.CmdArm.decode(bad)
        assert False
    except ValueError:
        pass
