#!/usr/bin/env python3
"""bert_cli - host side of the sfp-10g-bert-pocket tester.

Three jobs, one file, standard library only:

  ddm    decode the ten raw bytes an SFP+ module returns from SFF-8472 page A2h
         (temperature, supply, bias, TX and RX optical power) into units a
         person can check
  ber    turn an error count and a bit count into a bit error ratio plus a 95 %
         upper bound; with zero errors the bound is -ln(0.05)/bits, which is the
         number a datasheet actually means when it prints "< 1e-12"
  demo   run both against the values the RTL testbench uses, so the arithmetic
         in this file and the bytes on the simulated bus cannot drift apart

Examples
    python bert_cli.py demo
    python bert_cli.py ddm --hex 1a4080e800c803e800fa
    python bert_cli.py ber --errors 0 --seconds 100
    python bert_cli.py ber --errors 3 --bits 10312500000000

The unit conversions follow SFF-8472 rev 12.4, section 9:
    temperature  signed 16-bit, 1/256 degC
    Vcc          unsigned 16-bit, 100 uV
    TX bias      unsigned 16-bit, 2 uA
    TX/RX power  unsigned 16-bit, 0.1 uW
"""

from __future__ import annotations

import argparse
import math
import sys

LINE_RATE_BPS = 10_312_500_000.0          # 10GBASE-R, 64b/66b coded line rate
PAYLOAD_BPS = 10_000_000_000.0            # what the customer usually quotes
DDM_BYTES = 10
POISSON_ALPHA = 0.05                      # 95 % confidence


# --------------------------------------------------------------------------
# SFF-8472 decode
# --------------------------------------------------------------------------
def _s16(msb: int, lsb: int) -> int:
    raw = (msb << 8) | lsb
    return raw - 0x10000 if raw & 0x8000 else raw


def _u16(msb: int, lsb: int) -> int:
    return (msb << 8) | lsb


def uW_to_dbm(power_uW: float) -> float:
    if power_uW <= 0:
        return float("-inf")
    return 10.0 * math.log10(power_uW / 1000.0)


def decode_ddm(raw: bytes) -> dict:
    """Decode the ten A2h bytes 0x60..0x69 into engineering units."""
    if len(raw) != DDM_BYTES:
        raise ValueError("need exactly %d bytes, got %d" % (DDM_BYTES, len(raw)))

    temp_raw = _s16(raw[0], raw[1])
    vcc_raw = _u16(raw[2], raw[3])
    bias_raw = _u16(raw[4], raw[5])
    tx_raw = _u16(raw[6], raw[7])
    rx_raw = _u16(raw[8], raw[9])

    tx_uW = tx_raw * 0.1
    rx_uW = rx_raw * 0.1

    return {
        "temperature_c": temp_raw / 256.0,
        "vcc_v": vcc_raw * 100e-6,
        "tx_bias_ma": bias_raw * 2e-3,
        "tx_power_uW": tx_uW,
        "tx_power_dbm": uW_to_dbm(tx_uW),
        "rx_power_uW": rx_uW,
        "rx_power_dbm": uW_to_dbm(rx_uW),
        "raw": {
            "temperature": temp_raw,
            "vcc": vcc_raw,
            "tx_bias": bias_raw,
            "tx_power": tx_raw,
            "rx_power": rx_raw,
        },
    }


# --------------------------------------------------------------------------
# BER and confidence bound
# --------------------------------------------------------------------------
def poisson_cdf(k: int, lam: float) -> float:
    """P(X <= k) for X ~ Poisson(lam), summed term by term."""
    if lam <= 0.0:
        return 0.0 if k < 0 else 1.0
    term = math.exp(-lam)
    total = term
    for i in range(1, k + 1):
        term *= lam / i
        total += term
    return total


def poisson_upper_count(errors: int, confidence: float = 0.95) -> float:
    """Upper confidence bound on the Poisson mean given an observed count.

    With zero errors this is exactly -ln(1 - confidence) = 2.996, i.e. the usual
    "less than three errors could have happened" statement.
    """
    alpha = 1.0 - confidence
    if errors == 0:
        return -math.log(alpha)
    lo, hi = 0.0, errors + 10.0 * math.sqrt(errors) + 20.0
    while poisson_cdf(errors, hi) > alpha:
        hi *= 2.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if poisson_cdf(errors, mid) > alpha:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def ber_summary(errors: int, bits: float, confidence: float = 0.95) -> dict:
    if bits <= 0:
        raise ValueError("bit count must be positive")
    return {
        "errors": errors,
        "bits": bits,
        "ber": errors / bits,
        "ber_upper": poisson_upper_count(errors, confidence) / bits,
        "confidence": confidence,
        "clean": errors == 0,
    }


def bits_for(seconds: float, rate_bps: float = LINE_RATE_BPS) -> float:
    return seconds * rate_bps


# --------------------------------------------------------------------------
# formatting
# --------------------------------------------------------------------------
def fmt_ddm(d: dict) -> str:
    rows = [
        ("temperature", "%8.2f degC" % d["temperature_c"]),
        ("Vcc", "%8.3f V" % d["vcc_v"]),
        ("TX bias", "%8.3f mA" % d["tx_bias_ma"]),
        ("TX power", "%8.2f dBm  (%.1f uW)" % (d["tx_power_dbm"], d["tx_power_uW"])),
        ("RX power", "%8.2f dBm  (%.1f uW)" % (d["rx_power_dbm"], d["rx_power_uW"])),
    ]
    width = max(len(name) for name, _ in rows)
    return "\n".join("  %-*s  %s" % (width, name, value) for name, value in rows)


def fmt_ber(s: dict) -> str:
    lines = [
        "  errors            %d" % s["errors"],
        "  bits tested       %.4g  (%.1f s at %.4g Gbit/s)"
        % (s["bits"], s["bits"] / LINE_RATE_BPS, LINE_RATE_BPS / 1e9),
        "  BER               %.3e" % s["ber"],
        "  BER upper (%.0f%%)  %.3e" % (s["confidence"] * 100, s["ber_upper"]),
    ]
    if s["clean"]:
        lines.append("  note              no errors seen; the bound is -ln(0.05)/bits")
    return "\n".join(lines)


# --------------------------------------------------------------------------
# commands
# --------------------------------------------------------------------------
DEMO_HEX = "1a4080e800c803e800fa"


def cmd_demo(_args) -> int:
    raw = bytes.fromhex(DEMO_HEX)
    d = decode_ddm(raw)
    print("SFF-8472 A2h 0x60..0x69")
    print("  raw bytes         %s" % " ".join("%02x" % b for b in raw))
    print(fmt_ddm(d))
    print()
    print("BERT, 100 s at 10.3125 Gbit/s, zero errors")
    print(fmt_ber(ber_summary(0, bits_for(100.0))))
    print()
    print("BERT, 600 s, three errors")
    print(fmt_ber(ber_summary(3, bits_for(600.0))))
    return 0


def cmd_ddm(args) -> int:
    if args.hex:
        text = args.hex
    else:
        with open(args.file, "r", encoding="utf-8") as fh:
            text = fh.read()
    text = "".join(ch for ch in text if ch in "0123456789abcdefABCDEF")
    if len(text) < 20:
        print("error: need 10 bytes (20 hex digits)", file=sys.stderr)
        return 2
    raw = bytes.fromhex(text[:20])
    print("SFF-8472 A2h 0x60..0x69")
    print("  raw bytes         %s" % " ".join("%02x" % b for b in raw))
    print(fmt_ddm(decode_ddm(raw)))
    return 0


def cmd_ber(args) -> int:
    if args.bits is not None:
        bits = float(args.bits)
    else:
        bits = bits_for(args.seconds)
    print(fmt_ber(ber_summary(args.errors, bits)))
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="bert_cli",
        description="host side of the sfp-10g-bert-pocket tester",
    )
    sub = p.add_subparsers(dest="cmd")

    d = sub.add_parser("ddm", help="decode an SFF-8472 A2h dump")
    g = d.add_mutually_exclusive_group(required=True)
    g.add_argument("--hex", help="20 hex digits, 0x60..0x69")
    g.add_argument("--file", help="file containing the hex digits")
    d.set_defaults(func=cmd_ddm)

    b = sub.add_parser("ber", help="bit error ratio and confidence bound")
    b.add_argument("--errors", type=int, required=True)
    b.add_argument("--bits", type=float, default=None)
    b.add_argument("--seconds", type=float, default=100.0)
    b.set_defaults(func=cmd_ber)

    m = sub.add_parser("demo", help="run both paths on the testbench values")
    m.set_defaults(func=cmd_demo)

    return p


def main(argv=None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "func", None):
        return cmd_demo(args)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
