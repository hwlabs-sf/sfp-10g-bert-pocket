#!/usr/bin/env python3
"""Tests for sfp-10g-bert-pocket.

Four independent things are checked, each against a model that does not share
code with the thing under test:

  * the 95 rows of `rtl/prbs31_64b.v` are parsed out of the Verilog text and
    evaluated against a bit-serial x^31 + x^28 + 1 generator -- 10 000 bits,
    157 words, two different seeds.  This is the check that catches a typo in
    one of the 64 parallel rows, which is the failure mode that matters.
  * the self-synchronising checker is modelled at bit level, and a single
    flipped bit must produce exactly the burst the testbench sees (three
    reported errors: the bit itself and the two predictions that depend on it).
  * SFF-8472 unit conversion and the Poisson upper bound are checked against
    values worked out by hand, including the -ln(0.05)/bits case.
  * the KiCad board must come back clean from `kicad-dfm-precheck`.

If `iverilog` is on PATH the two testbenches are also compiled and run.  If the
`kicad-dfm-precheck` repository is not cloned next to this one, that test is
skipped with a message rather than silently passing.
"""

from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
SHOWCASE = PKG.parent

sys.path.insert(0, str(PKG / "host"))
import bert_cli  # noqa: E402

DEG = 31
MASK31 = (1 << DEG) - 1
SEED = 0x7FFF_FFFF


# --------------------------------------------------------------------------
# Verilog equation extraction
# --------------------------------------------------------------------------
ASSIGN_RE = re.compile(r"assign\s+(\w+)\s*\[\s*(\d+)\s*\]\s*=\s*([^;]+);")


def parse_rows(path: Path):
    """Return {name: {index: [state bit, ...]}} for every assign in the file."""
    text = path.read_text(encoding="utf-8")
    rows: dict[str, dict[int, list[int]]] = {}
    for name, index, rhs in ASSIGN_RE.findall(text):
        terms = [int(m) for m in re.findall(r"state\s*\[\s*(\d+)\s*\]", rhs)]
        if not terms:
            raise AssertionError("row %s[%s] has no state terms: %r" % (name, index, rhs))
        rows.setdefault(name, {})[int(index)] = terms
    return rows


def eval_row(terms, state: int) -> int:
    value = 0
    for bit in terms:
        value ^= (state >> bit) & 1
    return value


def rtl_word(rows, state: int):
    tx = rows["tx_bits"]
    out = 0
    for i in range(64):
        out |= eval_row(tx[i], state) << i
    return out


def rtl_next_state(rows, state: int) -> int:
    nxt = rows["state_next"]
    out = 0
    for i in range(DEG):
        out |= eval_row(nxt[i], state) << i
    return out


def serial_bits(seed: int, count: int):
    """Bit-serial x^31 + x^28 + 1, state[30] is the next bit out."""
    state = seed
    bits = []
    for _ in range(count):
        bits.append((state >> 30) & 1)
        feedback = ((state >> 30) ^ (state >> 27)) & 1
        state = ((state << 1) & MASK31) | feedback
    return bits


def bits_from_word_sequence(rows, seed: int, words: int):
    state = seed
    bits = []
    for _ in range(words):
        word = rtl_word(rows, state)
        for i in range(64):
            bits.append((word >> i) & 1)
        state = rtl_next_state(rows, state)
    return bits, state


class TestPrbsEquations(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows = parse_rows(PKG / "rtl" / "prbs31_64b.v")

    def test_row_count(self):
        self.assertEqual(len(self.rows["tx_bits"]), 64)
        self.assertEqual(len(self.rows["state_next"]), 31)

    def test_10000_bits_match_serial_model(self):
        words = 157                              # 10 048 bits
        golden = serial_bits(SEED, words * 64)
        produced, _ = bits_from_word_sequence(self.rows, SEED, words)
        self.assertEqual(len(produced), words * 64)
        for i, (a, b) in enumerate(zip(produced, golden)):
            if a != b:
                self.fail("bit %d differs: rtl=%d golden=%d" % (i, a, b))

    def test_second_seed(self):
        seed = 0x1234_5678 & MASK31
        golden = serial_bits(seed, 64 * 40)
        produced, _ = bits_from_word_sequence(self.rows, seed, 40)
        self.assertEqual(produced, golden)

    def test_next_state_equals_tail_of_stream(self):
        # state_next[30-k] must be output bit 64+k; this is the identity that
        # keeps the parallel generator and the checker on the same phase
        header = serial_bits(SEED, 64 + DEG)
        tail = header[64:]
        state_after = 0
        for k in range(DEG):
            state_after |= tail[k] << (DEG - 1 - k)
        self.assertEqual(rtl_next_state(self.rows, SEED), state_after)

    def test_stream_is_not_constant(self):
        produced, _ = bits_from_word_sequence(self.rows, SEED, 8)
        self.assertGreater(sum(produced), 100)
        self.assertLess(sum(produced), 512 - 100)


class TestCheckerBurst(unittest.TestCase):
    """Bit-level model of the self-synchronising checker in the RTL."""

    @staticmethod
    def reported_errors(bits):
        errs = []
        for n in range(DEG, len(bits)):
            predicted = bits[n - DEG] ^ bits[n - 28]
            errs.append(bits[n] ^ predicted)
        return errs

    def test_clean_stream_has_no_errors(self):
        self.assertEqual(sum(self.reported_errors(serial_bits(SEED, 2000))), 0)

    def test_single_flip_reports_three_errors(self):
        bits = serial_bits(SEED, 2000)
        position = 700
        bits[position] ^= 1
        errs = self.reported_errors(bits)
        hits = [DEG + i for i, e in enumerate(errs) if e]
        # the corrupted bit itself, then the two predictions that read it
        self.assertEqual(len(hits), 3)
        self.assertEqual(hits, [position, position + 28, position + 31])

    def test_burst_is_bounded(self):
        bits = serial_bits(SEED, 2000)
        for position in (400, 900, 1500):
            copy = list(bits)
            copy[position] ^= 1
            self.assertLessEqual(sum(self.reported_errors(copy)), 3)


class TestDdmMath(unittest.TestCase):
    def test_temp_is_signed(self):
        hot = bert_cli.decode_ddm(bytes.fromhex("1a4080e800c803e800fa"))
        self.assertAlmostEqual(hot["temperature_c"], 26.25, places=4)
        cold = bert_cli.decode_ddm(bytes.fromhex("f00080e800c803e800fa"))
        self.assertAlmostEqual(cold["temperature_c"], -16.0, places=4)

    def test_reference_block(self):
        d = bert_cli.decode_ddm(bytes.fromhex("1a4080e800c803e800fa"))
        self.assertAlmostEqual(d["vcc_v"], 3.300, places=5)
        self.assertAlmostEqual(d["tx_bias_ma"], 0.400, places=5)
        self.assertAlmostEqual(d["tx_power_dbm"], -10.0, places=3)
        self.assertAlmostEqual(d["rx_power_uW"], 25.0, places=5)
        self.assertAlmostEqual(d["rx_power_dbm"], 10 * math.log10(0.025), places=4)

    def test_zero_power_is_minus_infinity(self):
        self.assertEqual(bert_cli.uW_to_dbm(0.0), float("-inf"))

    def test_rejects_short_block(self):
        with self.assertRaises(ValueError):
            bert_cli.decode_ddm(b"\x00" * 9)


class TestBerBound(unittest.TestCase):
    def test_zero_errors_bound(self):
        bits = bert_cli.bits_for(100.0)
        summary = bert_cli.ber_summary(0, bits)
        self.assertAlmostEqual(summary["ber_upper"], -math.log(0.05) / bits, places=18)
        self.assertLess(summary["ber_upper"], 3.0e-12)
        self.assertGreater(summary["ber_upper"], 2.8e-12)

    def test_poisson_cdf_known_values(self):
        # P(X <= 2 | lambda = 2) = 0.676676...
        self.assertAlmostEqual(bert_cli.poisson_cdf(2, 2.0), 0.6766764, places=6)
        self.assertAlmostEqual(bert_cli.poisson_cdf(0, 0.0), 1.0, places=12)

    def test_upper_bound_exceeds_point_estimate(self):
        summary = bert_cli.ber_summary(5, 1e12)
        self.assertGreater(summary["ber_upper"], summary["ber"])

    def test_empty_run_is_rejected(self):
        with self.assertRaises(ValueError):
            bert_cli.ber_summary(0, 0)


class TestDfmClean(unittest.TestCase):
    def test_board_passes_prefab_lint(self):
        tool = SHOWCASE / "kicad-dfm-precheck" / "dfm_check.py"
        if not tool.is_file():
            self.skipTest("kicad-dfm-precheck not cloned next to this repo")
        cmd = [
            sys.executable,
            str(tool),
            str(PKG / "hardware" / "sfp_bert.kicad_pcb"),
            str(PKG / "hardware" / "sfp_bert.kicad_sch"),
        ]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(
            proc.returncode, 0,
            "dfm_check exited %d\n%s\n%s" % (proc.returncode, proc.stdout, proc.stderr),
        )
        self.assertIn("no findings", proc.stdout)
        self.assertIn("0 error(s)", proc.stdout)
        for code in ("DFM001", "DFM002", "DFM010", "DFM011"):
            self.assertNotIn(code, proc.stdout)


class TestRtlSimulation(unittest.TestCase):
    def _run_tb(self, top: str, sources):
        iverilog = shutil.which("iverilog")
        if not iverilog:
            self.skipTest("iverilog not installed")
        with tempfile.TemporaryDirectory() as tmp:
            exe = os.path.join(tmp, "sim.vvp")
            build = subprocess.run(
                [iverilog, "-g2005", "-o", exe] + [str(PKG / s) for s in sources],
                capture_output=True, text=True, cwd=tmp,
            )
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run(["vvp", exe], capture_output=True, text=True, cwd=tmp)
            self.assertIn("TEST PASS", run.stdout, run.stdout + run.stderr)

    def test_prbs_testbench(self):
        self._run_tb("tb_prbs31",
                     ["rtl/prbs31_64b.v", "rtl/ber_counter.v", "rtl/tb_prbs31.v"])

    def test_i2c_testbench(self):
        self._run_tb("tb_sff8472_i2c",
                     ["rtl/sff8472_ddm_i2c.v", "rtl/tb_sff8472_i2c.v"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
