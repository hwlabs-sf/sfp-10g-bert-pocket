# sfp-10g-bert-pocket

A 10.3125 Gbit/s SFP+ bit-error-rate tester and optical power monitor, small enough
to be a pocket board: a 64-bit parallel PRBS-31 generator and checker in Verilog,
an SFF-8472 I2C poller, a four-layer SFP+ carrier, and a host CLI that turns raw
diagnostic bytes into degrees, volts, milliamps and dBm.

It is a reference design, not a product. Everything in it is either a published
standard or arithmetic you can re-derive: IEEE 802.3ae clause 49 for the pattern
and the line rate, SFF-8472 rev 12.4 for the diagnostic registers. No vendor IP,
no encrypted blocks, no "contact us for the core".

---

## What the numbers are

| Quantity | Value | Where it comes from |
| :--- | :--- | :--- |
| Line rate | 10.3125 Gbit/s | 10GBASE-R, 64b/66b coded |
| Clock | 161.1328125 MHz | 10.3125 G / 64 |
| Pattern | PRBS-31, G(x) = x^31 + x^28 + 1 | IEEE 802.3ae 49.2.8 |
| Checker | self-synchronising, 31-bit window | predicts s[n] from s[n-28] ^ s[n-31] |
| DDM | SFF-8472 A2h, bytes 0x60..0x69 | 1/256 °C, 100 µV, 2 µA, 0.1 µW |
| BER, 100 s, zero errors | < 2.91e-12 (95 %) | −ln(0.05) / 1.031e12 bits |

At 10.3125 Gbit/s a 100-second run pushes 1.031e12 bits through the link. If the
counter sees nothing, the honest statement is not "BER = 0" but "BER is below
2.91e-12 at 95 % confidence", which is what `host/bert_cli.py ber` prints.

## Layout

```
rtl/prbs31_64b.v        64-bit parallel generator + self-sync checker
rtl/ber_counter.v       saturating bit/error counters, latch-on-read snapshot
rtl/sff8472_ddm_i2c.v   I2C master polling A2h 0x60..0x69
rtl/tb_prbs31.v         self-checking testbench, serial golden model inside
rtl/tb_sff8472_i2c.v    bus-level testbench with an A2h slave model
hardware/sfp_bert.kicad_sch / .kicad_pcb / BOM.csv
host/bert_cli.py        DDM decode, BER + Poisson bound, offline demo
tests/test_bert_and_dfm.py
architecture.svg
```

## The generator is written out, not looped

A bit-serial PRBS-31 is nine lines of Verilog. This one is unrolled 64 clocks
deep, because the parallel form is where the mistakes live: a single wrong tap in
one of the 64 rows still produces a plausible-looking stream, and you only find
out when a link fails BER at 1e-9 for no visible reason.

So each row is written as the XOR of the state bits that feed it:

```verilog
assign tx_bits[ 0] = state[30];
assign tx_bits[ 1] = state[29];
...
assign tx_bits[31] = state[27] ^ state[30];
```

and `tests/test_bert_and_dfm.py` parses all 95 rows straight out of the Verilog
source, evaluates them, and compares 10 048 bits against a bit-serial LFSR written
independently in Python, on two different seeds. If someone edits a row, the test
fails on the first bit that disagrees.

The checker is the textbook self-synchronising form: it predicts each received bit
from the two bits 28 and 31 positions earlier *in the received stream*. That means
no frame boundary and no handshake -- it locks after 31 bits and re-locks 31 bits
after a phase slip. It also means one corrupted bit is reported as three errors
(itself plus the two predictions that read it), which is a property of the
circuit, not a defect, and the testbench asserts exactly that.

## Hardware

A 60 × 40 mm four-layer carrier:

- **F.Cu / B.Cu** signal, **In1.Cu** solid ground, **In2.Cu** 3V3.
- TD and RD pairs are 0.15 mm wide with a 0.15 mm gap, referenced to In1 through
  100 µm of prepreg, which lands close to 100 Ω differential without a field
  solver. The stack-up callout is in `architecture.svg`.
- 100 nF 0201 AC coupling on all four lines, ground void under the pads.
- TPS7A9401 (4.4 µVrms) for the module rail, not a generic buck: SFP+ receivers
  care about rail noise more than they care about efficiency.
- CP2102N bridges I2C to USB so a laptop can poll the DDM without an FPGA.
- 161.1328125 MHz LVDS oscillator for boards that need to supply the reference
  clock instead of receiving it.

`hardware/BOM.csv` carries the primary part and an alternate for every line, with
the LCSC column filled where the code was verified and `tbd` where purchasing has
to confirm it at order time -- a BOM that pretends to know every distributor code
is a BOM that has never been ordered.

The board passes `kicad-dfm-precheck` with zero findings: 13 vias, 24 through-hole
pads, no annular ring under 0.15 mm, no placeholder values, no pad left on an
`unconnected-*` net.

## Running it

```bash
# everything that does not need a simulator
python -m unittest discover -s tests -v

# both testbenches, if iverilog is installed
iverilog -g2005 -o tb.vvp rtl/prbs31_64b.v rtl/ber_counter.v rtl/tb_prbs31.v && vvp tb.vvp
iverilog -g2005 -o i2c.vvp rtl/sff8472_ddm_i2c.v rtl/tb_sff8472_i2c.v && vvp i2c.vvp

# the host side
python host/bert_cli.py demo
python host/bert_cli.py ddm --hex 1a4080e800c803e800fa
python host/bert_cli.py ber --errors 0 --seconds 100
```

The DFM test looks for `kicad-dfm-precheck` next to this repository and skips
with a message if it is not there, rather than passing quietly.

## What is not here

No vendor-specific primitives, no SERDES transceiver wrapper (that is the FPGA
vendor's IP and it is not portable), no firmware for a specific USB bridge, and no
claim that this has been run against a live module -- the I2C master is verified
against a bus-level model, not against silicon. Those gaps are stated because a
reference design that hides its edges is worse than no reference design.

MIT licensed. Built and tested on a bench in Shenzhen.

[hengwu.online/labs](https://hengwu.online/labs/) · [hello@hengwu.online](mailto:hello@hengwu.online)
