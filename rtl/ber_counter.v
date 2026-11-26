// -----------------------------------------------------------------------------
// ber_counter.v -- 64-bit bit/error accumulator with saturating adders and a
//                  latch-on-read snapshot, sized for a 10G BERT run.
//
// At 10.3125 Gbit/s a 64-bit bit counter wraps every 2^64 / 1.03125e10 s, i.e.
// 1.78e9 s (~56 years), and a 64-bit error counter is far beyond any real link,
// so both saturate instead of wrapping: a counter that rolls over silently turns
// a dead link into a clean-looking report, which is the one failure mode a BERT
// must never have.
//
// A host read cannot be atomic with a free-running counter, so "read" is a
// strobe: snap latches both totals into bit_snap / err_snap and the host reads
// those two registers.  bit_total and err_total keep counting underneath, so the
// next snapshot is the interval, not a reset.
//
// Verilog-2001, no vendor primitives.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

`default_nettype none

module ber_counter (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,        // synchronous, returns both totals to 0
    input  wire        valid,        // one accumulated word this cycle
    input  wire [6:0]  bit_inc,      // bits in this word (0..64)
    input  wire [6:0]  err_inc,      // errors in this word (0..64)
    input  wire        snap,         // latch totals into the snapshot registers

    output reg  [63:0] bit_total,
    output reg  [63:0] err_total,
    output reg  [63:0] bit_snap,
    output reg  [63:0] err_snap
);

    wire [63:0] bit_sum = bit_total + {57'd0, bit_inc};
    wire [63:0] err_sum = err_total + {57'd0, err_inc};

    always @(posedge clk) begin
        if (!rst_n) begin
            bit_total <= 64'd0;
            err_total <= 64'd0;
            bit_snap  <= 64'd0;
            err_snap  <= 64'd0;
        end else if (clear) begin
            bit_total <= 64'd0;
            err_total <= 64'd0;
        end else if (valid) begin
            // saturate, never wrap
            bit_total <= (&bit_total) ? bit_total : bit_sum;
            err_total <= (&err_total) ? err_total : err_sum;
        end

        if (rst_n && snap) begin
            bit_snap <= bit_total;
            err_snap <= err_total;
        end
    end

endmodule

`default_nettype wire
