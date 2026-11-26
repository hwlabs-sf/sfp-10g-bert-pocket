// -----------------------------------------------------------------------------
// prbs31_64b.v -- 64-bit parallel PRBS-31 generator and self-synchronising
//                 checker for 10GBASE-R (IEEE 802.3ae clause 49).
//
// Polynomial   G(x) = x^31 + x^28 + 1   ->   s[n] = s[n-28] ^ s[n-31]
// Line rate    10.3125 Gbit/s = 64 bit x 161.1328125 MHz
//
// The generator is the bit-serial LFSR unrolled 64 clocks deep.  Every row below
// is the XOR of the state bits that feed it, derived from the recurrence; there
// is no loop and nothing is hidden in a function.  tests/test_bert_and_dfm.py
// re-derives all 95 rows from the polynomial and compares the resulting stream
// against a serial golden model over 10 000 bits, so the equations are checked,
// not trusted.
//
// The checker is the textbook self-synchronising form.  It predicts each
// received bit from the bits 28 and 31 positions earlier in the received stream
// itself, so it needs no frame boundary and no handshake: it locks after 31
// bits and re-locks 31 bits after a phase slip.  A single corrupted bit shows up
// as itself plus at most two follow-on predictions, which is the documented
// behaviour of this circuit and what the testbench asserts.
//
// Verilog-2001, vendor agnostic, no vendor primitives, no latches.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

`default_nettype none

module prbs31_64b #(
    parameter [30:0]  SEED         = 31'h7FFF_FFFF,
    parameter integer WARMUP_WORDS = 4,     // words dropped after reset
    parameter integer LOCK_WORDS   = 64     // clean words needed to assert lock
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,

    input  wire        inject_en,           // one strobe = one flipped bit
    input  wire [5:0]  inject_pos,

    output wire [63:0] tx_data,

    input  wire [63:0] rx_data,             // loop tx_data back, or the SERDES word
    input  wire        rx_valid,

    output wire [63:0] rx_err_vec,
    output wire [6:0]  rx_err_cnt,
    output reg         rx_locked
);

    reg  [30:0] state;
    wire [30:0] state_next;
    wire [63:0] tx_bits;

    // ---- transmit: 64 bits of the sequence, newest last ------------------
    assign tx_bits[ 0] = state[30];
    assign tx_bits[ 1] = state[29];
    assign tx_bits[ 2] = state[28];
    assign tx_bits[ 3] = state[27];
    assign tx_bits[ 4] = state[26];
    assign tx_bits[ 5] = state[25];
    assign tx_bits[ 6] = state[24];
    assign tx_bits[ 7] = state[23];
    assign tx_bits[ 8] = state[22];
    assign tx_bits[ 9] = state[21];
    assign tx_bits[10] = state[20];
    assign tx_bits[11] = state[19];
    assign tx_bits[12] = state[18];
    assign tx_bits[13] = state[17];
    assign tx_bits[14] = state[16];
    assign tx_bits[15] = state[15];
    assign tx_bits[16] = state[14];
    assign tx_bits[17] = state[13];
    assign tx_bits[18] = state[12];
    assign tx_bits[19] = state[11];
    assign tx_bits[20] = state[10];
    assign tx_bits[21] = state[9];
    assign tx_bits[22] = state[8];
    assign tx_bits[23] = state[7];
    assign tx_bits[24] = state[6];
    assign tx_bits[25] = state[5];
    assign tx_bits[26] = state[4];
    assign tx_bits[27] = state[3];
    assign tx_bits[28] = state[2];
    assign tx_bits[29] = state[1];
    assign tx_bits[30] = state[0];
    assign tx_bits[31] = state[27] ^ state[30];
    assign tx_bits[32] = state[26] ^ state[29];
    assign tx_bits[33] = state[25] ^ state[28];
    assign tx_bits[34] = state[24] ^ state[27];
    assign tx_bits[35] = state[23] ^ state[26];
    assign tx_bits[36] = state[22] ^ state[25];
    assign tx_bits[37] = state[21] ^ state[24];
    assign tx_bits[38] = state[20] ^ state[23];
    assign tx_bits[39] = state[19] ^ state[22];
    assign tx_bits[40] = state[18] ^ state[21];
    assign tx_bits[41] = state[17] ^ state[20];
    assign tx_bits[42] = state[16] ^ state[19];
    assign tx_bits[43] = state[15] ^ state[18];
    assign tx_bits[44] = state[14] ^ state[17];
    assign tx_bits[45] = state[13] ^ state[16];
    assign tx_bits[46] = state[12] ^ state[15];
    assign tx_bits[47] = state[11] ^ state[14];
    assign tx_bits[48] = state[10] ^ state[13];
    assign tx_bits[49] = state[9] ^ state[12];
    assign tx_bits[50] = state[8] ^ state[11];
    assign tx_bits[51] = state[7] ^ state[10];
    assign tx_bits[52] = state[6] ^ state[9];
    assign tx_bits[53] = state[5] ^ state[8];
    assign tx_bits[54] = state[4] ^ state[7];
    assign tx_bits[55] = state[3] ^ state[6];
    assign tx_bits[56] = state[2] ^ state[5];
    assign tx_bits[57] = state[1] ^ state[4];
    assign tx_bits[58] = state[0] ^ state[3];
    assign tx_bits[59] = state[2] ^ state[27] ^ state[30];
    assign tx_bits[60] = state[1] ^ state[26] ^ state[29];
    assign tx_bits[61] = state[0] ^ state[25] ^ state[28];
    assign tx_bits[62] = state[24] ^ state[30];
    assign tx_bits[63] = state[23] ^ state[29];
    // ---- state after 64 clocks (M^64 applied to the same state) ----------
    assign state_next[ 0] = state[20] ^ state[23] ^ state[26] ^ state[29];
    assign state_next[ 1] = state[21] ^ state[24] ^ state[27] ^ state[30];
    assign state_next[ 2] = state[0] ^ state[22] ^ state[25];
    assign state_next[ 3] = state[1] ^ state[23] ^ state[26];
    assign state_next[ 4] = state[2] ^ state[24] ^ state[27];
    assign state_next[ 5] = state[3] ^ state[25] ^ state[28];
    assign state_next[ 6] = state[4] ^ state[26] ^ state[29];
    assign state_next[ 7] = state[5] ^ state[27] ^ state[30];
    assign state_next[ 8] = state[0] ^ state[6];
    assign state_next[ 9] = state[1] ^ state[7];
    assign state_next[10] = state[2] ^ state[8];
    assign state_next[11] = state[3] ^ state[9];
    assign state_next[12] = state[4] ^ state[10];
    assign state_next[13] = state[5] ^ state[11];
    assign state_next[14] = state[6] ^ state[12];
    assign state_next[15] = state[7] ^ state[13];
    assign state_next[16] = state[8] ^ state[14];
    assign state_next[17] = state[9] ^ state[15];
    assign state_next[18] = state[10] ^ state[16];
    assign state_next[19] = state[11] ^ state[17];
    assign state_next[20] = state[12] ^ state[18];
    assign state_next[21] = state[13] ^ state[19];
    assign state_next[22] = state[14] ^ state[20];
    assign state_next[23] = state[15] ^ state[21];
    assign state_next[24] = state[16] ^ state[22];
    assign state_next[25] = state[17] ^ state[23];
    assign state_next[26] = state[18] ^ state[24];
    assign state_next[27] = state[19] ^ state[25];
    assign state_next[28] = state[20] ^ state[26];
    assign state_next[29] = state[21] ^ state[27];
    assign state_next[30] = state[22] ^ state[28];
    // ---- transmit output, with a one-bit error-injection door -------------
    wire [63:0] inject_mask = inject_en ? (64'd1 << inject_pos) : 64'd0;
    assign tx_data = tx_bits ^ inject_mask;

    // ---- receive: self-synchronising prediction --------------------------
    // rx_hist[0] is the oldest bit still held, rx_hist[30] the most recent.
    reg  [30:0] rx_hist;
    wire [94:0] rx_window = {rx_data, rx_hist};

    wire [63:0] rx_err_raw;
    genvar j;
    generate
        for (j = 0; j < 64; j = j + 1) begin : g_err
            assign rx_err_raw[j] = rx_data[j] ^ rx_window[j] ^ rx_window[j + 3];
        end
    endgenerate

    // ---- warm-up and lock -------------------------------------------------
    reg [7:0] warm_cnt;
    reg [7:0] lock_cnt;

    wire warm_done = (warm_cnt >= WARMUP_WORDS[7:0]);

    function [6:0] popcount64;
        input [63:0] v;
        integer i;
        begin
            popcount64 = 7'd0;
            for (i = 0; i < 64; i = i + 1)
                popcount64 = popcount64 + v[i];
        end
    endfunction

    assign rx_err_vec = warm_done ? rx_err_raw : 64'd0;
    assign rx_err_cnt = popcount64(rx_err_vec);

    always @(posedge clk) begin
        if (!rst_n) begin
            state     <= SEED;
            rx_hist   <= 31'd0;
            warm_cnt  <= 8'd0;
            lock_cnt  <= 8'd0;
            rx_locked <= 1'b0;
        end else if (enable) begin
            state <= state_next;

            if (rx_valid) begin
                rx_hist <= rx_data[63:33];
                if (!warm_done)
                    warm_cnt <= warm_cnt + 8'd1;
            end

            if (rx_valid && warm_done) begin
                if (rx_err_raw == 64'd0) begin
                    if (lock_cnt == LOCK_WORDS[7:0])
                        rx_locked <= 1'b1;
                    else
                        lock_cnt <= lock_cnt + 8'd1;
                end else begin
                    lock_cnt  <= 8'd0;
                    rx_locked <= 1'b0;
                end
            end
        end
    end

endmodule

`default_nettype wire
