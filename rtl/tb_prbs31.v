// -----------------------------------------------------------------------------
// tb_prbs31.v -- self-checking testbench for prbs31_64b + ber_counter.
//
// What it proves, in order:
//   1. every one of the 64 parallel generator rows produces the same stream as a
//      bit-serial x^31 + x^28 + 1 LFSR running inside this testbench, for
//      200 words = 12 800 bits (this is the independent model, not a copy of the
//      RTL equations);
//   2. the self-synchronising checker reports zero errors and asserts lock when
//      the loopback is clean;
//   3. a single injected bit produces between 1 and 3 reported errors, never
//      more, and the checker re-locks within a few words -- the documented
//      behaviour of a self-synchronising PRBS checker;
//   4. ber_counter accumulates exactly 64 bits per word.
//
// iverilog -g2005 -o tb.vvp rtl/prbs31_64b.v rtl/ber_counter.v rtl/tb_prbs31.v
// vvp tb.vvp
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

module tb_prbs31;

    localparam integer WARMUP_WORDS = 4;
    localparam integer CLEAN_WORDS  = 200;
    localparam integer INJECTIONS   = 8;
    localparam integer GAP_WORDS    = 32;
    localparam integer RECOVER_WORDS = 120;

    reg clk = 1'b0;
    always #3.103 clk = ~clk;              // 161.1328125 MHz

    reg         rst_n    = 1'b0;
    reg         enable   = 1'b0;
    reg         inject_en = 1'b0;
    reg  [5:0]  inject_pos = 6'd0;

    wire [63:0] tx_data;
    wire [63:0] rx_err_vec;
    wire [6:0]  rx_err_cnt;
    wire        rx_locked;

    prbs31_64b #(
        .WARMUP_WORDS (WARMUP_WORDS)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .enable     (enable),
        .inject_en  (inject_en),
        .inject_pos (inject_pos),
        .tx_data    (tx_data),
        .rx_data    (tx_data),
        .rx_valid   (enable),
        .rx_err_vec (rx_err_vec),
        .rx_err_cnt (rx_err_cnt),
        .rx_locked  (rx_locked)
    );

    wire [63:0] bit_total, err_total, bit_snap, err_snap;

    ber_counter u_cnt (
        .clk       (clk),
        .rst_n     (rst_n),
        .clear     (1'b0),
        .valid     (enable),
        .bit_inc   (7'd64),
        .err_inc   (rx_err_cnt),
        .snap      (1'b0),
        .bit_total (bit_total),
        .err_total (err_total),
        .bit_snap  (bit_snap),
        .err_snap  (err_snap)
    );

    // ---- independent serial golden model ---------------------------------
    function [30:0] adv64;
        input [30:0] st0;
        integer i;
        reg [30:0] t;
        begin
            t = st0;
            for (i = 0; i < 64; i = i + 1)
                t = {t[29:0], t[30] ^ t[27]};
            adv64 = t;
        end
    endfunction

    function [63:0] expand64;
        input [30:0] st0;
        integer i;
        reg [30:0] t;
        begin
            t = st0;
            for (i = 0; i < 64; i = i + 1) begin
                expand64[i] = t[30];
                t = {t[29:0], t[30] ^ t[27]};
            end
        end
    endfunction

    reg [30:0] g = 31'h7FFF_FFFF;
    reg [63:0] gword;

    // The injected bit is part of the transmitted stream, so the golden word is
    // compared with the same flip applied.  Sampling at the posedge mirrors what
    // the DUT sees on that edge.
    reg        inject_d = 1'b0;
    reg [5:0]  ipos_d   = 6'd0;
    always @(posedge clk) begin
        inject_d <= inject_en;
        ipos_d   <= inject_pos;
    end

    always @(posedge clk)
        if (rst_n && enable)
            g <= adv64(g);

    // ---- bookkeeping ------------------------------------------------------
    integer phase = 0;                 // 0 reset, 1 clean, 2 inject, 3 recover
    integer word_idx = 0;
    integer mismatches = 0;
    integer err_clean = 0;
    integer err_inject = 0;
    integer err_recover = 0;
    integer burst_max = 0;
    integer burst_min = 99;
    integer burst_this = 0;
    integer injections_done = 0;
    integer i;

    always @(negedge clk) begin
        if (rst_n && enable) begin
            gword = expand64(g) ^ (inject_d ? (64'd1 << ipos_d) : 64'd0);
            if (word_idx >= WARMUP_WORDS + 1) begin
                if (tx_data !== gword) begin
                    if (mismatches < 5)
                        $display("MISMATCH word %0d: rtl=%h golden=%h", word_idx, tx_data, gword);
                    mismatches = mismatches + 1;
                end
            end

            case (phase)
                1: err_clean   = err_clean   + rx_err_cnt;
                2: begin
                    err_inject  = err_inject  + rx_err_cnt;
                    burst_this  = burst_this  + rx_err_cnt;
                end
                3: err_recover = err_recover + rx_err_cnt;
            endcase

            word_idx = word_idx + 1;
        end
    end

    // ---- stimulus ---------------------------------------------------------
    initial begin
        $dumpfile("tb_prbs31.vcd");
        $dumpvars(0, tb_prbs31);

        repeat (4) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        @(negedge clk) enable = 1'b1;

        // phase 1: clean loopback
        phase = 1;
        repeat (CLEAN_WORDS) @(negedge clk);

        // phase 2: one flipped bit every GAP_WORDS words
        phase = 2;
        for (i = 0; i < INJECTIONS; i = i + 1) begin
            repeat (GAP_WORDS - 5) @(negedge clk);
            burst_this = 0;
            inject_pos = (i * 7) % 64;
            inject_en  = 1'b1;
            @(negedge clk);
            inject_en  = 1'b0;
            injections_done = injections_done + 1;
            repeat (5) @(negedge clk);          // let the 31-bit window flush
            if (burst_this < burst_min) burst_min = burst_this;
            if (burst_this > burst_max) burst_max = burst_this;
        end

        // phase 3: recovery
        phase = 3;
        repeat (RECOVER_WORDS) @(negedge clk);

        // ---- verdict ------------------------------------------------------
        $display("");
        $display("words compared      : %0d", word_idx);
        $display("stream mismatches   : %0d", mismatches);
        $display("errors, clean phase : %0d", err_clean);
        $display("errors, inject phase: %0d (from %0d injected bits)", err_inject, injections_done);
        $display("burst per injection : min %0d max %0d", burst_min, burst_max);
        $display("errors, recovery    : %0d", err_recover);
        $display("locked at end       : %0d", rx_locked);
        $display("bit_total           : %0d (expect %0d)", bit_total, (word_idx) * 64);

        if (mismatches == 0
            && err_clean == 0
            && burst_min >= 1 && burst_max <= 3
            && injections_done == INJECTIONS
            && err_recover == 0
            && rx_locked == 1'b1
            && bit_total == word_idx * 64)
            $display("TEST PASS");
        else
            $display("TEST FAIL");

        $finish;
    end

endmodule
