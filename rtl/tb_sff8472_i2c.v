// -----------------------------------------------------------------------------
// tb_sff8472_i2c.v -- bus-level testbench for sff8472_ddm_i2c.
//
// The slave is a small behavioural model that speaks only the transaction this
// master is allowed to make:
//
//   START  A2h(W)  ACK  0x60  ACK  START  A2h(R)  ACK  b0..b9  STOP
//
// It synchronises on the start condition (SDA falling while SCL is high) rather
// than on a free-running edge count, then samples on the falling edge of SCL --
// the instant when the incoming bit has finished being valid and the next one
// has not been armed yet.  If the master emits the wrong address, the wrong
// pointer or the wrong number of clocks, the byte check below fails.
//
//   iverilog -g2005 -o i2c.vvp rtl/sff8472_ddm_i2c.v rtl/tb_sff8472_i2c.v
//   vvp i2c.vvp
// -----------------------------------------------------------------------------

`timescale 1ns/1ps

module tb_sff8472_i2c;

    reg clk = 1'b0;
    always #5 clk = ~clk;                 // 100 MHz

    reg  rst_n  = 1'b0;
    reg  enable = 1'b0;
    reg  start  = 1'b0;

    wire scl;
    wire sda;
    wire dut_sda_o, dut_sda_oe;

    wire [15:0] ddm_temp, ddm_vcc, ddm_bias, ddm_tx_pwr, ddm_rx_pwr;
    wire        ddm_valid, busy, nak_error;

    pullup (sda);
    pullup (scl);

    sff8472_ddm_i2c #(
        .CLK_HZ  (100_000_000),
        .I2C_HZ  (400_000),
        .POLL_MS (100)
    ) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (enable),
        .start     (start),
        .scl       (scl),
        .sda_i     (sda),
        .sda_o     (dut_sda_o),
        .sda_oe    (dut_sda_oe),
        .ddm_temp  (ddm_temp),
        .ddm_vcc   (ddm_vcc),
        .ddm_bias  (ddm_bias),
        .ddm_tx_pwr(ddm_tx_pwr),
        .ddm_rx_pwr(ddm_rx_pwr),
        .ddm_valid (ddm_valid),
        .busy      (busy),
        .nak_error (nak_error)
    );

    assign sda = dut_sda_oe ? 1'b0 : 1'bz;

    // ---- slave model ------------------------------------------------------
    // 0x60/61 temp = 0x1A40 (26.25 C)
    // 0x62/63 Vcc  = 0x80E8 (3.300 V)
    // 0x64/65 bias = 0x00C8 (400 uA)
    // 0x66/67 TX   = 0x03E8 (-10.0 dBm)
    // 0x68/69 RX   = 0x00FA (-20.0 dBm)
    localparam [79:0] PAGE = 80'h1A40_80E8_00C8_03E8_00FA;

    reg  [79:0] page = PAGE;
    reg  [7:0]  sh   = 8'h00;
    reg  [7:0]  seen [0:2];
    reg  [7:0]  expect [0:2];

    reg         slave_drive = 1'b0;
    reg         slave_bit   = 1'b0;
    assign sda = slave_drive ? slave_bit : 1'bz;

    integer txn     = 0;      // 1 = address + pointer, 2 = read phase
    integer fall    = 0;      // falling edges since the last start condition
    integer byte_ix = 0;
    integer byte_errors = 0;
    integer nb, bi, k, j;
    integer skip_start = 0;   // the start state itself makes one SCL falling edge

    initial begin
        expect[0] = 8'hA2;
        expect[1] = 8'h60;
        expect[2] = 8'hA3;
    end

    // start condition: SDA falls while SCL is high
    always @(negedge sda) begin
        if (scl === 1'b1) begin
            txn     = txn + 1;
            fall    = 0;
            byte_ix = 0;
            sh      = 8'h00;
            slave_drive = 1'b0;
            skip_start  = 1;
        end
    end

    // arm the SDA level for bit n of the current transaction
    task arm_bit;
        input integer n;
        integer kk, jj;
        begin
            if (n < 0) begin
                slave_drive = 1'b0;
                slave_bit   = 1'b0;
            end else begin
                kk = n / 9;
                jj = n % 9;
                if (jj == 8) begin
                    // acknowledge phase
                    slave_drive = (txn == 1) || (txn == 2 && kk == 0);
                    slave_bit   = 1'b0;
                end else if (txn == 2 && kk >= 1) begin
                    slave_drive = 1'b1;
                    slave_bit   = page[79 - (8 * (kk - 1) + jj)];
                end else begin
                    slave_drive = 1'b0;
                    slave_bit   = 1'b0;
                end
            end
        end
    endtask

    always @(negedge scl) begin
        if (skip_start) begin
            // the start state's own falling edge carries no data bit
            skip_start = 0;
            fall       = 0;
            arm_bit(0);
        end else begin
            fall = fall + 1;
            bi   = fall - 1;              // bit that just finished

            if (bi >= 0 && (bi % 9) < 8)
                sh = {sh[6:0], sda};

            if (bi >= 0 && (bi % 9) == 7) begin
                if (txn == 1 && byte_ix < 2) begin
                    if (sh !== expect[byte_ix]) begin
                        $display("BYTE %0d MISMATCH: master sent %h, expected %h",
                                 byte_ix, sh, expect[byte_ix]);
                        byte_errors = byte_errors + 1;
                    end
                    seen[byte_ix] = sh;
                end
                if (txn == 2 && byte_ix == 0) begin
                    if (sh !== expect[2]) begin
                        $display("ADDR READ MISMATCH: master sent %h, expected %h", sh, expect[2]);
                        byte_errors = byte_errors + 1;
                    end
                    seen[2] = sh;
                end
                byte_ix = byte_ix + 1;
                sh = 8'h00;
            end

            arm_bit(bi + 1);
        end
    end

    // ---- stimulus ---------------------------------------------------------
    integer wait_clk;
    initial begin
        $dumpfile("tb_sff8472_i2c.vcd");
        $dumpvars(0, tb_sff8472_i2c);

        repeat (10) @(posedge clk);
        @(negedge clk) rst_n = 1'b1;
        repeat (20) @(negedge clk);

        @(negedge clk) start = 1'b1;
        @(negedge clk) start = 1'b0;

        wait_clk = 0;
        while (!ddm_valid && wait_clk < 400000) begin
            @(posedge clk);
            wait_clk = wait_clk + 1;
        end

        $display("");
        $display("start conditions: %0d (want 2)", txn);
        $display("clocks to valid : %0d", wait_clk);
        $display("bytes sent      : %h %h %h (want A2 60 A3)", seen[0], seen[1], seen[2]);
        $display("temp   = %h (want 1A40)", ddm_temp);
        $display("vcc    = %h (want 80E8)", ddm_vcc);
        $display("bias   = %h (want 00C8)", ddm_bias);
        $display("tx_pwr = %h (want 03E8)", ddm_tx_pwr);
        $display("rx_pwr = %h (want 00FA)", ddm_rx_pwr);
        $display("nak_error = %0d", nak_error);

        if (ddm_valid && byte_errors == 0 && txn == 2
            && ddm_temp == 16'h1A40 && ddm_vcc == 16'h80E8
            && ddm_bias == 16'h00C8 && ddm_tx_pwr == 16'h03E8
            && ddm_rx_pwr == 16'h00FA && nak_error == 1'b0)
            $display("TEST PASS");
        else
            $display("TEST FAIL");

        $finish;
    end

    initial begin
        #3000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
