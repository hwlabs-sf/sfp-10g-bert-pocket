// -----------------------------------------------------------------------------
// sff8472_ddm_i2c.v -- I2C master that polls the SFF-8472 (SFP+ MSA) digital
//                      diagnostic monitor and hands the host five raw words.
//
// Target device: 7-bit address 0x51, i.e. the A2h page of an SFP+ module.
// One sequential read of byte 0x60 .. 0x69 returns:
//
//   0x60/61  temperature        signed 16-bit, 1/256 degC
//   0x62/63  Vcc                unsigned 16-bit, 100 uV
//   0x64/65  TX bias            unsigned 16-bit, 2 uA
//   0x66/67  TX optical power   unsigned 16-bit, 0.1 uW
//   0x68/69  RX optical power   unsigned 16-bit, 0.1 uW
//
// This block deliberately does not convert units: it moves ten bytes and
// presents the raw words, because the scaling is the part that gets audited.
// host/bert_cli.py owns the arithmetic and its tests pin the numbers.
//
// Bus behaviour: 400 kHz fast mode, quarter-bit state machine, start, write of
// the register pointer, repeated start, read with ACK on every byte but the
// last, stop.  All outputs are open drain; the pull-ups sit on the carrier.
//
// Verilog-2001, no vendor primitives.
// -----------------------------------------------------------------------------

`timescale 1ns/1ps
`default_nettype none

module sff8472_ddm_i2c #(
    parameter integer CLK_HZ   = 100_000_000,
    parameter integer I2C_HZ   = 400_000,
    parameter integer POLL_MS  = 100,
    parameter [6:0]   DEV_ADDR = 7'h51,      // 0xA2 >> 1
    parameter [7:0]   REG_PTR  = 8'h60,
    parameter integer N_BYTES  = 10
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,        // free-run, re-poll every POLL_MS
    input  wire        start,         // manual trigger, ignored while busy

    output reg         scl,
    input  wire        sda_i,
    output reg         sda_o,
    output reg         sda_oe,        // 1 = drive low, 0 = release to pull-up

    output reg  [15:0] ddm_temp,
    output reg  [15:0] ddm_vcc,
    output reg  [15:0] ddm_bias,
    output reg  [15:0] ddm_tx_pwr,
    output reg  [15:0] ddm_rx_pwr,

    output reg         ddm_valid,     // one-cycle pulse, block is fresh
    output reg         busy,
    output reg         nak_error      // a device failed to acknowledge
);

    localparam integer QCLK      = (CLK_HZ / (I2C_HZ * 4) > 0) ? (CLK_HZ / (I2C_HZ * 4)) : 1;
    localparam integer POLL_CLKS = (CLK_HZ / 1000) * POLL_MS;

    localparam [2:0] S_IDLE   = 3'd0,
                     S_START  = 3'd1,
                     S_ADDR_W = 3'd2,
                     S_REG    = 3'd3,
                     S_RSTART = 3'd4,
                     S_ADDR_R = 3'd5,
                     S_READ   = 3'd6,
                     S_STOP   = 3'd7;

    reg [2:0]  state;
    reg [1:0]  qph;
    reg [15:0] div_cnt;
    reg [3:0]  bit_i;          // 0..7 data bits, 8 = acknowledge phase
    reg [3:0]  byte_i;
    reg [7:0]  tx_shift;
    reg [7:0]  rx_shift;
    reg [79:0] rx_block;
    reg [31:0] poll_cnt;

    wire tick     = (div_cnt == (QCLK - 1));
    wire step     = tick && (qph == 2'd3);      // one bit finished
    wire scl_high = (qph == 2'd1) || (qph == 2'd2);

    // ---- bit timing -------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            div_cnt <= 16'd0;
            qph     <= 2'd0;
        end else if (tick) begin
            div_cnt <= 16'd0;
            qph     <= qph + 2'd1;
        end else begin
            div_cnt <= div_cnt + 16'd1;
        end
    end

    // ---- what the master drives ------------------------------------------
    wire send_phase = (state == S_ADDR_W) || (state == S_REG) || (state == S_ADDR_R);
    wire ack_phase   = (bit_i == 4'd8);
    wire last_byte  = (byte_i == N_BYTES - 1);

    // data bits come from tx_shift[7]; the acknowledge phase is always released
    wire drive_low = (send_phase && !ack_phase) ? ~tx_shift[7] : 1'b0;

    always @(*) begin
        scl    = scl_high;
        sda_o  = 1'b0;
        sda_oe = drive_low;

        case (state)
            S_IDLE: begin
                scl    = 1'b1;
                sda_oe = 1'b0;
            end
            S_START: begin
                // SCL already high; drop SDA on the second quarter = start
                scl    = (qph <= 2'd1);
                sda_oe = (qph >= 2'd1);
            end
            S_RSTART: begin
                // SDA must be high before the edge, then fall while SCL is high
                scl    = (qph <= 2'd1);
                sda_oe = (qph >= 2'd1);
            end
            S_STOP: begin
                // SDA low, SCL high, then release SDA while SCL is high
                scl    = (qph >= 2'd1) && (qph <= 2'd2);
                sda_oe = (qph <= 2'd1);
            end
            S_READ: begin
                // master acknowledges every byte except the last
                sda_oe = ack_phase && !last_byte;
            end
            default: ;
        endcase
    end

    // ---- sequencer --------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            bit_i      <= 4'd0;
            byte_i     <= 4'd0;
            tx_shift   <= 8'hFF;
            rx_shift   <= 8'h00;
            rx_block   <= 80'd0;
            poll_cnt   <= 32'd0;
            busy       <= 1'b0;
            ddm_valid  <= 1'b0;
            nak_error  <= 1'b0;
            ddm_temp   <= 16'd0;
            ddm_vcc    <= 16'd0;
            ddm_bias   <= 16'd0;
            ddm_tx_pwr <= 16'd0;
            ddm_rx_pwr <= 16'd0;
        end else begin
            ddm_valid <= 1'b0;

            // idle: wait for a trigger, or for the poll timer to expire
            if (state == S_IDLE) begin
                busy <= 1'b0;
                if (poll_cnt != 32'd0)
                    poll_cnt <= poll_cnt - 32'd1;

                if (start || (enable && poll_cnt == 32'd0)) begin
                    state     <= S_START;
                    busy      <= 1'b1;
                    bit_i     <= 4'd0;
                    byte_i    <= 4'd0;
                    tx_shift  <= 8'hFF;
                    nak_error <= 1'b0;
                    poll_cnt  <= POLL_CLKS;
                end
            end

            // sample in the middle of the high phase
            if (tick && qph == 2'd2) begin
                case (state)
                    S_ADDR_W, S_REG, S_ADDR_R:
                        if (ack_phase && sda_i)
                            nak_error <= 1'b1;
                    S_READ:
                        if (!ack_phase)
                            rx_shift <= {rx_shift[6:0], sda_i};
                    default: ;
                endcase
            end

            if (step) begin
                case (state)
                    S_START: begin
                        state    <= S_ADDR_W;
                        bit_i    <= 4'd0;
                        tx_shift <= {DEV_ADDR, 1'b0};
                    end
                    S_ADDR_W: begin
                        if (ack_phase) begin
                            bit_i    <= 4'd0;
                            state    <= S_REG;
                            tx_shift <= REG_PTR;
                        end else begin
                            bit_i    <= bit_i + 4'd1;
                            tx_shift <= {tx_shift[6:0], 1'b1};
                        end
                    end
                    S_REG: begin
                        if (ack_phase) begin
                            bit_i    <= 4'd0;
                            state    <= S_RSTART;
                            tx_shift <= 8'hFF;
                        end else begin
                            bit_i    <= bit_i + 4'd1;
                            tx_shift <= {tx_shift[6:0], 1'b1};
                        end
                    end
                    S_RSTART: begin
                        state    <= S_ADDR_R;
                        bit_i    <= 4'd0;
                        tx_shift <= {DEV_ADDR, 1'b1};
                    end
                    S_ADDR_R: begin
                        if (ack_phase) begin
                            bit_i  <= 4'd0;
                            state  <= S_READ;
                        end else begin
                            bit_i    <= bit_i + 4'd1;
                            tx_shift <= {tx_shift[6:0], 1'b1};
                        end
                    end
                    S_READ: begin
                        if (ack_phase) begin
                            bit_i    <= 4'd0;
                            rx_block <= {rx_block[71:0], rx_shift};
                            if (last_byte)
                                state <= S_STOP;
                            else
                                byte_i <= byte_i + 4'd1;
                        end else begin
                            bit_i <= bit_i + 4'd1;
                        end
                    end
                    S_STOP: begin
                        state     <= S_IDLE;
                        busy      <= 1'b0;
                        ddm_valid <= 1'b1;
                        // ten bytes shift in MSB-first: rx_block[79:72] is
                        // the first byte read (0x60), rx_block[7:0] the last
                        ddm_temp   <= rx_block[79:64];
                        ddm_vcc    <= rx_block[63:48];
                        ddm_bias   <= rx_block[47:32];
                        ddm_tx_pwr <= rx_block[31:16];
                        ddm_rx_pwr <= rx_block[15:0];
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule

`default_nettype wire
