// =============================================================================
// Module: adar1000_spi.v
// Description: SPI master controller for Analog Devices ADAR1000 beamformer IC.
//
// ADAR1000 SPI characteristics:
//   - 24-bit transactions: [15:8]=register address, [7:0]=data (write)
//     or 3-byte read: address byte + 2 dummy + 1 data byte
//   - CPOL=0, CPHA=0 (Mode 0) or CPOL=1, CPHA=1 (Mode 3) - configurable
//   - MSB first
//   - Maximum SPI clock: 25 MHz
//   - Active-low chip select
//
// Transaction format (24 bits):
//   Bit [23]    : R/W# (1=read, 0=write)
//   Bits [22:16]: Register address (7 bits)
//   Bits [15:8] : Don't-care for write, register extension for burst
//   Bits [7:0]  : Data byte
// =============================================================================

`timescale 1ns / 1ps

module adar1000_spi #(
    parameter CLK_DIV   = 5,   // SPI clock = sys_clk / (2 * CLK_DIV), default 100MHz/10=10MHz
    parameter CPOL      = 0,   // Clock polarity
    parameter CPHA      = 0    // Clock phase
)(
    input  wire        clk,
    input  wire        rst_n,

    // User interface
    input  wire        tx_start,     // Pulse to start a transaction
    input  wire        rw,           // 1=read, 0=write
    input  wire [6:0]  addr,         // 7-bit register address
    input  wire [7:0]  wdata,        // Write data byte
    output reg  [7:0]  rdata,        // Read data byte
    output reg         tx_done,      // Transaction complete pulse
    output reg         busy,

    // SPI pins
    output reg         spi_csn,      // Chip select (active low)
    output reg         spi_clk,      // SPI clock
    output reg         spi_mosi,     // Master out slave in
    input  wire        spi_miso      // Master in slave out
);

    // -------------------------------------------------------------------------
    // SPI clock generation counter
    // -------------------------------------------------------------------------
    reg [$clog2(CLK_DIV)-1:0] clk_cnt;
    reg                        spi_clk_en;
    reg                        spi_clk_r;  // Rising edge strobe
    reg                        spi_clk_f;  // Falling edge strobe

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt    <= {$clog2(CLK_DIV){1'b0}};
            spi_clk_r  <= 1'b0;
            spi_clk_f  <= 1'b0;
        end else begin
            spi_clk_r <= 1'b0;
            spi_clk_f <= 1'b0;
            if (spi_clk_en) begin
                if (clk_cnt == CLK_DIV - 1) begin
                    clk_cnt   <= {$clog2(CLK_DIV){1'b0}};
                    spi_clk   <= ~spi_clk;
                    if (spi_clk == CPOL) spi_clk_r <= 1'b1;  // rising
                    else                 spi_clk_f <= 1'b1;  // falling
                end else begin
                    clk_cnt <= clk_cnt + 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    localparam ST_IDLE     = 3'd0;
    localparam ST_ASSERT   = 3'd1;   // Assert CSN
    localparam ST_SHIFT    = 3'd2;   // Shift 24 bits
    localparam ST_DEASSERT = 3'd3;   // Deassert CSN (hold)
    localparam ST_DONE     = 3'd4;

    reg [2:0]  state;
    reg [23:0] shift_reg;   // Shift register (TX)
    reg [23:0] rx_reg;      // Receive register
    reg [4:0]  bit_cnt;     // Bit counter (0..23)
    reg        rw_r;        // Latched R/W

    // Detect start pulse
    reg tx_start_d;
    wire tx_start_pulse = tx_start & ~tx_start_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) tx_start_d <= 1'b0;
        else        tx_start_d <= tx_start;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            spi_csn     <= 1'b1;
            spi_clk     <= CPOL[0];
            spi_mosi    <= 1'b0;
            spi_clk_en  <= 1'b0;
            busy        <= 1'b0;
            tx_done     <= 1'b0;
            rdata       <= 8'h00;
            shift_reg   <= 24'h0;
            rx_reg      <= 24'h0;
            bit_cnt     <= 5'd0;
            rw_r        <= 1'b0;
            clk_cnt     <= {$clog2(CLK_DIV){1'b0}};
        end else begin
            tx_done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy       <= 1'b0;
                    spi_clk_en <= 1'b0;
                    spi_csn    <= 1'b1;
                    spi_clk    <= CPOL[0];
                    if (tx_start_pulse) begin
                        // Build 24-bit transfer: {rw, addr[6:0], 8'h00, wdata[7:0]}
                        shift_reg <= {rw, addr, 8'h00, wdata};
                        rw_r      <= rw;
                        bit_cnt   <= 5'd0;
                        busy      <= 1'b1;
                        state     <= ST_ASSERT;
                    end
                end

                ST_ASSERT: begin
                    spi_csn    <= 1'b0;
                    spi_clk_en <= 1'b1;
                    // Pre-drive MOSI with MSB before first clock edge
                    spi_mosi   <= shift_reg[23];
                    state      <= ST_SHIFT;
                end

                ST_SHIFT: begin
                    // For CPHA=0: shift on falling edge, sample on rising edge
                    // For CPHA=1: shift on rising edge, sample on falling edge
                    if ((CPHA == 0 && spi_clk_f) || (CPHA == 1 && spi_clk_r)) begin
                        // Drive next bit
                        shift_reg <= {shift_reg[22:0], 1'b0};
                        spi_mosi  <= shift_reg[22];
                    end
                    if ((CPHA == 0 && spi_clk_r) || (CPHA == 1 && spi_clk_f)) begin
                        // Sample MISO
                        rx_reg  <= {rx_reg[22:0], spi_miso};
                        bit_cnt <= bit_cnt + 1'b1;
                        if (bit_cnt == 5'd23) begin
                            state <= ST_DEASSERT;
                        end
                    end
                end

                ST_DEASSERT: begin
                    spi_clk_en <= 1'b0;
                    spi_csn    <= 1'b1;
                    spi_clk    <= CPOL[0];
                    spi_mosi   <= 1'b0;
                    if (rw_r)
                        rdata <= rx_reg[7:0];
                    state <= ST_DONE;
                end

                ST_DONE: begin
                    tx_done <= 1'b1;
                    busy    <= 1'b0;
                    state   <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
