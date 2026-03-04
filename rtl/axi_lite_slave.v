// =============================================================================
// Module: axi_lite_slave.v
// Description: AXI4-Lite slave interface for PS <-> PL register access.
//              Provides memory-mapped control/status registers for the
//              null steering system.
//
// Register Map:
//   0x00: CTRL   [0]=start capture, [1]=start cov, [2]=start inversion,
//                [3]=start weights, [4]=apply beam, [7:5]=reserved
//   0x04: STATUS [0]=capture_done, [1]=cov_valid, [2]=inv_valid,
//                [3]=weights_valid, [4]=busy, [5]=singular_error
//   0x08: SNAP_COUNT  - Number of snapshots for covariance (default 256)
//   0x0C: DIAG_LOAD   - Diagonal loading value (Q16 fixed-point)
//   0x10: STEER_ANGLE - Desired steering angle (degrees, signed Q8.8)
//   0x14: SPI_CTRL    - [0]=SPI start, [8]=R/W, [15:9]=address, [23:16]=write data
//   0x18: SPI_STATUS  - [0]=SPI busy, [1]=SPI done
//   0x1C: SPI_RDATA   - SPI read data [7:0]
//   0x20-0x3C: WEIGHT_OUT[0..3] real/imag pairs (read-only, 32-bit each)
//   0x40: VERSION     - Build version (read-only)
//   0x44: GPIO_CTRL   - [0]=GPIO0/RX_LOAD, [1]=GPIO1/TX_LOAD,
//                       [2]=GPIO4/TR, [3]=GPIO5/PA_ON (P3 connector)
// =============================================================================

`timescale 1ns / 1ps

module axi_lite_slave #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 32
)(
    // AXI4-Lite slave ports
    input  wire                      s_axi_aclk,
    input  wire                      s_axi_aresetn,

    // Write address channel
    input  wire [ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  wire [2:0]                s_axi_awprot,
    input  wire                      s_axi_awvalid,
    output reg                       s_axi_awready,

    // Write data channel
    input  wire [DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [DATA_WIDTH/8-1:0]   s_axi_wstrb,
    input  wire                      s_axi_wvalid,
    output reg                       s_axi_wready,

    // Write response channel
    output reg  [1:0]                s_axi_bresp,
    output reg                       s_axi_bvalid,
    input  wire                      s_axi_bready,

    // Read address channel
    input  wire [ADDR_WIDTH-1:0]     s_axi_araddr,
    input  wire [2:0]                s_axi_arprot,
    input  wire                      s_axi_arvalid,
    output reg                       s_axi_arready,

    // Read data channel
    output reg  [DATA_WIDTH-1:0]     s_axi_rdata,
    output reg  [1:0]                s_axi_rresp,
    output reg                       s_axi_rvalid,
    input  wire                      s_axi_rready,

    // Register outputs to PL logic
    output reg                       ctrl_start_capture,
    output reg                       ctrl_start_cov,
    output reg                       ctrl_start_inversion,
    output reg                       ctrl_start_weights,
    output reg                       ctrl_apply_beam,
    output reg  [DATA_WIDTH-1:0]     reg_snap_count,
    output reg  [DATA_WIDTH-1:0]     reg_diag_load,
    output reg  [DATA_WIDTH-1:0]     reg_steer_angle,
    output reg                       spi_start,
    output reg                       spi_rw,
    output reg  [6:0]                spi_addr,
    output reg  [7:0]                spi_wdata,

    // GPIO control outputs: ADAR1000 P3 connector GPIO signals
    // [0]=GPIO0/RX_LOAD, [1]=GPIO1/TX_LOAD, [2]=GPIO4/TR, [3]=GPIO5/PA_ON
    output reg  [3:0]                gpio_ctrl,

    // Status inputs from PL logic
    input  wire                      status_capture_done,
    input  wire                      status_cov_valid,
    input  wire                      status_inv_valid,
    input  wire                      status_weights_valid,
    input  wire                      status_busy,
    input  wire                      status_singular,
    input  wire                      status_spi_busy,
    input  wire                      status_spi_done,
    input  wire [7:0]                status_spi_rdata,
    input  wire [DATA_WIDTH-1:0]     weight_re_0,
    input  wire [DATA_WIDTH-1:0]     weight_im_0,
    input  wire [DATA_WIDTH-1:0]     weight_re_1,
    input  wire [DATA_WIDTH-1:0]     weight_im_1,
    input  wire [DATA_WIDTH-1:0]     weight_re_2,
    input  wire [DATA_WIDTH-1:0]     weight_im_2,
    input  wire [DATA_WIDTH-1:0]     weight_re_3,
    input  wire [DATA_WIDTH-1:0]     weight_im_3
);

    // -------------------------------------------------------------------------
    // Internal register addresses (byte-addressed, 4-byte aligned)
    // -------------------------------------------------------------------------
    localparam ADDR_CTRL        = 7'h00;
    localparam ADDR_STATUS      = 7'h04;
    localparam ADDR_SNAP_COUNT  = 7'h08;
    localparam ADDR_DIAG_LOAD   = 7'h0C;
    localparam ADDR_STEER_ANGLE = 7'h10;
    localparam ADDR_SPI_CTRL    = 7'h14;
    localparam ADDR_SPI_STATUS  = 7'h18;
    localparam ADDR_SPI_RDATA   = 7'h1C;
    localparam ADDR_W0_RE       = 7'h20;
    localparam ADDR_W0_IM       = 7'h24;
    localparam ADDR_W1_RE       = 7'h28;
    localparam ADDR_W1_IM       = 7'h2C;
    localparam ADDR_W2_RE       = 7'h30;
    localparam ADDR_W2_IM       = 7'h34;
    localparam ADDR_W3_RE       = 7'h38;
    localparam ADDR_W3_IM       = 7'h3C;
    localparam ADDR_VERSION     = 7'h40;
    localparam ADDR_GPIO_CTRL   = 7'h44;

    localparam VERSION_NUM      = 32'h0001_0000;  // v1.0

    // -------------------------------------------------------------------------
    // Write address/data latching
    // -------------------------------------------------------------------------
    reg [ADDR_WIDTH-1:0] aw_addr_r;
    reg                  aw_valid_r;

    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            aw_addr_r     <= {ADDR_WIDTH{1'b0}};
            aw_valid_r    <= 1'b0;

            ctrl_start_capture   <= 1'b0;
            ctrl_start_cov       <= 1'b0;
            ctrl_start_inversion <= 1'b0;
            ctrl_start_weights   <= 1'b0;
            ctrl_apply_beam      <= 1'b0;
            reg_snap_count       <= 32'd256;
            reg_diag_load        <= 32'd0;
            reg_steer_angle      <= 32'd0;
            spi_start            <= 1'b0;
            spi_rw               <= 1'b0;
            spi_addr             <= 7'h00;
            spi_wdata            <= 8'h00;
            gpio_ctrl            <= 4'b0000;
            // Reset defaults: RX_LOAD=0, TX_LOAD=0, TR=0 (receive mode),
            // PA_ON=0 (power amplifier off — safe power-on state).
        end else begin
            // Auto-clear pulse signals
            ctrl_start_capture   <= 1'b0;
            ctrl_start_cov       <= 1'b0;
            ctrl_start_inversion <= 1'b0;
            ctrl_start_weights   <= 1'b0;
            spi_start            <= 1'b0;

            // Accept write address
            if (s_axi_awvalid && !aw_valid_r) begin
                s_axi_awready <= 1'b1;
                aw_addr_r     <= s_axi_awaddr;
                aw_valid_r    <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
            end

            // Accept write data and perform write
            if (s_axi_wvalid && aw_valid_r) begin
                s_axi_wready <= 1'b1;
                aw_valid_r   <= 1'b0;

                case (aw_addr_r[6:0])
                    ADDR_CTRL: begin
                        ctrl_start_capture   <= s_axi_wdata[0];
                        ctrl_start_cov       <= s_axi_wdata[1];
                        ctrl_start_inversion <= s_axi_wdata[2];
                        ctrl_start_weights   <= s_axi_wdata[3];
                        ctrl_apply_beam      <= s_axi_wdata[4];
                    end
                    ADDR_SNAP_COUNT:  reg_snap_count  <= s_axi_wdata;
                    ADDR_DIAG_LOAD:   reg_diag_load   <= s_axi_wdata;
                    ADDR_STEER_ANGLE: reg_steer_angle <= s_axi_wdata;
                    ADDR_SPI_CTRL: begin
                        spi_start <= s_axi_wdata[0];
                        spi_rw    <= s_axi_wdata[8];
                        spi_addr  <= s_axi_wdata[15:9];
                        spi_wdata <= s_axi_wdata[23:16];
                    end
                    ADDR_GPIO_CTRL: gpio_ctrl <= s_axi_wdata[3:0];
                    default: ; // Ignore writes to read-only registers
                endcase

                // Send write response
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00; // OKAY
            end else begin
                s_axi_wready <= 1'b0;
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Read logic
    // -------------------------------------------------------------------------
    always @(posedge s_axi_aclk or negedge s_axi_aresetn) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= {DATA_WIDTH{1'b0}};
            s_axi_rresp   <= 2'b00;
        end else begin
            if (s_axi_arvalid && !s_axi_rvalid) begin
                s_axi_arready <= 1'b1;
                s_axi_rvalid  <= 1'b1;
                s_axi_rresp   <= 2'b00;

                case (s_axi_araddr[6:0])
                    ADDR_CTRL:        s_axi_rdata <= {27'b0, ctrl_apply_beam,
                                                       ctrl_start_weights,
                                                       ctrl_start_inversion,
                                                       ctrl_start_cov,
                                                       ctrl_start_capture};
                    ADDR_STATUS:      s_axi_rdata <= {26'b0,
                                                       status_singular,
                                                       status_busy,
                                                       status_weights_valid,
                                                       status_inv_valid,
                                                       status_cov_valid,
                                                       status_capture_done};
                    ADDR_SNAP_COUNT:  s_axi_rdata <= reg_snap_count;
                    ADDR_DIAG_LOAD:   s_axi_rdata <= reg_diag_load;
                    ADDR_STEER_ANGLE: s_axi_rdata <= reg_steer_angle;
                    ADDR_SPI_CTRL:    s_axi_rdata <= {8'b0, spi_wdata, spi_rw,
                                                       spi_addr, 1'b0};
                    ADDR_SPI_STATUS:  s_axi_rdata <= {30'b0, status_spi_done,
                                                       status_spi_busy};
                    ADDR_SPI_RDATA:   s_axi_rdata <= {24'b0, status_spi_rdata};
                    ADDR_W0_RE:       s_axi_rdata <= weight_re_0;
                    ADDR_W0_IM:       s_axi_rdata <= weight_im_0;
                    ADDR_W1_RE:       s_axi_rdata <= weight_re_1;
                    ADDR_W1_IM:       s_axi_rdata <= weight_im_1;
                    ADDR_W2_RE:       s_axi_rdata <= weight_re_2;
                    ADDR_W2_IM:       s_axi_rdata <= weight_im_2;
                    ADDR_W3_RE:       s_axi_rdata <= weight_re_3;
                    ADDR_W3_IM:       s_axi_rdata <= weight_im_3;
                    ADDR_VERSION:     s_axi_rdata <= VERSION_NUM;
                    ADDR_GPIO_CTRL:   s_axi_rdata <= {28'b0, gpio_ctrl};
                    default:          s_axi_rdata <= 32'hDEAD_BEEF;
                endcase
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule
