// =============================================================================
// Module: top_null_steering.v
// Description: Top-level module for the anti-jamming null steering system.
//              Instantiates and connects all PL modules for the Xilinx ZC702
//              (Zynq-7000) platform.
//
// Architecture:
//   PS (ARM) <--> AXI-Lite Slave <--> Control/Status Registers
//   ADC I/F --> Signal Acquisition --> Covariance Matrix --> Matrix Inverse
//           --> Weight Compute --> Beam Apply --> Output
//   PS <--> SPI Controller --> ADAR1000
// =============================================================================

`timescale 1ns / 1ps

module top_null_steering #(
    parameter SAMPLE_WIDTH  = 16,
    parameter COV_WIDTH     = 32,
    parameter WIDE_WIDTH    = 64,
    parameter OUT_WIDTH     = 32,
    parameter N_SNAP_WIDTH  = 9,
    parameter SPI_CLK_DIV   = 5    // 100MHz / (2*5) = 10MHz SPI clock
)(
    // System clock and reset from Zynq PS
    input  wire        sys_clk,    // 100 MHz
    input  wire        sys_rst_n,

    // ADC parallel interface (4 channels, I/Q each)
    input  wire [SAMPLE_WIDTH-1:0] adc_i_ch0,
    input  wire [SAMPLE_WIDTH-1:0] adc_q_ch0,
    input  wire [SAMPLE_WIDTH-1:0] adc_i_ch1,
    input  wire [SAMPLE_WIDTH-1:0] adc_q_ch1,
    input  wire [SAMPLE_WIDTH-1:0] adc_i_ch2,
    input  wire [SAMPLE_WIDTH-1:0] adc_q_ch2,
    input  wire [SAMPLE_WIDTH-1:0] adc_i_ch3,
    input  wire [SAMPLE_WIDTH-1:0] adc_q_ch3,
    input  wire                    adc_valid,

    // Beamformed output
    output wire [OUT_WIDTH-1:0]    beam_out_i,
    output wire [OUT_WIDTH-1:0]    beam_out_q,
    output wire                    beam_out_valid,

    // SPI to ADAR1000
    output wire                    spi_csn,
    output wire                    spi_clk,
    output wire                    spi_mosi,
    input  wire                    spi_miso,

    // AXI4-Lite slave interface (from Zynq PS)
    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,
    input  wire [31:0] s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [1:0]  s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [31:0] s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready,

    // Status LEDs (optional, for ZC702 debug)
    output wire [3:0]  status_leds
);

    // =========================================================================
    // Internal wires
    // =========================================================================

    // AXI -> PL control signals
    wire        ctrl_start_capture;
    wire        ctrl_start_cov;
    wire [31:0] reg_snap_count;
    wire [31:0] reg_diag_load;
    wire [31:0] reg_steer_angle;
    wire        spi_start_from_axi;
    wire        spi_rw_from_axi;
    wire [6:0]  spi_addr_from_axi;
    wire [7:0]  spi_wdata_from_axi;
    wire        ctrl_apply_beam;

    // Status -> AXI
    wire        status_capture_done;
    wire        status_cov_valid;
    wire        status_inv_valid;
    wire        status_weights_valid;
    wire        status_busy;
    wire        status_singular;
    wire        status_spi_busy;
    wire        status_spi_done;
    wire [7:0]  status_spi_rdata;

    // Signal acquisition outputs
    wire [SAMPLE_WIDTH-1:0] sa_i_ch0, sa_q_ch0;
    wire [SAMPLE_WIDTH-1:0] sa_i_ch1, sa_q_ch1;
    wire [SAMPLE_WIDTH-1:0] sa_i_ch2, sa_q_ch2;
    wire [SAMPLE_WIDTH-1:0] sa_i_ch3, sa_q_ch3;
    wire                    sa_valid;

    // MVDR weight outputs
    wire [COV_WIDTH-1:0] w_re [0:3];
    wire [COV_WIDTH-1:0] w_im [0:3];
    wire                        weight_valid;

    // Steering vector (generated from steer_angle register)
    // For simplicity, use pre-computed unit steering vector from PS via AXI
    // PS writes steer_re/im as weight outputs; default: broadside (all ones)
    // The steering vector is computed in software and written via AXI
    // In hardware we use a simple LUT or pass-through from PS-computed values
    reg [COV_WIDTH-1:0] steer_re [0:3];
    reg [COV_WIDTH-1:0] steer_im [0:3];

    // Update steering vector when steer_angle register is written
    // Default: broadside (theta=0), a = [1, 1, 1, 1] normalized
    // Q16.16 fixed-point: 16 integer bits | 16 fractional bits; 1.0 = 0x00010000
    localparam STEER_UNIT = 32'h00010000;  // 1.0 in Q16.16 fixed-point
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            steer_re[0] <= STEER_UNIT; steer_im[0] <= 32'b0;
            steer_re[1] <= STEER_UNIT; steer_im[1] <= 32'b0;
            steer_re[2] <= STEER_UNIT; steer_im[2] <= 32'b0;
            steer_re[3] <= STEER_UNIT; steer_im[3] <= 32'b0;
        end
        // PS updates steering vector through dedicated weight registers
        // (steer_re/im are updated via mmapped registers or PS-side driver)
    end

    // =========================================================================
    // AXI-Lite Slave
    // =========================================================================
    axi_lite_slave #(
        .DATA_WIDTH (32),
        .ADDR_WIDTH (32)
    ) u_axi_slave (
        .s_axi_aclk          (s_axi_aclk),
        .s_axi_aresetn       (s_axi_aresetn),
        .s_axi_awaddr        (s_axi_awaddr),
        .s_axi_awprot        (s_axi_awprot),
        .s_axi_awvalid       (s_axi_awvalid),
        .s_axi_awready       (s_axi_awready),
        .s_axi_wdata         (s_axi_wdata),
        .s_axi_wstrb         (s_axi_wstrb),
        .s_axi_wvalid        (s_axi_wvalid),
        .s_axi_wready        (s_axi_wready),
        .s_axi_bresp         (s_axi_bresp),
        .s_axi_bvalid        (s_axi_bvalid),
        .s_axi_bready        (s_axi_bready),
        .s_axi_araddr        (s_axi_araddr),
        .s_axi_arprot        (s_axi_arprot),
        .s_axi_arvalid       (s_axi_arvalid),
        .s_axi_arready       (s_axi_arready),
        .s_axi_rdata         (s_axi_rdata),
        .s_axi_rresp         (s_axi_rresp),
        .s_axi_rvalid        (s_axi_rvalid),
        .s_axi_rready        (s_axi_rready),
        .ctrl_start_capture  (ctrl_start_capture),
        .ctrl_start_cov      (ctrl_start_cov),
        .ctrl_start_inversion(),
        .ctrl_start_weights  (),
        .ctrl_apply_beam     (ctrl_apply_beam),
        .reg_snap_count      (reg_snap_count),
        .reg_diag_load       (reg_diag_load),
        .reg_steer_angle     (reg_steer_angle),
        .spi_start           (spi_start_from_axi),
        .spi_rw              (spi_rw_from_axi),
        .spi_addr            (spi_addr_from_axi),
        .spi_wdata           (spi_wdata_from_axi),
        .status_capture_done (status_capture_done),
        .status_cov_valid    (status_cov_valid),
        .status_inv_valid    (status_inv_valid),
        .status_weights_valid(status_weights_valid),
        .status_busy         (status_busy),
        .status_singular     (status_singular),
        .status_spi_busy     (status_spi_busy),
        .status_spi_done     (status_spi_done),
        .status_spi_rdata    (status_spi_rdata),
        .weight_re_0         (w_re[0]),
        .weight_im_0         (w_im[0]),
        .weight_re_1         (w_re[1]),
        .weight_im_1         (w_im[1]),
        .weight_re_2         (w_re[2]),
        .weight_im_2         (w_im[2]),
        .weight_re_3         (w_re[3]),
        .weight_im_3         (w_im[3])
    );

    // =========================================================================
    // Signal Acquisition
    // =========================================================================
    signal_acquisition #(
        .DATA_WIDTH   (SAMPLE_WIDTH),
        .NUM_CHANNELS (4),
        .BUFFER_DEPTH (256),
        .ADDR_WIDTH   (8)
    ) u_sig_acq (
        .clk            (sys_clk),
        .rst_n          (sys_rst_n),
        .adc_i_ch0      (adc_i_ch0),
        .adc_q_ch0      (adc_q_ch0),
        .adc_i_ch1      (adc_i_ch1),
        .adc_q_ch1      (adc_q_ch1),
        .adc_i_ch2      (adc_i_ch2),
        .adc_q_ch2      (adc_q_ch2),
        .adc_i_ch3      (adc_i_ch3),
        .adc_q_ch3      (adc_q_ch3),
        .adc_valid      (adc_valid),
        .capture_en     (1'b1),
        .capture_start  (ctrl_start_capture),
        .sample_count   (reg_snap_count[7:0]),
        .out_i_ch0      (sa_i_ch0),
        .out_q_ch0      (sa_q_ch0),
        .out_i_ch1      (sa_i_ch1),
        .out_q_ch1      (sa_q_ch1),
        .out_i_ch2      (sa_i_ch2),
        .out_q_ch2      (sa_q_ch2),
        .out_i_ch3      (sa_i_ch3),
        .out_q_ch3      (sa_q_ch3),
        .out_valid      (sa_valid),
        .capture_done   (status_capture_done),
        .sample_index   ()
    );

    // =========================================================================
    // MVDR Beamformer Pipeline
    // =========================================================================
    mvdr_beamformer #(
        .SAMPLE_WIDTH (SAMPLE_WIDTH),
        .COV_WIDTH    (COV_WIDTH),
        .WIDE_WIDTH   (WIDE_WIDTH),
        .N_SNAP_WIDTH (N_SNAP_WIDTH)
    ) u_mvdr (
        .clk          (sys_clk),
        .rst_n        (sys_rst_n),
        .in_i_ch0     (sa_i_ch0),
        .in_q_ch0     (sa_q_ch0),
        .in_i_ch1     (sa_i_ch1),
        .in_q_ch1     (sa_q_ch1),
        .in_i_ch2     (sa_i_ch2),
        .in_q_ch2     (sa_q_ch2),
        .in_i_ch3     (sa_i_ch3),
        .in_q_ch3     (sa_q_ch3),
        .in_valid     (sa_valid),
        .steer_re     (steer_re),
        .steer_im     (steer_im),
        .start        (ctrl_start_cov),
        .n_snapshots  (reg_snap_count[N_SNAP_WIDTH-1:0]),
        .diag_load    (reg_diag_load),
        .weight_re    (w_re),
        .weight_im    (w_im),
        .weight_valid (status_weights_valid),
        .cov_valid    (status_cov_valid),
        .inv_valid    (status_inv_valid),
        .busy         (status_busy),
        .singular_error(status_singular)
    );

    // =========================================================================
    // Beam Apply (apply weights to live ADC stream)
    // =========================================================================
    beam_apply #(
        .DATA_WIDTH   (SAMPLE_WIDTH),
        .WEIGHT_WIDTH (COV_WIDTH),
        .OUT_WIDTH    (OUT_WIDTH)
    ) u_beam_apply (
        .clk           (sys_clk),
        .rst_n         (sys_rst_n),
        .in_i_ch0      (adc_i_ch0),
        .in_q_ch0      (adc_q_ch0),
        .in_i_ch1      (adc_i_ch1),
        .in_q_ch1      (adc_q_ch1),
        .in_i_ch2      (adc_i_ch2),
        .in_q_ch2      (adc_q_ch2),
        .in_i_ch3      (adc_i_ch3),
        .in_q_ch3      (adc_q_ch3),
        .in_valid      (adc_valid & ctrl_apply_beam),
        .w_re          (w_re),
        .w_im          (w_im),
        .weights_valid (status_weights_valid),
        .out_i         (beam_out_i),
        .out_q         (beam_out_q),
        .out_valid     (beam_out_valid)
    );

    // =========================================================================
    // ADAR1000 SPI Controller
    // =========================================================================
    adar1000_spi #(
        .CLK_DIV (SPI_CLK_DIV),
        .CPOL    (0),
        .CPHA    (0)
    ) u_spi (
        .clk      (sys_clk),
        .rst_n    (sys_rst_n),
        .tx_start (spi_start_from_axi),
        .rw       (spi_rw_from_axi),
        .addr     (spi_addr_from_axi),
        .wdata    (spi_wdata_from_axi),
        .rdata    (status_spi_rdata),
        .tx_done  (status_spi_done),
        .busy     (status_spi_busy),
        .spi_csn  (spi_csn),
        .spi_clk  (spi_clk),
        .spi_mosi (spi_mosi),
        .spi_miso (spi_miso)
    );

    // =========================================================================
    // Status LEDs (ZC702 GPIO)
    // =========================================================================
    assign status_leds[0] = status_capture_done;
    assign status_leds[1] = status_weights_valid;
    assign status_leds[2] = status_busy;
    assign status_leds[3] = status_singular;

    // weight_valid is an alias used here
    assign weight_valid = status_weights_valid;

endmodule
