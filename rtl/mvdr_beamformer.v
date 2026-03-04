// =============================================================================
// Module: mvdr_beamformer.v
// Description: Top-level MVDR beamformer engine connecting:
//   covariance_matrix -> matrix_inverse -> weight_compute
//
//   Orchestrates the MVDR pipeline:
//   1. Covariance matrix estimation from signal_acquisition output
//   2. 4x4 complex matrix inversion with diagonal loading
//   3. MVDR weight computation using steering vector
// =============================================================================

`timescale 1ns / 1ps

module mvdr_beamformer #(
    parameter SAMPLE_WIDTH  = 16,   // ADC I/Q sample width
    parameter COV_WIDTH     = 32,   // Covariance matrix element width
    parameter WIDE_WIDTH    = 64,   // Extended precision
    parameter N_SNAP_WIDTH  = 9     // Bits for snapshot counter
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // Input from signal_acquisition
    input  wire [SAMPLE_WIDTH-1:0] in_i_ch0,
    input  wire [SAMPLE_WIDTH-1:0] in_q_ch0,
    input  wire [SAMPLE_WIDTH-1:0] in_i_ch1,
    input  wire [SAMPLE_WIDTH-1:0] in_q_ch1,
    input  wire [SAMPLE_WIDTH-1:0] in_i_ch2,
    input  wire [SAMPLE_WIDTH-1:0] in_q_ch2,
    input  wire [SAMPLE_WIDTH-1:0] in_i_ch3,
    input  wire [SAMPLE_WIDTH-1:0] in_q_ch3,
    input  wire                           in_valid,

    // Steering vector (desired look direction)
    input  wire [COV_WIDTH-1:0]    steer_re [0:3],
    input  wire [COV_WIDTH-1:0]    steer_im [0:3],

    // Control (from AXI-Lite or PS)
    input  wire                           start,           // Begin processing
    input  wire [N_SNAP_WIDTH-1:0]        n_snapshots,     // Number of snapshots
    input  wire [COV_WIDTH-1:0]    diag_load,       // Diagonal loading

    // Output weights
    output wire [COV_WIDTH-1:0]    weight_re [0:3],
    output wire [COV_WIDTH-1:0]    weight_im [0:3],
    output wire                           weight_valid,

    // Status
    output wire                           cov_valid,
    output wire                           inv_valid,
    output wire                           busy,
    output wire                           singular_error
);

    // -------------------------------------------------------------------------
    // Covariance matrix outputs
    // -------------------------------------------------------------------------
    wire [COV_WIDTH-1:0] rxx_re [0:3][0:3];
    wire [COV_WIDTH-1:0] rxx_im [0:3][0:3];
    wire                 rxx_valid;
    wire                 cov_busy;

    // -------------------------------------------------------------------------
    // Inverse matrix outputs
    // -------------------------------------------------------------------------
    wire [COV_WIDTH-1:0] rinv_re [0:3][0:3];
    wire [COV_WIDTH-1:0] rinv_im [0:3][0:3];
    wire                        rinv_valid;
    wire                        inv_busy;

    // -------------------------------------------------------------------------
    // Covariance matrix estimation
    // -------------------------------------------------------------------------
    covariance_matrix #(
        .DATA_WIDTH  (SAMPLE_WIDTH),
        .ACCUM_WIDTH (COV_WIDTH),
        .N_SNAP      (256),
        .SNAP_WIDTH  (N_SNAP_WIDTH)
    ) u_cov_matrix (
        .clk         (clk),
        .rst_n       (rst_n),
        .in_i_ch0    (in_i_ch0),
        .in_q_ch0    (in_q_ch0),
        .in_i_ch1    (in_i_ch1),
        .in_q_ch1    (in_q_ch1),
        .in_i_ch2    (in_i_ch2),
        .in_q_ch2    (in_q_ch2),
        .in_i_ch3    (in_i_ch3),
        .in_q_ch3    (in_q_ch3),
        .in_valid    (in_valid),
        .start       (start),
        .n_snapshots (n_snapshots),
        .rxx_re      (rxx_re),
        .rxx_im      (rxx_im),
        .rxx_valid   (rxx_valid),
        .busy        (cov_busy),
        .snap_count  ()
    );

    // -------------------------------------------------------------------------
    // Matrix inversion
    // -------------------------------------------------------------------------
    matrix_inverse #(
        .DATA_WIDTH  (COV_WIDTH),
        .WIDE_WIDTH  (WIDE_WIDTH)
    ) u_mat_inv (
        .clk            (clk),
        .rst_n          (rst_n),
        .mat_re         (rxx_re),
        .mat_im         (rxx_im),
        .mat_valid      (rxx_valid),
        .diag_load      (diag_load),
        .inv_re         (rinv_re),
        .inv_im         (rinv_im),
        .inv_valid      (rinv_valid),
        .busy           (inv_busy),
        .error_singular (singular_error)
    );

    // -------------------------------------------------------------------------
    // Weight computation
    // -------------------------------------------------------------------------
    weight_compute #(
        .DATA_WIDTH (COV_WIDTH),
        .WIDE_WIDTH (WIDE_WIDTH)
    ) u_weight_compute (
        .clk          (clk),
        .rst_n        (rst_n),
        .rinv_re      (rinv_re),
        .rinv_im      (rinv_im),
        .rinv_valid   (rinv_valid),
        .steer_re     (steer_re),
        .steer_im     (steer_im),
        .weight_re    (weight_re),
        .weight_im    (weight_im),
        .weight_valid (weight_valid),
        .busy         ()
    );

    // -------------------------------------------------------------------------
    // Status aggregation
    // -------------------------------------------------------------------------
    assign cov_valid = rxx_valid;
    assign inv_valid = rinv_valid;
    assign busy      = cov_busy | inv_busy;

endmodule
