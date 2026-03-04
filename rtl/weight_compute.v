// =============================================================================
// Module: weight_compute.v
// Description: Computes MVDR beamforming weights.
//              w = (Rxx^-1 * a) / (a^H * Rxx^-1 * a)
//
//   Step 1: v = Rxx^-1 * a    (4-element complex vector)
//   Step 2: denom = a^H * v   (complex scalar)
//   Step 3: w = v / denom     (element-wise complex division)
//
// The steering vector a for direction theta (linear array, half-wavelength spacing):
//   a[n] = exp(j * pi * n * sin(theta)), n = 0,1,2,3
//
// Steering vector components are pre-computed and provided as input
// (allows both hardware and software steering vector generation).
// =============================================================================

`timescale 1ns / 1ps

module weight_compute #(
    parameter DATA_WIDTH = 32,   // Width of covariance matrix elements
    parameter WIDE_WIDTH = 64    // Extended precision for intermediate products
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // Inverse covariance matrix (from matrix_inverse)
    input  wire [DATA_WIDTH-1:0]  rinv_re [0:3][0:3],
    input  wire [DATA_WIDTH-1:0]  rinv_im [0:3][0:3],
    input  wire                          rinv_valid,

    // Steering vector for desired look direction (a[0..3], complex)
    input  wire [DATA_WIDTH-1:0]  steer_re [0:3],
    input  wire [DATA_WIDTH-1:0]  steer_im [0:3],

    // Output: MVDR weights (w[0..3], complex)
    output reg  [DATA_WIDTH-1:0]  weight_re [0:3],
    output reg  [DATA_WIDTH-1:0]  weight_im [0:3],
    output reg                           weight_valid,
    output reg                           busy
);

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    localparam ST_IDLE     = 3'd0;
    localparam ST_MVM      = 3'd1;   // Matrix-vector multiply: v = Rinv * a
    localparam ST_INNER    = 3'd2;   // Inner product: denom = a^H * v
    localparam ST_DIVIDE   = 3'd3;   // w = v / denom
    localparam ST_DONE     = 3'd4;

    reg [2:0] state;

    // Intermediate results
    reg signed [WIDE_WIDTH-1:0] v_re [0:3];  // v = Rinv * a
    reg signed [WIDE_WIDTH-1:0] v_im [0:3];
    reg signed [WIDE_WIDTH-1:0] denom_re;    // denom = a^H * v (should be real for Hermitian Rinv)
    reg signed [WIDE_WIDTH-1:0] denom_im;

    // Register inputs
    reg signed [DATA_WIDTH-1:0] a_re [0:3];
    reg signed [DATA_WIDTH-1:0] a_im [0:3];
    reg signed [DATA_WIDTH-1:0] R_re [0:3][0:3];
    reg signed [DATA_WIDTH-1:0] R_im [0:3][0:3];

    integer i, j;

    // Detect valid pulse
    reg rinv_valid_d;
    wire rinv_valid_pulse = rinv_valid & ~rinv_valid_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rinv_valid_d <= 1'b0;
        else        rinv_valid_d <= rinv_valid;
    end

    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            weight_valid <= 1'b0;
            busy         <= 1'b0;
            denom_re     <= {WIDE_WIDTH{1'b0}};
            denom_im     <= {WIDE_WIDTH{1'b0}};
            for (i = 0; i < 4; i = i + 1) begin
                v_re[i]      <= {WIDE_WIDTH{1'b0}};
                v_im[i]      <= {WIDE_WIDTH{1'b0}};
                weight_re[i] <= {DATA_WIDTH{1'b0}};
                weight_im[i] <= {DATA_WIDTH{1'b0}};
                a_re[i]      <= {DATA_WIDTH{1'b0}};
                a_im[i]      <= {DATA_WIDTH{1'b0}};
                for (j = 0; j < 4; j = j + 1) begin
                    R_re[i][j] <= {DATA_WIDTH{1'b0}};
                    R_im[i][j] <= {DATA_WIDTH{1'b0}};
                end
            end
        end else begin
            case (state)
                ST_IDLE: begin
                    weight_valid <= 1'b0;
                    busy         <= 1'b0;
                    if (rinv_valid_pulse) begin
                        // Latch inputs
                        for (i = 0; i < 4; i = i + 1) begin
                            a_re[i] <= steer_re[i];
                            a_im[i] <= steer_im[i];
                            for (j = 0; j < 4; j = j + 1) begin
                                R_re[i][j] <= rinv_re[i][j];
                                R_im[i][j] <= rinv_im[i][j];
                            end
                        end
                        busy  <= 1'b1;
                        state <= ST_MVM;
                    end
                end

                // v[i] = sum_j R[i][j] * a[j]
                // complex product: (R_re + j*R_im)(a_re + j*a_im)
                //   real = R_re*a_re - R_im*a_im
                //   imag = R_re*a_im + R_im*a_re
                ST_MVM: begin
                    for (i = 0; i < 4; i = i + 1) begin
                        v_re[i] <= (R_re[i][0]*a_re[0] - R_im[i][0]*a_im[0])
                                  +(R_re[i][1]*a_re[1] - R_im[i][1]*a_im[1])
                                  +(R_re[i][2]*a_re[2] - R_im[i][2]*a_im[2])
                                  +(R_re[i][3]*a_re[3] - R_im[i][3]*a_im[3]);
                        v_im[i] <= (R_re[i][0]*a_im[0] + R_im[i][0]*a_re[0])
                                  +(R_re[i][1]*a_im[1] + R_im[i][1]*a_re[1])
                                  +(R_re[i][2]*a_im[2] + R_im[i][2]*a_re[2])
                                  +(R_re[i][3]*a_im[3] + R_im[i][3]*a_re[3]);
                    end
                    state <= ST_INNER;
                end

                // denom = a^H * v = sum_i conj(a[i]) * v[i]
                //   real = a_re[i]*v_re[i] + a_im[i]*v_im[i]
                //   imag = a_re[i]*v_im[i] - a_im[i]*v_re[i]
                ST_INNER: begin
                    denom_re <= (a_re[0]*v_re[0] + a_im[0]*v_im[0])
                               +(a_re[1]*v_re[1] + a_im[1]*v_im[1])
                               +(a_re[2]*v_re[2] + a_im[2]*v_im[2])
                               +(a_re[3]*v_re[3] + a_im[3]*v_im[3]);
                    denom_im <= (a_re[0]*v_im[0] - a_im[0]*v_re[0])
                               +(a_re[1]*v_im[1] - a_im[1]*v_re[1])
                               +(a_re[2]*v_im[2] - a_im[2]*v_re[2])
                               +(a_re[3]*v_im[3] - a_im[3]*v_re[3]);
                    state <= ST_DIVIDE;
                end

                // w[i] = v[i] / denom
                // (v_re + j*v_im) / (d_re + j*d_im)
                //   real = (v_re*d_re + v_im*d_im) / (d_re^2 + d_im^2)
                //   imag = (v_im*d_re - v_re*d_im) / (d_re^2 + d_im^2)
                ST_DIVIDE: begin
                    begin : div_block
                        reg signed [WIDE_WIDTH-1:0] denom_sq;
                        denom_sq = denom_re*denom_re + denom_im*denom_im;
                        // Use threshold (> 0) rather than exact zero to detect
                        // near-zero denominators after fixed-point arithmetic
                        if (denom_sq <= 0) begin
                            // Fallback: set uniform weights if denominator is zero
                            for (i = 0; i < 4; i = i + 1) begin
                                weight_re[i] <= {{(DATA_WIDTH-2){1'b0}}, 2'b01};
                                weight_im[i] <= {DATA_WIDTH{1'b0}};
                            end
                        end else begin
                            for (i = 0; i < 4; i = i + 1) begin
                                weight_re[i] <= ($signed(v_re[i])*denom_re
                                               + $signed(v_im[i])*denom_im) / denom_sq;
                                weight_im[i] <= ($signed(v_im[i])*denom_re
                                               - $signed(v_re[i])*denom_im) / denom_sq;
                            end
                        end
                    end
                    state <= ST_DONE;
                end

                ST_DONE: begin
                    weight_valid <= 1'b1;
                    busy         <= 1'b0;
                    state        <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
