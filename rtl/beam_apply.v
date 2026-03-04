// =============================================================================
// Module: beam_apply.v
// Description: Applies complex beamforming weights to 4 antenna element signals.
//              Output = sum_i w[i] * x[i]  (complex dot product)
//
//   y = w[0]*x[0] + w[1]*x[1] + w[2]*x[2] + w[3]*x[3]
//
// Complex multiply: (w_re + j*w_im)(x_re + j*x_im)
//   real = w_re*x_re - w_im*x_im
//   imag = w_re*x_im + w_im*x_re
// =============================================================================

`timescale 1ns / 1ps

module beam_apply #(
    parameter DATA_WIDTH   = 16,  // Width of input I/Q samples
    parameter WEIGHT_WIDTH = 32,  // Width of beamforming weights
    parameter OUT_WIDTH    = 32   // Output width (saturated)
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // Input samples (4 channels)
    input  wire [DATA_WIDTH-1:0]  in_i_ch0,
    input  wire [DATA_WIDTH-1:0]  in_q_ch0,
    input  wire [DATA_WIDTH-1:0]  in_i_ch1,
    input  wire [DATA_WIDTH-1:0]  in_q_ch1,
    input  wire [DATA_WIDTH-1:0]  in_i_ch2,
    input  wire [DATA_WIDTH-1:0]  in_q_ch2,
    input  wire [DATA_WIDTH-1:0]  in_i_ch3,
    input  wire [DATA_WIDTH-1:0]  in_q_ch3,
    input  wire                          in_valid,

    // Beamforming weights (from weight_compute)
    input  wire [WEIGHT_WIDTH-1:0] w_re [0:3],
    input  wire [WEIGHT_WIDTH-1:0] w_im [0:3],
    input  wire                           weights_valid,  // Weights are stable

    // Beamformed output
    output reg  [OUT_WIDTH-1:0]   out_i,
    output reg  [OUT_WIDTH-1:0]   out_q,
    output reg                           out_valid
);

    // -------------------------------------------------------------------------
    // Pipeline stage 1: Complex multiply for each element
    // -------------------------------------------------------------------------
    localparam PROD_WIDTH = DATA_WIDTH + WEIGHT_WIDTH;

    reg signed [PROD_WIDTH-1:0] prod_re [0:3];
    reg signed [PROD_WIDTH-1:0] prod_im [0:3];
    reg                         stage1_valid;

    // Registered weights
    reg signed [WEIGHT_WIDTH-1:0] w_re_r [0:3];
    reg signed [WEIGHT_WIDTH-1:0] w_im_r [0:3];

    // Latch weights when valid
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_re_r[0] <= {WEIGHT_WIDTH{1'b0}};
            w_re_r[1] <= {WEIGHT_WIDTH{1'b0}};
            w_re_r[2] <= {WEIGHT_WIDTH{1'b0}};
            w_re_r[3] <= {WEIGHT_WIDTH{1'b0}};
            w_im_r[0] <= {WEIGHT_WIDTH{1'b0}};
            w_im_r[1] <= {WEIGHT_WIDTH{1'b0}};
            w_im_r[2] <= {WEIGHT_WIDTH{1'b0}};
            w_im_r[3] <= {WEIGHT_WIDTH{1'b0}};
        end else if (weights_valid) begin
            w_re_r[0] <= w_re[0]; w_im_r[0] <= w_im[0];
            w_re_r[1] <= w_re[1]; w_im_r[1] <= w_im[1];
            w_re_r[2] <= w_re[2]; w_im_r[2] <= w_im[2];
            w_re_r[3] <= w_re[3]; w_im_r[3] <= w_im[3];
        end
    end

    // Stage 1: multiply
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod_re[0] <= {PROD_WIDTH{1'b0}};
            prod_re[1] <= {PROD_WIDTH{1'b0}};
            prod_re[2] <= {PROD_WIDTH{1'b0}};
            prod_re[3] <= {PROD_WIDTH{1'b0}};
            prod_im[0] <= {PROD_WIDTH{1'b0}};
            prod_im[1] <= {PROD_WIDTH{1'b0}};
            prod_im[2] <= {PROD_WIDTH{1'b0}};
            prod_im[3] <= {PROD_WIDTH{1'b0}};
            stage1_valid <= 1'b0;
        end else begin
            stage1_valid <= in_valid;
            if (in_valid) begin
                // w_re*x_re - w_im*x_im
                prod_re[0] <= w_re_r[0]*in_i_ch0 - w_im_r[0]*in_q_ch0;
                prod_re[1] <= w_re_r[1]*in_i_ch1 - w_im_r[1]*in_q_ch1;
                prod_re[2] <= w_re_r[2]*in_i_ch2 - w_im_r[2]*in_q_ch2;
                prod_re[3] <= w_re_r[3]*in_i_ch3 - w_im_r[3]*in_q_ch3;
                // w_re*x_im + w_im*x_re
                prod_im[0] <= w_re_r[0]*in_q_ch0 + w_im_r[0]*in_i_ch0;
                prod_im[1] <= w_re_r[1]*in_q_ch1 + w_im_r[1]*in_i_ch1;
                prod_im[2] <= w_re_r[2]*in_q_ch2 + w_im_r[2]*in_i_ch2;
                prod_im[3] <= w_re_r[3]*in_q_ch3 + w_im_r[3]*in_i_ch3;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Pipeline stage 2: Accumulate (sum) across 4 elements + saturation
    // -------------------------------------------------------------------------
    localparam SUM_WIDTH = PROD_WIDTH + 2;  // Extra bits for 4-element sum

    reg signed [SUM_WIDTH-1:0] sum_re;
    reg signed [SUM_WIDTH-1:0] sum_q;

    // Saturation limits for output
    localparam signed [OUT_WIDTH-1:0] SAT_MAX =  {1'b0, {(OUT_WIDTH-1){1'b1}}};
    localparam signed [OUT_WIDTH-1:0] SAT_MIN =  {1'b1, {(OUT_WIDTH-1){1'b0}}};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_i     <= {OUT_WIDTH{1'b0}};
            out_q     <= {OUT_WIDTH{1'b0}};
            out_valid <= 1'b0;
        end else begin
            out_valid <= stage1_valid;
            if (stage1_valid) begin
                sum_re <= prod_re[0] + prod_re[1] + prod_re[2] + prod_re[3];
                sum_q  <= prod_im[0] + prod_im[1] + prod_im[2] + prod_im[3];

                // Saturating truncation to OUT_WIDTH
                if ($signed(sum_re) > $signed({{(SUM_WIDTH-OUT_WIDTH){1'b0}}, SAT_MAX}))
                    out_i <= SAT_MAX;
                else if ($signed(sum_re) < $signed({{(SUM_WIDTH-OUT_WIDTH){1'b1}}, SAT_MIN}))
                    out_i <= SAT_MIN;
                else
                    out_i <= sum_re[OUT_WIDTH-1:0];

                if ($signed(sum_q) > $signed({{(SUM_WIDTH-OUT_WIDTH){1'b0}}, SAT_MAX}))
                    out_q <= SAT_MAX;
                else if ($signed(sum_q) < $signed({{(SUM_WIDTH-OUT_WIDTH){1'b1}}, SAT_MIN}))
                    out_q <= SAT_MIN;
                else
                    out_q <= sum_q[OUT_WIDTH-1:0];
            end
        end
    end

endmodule
