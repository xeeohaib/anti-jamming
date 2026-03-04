// =============================================================================
// Module: covariance_matrix.v
// Description: Real-time spatial covariance matrix (Rxx) computation.
//              Computes the 4x4 complex Hermitian covariance matrix from
//              4-element antenna array snapshots.
//              Rxx[i][j] = sum_k( x_i[k] * conj(x_j[k]) ) / N
//
// Fixed-point format: 16-bit I/Q input, 32-bit accumulation
// Matrix is Hermitian: only upper triangle is computed; lower is conjugate
// =============================================================================

`timescale 1ns / 1ps

module covariance_matrix #(
    parameter DATA_WIDTH  = 16,  // Width of I/Q input samples
    parameter ACCUM_WIDTH = 32,  // Accumulator width (must be >= 2*DATA_WIDTH)
    parameter N_SNAP      = 256, // Number of snapshots for averaging
    parameter SNAP_WIDTH  = 9    // ceil(log2(N_SNAP + 1))
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // Input samples from signal_acquisition (4 channels, I/Q)
    input  wire [DATA_WIDTH-1:0]  in_i_ch0,
    input  wire [DATA_WIDTH-1:0]  in_q_ch0,
    input  wire [DATA_WIDTH-1:0]  in_i_ch1,
    input  wire [DATA_WIDTH-1:0]  in_q_ch1,
    input  wire [DATA_WIDTH-1:0]  in_i_ch2,
    input  wire [DATA_WIDTH-1:0]  in_q_ch2,
    input  wire [DATA_WIDTH-1:0]  in_i_ch3,
    input  wire [DATA_WIDTH-1:0]  in_q_ch3,
    input  wire                   in_valid,

    // Control
    input  wire                   start,       // Begin new covariance estimation
    input  wire [SNAP_WIDTH-1:0]  n_snapshots, // Number of snapshots to use

    // Output: 4x4 complex Hermitian covariance matrix (upper triangle + diagonal)
    // Indexed as Rxx[row][col], col >= row
    // Real and imaginary parts, 32-bit each
    // 10 unique elements: (0,0),(0,1),(0,2),(0,3),(1,1),(1,2),(1,3),(2,2),(2,3),(3,3)
    output reg  [ACCUM_WIDTH-1:0] rxx_re [0:3][0:3],
    output reg  [ACCUM_WIDTH-1:0] rxx_im [0:3][0:3],
    output reg                    rxx_valid,  // Covariance matrix is ready

    // Status
    output reg                    busy,
    output reg  [SNAP_WIDTH-1:0]  snap_count
);

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    localparam ST_IDLE     = 2'b00;
    localparam ST_ACCUMULATE = 2'b01;
    localparam ST_NORMALIZE  = 2'b10;
    localparam ST_DONE     = 2'b11;

    reg [1:0]            state;
    reg [SNAP_WIDTH-1:0] count;
    reg [SNAP_WIDTH-1:0] target_snaps;

    // Detect rising edge of start
    reg start_d;
    wire start_pulse = start & ~start_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) start_d <= 1'b0;
        else        start_d <= start;
    end

    // -------------------------------------------------------------------------
    // Accumulators: 4x4 complex, stored as separate real/imag arrays
    // Only upper triangle + diagonal computed (Hermitian symmetry).
    // Lower triangle: rxx[i][j] = conj(rxx[j][i])
    // -------------------------------------------------------------------------
    reg signed [ACCUM_WIDTH-1:0] accum_re [0:3][0:3];
    reg signed [ACCUM_WIDTH-1:0] accum_im [0:3][0:3];

    // Registered input samples
    reg signed [DATA_WIDTH-1:0]  x_i [0:3];
    reg signed [DATA_WIDTH-1:0]  x_q [0:3];
    reg                          valid_d;

    // Register inputs
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_i[0] <= {DATA_WIDTH{1'b0}}; x_q[0] <= {DATA_WIDTH{1'b0}};
            x_i[1] <= {DATA_WIDTH{1'b0}}; x_q[1] <= {DATA_WIDTH{1'b0}};
            x_i[2] <= {DATA_WIDTH{1'b0}}; x_q[2] <= {DATA_WIDTH{1'b0}};
            x_i[3] <= {DATA_WIDTH{1'b0}}; x_q[3] <= {DATA_WIDTH{1'b0}};
            valid_d <= 1'b0;
        end else begin
            x_i[0] <= $signed(in_i_ch0); x_q[0] <= $signed(in_q_ch0);
            x_i[1] <= $signed(in_i_ch1); x_q[1] <= $signed(in_q_ch1);
            x_i[2] <= $signed(in_i_ch2); x_q[2] <= $signed(in_q_ch2);
            x_i[3] <= $signed(in_i_ch3); x_q[3] <= $signed(in_q_ch3);
            valid_d <= in_valid;
        end
    end

    // -------------------------------------------------------------------------
    // Products: x_i[row] * x_i[col] + x_q[row] * x_q[col] (real part of x*conj(x))
    //           x_q[row] * x_i[col] - x_i[row] * x_q[col] (imag part of x*conj(x))
    // x_row * conj(x_col) = (xi_row + j*xq_row)(xi_col - j*xq_col)
    //   real = xi_row*xi_col + xq_row*xq_col
    //   imag = xq_row*xi_col - xi_row*xq_col
    // -------------------------------------------------------------------------
    integer row_i, col_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            count        <= {SNAP_WIDTH{1'b0}};
            target_snaps <= {SNAP_WIDTH{1'b0}};
            rxx_valid    <= 1'b0;
            busy         <= 1'b0;
            snap_count   <= {SNAP_WIDTH{1'b0}};
            for (row_i = 0; row_i < 4; row_i = row_i + 1)
                for (col_i = 0; col_i < 4; col_i = col_i + 1) begin
                    accum_re[row_i][col_i] <= {ACCUM_WIDTH{1'b0}};
                    accum_im[row_i][col_i] <= {ACCUM_WIDTH{1'b0}};
                    rxx_re[row_i][col_i]   <= {ACCUM_WIDTH{1'b0}};
                    rxx_im[row_i][col_i]   <= {ACCUM_WIDTH{1'b0}};
                end
        end else begin
            case (state)
                ST_IDLE: begin
                    // rxx_valid holds its value from ST_DONE until a new run starts
                    busy <= 1'b0;
                    if (start_pulse) begin
                        rxx_valid <= 1'b0;  // Clear on new measurement start
                        // Clear accumulators
                        for (row_i = 0; row_i < 4; row_i = row_i + 1)
                            for (col_i = 0; col_i < 4; col_i = col_i + 1) begin
                                accum_re[row_i][col_i] <= {ACCUM_WIDTH{1'b0}};
                                accum_im[row_i][col_i] <= {ACCUM_WIDTH{1'b0}};
                            end
                        count        <= {SNAP_WIDTH{1'b0}};
                        target_snaps <= n_snapshots;
                        busy         <= 1'b1;
                        state        <= ST_ACCUMULATE;
                    end
                end

                ST_ACCUMULATE: begin
                    if (valid_d) begin
                        // Accumulate upper triangle (including diagonal).
                        // Products are evaluated in the context-determined width of
                        // ACCUM_WIDTH (the NBA target), which correctly widens the
                        // 16-bit × 16-bit product via sign extension.  Avoid
                        // concatenation with a zero-width replication ({0{...}})
                        // which is illegal per IEEE 1364-2001 §4.1.14 and causes
                        // simulators to silently produce a 0-bit (zero) result.
                        for (row_i = 0; row_i < 4; row_i = row_i + 1) begin
                            for (col_i = row_i; col_i < 4; col_i = col_i + 1) begin
                                accum_re[row_i][col_i] <= accum_re[row_i][col_i] +
                                    (x_i[row_i] * x_i[col_i] + x_q[row_i] * x_q[col_i]);
                                accum_im[row_i][col_i] <= accum_im[row_i][col_i] +
                                    (x_q[row_i] * x_i[col_i] - x_i[row_i] * x_q[col_i]);
                            end
                        end
                        snap_count <= count + 1'b1;
                        if (count == target_snaps - 1) begin
                            count <= {SNAP_WIDTH{1'b0}};
                            state <= ST_NORMALIZE;
                        end else begin
                            count <= count + 1'b1;
                        end
                    end
                end

                ST_NORMALIZE: begin
                    // Copy upper triangle to output; mirror for lower triangle
                    for (row_i = 0; row_i < 4; row_i = row_i + 1) begin
                        for (col_i = 0; col_i < 4; col_i = col_i + 1) begin
                            if (col_i >= row_i) begin
                                // Upper triangle: divide by N (arithmetic right shift)
                                rxx_re[row_i][col_i] <= $signed(accum_re[row_i][col_i]) / $signed(target_snaps);
                                rxx_im[row_i][col_i] <= $signed(accum_im[row_i][col_i]) / $signed(target_snaps);
                            end else begin
                                // Lower triangle: conjugate of upper
                                rxx_re[row_i][col_i] <=  $signed(accum_re[col_i][row_i]) / $signed(target_snaps);
                                rxx_im[row_i][col_i] <= -($signed(accum_im[col_i][row_i]) / $signed(target_snaps));
                            end
                        end
                    end
                    state <= ST_DONE;
                end

                ST_DONE: begin
                    rxx_valid <= 1'b1;  // Stays asserted until next start_pulse clears it
                    busy      <= 1'b0;
                    state     <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
