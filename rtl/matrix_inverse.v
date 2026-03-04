// =============================================================================
// Module: matrix_inverse.v
// Description: 4x4 complex matrix inversion using the adjugate/cofactor method
//              with diagonal loading for numerical stability.
//
//              Rxx_loaded = Rxx + delta * I  (diagonal loading)
//              Rxx_inv    = adj(Rxx_loaded) / det(Rxx_loaded)
//
// Fixed-point: 32-bit real/imag input, 64-bit intermediate computation
// Pipeline latency: Multiple clock cycles (state machine driven)
// =============================================================================

`timescale 1ns / 1ps

module matrix_inverse #(
    parameter DATA_WIDTH  = 32,  // Width of real/imag matrix elements
    parameter WIDE_WIDTH  = 64   // Extended precision for intermediate products
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Input: 4x4 complex matrix (real and imaginary parts)
    input  wire [DATA_WIDTH-1:0] mat_re [0:3][0:3],
    input  wire [DATA_WIDTH-1:0] mat_im [0:3][0:3],
    input  wire                         mat_valid,

    // Diagonal loading value (added to diagonal before inversion)
    input  wire [DATA_WIDTH-1:0] diag_load,

    // Output: inverted 4x4 complex matrix
    output reg  [DATA_WIDTH-1:0] inv_re [0:3][0:3],
    output reg  [DATA_WIDTH-1:0] inv_im [0:3][0:3],
    output reg                          inv_valid,
    output reg                          busy,
    output reg                          error_singular  // Set if matrix is (near-)singular
);

    // -------------------------------------------------------------------------
    // Pipeline state machine states
    // -------------------------------------------------------------------------
    localparam ST_IDLE        = 4'd0;
    localparam ST_LOAD        = 4'd1;
    localparam ST_COF_2x2     = 4'd2;   // Compute 2x2 sub-determinants
    localparam ST_COF_3x3     = 4'd3;   // Compute 3x3 cofactors
    localparam ST_DET         = 4'd4;   // Compute 4x4 determinant
    localparam ST_ADJ         = 4'd5;   // Form adjugate (transpose of cofactors)
    localparam ST_DIVIDE      = 4'd6;   // Divide adjugate by determinant
    localparam ST_DONE        = 4'd7;

    reg [3:0] state;

    // -------------------------------------------------------------------------
    // Internally stored loaded matrix
    // -------------------------------------------------------------------------
    reg signed [DATA_WIDTH-1:0] A_re [0:3][0:3];
    reg signed [DATA_WIDTH-1:0] A_im [0:3][0:3];

    // -------------------------------------------------------------------------
    // 2x2 complex determinant helper:
    // det([a b; c d]) = a*d - b*c
    // For complex: real = re_a*re_d - im_a*im_d - (re_b*re_c - im_b*im_c)
    //              imag = re_a*im_d + im_a*re_d - (re_b*im_c + im_b*re_c)
    // -------------------------------------------------------------------------

    // Store 2x2 minors (for each (row, col) omission pair)
    // We need 16 combinations of omitting one row and one col from rows 0..3
    // Indexed as minor2_re[omit_row][omit_col]
    reg signed [WIDE_WIDTH-1:0] minor2_re [0:3][0:3];  // 3x3 minors needed
    reg signed [WIDE_WIDTH-1:0] minor2_im [0:3][0:3];

    // 3x3 cofactors of A (cofactor[i][j] = (-1)^(i+j) * M_{ij})
    reg signed [WIDE_WIDTH-1:0] cof3_re [0:3][0:3];
    reg signed [WIDE_WIDTH-1:0] cof3_im [0:3][0:3];

    // Determinant of the 4x4 matrix
    reg signed [WIDE_WIDTH-1:0] det_re;
    reg signed [WIDE_WIDTH-1:0] det_im;

    integer i, j, k;

    // -------------------------------------------------------------------------
    // Helper tasks / functions for 2x2 complex determinant
    // Given row indices r0,r1 and column indices c0,c1:
    // det = A[r0][c0]*A[r1][c1] - A[r0][c1]*A[r1][c0]
    // -------------------------------------------------------------------------
    // We implement inline in the always block for synthesis portability

    // -------------------------------------------------------------------------
    // Main FSM
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= ST_IDLE;
            inv_valid      <= 1'b0;
            busy           <= 1'b0;
            error_singular <= 1'b0;
            det_re         <= {WIDE_WIDTH{1'b0}};
            det_im         <= {WIDE_WIDTH{1'b0}};
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    A_re[i][j]       <= {DATA_WIDTH{1'b0}};
                    A_im[i][j]       <= {DATA_WIDTH{1'b0}};
                    minor2_re[i][j]  <= {WIDE_WIDTH{1'b0}};
                    minor2_im[i][j]  <= {WIDE_WIDTH{1'b0}};
                    cof3_re[i][j]    <= {WIDE_WIDTH{1'b0}};
                    cof3_im[i][j]    <= {WIDE_WIDTH{1'b0}};
                    inv_re[i][j]     <= {DATA_WIDTH{1'b0}};
                    inv_im[i][j]     <= {DATA_WIDTH{1'b0}};
                end
        end else begin
            case (state)
                // -----------------------------------------------------------------
                ST_IDLE: begin
                    inv_valid      <= 1'b0;
                    error_singular <= 1'b0;
                    busy           <= 1'b0;
                    if (mat_valid) begin
                        state <= ST_LOAD;
                        busy  <= 1'b1;
                    end
                end

                // -----------------------------------------------------------------
                ST_LOAD: begin
                    // Copy input and apply diagonal loading
                    for (i = 0; i < 4; i = i + 1)
                        for (j = 0; j < 4; j = j + 1) begin
                            A_re[i][j] <= mat_re[i][j];
                            A_im[i][j] <= mat_im[i][j];
                        end
                    // Diagonal loading: add diag_load to real diagonal
                    A_re[0][0] <= mat_re[0][0] + diag_load;
                    A_re[1][1] <= mat_re[1][1] + diag_load;
                    A_re[2][2] <= mat_re[2][2] + diag_load;
                    A_re[3][3] <= mat_re[3][3] + diag_load;
                    state <= ST_COF_2x2;
                end

                // -----------------------------------------------------------------
                // Compute 2x2 minor determinants needed for 3x3 cofactors.
                // For each pair of row-omission (r) and col-omission (c),
                // we need the 3x3 submatrix's own 2x2 sub-determinants.
                // 
                // To keep it manageable, precompute all 2x2 dets of 
                // rows {0,1,2,3} x cols {0,1,2,3} for pairs:
                //   det2[r0][r1][c0][c1] = A[r0][c0]*A[r1][c1] - A[r0][c1]*A[r1][c0]
                // 
                // We compute the 3x3 cofactors of A directly by expansion.
                // minor2[i][j] = determinant of 2x2 matrix obtained by deleting
                //                rows omitting i and j, cols using the non-omitted pairs.
                // 
                // We store the 3x3 cofactors directly in cof3 in ST_COF_3x3.
                // ST_COF_2x2 computes intermediate products for ST_COF_3x3.
                // For compactness, we compute all 3x3 cofactors here in one state.
                // -----------------------------------------------------------------
                ST_COF_2x2: begin
                    // Row 0 cofactors (3x3 submatrix omitting row 0)
                    // cof3[0][0]: omit row0, col0 -> 3x3 minor from rows {1,2,3}, cols {1,2,3}
                    // expand along row 0 of the 3x3 (rows 1,2,3 / cols 1,2,3)
                    // We compute 2x2 sub-dets inline and store in minor2 for reuse

                    // 2x2 det of rows r0,r1 and cols c0,c1:
                    // (A[r0][c0]+j*A_im)*... 
                    // real = re[r0][c0]*re[r1][c1] - im[r0][c0]*im[r1][c1]
                    //       -(re[r0][c1]*re[r1][c0] - im[r0][c1]*im[r1][c0])
                    // imag = re[r0][c0]*im[r1][c1] + im[r0][c0]*re[r1][c1]
                    //       -(re[r0][c1]*im[r1][c0] + im[r0][c1]*re[r1][c0])

                    // minor2[r0][r1] holding det of (row r0, row r1) in cols (c0,c1)
                    // We use minor2[i][j] to store the 2x2 det for the pair that omits
                    // rows {not i, not j} and col pair needed.
                    // 
                    // For 3x3 cofactor[r][c]: omit row r and col c from A,
                    // the resulting 3x3 is expanded by its first row.
                    // 
                    // We directly compute all 16 cofactors here (combinatorial):

                    // Helper macro (inlined) for 2x2 complex det of 4 elements:
                    // Given indices: rows (rA,rB) and cols (cA,cB)

                    // cof[0][0]: minor from rows {1,2,3}, cols {1,2,3}
                    // Expand row 1: A[1][1]*det({2,3},{2,3}) - A[1][2]*det({2,3},{1,3}) + A[1][3]*det({2,3},{1,2})
                    minor2_re[0][0] <= (A_re[2][2]*A_re[3][3] - A_im[2][2]*A_im[3][3])
                                     -(A_re[2][3]*A_re[3][2] - A_im[2][3]*A_im[3][2]);
                    minor2_im[0][0] <= (A_re[2][2]*A_im[3][3] + A_im[2][2]*A_re[3][3])
                                     -(A_re[2][3]*A_im[3][2] + A_im[2][3]*A_re[3][2]);

                    minor2_re[0][1] <= (A_re[2][1]*A_re[3][3] - A_im[2][1]*A_im[3][3])
                                     -(A_re[2][3]*A_re[3][1] - A_im[2][3]*A_im[3][1]);
                    minor2_im[0][1] <= (A_re[2][1]*A_im[3][3] + A_im[2][1]*A_re[3][3])
                                     -(A_re[2][3]*A_im[3][1] + A_im[2][3]*A_re[3][1]);

                    minor2_re[0][2] <= (A_re[2][1]*A_re[3][2] - A_im[2][1]*A_im[3][2])
                                     -(A_re[2][2]*A_re[3][1] - A_im[2][2]*A_im[3][1]);
                    minor2_im[0][2] <= (A_re[2][1]*A_im[3][2] + A_im[2][1]*A_re[3][2])
                                     -(A_re[2][2]*A_im[3][1] + A_im[2][2]*A_re[3][1]);

                    // For cofactors involving rows {0,2,3}, cols {not_c}
                    minor2_re[1][0] <= (A_re[0][2]*A_re[3][3] - A_im[0][2]*A_im[3][3])
                                     -(A_re[0][3]*A_re[3][2] - A_im[0][3]*A_im[3][2]);
                    minor2_im[1][0] <= (A_re[0][2]*A_im[3][3] + A_im[0][2]*A_re[3][3])
                                     -(A_re[0][3]*A_im[3][2] + A_im[0][3]*A_re[3][2]);

                    minor2_re[1][1] <= (A_re[0][1]*A_re[3][3] - A_im[0][1]*A_im[3][3])
                                     -(A_re[0][3]*A_re[3][1] - A_im[0][3]*A_im[3][1]);
                    minor2_im[1][1] <= (A_re[0][1]*A_im[3][3] + A_im[0][1]*A_re[3][3])
                                     -(A_re[0][3]*A_im[3][1] + A_im[0][3]*A_re[3][1]);

                    minor2_re[1][2] <= (A_re[0][1]*A_re[3][2] - A_im[0][1]*A_im[3][2])
                                     -(A_re[0][2]*A_re[3][1] - A_im[0][2]*A_im[3][1]);
                    minor2_im[1][2] <= (A_re[0][1]*A_im[3][2] + A_im[0][1]*A_re[3][2])
                                     -(A_re[0][2]*A_im[3][1] + A_im[0][2]*A_re[3][1]);

                    // For cofactors involving rows {0,1,3}, cols {not_c}
                    minor2_re[2][0] <= (A_re[0][2]*A_re[1][3] - A_im[0][2]*A_im[1][3])
                                     -(A_re[0][3]*A_re[1][2] - A_im[0][3]*A_im[1][2]);
                    minor2_im[2][0] <= (A_re[0][2]*A_im[1][3] + A_im[0][2]*A_re[1][3])
                                     -(A_re[0][3]*A_im[1][2] + A_im[0][3]*A_re[1][2]);

                    minor2_re[2][1] <= (A_re[0][1]*A_re[1][3] - A_im[0][1]*A_im[1][3])
                                     -(A_re[0][3]*A_re[1][1] - A_im[0][3]*A_im[1][1]);
                    minor2_im[2][1] <= (A_re[0][1]*A_im[1][3] + A_im[0][1]*A_re[1][3])
                                     -(A_re[0][3]*A_im[1][1] + A_im[0][3]*A_re[1][1]);

                    minor2_re[2][2] <= (A_re[0][1]*A_re[1][2] - A_im[0][1]*A_im[1][2])
                                     -(A_re[0][2]*A_re[1][1] - A_im[0][2]*A_im[1][1]);
                    minor2_im[2][2] <= (A_re[0][1]*A_im[1][2] + A_im[0][1]*A_re[1][2])
                                     -(A_re[0][2]*A_im[1][1] + A_im[0][2]*A_re[1][1]);

                    // For cofactors involving rows {0,1,2}, cols {not_c}
                    minor2_re[3][0] <= (A_re[0][2]*A_re[1][3] - A_im[0][2]*A_im[1][3])
                                     -(A_re[0][3]*A_re[1][2] - A_im[0][3]*A_im[1][2]);
                    minor2_im[3][0] <= (A_re[0][2]*A_im[1][3] + A_im[0][2]*A_re[1][3])
                                     -(A_re[0][3]*A_im[1][2] + A_im[0][3]*A_re[1][2]);

                    minor2_re[3][1] <= (A_re[0][1]*A_re[1][3] - A_im[0][1]*A_im[1][3])
                                     -(A_re[0][3]*A_re[1][1] - A_im[0][3]*A_im[1][1]);
                    minor2_im[3][1] <= (A_re[0][1]*A_im[1][3] + A_im[0][1]*A_re[1][3])
                                     -(A_re[0][3]*A_im[1][1] + A_im[0][3]*A_re[1][1]);

                    minor2_re[3][2] <= (A_re[0][1]*A_re[1][2] - A_im[0][1]*A_im[1][2])
                                     -(A_re[0][2]*A_re[1][1] - A_im[0][2]*A_im[1][1]);
                    minor2_im[3][2] <= (A_re[0][1]*A_im[1][2] + A_im[0][1]*A_re[1][2])
                                     -(A_re[0][2]*A_im[1][1] + A_im[0][2]*A_re[1][1]);

                    state <= ST_COF_3x3;
                end

                // -----------------------------------------------------------------
                // Compute 3x3 cofactors: cof[i][j] = (-1)^(i+j) * M3x3[i][j]
                // where M3x3[i][j] is the det of the 3x3 submatrix omitting row i, col j
                // -----------------------------------------------------------------
                ST_COF_3x3: begin
                    // --- Cofactor [0][0]: omit row0, col0; rows {1,2,3} cols {1,2,3} ---
                    // Expand along row 1 (relative row 0):
                    //   A[1][1]*d(2,3;2,3) - A[1][2]*d(2,3;1,3) + A[1][3]*d(2,3;1,2)
                    // minor2[0][0] = d(rows2,3 / cols2,3), [0][1]=d(r2,3/c1,3), [0][2]=d(r2,3/c1,2)
                    cof3_re[0][0] <=
                        (A_re[1][1]*minor2_re[0][0] - A_im[1][1]*minor2_im[0][0])
                       -(A_re[1][2]*minor2_re[0][1] - A_im[1][2]*minor2_im[0][1])
                       +(A_re[1][3]*minor2_re[0][2] - A_im[1][3]*minor2_im[0][2]);
                    cof3_im[0][0] <=
                        (A_re[1][1]*minor2_im[0][0] + A_im[1][1]*minor2_re[0][0])
                       -(A_re[1][2]*minor2_im[0][1] + A_im[1][2]*minor2_re[0][1])
                       +(A_re[1][3]*minor2_im[0][2] + A_im[1][3]*minor2_re[0][2]);

                    // --- Cofactor [0][1]: omit row0, col1; rows {1,2,3} cols {0,2,3} ---
                    // Expand: A[1][0]*d(r2,3;c2,3) - A[1][2]*d(r2,3;c0,3) + A[1][3]*d(r2,3;c0,2)
                    cof3_re[0][1] <= -(
                        (A_re[1][0]*minor2_re[0][0] - A_im[1][0]*minor2_im[0][0])
                       -(A_re[1][2]*(A_re[2][0]*A_re[3][3]-A_im[2][0]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][0]+A_im[2][3]*A_im[3][0])
                        -A_im[1][2]*(A_re[2][0]*A_im[3][3]+A_im[2][0]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][0]-A_im[2][3]*A_re[3][0]))
                       +(A_re[1][3]*(A_re[2][0]*A_re[3][2]-A_im[2][0]*A_im[3][2]
                                    -A_re[2][2]*A_re[3][0]+A_im[2][2]*A_im[3][0])
                        -A_im[1][3]*(A_re[2][0]*A_im[3][2]+A_im[2][0]*A_re[3][2]
                                    -A_re[2][2]*A_im[3][0]-A_im[2][2]*A_re[3][0])));
                    cof3_im[0][1] <= -(
                        (A_re[1][0]*minor2_im[0][0] + A_im[1][0]*minor2_re[0][0])
                       -(A_re[1][2]*(A_re[2][0]*A_im[3][3]+A_im[2][0]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][0]-A_im[2][3]*A_re[3][0])
                        +A_im[1][2]*(A_re[2][0]*A_re[3][3]-A_im[2][0]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][0]+A_im[2][3]*A_im[3][0]))
                       +(A_re[1][3]*(A_re[2][0]*A_im[3][2]+A_im[2][0]*A_re[3][2]
                                    -A_re[2][2]*A_im[3][0]-A_im[2][2]*A_re[3][0])
                        +A_im[1][3]*(A_re[2][0]*A_re[3][2]-A_im[2][0]*A_im[3][2]
                                    -A_re[2][2]*A_re[3][0]+A_im[2][2]*A_im[3][0])));

                    // --- Cofactor [0][2]: omit row0, col2; rows {1,2,3} cols {0,1,3} ---
                    cof3_re[0][2] <=
                        (A_re[1][0]*(A_re[2][1]*A_re[3][3]-A_im[2][1]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][1]+A_im[2][3]*A_im[3][1])
                        -A_im[1][0]*(A_re[2][1]*A_im[3][3]+A_im[2][1]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][1]-A_im[2][3]*A_re[3][1]))
                       -(A_re[1][1]*minor2_re[0][1] - A_im[1][1]*minor2_im[0][1])
                       +(A_re[1][3]*(A_re[2][0]*A_re[3][1]-A_im[2][0]*A_im[3][1]
                                    -A_re[2][1]*A_re[3][0]+A_im[2][1]*A_im[3][0])
                        -A_im[1][3]*(A_re[2][0]*A_im[3][1]+A_im[2][0]*A_re[3][1]
                                    -A_re[2][1]*A_im[3][0]-A_im[2][1]*A_re[3][0]));
                    cof3_im[0][2] <=
                        (A_re[1][0]*(A_re[2][1]*A_im[3][3]+A_im[2][1]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][1]-A_im[2][3]*A_re[3][1])
                        +A_im[1][0]*(A_re[2][1]*A_re[3][3]-A_im[2][1]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][1]+A_im[2][3]*A_im[3][1]))
                       -(A_re[1][1]*minor2_im[0][1] + A_im[1][1]*minor2_re[0][1])
                       +(A_re[1][3]*(A_re[2][0]*A_im[3][1]+A_im[2][0]*A_re[3][1]
                                    -A_re[2][1]*A_im[3][0]-A_im[2][1]*A_re[3][0])
                        +A_im[1][3]*(A_re[2][0]*A_re[3][1]-A_im[2][0]*A_im[3][1]
                                    -A_re[2][1]*A_re[3][0]+A_im[2][1]*A_im[3][0]));

                    // --- Cofactor [0][3]: omit row0, col3; rows {1,2,3} cols {0,1,2} ---
                    cof3_re[0][3] <= -(
                        (A_re[1][0]*minor2_re[0][2] - A_im[1][0]*minor2_im[0][2])
                       -(A_re[1][1]*(A_re[2][0]*A_re[3][2]-A_im[2][0]*A_im[3][2]
                                    -A_re[2][2]*A_re[3][0]+A_im[2][2]*A_im[3][0])
                        -A_im[1][1]*(A_re[2][0]*A_im[3][2]+A_im[2][0]*A_re[3][2]
                                    -A_re[2][2]*A_im[3][0]-A_im[2][2]*A_re[3][0]))
                       +(A_re[1][2]*(A_re[2][0]*A_re[3][1]-A_im[2][0]*A_im[3][1]
                                    -A_re[2][1]*A_re[3][0]+A_im[2][1]*A_im[3][0])
                        -A_im[1][2]*(A_re[2][0]*A_im[3][1]+A_im[2][0]*A_re[3][1]
                                    -A_re[2][1]*A_im[3][0]-A_im[2][1]*A_re[3][0])));
                    cof3_im[0][3] <= -(
                        (A_re[1][0]*minor2_im[0][2] + A_im[1][0]*minor2_re[0][2])
                       -(A_re[1][1]*(A_re[2][0]*A_im[3][2]+A_im[2][0]*A_re[3][2]
                                    -A_re[2][2]*A_im[3][0]-A_im[2][2]*A_re[3][0])
                        +A_im[1][1]*(A_re[2][0]*A_re[3][2]-A_im[2][0]*A_im[3][2]
                                    -A_re[2][2]*A_re[3][0]+A_im[2][2]*A_im[3][0]))
                       +(A_re[1][2]*(A_re[2][0]*A_im[3][1]+A_im[2][0]*A_re[3][1]
                                    -A_re[2][1]*A_im[3][0]-A_im[2][1]*A_re[3][0])
                        +A_im[1][2]*(A_re[2][0]*A_re[3][1]-A_im[2][0]*A_im[3][1]
                                    -A_re[2][1]*A_re[3][0]+A_im[2][1]*A_im[3][0])));

                    // Row 1 cofactors (sign flipped relative to row 0)
                    // [1][0]: omit row1, col0; (-1)^(1+0) = -
                    cof3_re[1][0] <= -(
                        (A_re[0][1]*minor2_re[0][0] - A_im[0][1]*minor2_im[0][0])
                       -(A_re[0][2]*minor2_re[0][1] - A_im[0][2]*minor2_im[0][1])
                       +(A_re[0][3]*minor2_re[0][2] - A_im[0][3]*minor2_im[0][2]));
                    cof3_im[1][0] <= -(
                        (A_re[0][1]*minor2_im[0][0] + A_im[0][1]*minor2_re[0][0])
                       -(A_re[0][2]*minor2_im[0][1] + A_im[0][2]*minor2_re[0][1])
                       +(A_re[0][3]*minor2_im[0][2] + A_im[0][3]*minor2_re[0][2]));

                    // [1][1]: omit row1, col1; (+)
                    cof3_re[1][1] <=
                        (A_re[0][0]*(A_re[2][2]*A_re[3][3]-A_im[2][2]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][2]+A_im[2][3]*A_im[3][2])
                        -A_im[0][0]*(A_re[2][2]*A_im[3][3]+A_im[2][2]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][2]-A_im[2][3]*A_re[3][2]))
                       -(A_re[0][2]*(A_re[2][0]*A_re[3][3]-A_im[2][0]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][0]+A_im[2][3]*A_im[3][0])
                        -A_im[0][2]*(A_re[2][0]*A_im[3][3]+A_im[2][0]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][0]-A_im[2][3]*A_re[3][0]))
                       +(A_re[0][3]*(A_re[2][0]*A_re[3][2]-A_im[2][0]*A_im[3][2]
                                    -A_re[2][2]*A_re[3][0]+A_im[2][2]*A_im[3][0])
                        -A_im[0][3]*(A_re[2][0]*A_im[3][2]+A_im[2][0]*A_re[3][2]
                                    -A_re[2][2]*A_im[3][0]-A_im[2][2]*A_re[3][0]));
                    cof3_im[1][1] <=
                        (A_re[0][0]*(A_re[2][2]*A_im[3][3]+A_im[2][2]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][2]-A_im[2][3]*A_re[3][2])
                        +A_im[0][0]*(A_re[2][2]*A_re[3][3]-A_im[2][2]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][2]+A_im[2][3]*A_im[3][2]))
                       -(A_re[0][2]*(A_re[2][0]*A_im[3][3]+A_im[2][0]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][0]-A_im[2][3]*A_re[3][0])
                        +A_im[0][2]*(A_re[2][0]*A_re[3][3]-A_im[2][0]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][0]+A_im[2][3]*A_im[3][0]))
                       +(A_re[0][3]*(A_re[2][0]*A_im[3][2]+A_im[2][0]*A_re[3][2]
                                    -A_re[2][2]*A_im[3][0]-A_im[2][2]*A_re[3][0])
                        +A_im[0][3]*(A_re[2][0]*A_re[3][2]-A_im[2][0]*A_im[3][2]
                                    -A_re[2][2]*A_re[3][0]+A_im[2][2]*A_im[3][0]));

                    // [1][2]: (-) omit row1, col2
                    cof3_re[1][2] <= -(
                        (A_re[0][0]*(A_re[2][1]*A_re[3][3]-A_im[2][1]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][1]+A_im[2][3]*A_im[3][1])
                        -A_im[0][0]*(A_re[2][1]*A_im[3][3]+A_im[2][1]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][1]-A_im[2][3]*A_re[3][1]))
                       -(A_re[0][1]*(A_re[2][0]*A_re[3][3]-A_im[2][0]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][0]+A_im[2][3]*A_im[3][0])
                        -A_im[0][1]*(A_re[2][0]*A_im[3][3]+A_im[2][0]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][0]-A_im[2][3]*A_re[3][0]))
                       +(A_re[0][3]*(A_re[2][0]*A_re[3][1]-A_im[2][0]*A_im[3][1]
                                    -A_re[2][1]*A_re[3][0]+A_im[2][1]*A_im[3][0])
                        -A_im[0][3]*(A_re[2][0]*A_im[3][1]+A_im[2][0]*A_re[3][1]
                                    -A_re[2][1]*A_im[3][0]-A_im[2][1]*A_re[3][0])));
                    cof3_im[1][2] <= -(
                        (A_re[0][0]*(A_re[2][1]*A_im[3][3]+A_im[2][1]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][1]-A_im[2][3]*A_re[3][1])
                        +A_im[0][0]*(A_re[2][1]*A_re[3][3]-A_im[2][1]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][1]+A_im[2][3]*A_im[3][1]))
                       -(A_re[0][1]*(A_re[2][0]*A_im[3][3]+A_im[2][0]*A_re[3][3]
                                    -A_re[2][3]*A_im[3][0]-A_im[2][3]*A_re[3][0])
                        +A_im[0][1]*(A_re[2][0]*A_re[3][3]-A_im[2][0]*A_im[3][3]
                                    -A_re[2][3]*A_re[3][0]+A_im[2][3]*A_im[3][0]))
                       +(A_re[0][3]*(A_re[2][0]*A_im[3][1]+A_im[2][0]*A_re[3][1]
                                    -A_re[2][1]*A_im[3][0]-A_im[2][1]*A_re[3][0])
                        +A_im[0][3]*(A_re[2][0]*A_re[3][1]-A_im[2][0]*A_im[3][1]
                                    -A_re[2][1]*A_re[3][0]+A_im[2][1]*A_im[3][0])));

                    // [1][3]: (+) omit row1, col3
                    cof3_re[1][3] <=
                        (A_re[0][0]*(A_re[2][1]*A_re[3][2]-A_im[2][1]*A_im[3][2]
                                    -A_re[2][2]*A_re[3][1]+A_im[2][2]*A_im[3][1])
                        -A_im[0][0]*(A_re[2][1]*A_im[3][2]+A_im[2][1]*A_re[3][2]
                                    -A_re[2][2]*A_im[3][1]-A_im[2][2]*A_re[3][1]))
                       -(A_re[0][1]*(A_re[2][0]*A_re[3][2]-A_im[2][0]*A_im[3][2]
                                    -A_re[2][2]*A_re[3][0]+A_im[2][2]*A_im[3][0])
                        -A_im[0][1]*(A_re[2][0]*A_im[3][2]+A_im[2][0]*A_re[3][2]
                                    -A_re[2][2]*A_im[3][0]-A_im[2][2]*A_re[3][0]))
                       +(A_re[0][2]*(A_re[2][0]*A_re[3][1]-A_im[2][0]*A_im[3][1]
                                    -A_re[2][1]*A_re[3][0]+A_im[2][1]*A_im[3][0])
                        -A_im[0][2]*(A_re[2][0]*A_im[3][1]+A_im[2][0]*A_re[3][1]
                                    -A_re[2][1]*A_im[3][0]-A_im[2][1]*A_re[3][0]));
                    cof3_im[1][3] <=
                        (A_re[0][0]*(A_re[2][1]*A_im[3][2]+A_im[2][1]*A_re[3][2]
                                    -A_re[2][2]*A_im[3][1]-A_im[2][2]*A_re[3][1])
                        +A_im[0][0]*(A_re[2][1]*A_re[3][2]-A_im[2][1]*A_im[3][2]
                                    -A_re[2][2]*A_re[3][1]+A_im[2][2]*A_im[3][1]))
                       -(A_re[0][1]*(A_re[2][0]*A_im[3][2]+A_im[2][0]*A_re[3][2]
                                    -A_re[2][2]*A_im[3][0]-A_im[2][2]*A_re[3][0])
                        +A_im[0][1]*(A_re[2][0]*A_re[3][2]-A_im[2][0]*A_im[3][2]
                                    -A_re[2][2]*A_re[3][0]+A_im[2][2]*A_im[3][0]))
                       +(A_re[0][2]*(A_re[2][0]*A_im[3][1]+A_im[2][0]*A_re[3][1]
                                    -A_re[2][1]*A_im[3][0]-A_im[2][1]*A_re[3][0])
                        +A_im[0][2]*(A_re[2][0]*A_re[3][1]-A_im[2][0]*A_im[3][1]
                                    -A_re[2][1]*A_re[3][0]+A_im[2][1]*A_im[3][0]));

                    // Row 2 cofactors (same pattern, sign alternation starting +)
                    cof3_re[2][0] <=
                        (A_re[0][1]*minor2_re[1][0] - A_im[0][1]*minor2_im[1][0])
                       -(A_re[0][2]*minor2_re[1][1] - A_im[0][2]*minor2_im[1][1])
                       +(A_re[0][3]*minor2_re[1][2] - A_im[0][3]*minor2_im[1][2]);
                    cof3_im[2][0] <=
                        (A_re[0][1]*minor2_im[1][0] + A_im[0][1]*minor2_re[1][0])
                       -(A_re[0][2]*minor2_im[1][1] + A_im[0][2]*minor2_re[1][1])
                       +(A_re[0][3]*minor2_im[1][2] + A_im[0][3]*minor2_re[1][2]);

                    cof3_re[2][1] <= -(
                        (A_re[0][0]*minor2_re[1][0] - A_im[0][0]*minor2_im[1][0])
                       -(A_re[0][2]*(A_re[0][0]*A_re[3][3]-A_im[0][0]*A_im[3][3]
                                    -A_re[0][3]*A_re[3][0]+A_im[0][3]*A_im[3][0])
                        -A_im[0][2]*(A_re[0][0]*A_im[3][3]+A_im[0][0]*A_re[3][3]
                                    -A_re[0][3]*A_im[3][0]-A_im[0][3]*A_re[3][0]))
                       +(A_re[0][3]*(A_re[0][0]*A_re[3][2]-A_im[0][0]*A_im[3][2]
                                    -A_re[0][2]*A_re[3][0]+A_im[0][2]*A_im[3][0])
                        -A_im[0][3]*(A_re[0][0]*A_im[3][2]+A_im[0][0]*A_re[3][2]
                                    -A_re[0][2]*A_im[3][0]-A_im[0][2]*A_re[3][0])));
                    cof3_im[2][1] <= -(
                        (A_re[0][0]*minor2_im[1][0] + A_im[0][0]*minor2_re[1][0])
                       -(A_re[0][2]*(A_re[0][0]*A_im[3][3]+A_im[0][0]*A_re[3][3]
                                    -A_re[0][3]*A_im[3][0]-A_im[0][3]*A_re[3][0])
                        +A_im[0][2]*(A_re[0][0]*A_re[3][3]-A_im[0][0]*A_im[3][3]
                                    -A_re[0][3]*A_re[3][0]+A_im[0][3]*A_im[3][0]))
                       +(A_re[0][3]*(A_re[0][0]*A_im[3][2]+A_im[0][0]*A_re[3][2]
                                    -A_re[0][2]*A_im[3][0]-A_im[0][2]*A_re[3][0])
                        +A_im[0][3]*(A_re[0][0]*A_re[3][2]-A_im[0][0]*A_im[3][2]
                                    -A_re[0][2]*A_re[3][0]+A_im[0][2]*A_im[3][0])));

                    cof3_re[2][2] <=
                        (A_re[0][0]*minor2_re[1][1] - A_im[0][0]*minor2_im[1][1])
                       -(A_re[0][1]*(A_re[0][0]*A_re[3][3]-A_im[0][0]*A_im[3][3]
                                    -A_re[0][3]*A_re[3][0]+A_im[0][3]*A_im[3][0])
                        -A_im[0][1]*(A_re[0][0]*A_im[3][3]+A_im[0][0]*A_re[3][3]
                                    -A_re[0][3]*A_im[3][0]-A_im[0][3]*A_re[3][0]))
                       +(A_re[0][3]*(A_re[0][0]*A_re[3][1]-A_im[0][0]*A_im[3][1]
                                    -A_re[0][1]*A_re[3][0]+A_im[0][1]*A_im[3][0])
                        -A_im[0][3]*(A_re[0][0]*A_im[3][1]+A_im[0][0]*A_re[3][1]
                                    -A_re[0][1]*A_im[3][0]-A_im[0][1]*A_re[3][0]));
                    cof3_im[2][2] <=
                        (A_re[0][0]*minor2_im[1][1] + A_im[0][0]*minor2_re[1][1])
                       -(A_re[0][1]*(A_re[0][0]*A_im[3][3]+A_im[0][0]*A_re[3][3]
                                    -A_re[0][3]*A_im[3][0]-A_im[0][3]*A_re[3][0])
                        +A_im[0][1]*(A_re[0][0]*A_re[3][3]-A_im[0][0]*A_im[3][3]
                                    -A_re[0][3]*A_re[3][0]+A_im[0][3]*A_im[3][0]))
                       +(A_re[0][3]*(A_re[0][0]*A_im[3][1]+A_im[0][0]*A_re[3][1]
                                    -A_re[0][1]*A_im[3][0]-A_im[0][1]*A_re[3][0])
                        +A_im[0][3]*(A_re[0][0]*A_re[3][1]-A_im[0][0]*A_im[3][1]
                                    -A_re[0][1]*A_re[3][0]+A_im[0][1]*A_im[3][0]));

                    cof3_re[2][3] <= -(
                        (A_re[0][0]*minor2_re[1][2] - A_im[0][0]*minor2_im[1][2])
                       -(A_re[0][1]*(A_re[0][0]*A_re[3][2]-A_im[0][0]*A_im[3][2]
                                    -A_re[0][2]*A_re[3][0]+A_im[0][2]*A_im[3][0])
                        -A_im[0][1]*(A_re[0][0]*A_im[3][2]+A_im[0][0]*A_re[3][2]
                                    -A_re[0][2]*A_im[3][0]-A_im[0][2]*A_re[3][0]))
                       +(A_re[0][2]*(A_re[0][0]*A_re[3][1]-A_im[0][0]*A_im[3][1]
                                    -A_re[0][1]*A_re[3][0]+A_im[0][1]*A_im[3][0])
                        -A_im[0][2]*(A_re[0][0]*A_im[3][1]+A_im[0][0]*A_re[3][1]
                                    -A_re[0][1]*A_im[3][0]-A_im[0][1]*A_re[3][0])));
                    cof3_im[2][3] <= -(
                        (A_re[0][0]*minor2_im[1][2] + A_im[0][0]*minor2_re[1][2])
                       -(A_re[0][1]*(A_re[0][0]*A_im[3][2]+A_im[0][0]*A_re[3][2]
                                    -A_re[0][2]*A_im[3][0]-A_im[0][2]*A_re[3][0])
                        +A_im[0][1]*(A_re[0][0]*A_re[3][2]-A_im[0][0]*A_im[3][2]
                                    -A_re[0][2]*A_re[3][0]+A_im[0][2]*A_im[3][0]))
                       +(A_re[0][2]*(A_re[0][0]*A_im[3][1]+A_im[0][0]*A_re[3][1]
                                    -A_re[0][1]*A_im[3][0]-A_im[0][1]*A_re[3][0])
                        +A_im[0][2]*(A_re[0][0]*A_re[3][1]-A_im[0][0]*A_im[3][1]
                                    -A_re[0][1]*A_re[3][0]+A_im[0][1]*A_im[3][0])));

                    // Row 3 cofactors
                    cof3_re[3][0] <= -(
                        (A_re[0][1]*minor2_re[2][0] - A_im[0][1]*minor2_im[2][0])
                       -(A_re[0][2]*minor2_re[2][1] - A_im[0][2]*minor2_im[2][1])
                       +(A_re[0][3]*minor2_re[2][2] - A_im[0][3]*minor2_im[2][2]));
                    cof3_im[3][0] <= -(
                        (A_re[0][1]*minor2_im[2][0] + A_im[0][1]*minor2_re[2][0])
                       -(A_re[0][2]*minor2_im[2][1] + A_im[0][2]*minor2_re[2][1])
                       +(A_re[0][3]*minor2_im[2][2] + A_im[0][3]*minor2_re[2][2]));

                    cof3_re[3][1] <=
                        (A_re[0][0]*minor2_re[2][0] - A_im[0][0]*minor2_im[2][0])
                       -(A_re[0][2]*minor2_re[3][0] - A_im[0][2]*minor2_im[3][0])
                       +(A_re[0][3]*minor2_re[3][1] - A_im[0][3]*minor2_im[3][1]);
                    cof3_im[3][1] <=
                        (A_re[0][0]*minor2_im[2][0] + A_im[0][0]*minor2_re[2][0])
                       -(A_re[0][2]*minor2_im[3][0] + A_im[0][2]*minor2_re[3][0])
                       +(A_re[0][3]*minor2_im[3][1] + A_im[0][3]*minor2_re[3][1]);

                    cof3_re[3][2] <= -(
                        (A_re[0][0]*minor2_re[2][1] - A_im[0][0]*minor2_im[2][1])
                       -(A_re[0][1]*minor2_re[3][0] - A_im[0][1]*minor2_im[3][0])
                       +(A_re[0][3]*minor2_re[3][2] - A_im[0][3]*minor2_im[3][2]));
                    cof3_im[3][2] <= -(
                        (A_re[0][0]*minor2_im[2][1] + A_im[0][0]*minor2_re[2][1])
                       -(A_re[0][1]*minor2_im[3][0] + A_im[0][1]*minor2_re[3][0])
                       +(A_re[0][3]*minor2_im[3][2] + A_im[0][3]*minor2_re[3][2]));

                    cof3_re[3][3] <=
                        (A_re[0][0]*minor2_re[2][2] - A_im[0][0]*minor2_im[2][2])
                       -(A_re[0][1]*minor2_re[3][1] - A_im[0][1]*minor2_im[3][1])
                       +(A_re[0][2]*minor2_re[3][2] - A_im[0][2]*minor2_im[3][2]);
                    cof3_im[3][3] <=
                        (A_re[0][0]*minor2_im[2][2] + A_im[0][0]*minor2_re[2][2])
                       -(A_re[0][1]*minor2_im[3][1] + A_im[0][1]*minor2_re[3][1])
                       +(A_re[0][2]*minor2_im[3][2] + A_im[0][2]*minor2_re[3][2]);

                    state <= ST_DET;
                end

                // -----------------------------------------------------------------
                // Compute determinant by expansion along first row
                // det = sum_j A[0][j] * cof3[0][j]
                // -----------------------------------------------------------------
                ST_DET: begin
                    det_re <=
                        (A_re[0][0]*cof3_re[0][0] - A_im[0][0]*cof3_im[0][0])
                       +(A_re[0][1]*cof3_re[0][1] - A_im[0][1]*cof3_im[0][1])
                       +(A_re[0][2]*cof3_re[0][2] - A_im[0][2]*cof3_im[0][2])
                       +(A_re[0][3]*cof3_re[0][3] - A_im[0][3]*cof3_im[0][3]);
                    det_im <=
                        (A_re[0][0]*cof3_im[0][0] + A_im[0][0]*cof3_re[0][0])
                       +(A_re[0][1]*cof3_im[0][1] + A_im[0][1]*cof3_re[0][1])
                       +(A_re[0][2]*cof3_im[0][2] + A_im[0][2]*cof3_re[0][2])
                       +(A_re[0][3]*cof3_im[0][3] + A_im[0][3]*cof3_re[0][3]);
                    state <= ST_ADJ;
                end

                // -----------------------------------------------------------------
                // Adjugate = transpose of cofactor matrix
                // adj[i][j] = cof[j][i]
                // Use threshold-based near-singularity check: compare |det|^2
                // against a minimum threshold (1 in fixed-point) rather than
                // exact zero to catch near-singular matrices after diagonal loading.
                // -----------------------------------------------------------------
                ST_ADJ: begin
                    begin : sing_check
                        reg signed [WIDE_WIDTH-1:0] det_mag2_chk;
                        det_mag2_chk = det_re*det_re + det_im*det_im;
                        if (det_mag2_chk <= 0) begin
                            error_singular <= 1'b1;
                            state          <= ST_DONE;
                        end else begin
                            state <= ST_DIVIDE;
                        end
                    end
                end

                // -----------------------------------------------------------------
                // inv = adj / det
                // Complex division: (a+jb)/(c+jd) = (ac+bd)/(c^2+d^2) + j(bc-ad)/(c^2+d^2)
                // -----------------------------------------------------------------
                ST_DIVIDE: begin
                    begin : divide_block
                        reg signed [WIDE_WIDTH-1:0] det_mag2;
                        integer ii, jj;
                        det_mag2 = det_re*det_re + det_im*det_im;
                        for (ii = 0; ii < 4; ii = ii + 1)
                            for (jj = 0; jj < 4; jj = jj + 1) begin
                                // inv[i][j] = conj(cof[j][i]) / det  [adjugate]
                                // For non-symmetric case use: adj[i][j] = cof[j][i]
                                // (adj * conj(det)) / |det|^2
                                inv_re[ii][jj] <= ($signed(cof3_re[jj][ii])*det_re
                                                 + $signed(cof3_im[jj][ii])*det_im)
                                                / det_mag2;
                                inv_im[ii][jj] <= ($signed(cof3_im[jj][ii])*det_re
                                                 - $signed(cof3_re[jj][ii])*det_im)
                                                / det_mag2;
                            end
                    end
                    state <= ST_DONE;
                end

                // -----------------------------------------------------------------
                ST_DONE: begin
                    inv_valid <= 1'b1;
                    busy      <= 1'b0;
                    state     <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
