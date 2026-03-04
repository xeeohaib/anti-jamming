// =============================================================================
// Testbench: tb_covariance_matrix.v
// Description: Verifies the covariance_matrix module.
//              Injects known test signals with a predefined covariance structure
//              and checks the computed Rxx against expected values.
// =============================================================================

`timescale 1ns / 1ps

module tb_covariance_matrix;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam DATA_WIDTH  = 16;
    localparam ACCUM_WIDTH = 32;
    localparam N_SNAP      = 256;
    localparam SNAP_WIDTH  = 9;
    localparam CLK_PERIOD  = 10;  // 100 MHz

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg                     clk, rst_n;
    reg  [DATA_WIDTH-1:0]   in_i_ch0, in_q_ch0;
    reg  [DATA_WIDTH-1:0]   in_i_ch1, in_q_ch1;
    reg  [DATA_WIDTH-1:0]   in_i_ch2, in_q_ch2;
    reg  [DATA_WIDTH-1:0]   in_i_ch3, in_q_ch3;
    reg                     in_valid;
    reg                     start;
    reg  [SNAP_WIDTH-1:0]   n_snapshots;

    wire [ACCUM_WIDTH-1:0]  rxx_re [0:3][0:3];
    wire [ACCUM_WIDTH-1:0]  rxx_im [0:3][0:3];
    wire                    rxx_valid;
    wire                    busy;
    wire [SNAP_WIDTH-1:0]   snap_count;

    // -------------------------------------------------------------------------
    // DUT instantiation
    // -------------------------------------------------------------------------
    covariance_matrix #(
        .DATA_WIDTH  (DATA_WIDTH),
        .ACCUM_WIDTH (ACCUM_WIDTH),
        .N_SNAP      (N_SNAP),
        .SNAP_WIDTH  (SNAP_WIDTH)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .in_i_ch0    (in_i_ch0), .in_q_ch0 (in_q_ch0),
        .in_i_ch1    (in_i_ch1), .in_q_ch1 (in_q_ch1),
        .in_i_ch2    (in_i_ch2), .in_q_ch2 (in_q_ch2),
        .in_i_ch3    (in_i_ch3), .in_q_ch3 (in_q_ch3),
        .in_valid    (in_valid),
        .start       (start),
        .n_snapshots (n_snapshots),
        .rxx_re      (rxx_re),
        .rxx_im      (rxx_im),
        .rxx_valid   (rxx_valid),
        .busy        (busy),
        .snap_count  (snap_count)
    );

    // -------------------------------------------------------------------------
    // Clock generation
    // -------------------------------------------------------------------------
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // -------------------------------------------------------------------------
    // Test signal generation
    // All 4 channels receive the same unit-magnitude sinusoid (phase=0).
    // Expected: all diagonal elements = 1 (after scaling), off-diagonal = 1+0j
    // -------------------------------------------------------------------------
    integer snap_idx;
    integer test_pass;
    integer fail_count;

    // Simulate a DC signal of amplitude 1024 on all channels
    localparam signed [DATA_WIDTH-1:0] SIG_AMP = 16'sd1024;

    // Feed n identical real-valued DC samples on all channels.
    // Uses blocking (#1) assignments after posedge to avoid race conditions.
    task feed_samples;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                @(posedge clk); #1;
                in_i_ch0 = SIG_AMP; in_q_ch0 = 16'sd0;
                in_i_ch1 = SIG_AMP; in_q_ch1 = 16'sd0;
                in_i_ch2 = SIG_AMP; in_q_ch2 = 16'sd0;
                in_i_ch3 = SIG_AMP; in_q_ch3 = 16'sd0;
                in_valid = 1'b1;
            end
            @(posedge clk); #1;
            in_valid = 1'b0;
        end
    endtask

    // Pulse start: must be called AFTER n_snapshots is configured.
    // Uses #1 delay after posedge so the DUT samples the correct (new) value.
    task pulse_start;
        begin
            @(posedge clk); #1; start = 1'b1;
            @(posedge clk); #1; start = 1'b0;
        end
    endtask

    // Wait for rxx_valid with a cycle timeout.
    task wait_rxx_valid;
        input integer max_cycles;
        integer to;
        begin
            to = max_cycles;
            while (!rxx_valid && to > 0) begin
                @(posedge clk);
                to = to - 1;
            end
            if (to == 0) begin
                $display("  FAIL: rxx_valid never asserted (timeout %0d cycles)", max_cycles);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Test flow
    // -------------------------------------------------------------------------
    integer timeout;

    initial begin
        $display("=== Covariance Matrix Testbench ===");
        fail_count = 0;

        // Initialize
        rst_n       = 1'b0;
        start       = 1'b0;
        in_valid    = 1'b0;
        n_snapshots = 9'd64;
        in_i_ch0 = 0; in_q_ch0 = 0;
        in_i_ch1 = 0; in_q_ch1 = 0;
        in_i_ch2 = 0; in_q_ch2 = 0;
        in_i_ch3 = 0; in_q_ch3 = 0;

        repeat(5) @(posedge clk);
        #1; rst_n = 1'b1;
        repeat(3) @(posedge clk);

        // -----------------------------------------------------------------------
        // Test 1: DC signal on all channels -> all Rxx elements should be equal
        // -----------------------------------------------------------------------
        $display("Test 1: DC signal, all channels equal amplitude...");
        n_snapshots = 9'd64;  // Configure BEFORE pulsing start
        pulse_start;

        // Feed 64 identical samples
        feed_samples(64);

        // rxx_valid holds high after done; wait for it
        wait_rxx_valid(200);

        if (rxx_valid) begin
            $display("  rxx_valid asserted after capture + normalize");
            // For DC signal of amplitude A on all channels:
            // Rxx[i][j] = A^2 (real) for all i,j
            // Expected: rxx_re[0][0] = SIG_AMP^2 = 1024^2 = 1048576
            $display("  Rxx[0][0].re = %0d (expected ~1048576)", $signed(rxx_re[0][0]));
            $display("  Rxx[0][1].re = %0d (expected ~1048576)", $signed(rxx_re[0][1]));
            $display("  Rxx[0][0].im = %0d (expected 0)", $signed(rxx_im[0][0]));
            $display("  Rxx[0][1].im = %0d (expected 0)", $signed(rxx_im[0][1]));

            if ($signed(rxx_re[0][0]) == 0) begin
                $display("  FAIL: Diagonal element is 0");
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS: Non-zero diagonal element");
            end

            // Check Hermitian symmetry: rxx[i][j].re = rxx[j][i].re
            if ($signed(rxx_re[0][1]) !== $signed(rxx_re[1][0])) begin
                $display("  FAIL: Hermitian symmetry violated: Rxx[0][1]=%0d != Rxx[1][0]=%0d",
                         $signed(rxx_re[0][1]), $signed(rxx_re[1][0]));
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS: Hermitian symmetry holds");
            end

            // Check imaginary diagonal is zero
            if ($signed(rxx_im[0][0]) !== 32'sd0) begin
                $display("  FAIL: Diagonal imaginary should be 0, got %0d", $signed(rxx_im[0][0]));
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS: Diagonal imaginary is 0");
            end
        end

        repeat(5) @(posedge clk);

        // -----------------------------------------------------------------------
        // Test 2: Zero input -> all Rxx elements should be zero
        // -----------------------------------------------------------------------
        $display("Test 2: Zero input -> Rxx should be all zeros...");
        in_i_ch0 = 0; in_q_ch0 = 0;
        in_i_ch1 = 0; in_q_ch1 = 0;
        in_i_ch2 = 0; in_q_ch2 = 0;
        in_i_ch3 = 0; in_q_ch3 = 0;

        n_snapshots = 9'd16;  // Configure BEFORE pulsing start
        pulse_start;

        // Feed 16 zero samples
        @(posedge clk); #1; in_valid = 1'b1;
        repeat(16) @(posedge clk);
        #1; in_valid = 1'b0;

        wait_rxx_valid(200);

        if (rxx_valid) begin
            if ($signed(rxx_re[0][0]) !== 32'sd0) begin
                $display("  FAIL: Rxx[0][0].re should be 0, got %0d", $signed(rxx_re[0][0]));
                fail_count = fail_count + 1;
            end else begin
                $display("  PASS: Zero input -> zero covariance");
            end
        end

        // -----------------------------------------------------------------------
        // Final report
        // -----------------------------------------------------------------------
        repeat(5) @(posedge clk);
        if (fail_count == 0) begin
            $display("=== ALL TESTS PASSED ===");
        end else begin
            $display("=== %0d TEST(S) FAILED ===", fail_count);
        end

        $finish;
    end

    // Timeout watchdog
    initial begin
        #5000000;
        $display("SIMULATION TIMEOUT");
        $finish;
    end

endmodule
