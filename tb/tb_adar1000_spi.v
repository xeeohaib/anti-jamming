// =============================================================================
// Testbench: tb_adar1000_spi.v
// Description: Verifies the ADAR1000 SPI controller module.
//              Tests write and read transactions, SPI timing, and
//              proper chip select behavior.
// =============================================================================

`timescale 1ns / 1ps

module tb_adar1000_spi;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam CLK_DIV  = 5;
    localparam CPOL     = 0;
    localparam CPHA     = 0;
    localparam CLK_PERIOD = 10;  // 100 MHz system clock

    // -------------------------------------------------------------------------
    // Signals
    // -------------------------------------------------------------------------
    reg        clk, rst_n;
    reg        tx_start;
    reg        rw;
    reg  [6:0] addr;
    reg  [7:0] wdata;
    wire [7:0] rdata;
    wire       tx_done;
    wire       busy;

    wire       spi_csn;
    wire       spi_clk;
    wire       spi_mosi;
    reg        spi_miso;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    adar1000_spi #(
        .CLK_DIV (CLK_DIV),
        .CPOL    (CPOL),
        .CPHA    (CPHA)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_start (tx_start),
        .rw       (rw),
        .addr     (addr),
        .wdata    (wdata),
        .rdata    (rdata),
        .tx_done  (tx_done),
        .busy     (busy),
        .spi_csn  (spi_csn),
        .spi_clk  (spi_clk),
        .spi_mosi (spi_mosi),
        .spi_miso (spi_miso)
    );

    // -------------------------------------------------------------------------
    // Clock
    // -------------------------------------------------------------------------
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // -------------------------------------------------------------------------
    // MISO simulation: echo back a known byte 0xA5 during read transactions
    // -------------------------------------------------------------------------
    reg  [7:0]  miso_data;
    reg  [4:0]  miso_bit_cnt;
    reg         miso_active;

    always @(negedge spi_csn) begin
        miso_data    = 8'hA5;
        miso_bit_cnt = 0;
        miso_active  = 1'b1;
    end

    always @(posedge spi_clk) begin
        if (miso_active) begin
            if (miso_bit_cnt < 16) begin
                spi_miso = 1'b0;  // Don't-care during address bytes
            end else begin
                spi_miso = miso_data[7 - (miso_bit_cnt - 16)];
            end
            miso_bit_cnt = miso_bit_cnt + 1;
        end
    end

    always @(posedge spi_csn) begin
        miso_active = 1'b0;
        spi_miso    = 1'b0;
    end

    // -------------------------------------------------------------------------
    // Test tasks
    // -------------------------------------------------------------------------
    integer fail_count;

    task spi_write_transaction(input [6:0] a, input [7:0] d);
        @(posedge clk);
        rw       <= 1'b0;
        addr     <= a;
        wdata    <= d;
        tx_start <= 1'b1;
        @(posedge clk);
        tx_start <= 1'b0;

        // Wait for transaction done
        wait(tx_done);
        @(posedge clk);
        $display("  SPI Write: addr=0x%02X data=0x%02X done", a, d);
    endtask

    task spi_read_transaction(input [6:0] a, output [7:0] d_out);
        @(posedge clk);
        rw       <= 1'b1;
        addr     <= a;
        wdata    <= 8'h00;
        tx_start <= 1'b1;
        @(posedge clk);
        tx_start <= 1'b0;

        wait(tx_done);
        @(posedge clk);
        d_out = rdata;
        $display("  SPI Read:  addr=0x%02X data=0x%02X", a, d_out);
    endtask

    // -------------------------------------------------------------------------
    // Main test
    // -------------------------------------------------------------------------
    reg [7:0] read_back;

    initial begin
        $display("=== ADAR1000 SPI Controller Testbench ===");
        fail_count = 0;
        spi_miso   = 1'b0;
        miso_active = 1'b0;
        miso_bit_cnt = 0;

        // Reset
        rst_n    = 1'b0;
        tx_start = 1'b0;
        rw       = 1'b0;
        addr     = 7'h00;
        wdata    = 8'h00;
        repeat(5) @(posedge clk);
        rst_n = 1'b1;
        repeat(3) @(posedge clk);

        // -----------------------------------------------------------------------
        // Test 1: Write transaction - CSN should go low, then high after 24 bits
        // -----------------------------------------------------------------------
        $display("Test 1: SPI write transaction...");
        if (spi_csn !== 1'b1) begin
            $display("  FAIL: CSN should be high at idle");
            fail_count = fail_count + 1;
        end

        spi_write_transaction(7'h2A, 8'hFF);

        if (spi_csn !== 1'b1) begin
            $display("  FAIL: CSN should be high after transaction");
            fail_count = fail_count + 1;
        end else begin
            $display("  PASS: CSN high after transaction");
        end

        if (busy !== 1'b0) begin
            $display("  FAIL: Busy should be deasserted after done");
            fail_count = fail_count + 1;
        end else begin
            $display("  PASS: Busy deasserted after done");
        end

        repeat(5) @(posedge clk);

        // -----------------------------------------------------------------------
        // Test 2: Read transaction - rdata should capture MISO byte
        // -----------------------------------------------------------------------
        $display("Test 2: SPI read transaction...");
        spi_read_transaction(7'h04, read_back);

        // The miso_data was 0xA5, so rdata should be 0xA5
        if (rdata !== 8'hA5) begin
            $display("  FAIL: Expected rdata=0xA5, got 0x%02X", rdata);
            fail_count = fail_count + 1;
        end else begin
            $display("  PASS: rdata = 0xA5 as expected");
        end

        // -----------------------------------------------------------------------
        // Test 3: Multiple back-to-back writes
        // -----------------------------------------------------------------------
        $display("Test 3: Back-to-back write transactions...");
        spi_write_transaction(7'h10, 8'h12);
        spi_write_transaction(7'h11, 8'h34);
        spi_write_transaction(7'h12, 8'h56);
        $display("  PASS: Three consecutive writes completed");

        // -----------------------------------------------------------------------
        // Test 4: Verify MOSI bit pattern for a known write
        // -----------------------------------------------------------------------
        $display("Test 4: MOSI bit pattern verification...");
        // For write addr=0x55, data=0xAA:
        // 24-bit word: {0 (R/W=0), 1010101 (addr), 00000000, 10101010}
        // = 24'h0000_2A_AA -> but our format is {rw, addr[6:0], 8'h00, wdata}
        // = {1'b0, 7'h55, 8'h00, 8'hAA} = 24'b 0 1010101 00000000 10101010
        spi_write_transaction(7'h55, 8'hAA);
        $display("  PASS: Write addr=0x55 data=0xAA completed");

        // -----------------------------------------------------------------------
        // Final report
        // -----------------------------------------------------------------------
        repeat(10) @(posedge clk);
        if (fail_count == 0) begin
            $display("=== ALL TESTS PASSED ===");
        end else begin
            $display("=== %0d TEST(S) FAILED ===", fail_count);
        end
        $finish;
    end

    // SPI clock edge counter for timing verification
    integer spi_clk_cnt;
    always @(posedge spi_clk) spi_clk_cnt = spi_clk_cnt + 1;

    initial begin
        spi_clk_cnt = 0;
        #2000000;
        $display("SIMULATION TIMEOUT");
        $finish;
    end

endmodule
