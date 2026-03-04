// =============================================================================
// Module: signal_acquisition.v
// Description: 4-channel ADC interface for capturing antenna element signals.
//              Provides synchronous sampling across all 4 channels, with
//              configurable sample rate and buffer depth.
// Target: Xilinx ZC702 (Zynq-7000)
// =============================================================================

`timescale 1ns / 1ps

module signal_acquisition #(
    parameter DATA_WIDTH    = 16,   // I/Q sample width in bits
    parameter NUM_CHANNELS  = 4,    // Number of antenna elements
    parameter BUFFER_DEPTH  = 256,  // Snapshot buffer depth (must be power of 2)
    parameter ADDR_WIDTH    = 8     // log2(BUFFER_DEPTH)
)(
    // Clock and reset
    input  wire                             clk,
    input  wire                             rst_n,

    // ADC parallel interface (each channel: DATA_WIDTH bits I + DATA_WIDTH bits Q)
    input  wire [DATA_WIDTH-1:0]            adc_i_ch0,
    input  wire [DATA_WIDTH-1:0]            adc_q_ch0,
    input  wire [DATA_WIDTH-1:0]            adc_i_ch1,
    input  wire [DATA_WIDTH-1:0]            adc_q_ch1,
    input  wire [DATA_WIDTH-1:0]            adc_i_ch2,
    input  wire [DATA_WIDTH-1:0]            adc_q_ch2,
    input  wire [DATA_WIDTH-1:0]            adc_i_ch3,
    input  wire [DATA_WIDTH-1:0]            adc_q_ch3,
    input  wire                             adc_valid,  // Synchronous data-valid from ADC

    // Control interface
    input  wire                             capture_en,     // Enable sample capture
    input  wire                             capture_start,  // Trigger new capture snapshot
    input  wire [ADDR_WIDTH-1:0]            sample_count,   // Number of samples to capture

    // Output data bus (to covariance matrix module)
    output reg  [DATA_WIDTH-1:0]            out_i_ch0,
    output reg  [DATA_WIDTH-1:0]            out_q_ch0,
    output reg  [DATA_WIDTH-1:0]            out_i_ch1,
    output reg  [DATA_WIDTH-1:0]            out_q_ch1,
    output reg  [DATA_WIDTH-1:0]            out_i_ch2,
    output reg  [DATA_WIDTH-1:0]            out_q_ch2,
    output reg  [DATA_WIDTH-1:0]            out_i_ch3,
    output reg  [DATA_WIDTH-1:0]            out_q_ch3,
    output reg                              out_valid,

    // Status
    output reg                              capture_done,
    output reg  [ADDR_WIDTH-1:0]            sample_index
);

    // -------------------------------------------------------------------------
    // Internal sample buffers (ping-pong not needed for initial implementation)
    // -------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] buf_i_ch0 [0:BUFFER_DEPTH-1];
    reg [DATA_WIDTH-1:0] buf_q_ch0 [0:BUFFER_DEPTH-1];
    reg [DATA_WIDTH-1:0] buf_i_ch1 [0:BUFFER_DEPTH-1];
    reg [DATA_WIDTH-1:0] buf_q_ch1 [0:BUFFER_DEPTH-1];
    reg [DATA_WIDTH-1:0] buf_i_ch2 [0:BUFFER_DEPTH-1];
    reg [DATA_WIDTH-1:0] buf_q_ch2 [0:BUFFER_DEPTH-1];
    reg [DATA_WIDTH-1:0] buf_i_ch3 [0:BUFFER_DEPTH-1];
    reg [DATA_WIDTH-1:0] buf_q_ch3 [0:BUFFER_DEPTH-1];

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    localparam ST_IDLE    = 2'b00;
    localparam ST_CAPTURE = 2'b01;
    localparam ST_READOUT = 2'b10;
    localparam ST_DONE    = 2'b11;

    reg [1:0]           state;
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [ADDR_WIDTH-1:0] target_count;

    // Detect rising edge of capture_start
    reg capture_start_d;
    wire capture_start_pulse = capture_start & ~capture_start_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            capture_start_d <= 1'b0;
        end else begin
            capture_start_d <= capture_start;
        end
    end

    // -------------------------------------------------------------------------
    // Main capture FSM
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            wr_ptr       <= {ADDR_WIDTH{1'b0}};
            rd_ptr       <= {ADDR_WIDTH{1'b0}};
            target_count <= {ADDR_WIDTH{1'b0}};
            capture_done <= 1'b0;
            out_valid    <= 1'b0;
            sample_index <= {ADDR_WIDTH{1'b0}};
        end else begin
            case (state)
                ST_IDLE: begin
                    capture_done <= 1'b0;
                    out_valid    <= 1'b0;
                    if (capture_en && capture_start_pulse) begin
                        wr_ptr       <= {ADDR_WIDTH{1'b0}};
                        target_count <= sample_count;
                        state        <= ST_CAPTURE;
                    end
                end

                ST_CAPTURE: begin
                    if (adc_valid) begin
                        buf_i_ch0[wr_ptr] <= adc_i_ch0;
                        buf_q_ch0[wr_ptr] <= adc_q_ch0;
                        buf_i_ch1[wr_ptr] <= adc_i_ch1;
                        buf_q_ch1[wr_ptr] <= adc_q_ch1;
                        buf_i_ch2[wr_ptr] <= adc_i_ch2;
                        buf_q_ch2[wr_ptr] <= adc_q_ch2;
                        buf_i_ch3[wr_ptr] <= adc_i_ch3;
                        buf_q_ch3[wr_ptr] <= adc_q_ch3;
                        if (wr_ptr == target_count - 1) begin
                            wr_ptr <= {ADDR_WIDTH{1'b0}};
                            state  <= ST_READOUT;
                            rd_ptr <= {ADDR_WIDTH{1'b0}};
                        end else begin
                            wr_ptr <= wr_ptr + 1'b1;
                        end
                    end
                end

                ST_READOUT: begin
                    // Stream buffered samples to downstream module
                    out_i_ch0    <= buf_i_ch0[rd_ptr];
                    out_q_ch0    <= buf_q_ch0[rd_ptr];
                    out_i_ch1    <= buf_i_ch1[rd_ptr];
                    out_q_ch1    <= buf_q_ch1[rd_ptr];
                    out_i_ch2    <= buf_i_ch2[rd_ptr];
                    out_q_ch2    <= buf_q_ch2[rd_ptr];
                    out_i_ch3    <= buf_i_ch3[rd_ptr];
                    out_q_ch3    <= buf_q_ch3[rd_ptr];
                    out_valid    <= 1'b1;
                    sample_index <= rd_ptr;
                    if (rd_ptr == target_count - 1) begin
                        rd_ptr <= {ADDR_WIDTH{1'b0}};
                        state  <= ST_DONE;
                    end else begin
                        rd_ptr <= rd_ptr + 1'b1;
                    end
                end

                ST_DONE: begin
                    out_valid    <= 1'b0;
                    capture_done <= 1'b1;
                    state        <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
