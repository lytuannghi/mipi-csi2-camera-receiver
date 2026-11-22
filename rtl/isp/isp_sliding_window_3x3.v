// =============================================================================
// File: isp_sliding_window_3x3.v
// Module: isp_sliding_window_3x3
// Project: MIPI CSI-2 v2.0 Camera Receiver ASIC Core
// Description: 3x3 Sliding Window Matrix Generator with Dual Line Buffers.
//              Provides a continuous 3x3 pixel spatial neighborhood for 2D
//              filtering and Sobel edge detection in streaming video flow.
// =============================================================================

`timescale 1ns / 1ps

module isp_sliding_window_3x3 #(
    parameter LINE_WIDTH = 640 // Horizontal line width in pixels
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        pixel_valid,
    input  wire [7:0]  pixel_in,

    output reg         window_valid,
    output reg  [7:0]  p00, p01, p02, // Top row
    output reg  [7:0]  p10, p11, p12, // Middle row (center: p11)
    output reg  [7:0]  p20, p21, p22  // Bottom row
);

    // Dual Line Buffers (Row 0 and Row 1)
    reg [7:0] line_buf0 [0:LINE_WIDTH-1];
    reg [7:0] line_buf1 [0:LINE_WIDTH-1];

    reg [9:0] col_cnt;
    reg [9:0] row_cnt;

    reg [7:0] lb0_out, lb1_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_cnt      <= 10'd0;
            row_cnt      <= 10'd0;
            window_valid <= 1'b0;
            p00 <= 8'd0; p01 <= 8'd0; p02 <= 8'd0;
            p10 <= 8'd0; p11 <= 8'd0; p12 <= 8'd0;
            p20 <= 8'd0; p21 <= 8'd0; p22 <= 8'd0;
            lb0_out      <= 8'd0;
            lb1_out      <= 8'd0;
        end else begin
            window_valid <= 1'b0;

            if (pixel_valid) begin
                // Read from line buffers
                lb0_out <= line_buf0[col_cnt];
                lb1_out <= line_buf1[col_cnt];

                // Write into line buffers
                line_buf0[col_cnt] <= lb1_out;
                line_buf1[col_cnt] <= pixel_in;

                // Shift 3x3 window horizontally
                p02 <= lb0_out;  p01 <= p02;  p00 <= p01;
                p12 <= lb1_out;  p11 <= p12;  p10 <= p11;
                p22 <= pixel_in; p21 <= p22;  p20 <= p21;

                if (col_cnt >= LINE_WIDTH - 1) begin
                    col_cnt <= 10'd0;
                    row_cnt <= row_cnt + 10'd1;
                end else begin
                    col_cnt <= col_cnt + 10'd1;
                end

                // Window valid once past line 2 and col 2
                if (row_cnt >= 10'd2 && col_cnt >= 10'd2) begin
                    window_valid <= 1'b1;
                end
            end
        end
    end

endmodule
