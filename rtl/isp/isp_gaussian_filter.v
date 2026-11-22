// =============================================================================
// File: isp_gaussian_filter.v
// Module: isp_gaussian_filter
// Project: MIPI CSI-2 v2.0 Camera Receiver ASIC Core
// Description: 3x3 2D Gaussian Spatial Smoothing Filter.
//              Kernel matrix:
//              [ 1  2  1 ]
//              [ 2  4  2 ] * (1/16)
//              [ 1  2  1 ]
//              Implemented with shift-and-add arithmetic without DSP multipliers.
// =============================================================================

`timescale 1ns / 1ps

module isp_gaussian_filter (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_valid,
    input  wire [7:0]  p00, p01, p02,
    input  wire [7:0]  p10, p11, p12,
    input  wire [7:0]  p20, p21, p22,

    output reg         out_valid,
    output reg  [7:0]  filtered_pixel
);

    reg [11:0] sum_stg1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid      <= 1'b0;
            filtered_pixel <= 8'd0;
            sum_stg1       <= 12'd0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                // Weighted sum = p00 + 2*p01 + p02 + 2*p10 + 4*p11 + 2*p12 + p20 + 2*p21 + p22
                sum_stg1 = {4'd0, p00} + ({3'd0, p01} << 1) + {4'd0, p02} +
                           ({3'd0, p10} << 1) + ({2'd0, p11} << 2) + ({3'd0, p12} << 1) +
                           {4'd0, p20} + ({3'd0, p21} << 1) + {4'd0, p22};

                // Divide by 16 (>>> 4)
                filtered_pixel <= sum_stg1[11:4];
            end
        end
    end

endmodule
