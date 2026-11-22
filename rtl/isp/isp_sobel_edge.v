// =============================================================================
// File: isp_sobel_edge.v
// Module: isp_sobel_edge
// Project: MIPI CSI-2 v2.0 Camera Receiver ASIC Core
// Description: Dual-Axis 3x3 Sobel Gradient Convolution & Edge Detection Engine.
//              Gx Kernel:
//              [-1  0 +1]
//              [-2  0 +2]
//              [-1  0 +1]
//              Gy Kernel:
//              [+1 +2 +1]
//              [ 0  0  0]
//              [-1 -2 -1]
//              Gradient Magnitude = |Gx| + |Gy| (L1 Manhattan Norm)
// =============================================================================

`timescale 1ns / 1ps

module isp_sobel_edge (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_valid,
    input  wire [7:0]  p00, p01, p02,
    input  wire [7:0]  p10, p11, p12,
    input  wire [7:0]  p20, p21, p22,
    input  wire [7:0]  edge_threshold,

    output reg         out_valid,
    output reg  [7:0]  edge_magnitude,
    output reg         is_edge
);

    reg signed [10:0] gx;
    reg signed [10:0] gy;
    reg [10:0] abs_gx;
    reg [10:0] abs_gy;
    reg [11:0] grad_sum;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid      <= 1'b0;
            edge_magnitude <= 8'd0;
            is_edge        <= 1'b0;
            gx             <= 11'sd0;
            gy             <= 11'sd0;
            abs_gx         <= 11'd0;
            abs_gy         <= 11'd0;
            grad_sum       <= 12'd0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                // Gx = (p02 + 2*p12 + p22) - (p00 + 2*p10 + p20)
                gx = ($signed({3'd0, p02}) + ($signed({3'd0, p12}) <<< 1) + $signed({3'd0, p22})) -
                     ($signed({3'd0, p00}) + ($signed({3'd0, p10}) <<< 1) + $signed({3'd0, p20}));

                // Gy = (p00 + 2*p01 + p02) - (p20 + 2*p21 + p22)
                gy = ($signed({3'd0, p00}) + ($signed({3'd0, p01}) <<< 1) + $signed({3'd0, p02})) -
                     ($signed({3'd0, p20}) + ($signed({3'd0, p21}) <<< 1) + $signed({3'd0, p22}));

                abs_gx = (gx < 0) ? -gx : gx;
                abs_gy = (gy < 0) ? -gy : gy;

                grad_sum = abs_gx + abs_gy;

                // Clamp to 8-bit saturating range [0, 255]
                if (grad_sum > 12'd255)
                    edge_magnitude <= 8'd255;
                else
                    edge_magnitude <= grad_sum[7:0];

                // Thresholding
                is_edge <= (grad_sum >= {4'd0, edge_threshold});
            end
        end
    end

endmodule
