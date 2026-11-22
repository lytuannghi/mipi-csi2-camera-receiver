// =============================================================================
// File: raw10_depacker.v
// Module: raw10_depacker
// Project: MIPI CSI-2 v2.0 Camera Receiver ASIC Core
// Description: MIPI RAW10 Byte-to-Pixel Unpacking Unit.
//              Converts 5 packed bytes into 4 independent 10-bit pixels:
//              Byte 0: Pixel 0 [9:2]
//              Byte 1: Pixel 1 [9:2]
//              Byte 2: Pixel 2 [9:2]
//              Byte 3: Pixel 3 [9:2]
//              Byte 4: {Pixel 3 [1:0], Pixel 2 [1:0], Pixel 1 [1:0], Pixel 0 [1:0]}
// =============================================================================

`timescale 1ns / 1ps

module raw10_depacker (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        in_valid,
    input  wire [7:0]  in_byte,

    output reg         pixel_valid,
    output reg  [9:0]  pixel_out,    // 10-bit Unpacked Pixel (0 to 1023)
    output wire [7:0]  pixel_8bit    // 8-bit normalized luminance (MSBs)
);

    reg [2:0] byte_idx;
    reg [7:0] b0, b1, b2, b3;

    assign pixel_8bit = pixel_out[9:2];

    // Pixel FIFO / sequencer for 4 unpacked pixels
    reg [9:0] p0, p1, p2, p3;
    reg [1:0] out_seq;
    reg       seq_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_idx    <= 3'd0;
            b0          <= 8'd0;
            b1          <= 8'd0;
            b2          <= 8'd0;
            b3          <= 8'd0;
            p0          <= 10'd0;
            p1          <= 10'd0;
            p2          <= 10'd0;
            p3          <= 10'd0;
            out_seq     <= 2'd0;
            seq_active  <= 1'b0;
            pixel_valid <= 1'b0;
            pixel_out   <= 10'd0;
        end else begin
            pixel_valid <= 1'b0;

            if (seq_active) begin
                pixel_valid <= 1'b1;
                case (out_seq)
                    2'd0: begin pixel_out <= p0; out_seq <= 2'd1; end
                    2'd1: begin pixel_out <= p1; out_seq <= 2'd2; end
                    2'd2: begin pixel_out <= p2; out_seq <= 2'd3; end
                    2'd3: begin pixel_out <= p3; seq_active <= 1'b0; end
                endcase
            end

            if (in_valid) begin
                case (byte_idx)
                    3'd0: begin b0 <= in_byte; byte_idx <= 3'd1; end
                    3'd1: begin b1 <= in_byte; byte_idx <= 3'd2; end
                    3'd2: begin b2 <= in_byte; byte_idx <= 3'd3; end
                    3'd3: begin b3 <= in_byte; byte_idx <= 3'd4; end
                    3'd4: begin
                        // 5th byte contains the 2 LSBs of pixels 0 to 3
                        p0 <= {b0, in_byte[1:0]};
                        p1 <= {b1, in_byte[3:2]};
                        p2 <= {b2, in_byte[5:4]};
                        p3 <= {b3, in_byte[7:6]};
                        seq_active <= 1'b1;
                        out_seq    <= 2'd0;
                        byte_idx   <= 3'd0;
                    end
                    default: byte_idx <= 3'd0;
                endcase
            end
        end
    end

endmodule
