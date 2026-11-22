// =============================================================================
// File: mipi_dphy_lane_rx.v
// Module: mipi_dphy_lane_rx
// Project: MIPI CSI-2 v2.0 Camera Receiver ASIC Core
// Description: D-PHY High-Speed (HS) Receiver & Deserializer for 1 Lane.
//              Detects Start-of-Transmission (SoT) sync pattern (0xB8)
//              and deserializes serial DDR data stream into 8-bit parallel bytes.
// =============================================================================

`timescale 1ns / 1ps

module mipi_dphy_lane_rx (
    input  wire        byte_clk,      // Parallel Byte Clock (e.g. 100 MHz - 250 MHz)
    input  wire        rst_n,

    // High-Speed Serial Bit Inputs (from differential analog frontend)
    input  wire [7:0]  hs_rx_byte_in, // Raw byte stream from serdes
    input  wire        hs_enable,     // HS Mode active (LP-01 -> LP-00 transition complete)

    output reg         byte_valid,    // Valid byte payload strobe
    output reg  [7:0]  byte_data,     // Deserialized byte
    output reg         sot_detected,  // SoT sync 0xB8 pattern detected
    output reg         err_sot_sync   // Synchronization error
);

    // MIPI D-PHY Start-of-Transmission (SoT) Sync Pattern = 8'hB8 (b10111000)
    localparam [7:0] SOT_PATTERN = 8'hB8;

    reg [1:0] rx_state;
    localparam ST_WAIT_SOT = 2'd0;
    localparam ST_ACTIVE   = 2'd1;

    always @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state     <= ST_WAIT_SOT;
            byte_valid   <= 1'b0;
            byte_data    <= 8'd0;
            sot_detected <= 1'b0;
            err_sot_sync <= 1'b0;
        end else begin
            byte_valid   <= 1'b0;
            sot_detected <= 1'b0;
            err_sot_sync <= 1'b0;

            if (!hs_enable) begin
                rx_state <= ST_WAIT_SOT;
            end else begin
                case (rx_state)
                    ST_WAIT_SOT: begin
                        if (hs_rx_byte_in == SOT_PATTERN) begin
                            sot_detected <= 1'b1;
                            rx_state     <= ST_ACTIVE;
                        end else if (hs_rx_byte_in != 8'h00 && hs_rx_byte_in != 8'hFF) begin
                            err_sot_sync <= 1'b1;
                        end
                    end

                    ST_ACTIVE: begin
                        byte_valid <= 1'b1;
                        byte_data  <= hs_rx_byte_in;
                    end
                endcase
            end
        end
    end

endmodule
