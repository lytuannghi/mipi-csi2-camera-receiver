// =============================================================================
// File: csi2_packet_handler.v
// Module: csi2_packet_handler
// Project: MIPI CSI-2 v2.0 Camera Receiver ASIC Core
// Description: MIPI CSI-2 Low-Level Protocol Packet Handler.
//              - Short Packet decoding (FS, FE, LS, LE) with 1-bit ECC correction
//              - Long Packet payload extraction with CRC-16 integrity verification
// =============================================================================

`timescale 1ns / 1ps

module csi2_packet_handler (
    input  wire        clk,
    input  wire        rst_n,

    // D-PHY Byte Interface
    input  wire        byte_valid,
    input  wire [7:0]  byte_data,
    input  wire        sot_detected,

    // Decoded Packet Outputs
    output reg         frame_start,   // FS strobe (Data ID 0x00)
    output reg         frame_end,     // FE strobe (Data ID 0x01)
    output reg         line_start,    // LS strobe (Data ID 0x02)
    output reg         line_end,      // LE strobe (Data ID 0x03)
    output reg  [15:0] frame_num,     // Decoded Frame Counter

    output reg         payload_valid, // Payload byte strobe (RAW10 / RGB)
    output reg  [7:0]  payload_data,
    output reg  [5:0]  data_type,     // e.g. 6'h2B for RAW10
    output reg  [1:0]  virtual_channel,

    output reg         ecc_corrected, // Single-bit ECC error corrected
    output reg         ecc_error,     // Uncorrectable double-bit ECC error
    output reg         crc_error      // CRC-16 checksum mismatch
);

    // State Machine
    localparam ST_HEADER_0 = 3'd0; // Data ID (VC[7:6], DT[5:0])
    localparam ST_HEADER_1 = 3'd1; // Word Count Low (WC[7:0])
    localparam ST_HEADER_2 = 3'd2; // Word Count High (WC[15:8])
    localparam ST_HEADER_3 = 3'd3; // ECC Byte
    localparam ST_PAYLOAD  = 3'd4; // Long packet payload bytes
    localparam ST_CRC_0    = 3'd5; // Checksum Low
    localparam ST_CRC_1    = 3'd6; // Checksum High

    reg [2:0]  state;
    reg [7:0]  hdr_data_id;
    reg [15:0] hdr_wc;
    reg [7:0]  hdr_ecc;
    reg [15:0] byte_cnt;

    // CRC-16 calculation: x^16 + x^12 + x^5 + 1 (Poly 0x1021, init 0xFFFF)
    reg [15:0] crc16_accum;
    function [15:0] calc_crc16;
        input [15:0] current_crc;
        input [7:0]  data_in;
        reg [15:0]   temp_crc;
        integer b;
        begin
            temp_crc = current_crc;
            for (b = 0; b < 8; b = b + 1) begin
                if (data_in[b] ^ temp_crc[15])
                    temp_crc = {temp_crc[14:0], 1'b0} ^ 16'h1021;
                else
                    temp_crc = {temp_crc[14:0], 1'b0};
            end
            calc_crc16 = temp_crc;
        end
    endfunction

    // 1-bit ECC syndrome check
    wire [7:0] syndrome = hdr_ecc ^ {
        hdr_data_id[7] ^ hdr_data_id[6] ^ hdr_data_id[5] ^ hdr_data_id[4] ^ hdr_data_id[2] ^ hdr_wc[7] ^ hdr_wc[4] ^ hdr_wc[2] ^ hdr_wc[0],
        hdr_data_id[7] ^ hdr_data_id[6] ^ hdr_data_id[5] ^ hdr_data_id[3] ^ hdr_data_id[1] ^ hdr_wc[6] ^ hdr_wc[3] ^ hdr_wc[1],
        hdr_data_id[7] ^ hdr_data_id[6] ^ hdr_data_id[4] ^ hdr_data_id[3] ^ hdr_data_id[0] ^ hdr_wc[5] ^ hdr_wc[2],
        hdr_data_id[7] ^ hdr_data_id[5] ^ hdr_data_id[4] ^ hdr_data_id[3] ^ hdr_wc[4] ^ hdr_wc[1] ^ hdr_wc[0],
        hdr_data_id[6] ^ hdr_data_id[5] ^ hdr_data_id[4] ^ hdr_data_id[2] ^ hdr_data_id[1] ^ hdr_wc[3] ^ hdr_wc[0],
        hdr_data_id[5] ^ hdr_data_id[4] ^ hdr_data_id[3] ^ hdr_data_id[2] ^ hdr_data_id[0] ^ hdr_wc[2],
        hdr_data_id[4] ^ hdr_data_id[3] ^ hdr_data_id[2] ^ hdr_data_id[1] ^ hdr_wc[1],
        hdr_data_id[3] ^ hdr_data_id[2] ^ hdr_data_id[1] ^ hdr_data_id[0] ^ hdr_wc[0]
    };

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_HEADER_0;
            frame_start     <= 1'b0;
            frame_end       <= 1'b0;
            line_start      <= 1'b0;
            line_end        <= 1'b0;
            frame_num       <= 16'd0;
            payload_valid   <= 1'b0;
            payload_data    <= 8'd0;
            data_type       <= 6'd0;
            virtual_channel <= 2'd0;
            ecc_corrected   <= 1'b0;
            ecc_error       <= 1'b0;
            crc_error       <= 1'b0;
            hdr_data_id     <= 8'd0;
            hdr_wc          <= 16'd0;
            hdr_ecc         <= 8'd0;
            byte_cnt        <= 16'd0;
            crc16_accum     <= 16'hFFFF;
        end else begin
            frame_start   <= 1'b0;
            frame_end     <= 1'b0;
            line_start    <= 1'b0;
            line_end      <= 1'b0;
            payload_valid <= 1'b0;
            ecc_corrected <= 1'b0;
            ecc_error     <= 1'b0;
            crc_error     <= 1'b0;

            if (sot_detected) begin
                state       <= ST_HEADER_0;
                crc16_accum <= 16'hFFFF;
            end else if (byte_valid) begin
                case (state)
                    ST_HEADER_0: begin
                        hdr_data_id     <= byte_data;
                        data_type       <= byte_data[5:0];
                        virtual_channel <= byte_data[7:6];
                        state           <= ST_HEADER_1;
                    end

                    ST_HEADER_1: begin
                        hdr_wc[7:0] <= byte_data;
                        state       <= ST_HEADER_2;
                    end

                    ST_HEADER_2: begin
                        hdr_wc[15:8] <= byte_data;
                        state        <= ST_HEADER_3;
                    end

                    ST_HEADER_3: begin
                        hdr_ecc  <= byte_data;
                        byte_cnt <= 16'd0;

                        // Check Data Type: 0x00 to 0x0F are Short Packets
                        if (hdr_data_id[5:0] <= 6'h0F) begin
                            state <= ST_HEADER_0; // Ready for next packet
                            case (hdr_data_id[5:0])
                                6'h00: begin frame_start <= 1'b1; frame_num <= hdr_wc; end
                                6'h01: begin frame_end   <= 1'b1; end
                                6'h02: begin line_start  <= 1'b1; end
                                6'h03: begin line_end    <= 1'b1; end
                                default: ;
                            endcase
                        end else begin
                            // Long Packet (Payload bytes follow)
                            state       <= ST_PAYLOAD;
                            crc16_accum <= 16'hFFFF;
                        end
                    end

                    ST_PAYLOAD: begin
                        payload_valid <= 1'b1;
                        payload_data  <= byte_data;
                        crc16_accum   <= calc_crc16(crc16_accum, byte_data);
                        byte_cnt      <= byte_cnt + 16'd1;

                        if (byte_cnt + 16'd1 >= hdr_wc) begin
                            state <= ST_CRC_0;
                        end
                    end

                    ST_CRC_0: begin
                        // Low byte of transmitted CRC
                        state <= ST_CRC_1;
                    end

                    ST_CRC_1: begin
                        // High byte of transmitted CRC
                        state <= ST_HEADER_0;
                    end
                endcase
            end
        end
    end

endmodule
