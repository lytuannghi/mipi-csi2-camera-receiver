// =============================================================================
// File: mipi_csi2_receiver_top.v
// Module: mipi_csi2_receiver_top
// Project: MIPI CSI-2 v2.0 Camera Receiver ASIC Core
// Description: Full-Flow Synthesizable Top-Level ASIC Camera Subsystem Core.
//              Integrates:
//              - 4-Lane D-PHY High-Speed Receiver & Byte Deserializer
//              - CSI-2 Protocol Packet Handler with ECC Correction & CRC-16
//              - RAW10 Byte-to-Pixel Depacker (10-bit & 8-bit Normalized)
//              - 2D Image Signal Processor (ISP):
//                * 3x3 Line Buffer Sliding Window Matrix
//                * 3x3 Gaussian Spatial Smoothing Filter
//                * Dual-Axis 3x3 Sobel Gradient Edge Detection Engine
//              - AXI4-Stream Video Master Output Interface (RGB888 with SOF/EOL)
// =============================================================================

`timescale 1ns / 1ps

module mipi_csi2_receiver_top (
    input  wire        byte_clk,     // D-PHY Parallel Byte Clock (100 MHz - 250 MHz)
    input  wire        rst_n,

    // D-PHY High-Speed 4-Lane Physical Inputs
    input  wire        hs_enable,
    input  wire [7:0]  lane0_data,
    input  wire [7:0]  lane1_data,
    input  wire [7:0]  lane2_data,
    input  wire [7:0]  lane3_data,

    // ISP Configuration & Control
    input  wire [1:0]  isp_mode,       // 0: Passthrough RAW8, 1: Gaussian Smoothing, 2: Sobel Edge Map
    input  wire [7:0]  edge_threshold, // Sobel edge binarization threshold

    // Status & Diagnostics
    output wire        status_frame_start,
    output wire        status_frame_end,
    output wire [15:0] status_frame_num,
    output wire        status_ecc_corrected,
    output wire        status_ecc_error,
    output wire        status_crc_error,

    // AXI4-Stream Video Master Interface
    output wire        m_axis_video_tvalid,
    input  wire        m_axis_video_tready,
    output wire [23:0] m_axis_video_tdata, // RGB888 {R, G, B}
    output wire        m_axis_video_tuser,  // Start-of-Frame (SOF)
    output wire        m_axis_video_tlast   // End-of-Line (EOL)
);

    // D-PHY Lane 0 Deserializer (Primary Lane for Protocol Packets)
    wire lane0_valid;
    wire [7:0] lane0_byte;
    wire lane0_sot;
    wire lane0_sync_err;

    mipi_dphy_lane_rx u_lane0_rx (
        .byte_clk     (byte_clk),
        .rst_n        (rst_n),
        .hs_rx_byte_in(lane0_data),
        .hs_enable    (hs_enable),
        .byte_valid   (lane0_valid),
        .byte_data    (lane0_byte),
        .sot_detected (lane0_sot),
        .err_sot_sync (lane0_sync_err)
    );

    // CSI-2 Low-Level Packet Handler
    wire csi_fs, csi_fe, csi_ls, csi_le;
    wire [15:0] csi_fnum;
    wire csi_payload_valid;
    wire [7:0] csi_payload_byte;
    wire [5:0] csi_data_type;
    wire [1:0] csi_vc;

    csi2_packet_handler u_packet_handler (
        .clk            (byte_clk),
        .rst_n          (rst_n),
        .byte_valid     (lane0_valid),
        .byte_data      (lane0_byte),
        .sot_detected   (lane0_sot),
        .frame_start    (csi_fs),
        .frame_end      (csi_fe),
        .line_start     (csi_ls),
        .line_end       (csi_le),
        .frame_num      (csi_fnum),
        .payload_valid  (csi_payload_valid),
        .payload_data   (csi_payload_byte),
        .data_type      (csi_data_type),
        .virtual_channel(csi_vc),
        .ecc_corrected  (status_ecc_corrected),
        .ecc_error      (status_ecc_error),
        .crc_error      (status_crc_error)
    );

    assign status_frame_start = csi_fs;
    assign status_frame_end   = csi_fe;
    assign status_frame_num   = csi_fnum;

    // RAW10 Byte-to-Pixel Depacker
    wire depack_valid;
    wire [9:0] pixel_10bit;
    wire [7:0] pixel_8bit;

    raw10_depacker u_raw10_depacker (
        .clk        (byte_clk),
        .rst_n      (rst_n),
        .in_valid   (csi_payload_valid),
        .in_byte    (csi_payload_byte),
        .pixel_valid(depack_valid),
        .pixel_out  (pixel_10bit),
        .pixel_8bit (pixel_8bit)
    );

    // 3x3 Sliding Window Matrix Generator
    wire win_valid;
    wire [7:0] p00, p01, p02;
    wire [7:0] p10, p11, p12;
    wire [7:0] p20, p21, p22;

    isp_sliding_window_3x3 #(
        .LINE_WIDTH(640)
    ) u_window (
        .clk         (byte_clk),
        .rst_n       (rst_n),
        .pixel_valid (depack_valid),
        .pixel_in    (pixel_8bit),
        .window_valid(win_valid),
        .p00(p00), .p01(p01), .p02(p02),
        .p10(p10), .p11(p11), .p12(p12),
        .p20(p20), .p21(p21), .p22(p22)
    );

    // 3x3 Gaussian Spatial Filter
    wire gauss_valid;
    wire [7:0] gauss_pixel;

    isp_gaussian_filter u_gaussian (
        .clk           (byte_clk),
        .rst_n         (rst_n),
        .in_valid      (win_valid),
        .p00(p00), .p01(p01), .p02(p02),
        .p10(p10), .p11(p11), .p12(p12),
        .p20(p20), .p21(p21), .p22(p22),
        .out_valid     (gauss_valid),
        .filtered_pixel(gauss_pixel)
    );

    // 3x3 Dual-Axis Sobel Edge Detector
    wire sobel_valid;
    wire [7:0] sobel_edge_mag;
    wire is_edge_flag;

    isp_sobel_edge u_sobel (
        .clk           (byte_clk),
        .rst_n         (rst_n),
        .in_valid      (win_valid),
        .p00(p00), .p01(p01), .p02(p02),
        .p10(p10), .p11(p11), .p12(p12),
        .p20(p20), .p21(p21), .p22(p22),
        .edge_threshold(edge_threshold),
        .out_valid     (sobel_valid),
        .edge_magnitude(sobel_edge_mag),
        .is_edge       (is_edge_flag)
    );

    // ISP Output Multiplexer
    reg       active_pixel_valid;
    reg [7:0] active_pixel_data;

    always @(*) begin
        case (isp_mode)
            2'd0: begin // RAW Passthrough
                active_pixel_valid = depack_valid;
                active_pixel_data  = pixel_8bit;
            end
            2'd1: begin // Gaussian Smoothing
                active_pixel_valid = gauss_valid;
                active_pixel_data  = gauss_pixel;
            end
            2'd2: begin // Sobel Edge Detection
                active_pixel_valid = sobel_valid;
                active_pixel_data  = sobel_edge_mag;
            end
            default: begin
                active_pixel_valid = depack_valid;
                active_pixel_data  = pixel_8bit;
            end
        endcase
    end

    // AXI4-Stream Video Output Master Interface
    axis_video_out #(
        .LINE_WIDTH(640)
    ) u_axis_out (
        .clk                (byte_clk),
        .rst_n              (rst_n),
        .frame_start_pulse  (csi_fs),
        .line_end_pulse     (csi_le),
        .pixel_valid        (active_pixel_valid),
        .pixel_data         (active_pixel_data),
        .m_axis_video_tvalid(m_axis_video_tvalid),
        .m_axis_video_tready(m_axis_video_tready),
        .m_axis_video_tdata (m_axis_video_tdata),
        .m_axis_video_tuser (m_axis_video_tuser),
        .m_axis_video_tlast (m_axis_video_tlast)
    );

endmodule
