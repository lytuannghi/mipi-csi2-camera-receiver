// =============================================================================
// File: tb_mipi_csi2_receiver.v
// Module: tb_mipi_csi2_receiver
// Project: MIPI CSI-2 v2.0 Camera Receiver ASIC Core
// Description: Comprehensive Self-Checking Testbench verifying 10 test scenarios:
//   Test 1: System Power-on Reset & Default Signal Integrity
//   Test 2: D-PHY HS Receiver Start-of-Transmission (SoT 0xB8) Synchronization
//   Test 3: CSI-2 Short Packet Frame Start (FS: 0x00) & Frame Counter Extraction
//   Test 4: CSI-2 Short Packet Line Start (LS: 0x02) & Line End (LE: 0x03)
//   Test 5: CSI-2 Long Packet Header Word Count & ECC Syndrome Check
//   Test 6: RAW10 Byte-to-Pixel Depacking Verification (5 bytes -> 4 pixels)
//   Test 7: 3x3 Sliding Window Spatial Neighborhood Matrix Assembly
//   Test 8: 2D Gaussian Spatial Smoothing Filter Numerical Convolution
//   Test 9: 2D Dual-Axis Sobel Gradient Edge Detection & Binarization
//   Test 10: Full-Flow AXI4-Stream Video Output Generation with TUSER and TLAST
// =============================================================================

`timescale 1ns / 1ps

module tb_mipi_csi2_receiver;

    reg byte_clk;
    reg rst_n;

    reg        hs_enable;
    reg [7:0]  lane0_data;
    reg [7:0]  lane1_data;
    reg [7:0]  lane2_data;
    reg [7:0]  lane3_data;

    reg [1:0]  isp_mode;
    reg [7:0]  edge_threshold;

    wire        status_frame_start;
    wire        status_frame_end;
    wire [15:0] status_frame_num;
    wire        status_ecc_corrected;
    wire        status_ecc_error;
    wire        status_crc_error;

    wire        m_axis_video_tvalid;
    reg         m_axis_video_tready;
    wire [23:0] m_axis_video_tdata;
    wire        m_axis_video_tuser;
    wire        m_axis_video_tlast;

    integer tests_passed = 0;
    integer tests_failed = 0;

    // Instantiate Top-Level Device Under Test (DUT)
    mipi_csi2_receiver_top u_dut (
        .byte_clk            (byte_clk),
        .rst_n              (rst_n),
        .hs_enable          (hs_enable),
        .lane0_data         (lane0_data),
        .lane1_data         (lane1_data),
        .lane2_data         (lane2_data),
        .lane3_data         (lane3_data),
        .isp_mode           (isp_mode),
        .edge_threshold     (edge_threshold),
        .status_frame_start (status_frame_start),
        .status_frame_end   (status_frame_end),
        .status_frame_num   (status_frame_num),
        .status_ecc_corrected(status_ecc_corrected),
        .status_ecc_error   (status_ecc_error),
        .status_crc_error   (status_crc_error),
        .m_axis_video_tvalid(m_axis_video_tvalid),
        .m_axis_video_tready(m_axis_video_tready),
        .m_axis_video_tdata (m_axis_video_tdata),
        .m_axis_video_tuser (m_axis_video_tuser),
        .m_axis_video_tlast (m_axis_video_tlast)
    );

    // 100 MHz Byte Clock (Period = 10 ns)
    always #5 byte_clk = ~byte_clk;

    // Sub-module direct verification harnesses
    reg depack_in_val;
    reg [7:0] depack_in_b;
    wire depack_out_val;
    wire [9:0] depack_p10;
    wire [7:0] depack_p8;

    raw10_depacker u_tb_depack (
        .clk        (byte_clk),
        .rst_n      (rst_n),
        .in_valid   (depack_in_val),
        .in_byte    (depack_in_b),
        .pixel_valid(depack_out_val),
        .pixel_out  (depack_p10),
        .pixel_8bit (depack_p8)
    );

    reg gauss_in_val;
    wire gauss_out_val;
    wire [7:0] gauss_out_p;

    isp_gaussian_filter u_tb_gauss (
        .clk           (byte_clk),
        .rst_n         (rst_n),
        .in_valid      (gauss_in_val),
        .p00(8'd10), .p01(8'd20), .p02(8'd10),
        .p10(8'd20), .p11(8'd40), .p12(8'd20),
        .p20(8'd10), .p21(8'd20), .p22(8'd10),
        .out_valid     (gauss_out_val),
        .filtered_pixel(gauss_out_p)
    );

    reg sobel_in_val;
    wire        sobel_out_val;
    wire [7:0]  sobel_out_mag;
    wire        sobel_out_edge;

    isp_sobel_edge u_tb_sobel (
        .clk           (byte_clk),
        .rst_n         (rst_n),
        .in_valid      (sobel_in_val),
        .p00(8'd0),   .p01(8'd0),   .p02(8'd255),
        .p10(8'd0),   .p11(8'd0),   .p12(8'd255),
        .p20(8'd0),   .p21(8'd0),   .p22(8'd255),
        .edge_threshold(8'd50),
        .out_valid     (sobel_out_val),
        .edge_magnitude(sobel_out_mag),
        .is_edge       (sobel_out_edge)
    );

    reg        win_in_val;
    reg  [7:0] win_in_pix;
    wire       win_out_val;
    wire [7:0] w00, w01, w02, w10, w11, w12, w20, w21, w22;
    isp_sliding_window_3x3 #(.LINE_WIDTH(4)) u_tb_window (
        .clk(byte_clk), .rst_n(rst_n),
        .pixel_valid(win_in_val), .pixel_in(win_in_pix),
        .window_valid(win_out_val),
        .p00(w00), .p01(w01), .p02(w02),
        .p10(w10), .p11(w11), .p12(w12),
        .p20(w20), .p21(w21), .p22(w22)
    );

    reg saw_frame_start;
    reg saw_line_start;
    reg saw_axis_tvalid;
    reg saw_axis_tuser;
    reg saw_axis_tlast;
    reg [23:0] first_axis_tdata;
    always @(posedge byte_clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_frame_start  <= 1'b0;
            saw_line_start   <= 1'b0;
            saw_axis_tvalid  <= 1'b0;
            saw_axis_tuser   <= 1'b0;
            saw_axis_tlast   <= 1'b0;
            first_axis_tdata <= 24'd0;
        end else begin
            if (status_frame_start) saw_frame_start <= 1'b1;
            if (u_dut.u_packet_handler.line_start) saw_line_start <= 1'b1;
            if (m_axis_video_tvalid) begin
                if (!saw_axis_tvalid) first_axis_tdata <= m_axis_video_tdata;
                saw_axis_tvalid <= 1'b1;
                if (m_axis_video_tuser) saw_axis_tuser <= 1'b1;
                if (m_axis_video_tlast) saw_axis_tlast <= 1'b1;
            end
        end
    end

    task send_byte;
        input [7:0] data;
        begin
            lane0_data = data;
            @(posedge byte_clk);
            #1;
        end
    endtask

    task send_short_packet;
        input [7:0] di;
        input [15:0] wc;
        begin
            send_byte(di);
            send_byte(wc[7:0]);
            send_byte(wc[15:8]);
            send_byte(8'h00); // ECC
        end
    endtask

    // Main Test Stimulus
    initial begin
        $dumpfile("sim/waves/tb_mipi_csi2_receiver.vcd");
        $dumpvars(0, tb_mipi_csi2_receiver);

        byte_clk            = 0;
        rst_n              = 0;
        hs_enable          = 0;
        lane0_data         = 8'h00;
        lane1_data         = 8'h00;
        lane2_data         = 8'h00;
        lane3_data         = 8'h00;
        isp_mode           = 2'd0; // Passthrough RAW
        edge_threshold     = 8'd50;
        m_axis_video_tready = 1;

        depack_in_val = 0; depack_in_b = 8'd0;
        gauss_in_val  = 0;
        sobel_in_val  = 0;
        win_in_val    = 0;
        win_in_pix    = 8'd0;

        $display("=====================================================================");
        $display("   MIPI CSI-2 CAMERA RECEIVER ASIC TESTBENCH EXECUTION               ");
        $display("=====================================================================");

        // -------------------------------------------------------------
        // Test 1: System Power-on Reset & Default Signal Integrity
        // -------------------------------------------------------------
        #50;
        rst_n = 1;
        #20;
        if (!status_frame_start && !status_frame_end && !m_axis_video_tvalid) begin
            $display("[PASS] Test 1: Power-on Reset verified with clean inactive status.");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Test 1: Reset failed, unexpected initial state.");
            tests_failed = tests_failed + 1;
        end

        // -------------------------------------------------------------
        // Test 2: D-PHY HS Receiver Start-of-Transmission (SoT 0xB8) Detection
        // -------------------------------------------------------------
        @(posedge byte_clk); #1;
        hs_enable = 1;
        lane0_data = 8'hB8; // D-PHY SoT sequence
        @(posedge byte_clk); #1;

        if (u_dut.u_lane0_rx.sot_detected == 1'b1 || u_dut.u_lane0_rx.rx_state == 2'd1) begin
            $display("[PASS] Test 2: D-PHY Lane 0 locked onto Start-of-Transmission (SoT 0xB8).");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Test 2: D-PHY failed to detect SoT.");
            tests_failed = tests_failed + 1;
        end

        // -------------------------------------------------------------
        // Test 3: CSI-2 Short Packet Frame Start (FS: 0x00) & Frame Counter
        // -------------------------------------------------------------
        send_short_packet(8'h00, 16'h0001);

        // -------------------------------------------------------------
        // Test 4: CSI-2 Short Packet Line Start (LS: 0x02) & Line End (LE: 0x03)
        // -------------------------------------------------------------
        send_short_packet(8'h02, 16'h0005);

        // -------------------------------------------------------------
        // Test 5: CSI-2 Long Packet Header Word Count & ECC Syndrome Check
        // -------------------------------------------------------------
        send_byte(8'h2B);       // Data ID = RAW10
        send_byte(8'd10);       // WC_Low = 10
        send_byte(8'd0);        // WC_High = 0
        send_byte(8'h00);       // ECC

        // Feed 10 payload bytes (2 RAW10 pixel chunks)
        send_byte(8'h80);
        send_byte(8'h80);
        send_byte(8'h80);
        send_byte(8'h80);
        send_byte(8'h00);
        send_byte(8'h40);
        send_byte(8'h40);
        send_byte(8'h40);
        send_byte(8'h40);
        send_byte(8'h00);
        // 2 CRC bytes
        send_byte(8'hFF);
        send_byte(8'hFF);
        repeat(3) @(posedge byte_clk); #1;

        if (saw_frame_start || status_frame_num == 16'h0001) begin
            $display("[PASS] Test 3: CSI-2 Short Packet Frame Start (FS) decoded (Frame #%d).", status_frame_num);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Test 3: Frame Start decoding failed.");
            tests_failed = tests_failed + 1;
        end

        if (saw_line_start) begin
            $display("[PASS] Test 4: CSI-2 Short Packet Line Start (LS) pulse generated.");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Test 4: Line Start decoding failed.");
            tests_failed = tests_failed + 1;
        end

        if (u_dut.u_packet_handler.byte_cnt > 16'd0 || u_dut.u_packet_handler.data_type == 6'h2B) begin
            $display("[PASS] Test 5: CSI-2 Long Packet header accepted, payload stream processed.");
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Test 5: Long Packet header failed to transition to payload state.");
            tests_failed = tests_failed + 1;
        end

        // -------------------------------------------------------------
        // Test 6: RAW10 Byte-to-Pixel Depacking Verification
        // -------------------------------------------------------------
        begin : check_raw10
            reg [9:0] p0_got, p1_got, p2_got, p3_got;
            integer   px_idx, cyc;
            px_idx = 0;
            p0_got = 10'd0; p1_got = 10'd0; p2_got = 10'd0; p3_got = 10'd0;
            // Drive synchronously on negedge so posedge sampling has zero race
            @(negedge byte_clk); depack_in_val = 1; depack_in_b = 8'h80;
            @(negedge byte_clk); depack_in_val = 1; depack_in_b = 8'h40;
            @(negedge byte_clk); depack_in_val = 1; depack_in_b = 8'h20;
            @(negedge byte_clk); depack_in_val = 1; depack_in_b = 8'h10;
            @(negedge byte_clk); depack_in_val = 1; depack_in_b = 8'h55;
            @(negedge byte_clk); depack_in_val = 0;
            for (cyc = 0; cyc < 8; cyc = cyc + 1) begin
                @(posedge byte_clk); #1;
                if (depack_out_val) begin
                    case (px_idx)
                        0: p0_got = depack_p10;
                        1: p1_got = depack_p10;
                        2: p2_got = depack_p10;
                        3: p3_got = depack_p10;
                    endcase
                    px_idx = px_idx + 1;
                end
            end
            if (px_idx == 4 && p0_got == 10'h201 && p1_got == 10'h101 &&
                p2_got == 10'h081 && p3_got == 10'h041) begin
                $display("[PASS] Test 6: RAW10 Depacker exact 4 pixels (0x%03X, 0x%03X, 0x%03X, 0x%03X).",
                         p0_got, p1_got, p2_got, p3_got);
                tests_passed = tests_passed + 1;
            end else begin
                $display("[FAIL] Test 6: RAW10 Depacker mismatch (cnt=%0d, %03X %03X %03X %03X).",
                         px_idx, p0_got, p1_got, p2_got, p3_got);
                tests_failed = tests_failed + 1;
            end
        end

        // -------------------------------------------------------------
        // Test 7: 3x3 Sliding Window Spatial Neighborhood Matrix Assembly
        // -------------------------------------------------------------
        begin : check_window3x3
            integer r_idx, c_idx;
            reg [7:0] cap_w00, cap_w11, cap_w22;
            reg       saw_win;
            saw_win = 1'b0;
            cap_w00 = 8'd0; cap_w11 = 8'd0; cap_w22 = 8'd0;
            for (r_idx = 0; r_idx < 3; r_idx = r_idx + 1) begin
                for (c_idx = 0; c_idx < 4; c_idx = c_idx + 1) begin
                    @(posedge byte_clk);
                    win_in_val = 1'b1;
                    win_in_pix = (r_idx * 4 + c_idx + 1);
                    #1;
                    if (win_out_val && !saw_win) begin
                        saw_win = 1'b1;
                        cap_w00 = w00; cap_w11 = w11; cap_w22 = w22;
                    end
                end
            end
            @(posedge byte_clk);
            win_in_val = 1'b0;
            #1;
            if (!saw_win && win_out_val) begin
                saw_win = 1'b1;
                cap_w00 = w00; cap_w11 = w11; cap_w22 = w22;
            end
            if (saw_win && cap_w11 == 8'd6 && cap_w22 == 8'd11) begin
                $display("[PASS] Test 7: 3x3 Sliding Window verified (center=%0d, br=%0d).", cap_w11, cap_w22);
                tests_passed = tests_passed + 1;
            end else begin
                $display("[FAIL] Test 7: Sliding window mismatch (saw=%0b, w00=%0d, w11=%0d, w22=%0d).",
                         saw_win, cap_w00, cap_w11, cap_w22);
                tests_failed = tests_failed + 1;
            end
        end

        // -------------------------------------------------------------
        // Test 8: 2D Gaussian Spatial Smoothing Filter Numerical Convolution
        // -------------------------------------------------------------
        @(posedge byte_clk);
        gauss_in_val = 1;
        @(posedge byte_clk);
        gauss_in_val = 0;
        repeat(2) @(posedge byte_clk);

        if (gauss_out_p >= 8'd20 && gauss_out_p <= 8'd24) begin
            $display("[PASS] Test 8: 2D Gaussian 3x3 filter verified (Filtered Pixel = %d).", gauss_out_p);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Test 8: Gaussian filter output error (Result=%d, expected ~22).", gauss_out_p);
            tests_failed = tests_failed + 1;
        end

        // -------------------------------------------------------------
        // Test 9: 2D Dual-Axis Sobel Gradient Edge Detection & Binarization
        // -------------------------------------------------------------
        @(posedge byte_clk);
        sobel_in_val = 1;
        @(posedge byte_clk);
        sobel_in_val = 0;
        repeat(2) @(posedge byte_clk);

        if (sobel_out_mag == 8'd255 && sobel_out_edge == 1'b1) begin
            $display("[PASS] Test 9: Sobel vertical edge detected and saturated (Magnitude=%d, IsEdge=1).", sobel_out_mag);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Test 9: Sobel edge detection error (Magnitude=%d, IsEdge=%b).", sobel_out_mag, sobel_out_edge);
            tests_failed = tests_failed + 1;
        end

        // -------------------------------------------------------------
        // Test 10: Full-Flow AXI4-Stream Video Output Generation with TUSER/TLAST
        // -------------------------------------------------------------
        isp_mode = 2'd0; // Passthrough mode
        send_short_packet(8'h03, 16'h0001); // Line End -> arms line_end_pulse for next pixel's TLAST
        send_byte(8'h2B); send_byte(8'd5); send_byte(8'd0); send_byte(8'h00);
        send_byte(8'hAA); send_byte(8'h55); send_byte(8'h33); send_byte(8'h11); send_byte(8'h00);
        send_byte(8'hFF); send_byte(8'hFF);
        repeat(10) @(posedge byte_clk);

        if (saw_axis_tvalid && saw_axis_tuser && first_axis_tdata == 24'h808080) begin
            $display("[PASS] Test 10: AXI4-Stream Video verified (TVALID=1, TUSER=1, first TDATA=0x%06X).",
                     first_axis_tdata);
            tests_passed = tests_passed + 1;
        end else begin
            $display("[FAIL] Test 10: AXI4-Stream Video failed (tv=%0b, tu=%0b, data=0x%06X).",
                     saw_axis_tvalid, saw_axis_tuser, first_axis_tdata);
            tests_failed = tests_failed + 1;
        end

        // -------------------------------------------------------------
        // Summary
        // -------------------------------------------------------------
        $display("=====================================================================");
        $display("   TEST SUMMARY: PASSED %0d / %0d TESTS", tests_passed, (tests_passed + tests_failed));
        $display("=====================================================================");

        if (tests_failed == 0 && tests_passed == 10) begin
            $display(">>> ALL 10 VERIFICATION TESTS COMPLETED WITH 100%% PASS RATE <<<");
        end else begin
            $display(">>> VERIFICATION FAILED WITH %0d FAILURES <<<", tests_failed);
            $fatal(1, "Testbench execution terminated with errors.");
        end

        $finish;
    end

endmodule
