// =============================================================================
// File: axis_video_out.v
// Module: axis_video_out
// Project: MIPI CSI-2 v2.0 Camera Receiver ASIC Core
// Description: AXI4-Stream Video Protocol Master Interface.
//              Formats processed camera frames to standard AXI4-Stream Video:
//              - TUSER: Start-of-Frame (SOF) strobe
//              - TLAST: End-of-Line (EOL) strobe
//              - TDATA: 24-bit RGB888 or 8-bit edge/luminance replicated
// =============================================================================

`timescale 1ns / 1ps

module axis_video_out #(
    parameter LINE_WIDTH = 640
)(
    input  wire        clk,
    input  wire        rst_n,

    // Video Source Inputs
    input  wire        frame_start_pulse, // SOF from packet handler
    input  wire        line_end_pulse,    // EOL from packet handler
    input  wire        pixel_valid,
    input  wire [7:0]  pixel_data,        // Filtered pixel / edge map

    // AXI4-Stream Video Interface
    output reg         m_axis_video_tvalid,
    input  wire        m_axis_video_tready,
    output reg  [23:0] m_axis_video_tdata, // RGB888 {R, G, B}
    output reg         m_axis_video_tuser,  // SOF
    output reg         m_axis_video_tlast   // EOL
);

    reg [9:0] col_counter;
    reg       sof_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axis_video_tvalid <= 1'b0;
            m_axis_video_tdata  <= 24'd0;
            m_axis_video_tuser  <= 1'b0;
            m_axis_video_tlast  <= 1'b0;
            col_counter         <= 10'd0;
            sof_pending         <= 1'b0;
        end else begin
            if (frame_start_pulse)
                sof_pending <= 1'b1;

            if (pixel_valid && (m_axis_video_tready || !m_axis_video_tvalid)) begin
                m_axis_video_tvalid <= 1'b1;
                // Replicate monochrome 8-bit to RGB888
                m_axis_video_tdata  <= {pixel_data, pixel_data, pixel_data};

                // TUSER asserted on the very first pixel of the frame
                if (sof_pending && col_counter == 10'd0) begin
                    m_axis_video_tuser <= 1'b1;
                    sof_pending        <= 1'b0;
                end else begin
                    m_axis_video_tuser <= 1'b0;
                end

                // TLAST asserted on the last pixel of each line
                if (col_counter >= LINE_WIDTH - 1 || line_end_pulse) begin
                    m_axis_video_tlast <= 1'b1;
                    col_counter        <= 10'd0;
                end else begin
                    m_axis_video_tlast <= 1'b0;
                    col_counter        <= col_counter + 10'd1;
                end
            end else if (m_axis_video_tready) begin
                m_axis_video_tvalid <= 1'b0;
                m_axis_video_tuser  <= 1'b0;
                m_axis_video_tlast  <= 1'b0;
            end
        end
    end

endmodule
