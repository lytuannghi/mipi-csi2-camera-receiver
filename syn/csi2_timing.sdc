# ==============================================================================
# Synopsys Design Constraints (SDC) - MIPI CSI-2 Camera Receiver ASIC Core
# Target Technology: SkyWater 130nm HD Standard Cell Library (1.80V, 25C)
# Operating Condition: Worst-Case Slow-Slow Corner (SSG, 0.9V, 125C)
# Target Frequency: 200.0 MHz (Parallel Byte Clock)
# ==============================================================================

# Primary Byte Clock Definition (Period = 5.0 ns -> 200 MHz)
create_clock -name byte_clk -period 5.000 [get_ports byte_clk]

# Clock Uncertainty & Jitter Modeling
set_clock_uncertainty -setup 0.250 [get_clocks byte_clk]
set_clock_uncertainty -hold  0.100 [get_clocks byte_clk]
set_clock_transition 0.080 [get_clocks byte_clk]

# Input Delays (High-Speed D-PHY PHY interface and configuration)
set_input_delay -clock byte_clk -max 1.200 [get_ports {lane0_data[*] lane1_data[*] lane2_data[*] lane3_data[*] hs_enable}]
set_input_delay -clock byte_clk -min 0.300 [get_ports {lane0_data[*] lane1_data[*] lane2_data[*] lane3_data[*] hs_enable}]
set_input_delay -clock byte_clk -max 1.500 [get_ports {isp_mode[*] edge_threshold[*]}]
set_input_delay -clock byte_clk -min 0.300 [get_ports {isp_mode[*] edge_threshold[*]}]
set_input_delay -clock byte_clk -max 1.000 [get_ports m_axis_video_tready]
set_input_delay -clock byte_clk -min 0.200 [get_ports m_axis_video_tready]

# Output Delays (AXI4-Stream Video Master interface and Status indicators)
set_output_delay -clock byte_clk -max 1.200 [get_ports {m_axis_video_tvalid m_axis_video_tdata[*] m_axis_video_tuser m_axis_video_tlast}]
set_output_delay -clock byte_clk -min 0.200 [get_ports {m_axis_video_tvalid m_axis_video_tdata[*] m_axis_video_tuser m_axis_video_tlast}]
set_output_delay -clock byte_clk -max 1.500 [get_ports {status_frame_start status_frame_end status_frame_num[*] status_ecc_corrected status_ecc_error status_crc_error}]
set_output_delay -clock byte_clk -min 0.200 [get_ports {status_frame_start status_frame_end status_frame_num[*] status_ecc_corrected status_ecc_error status_crc_error}]

# Drive Strength & Load Modeling
set_driving_cell -lib_cell INVX4 [get_ports {lane0_data[*] lane1_data[*] lane2_data[*] lane3_data[*] hs_enable isp_mode[*] edge_threshold[*] m_axis_video_tready}]
set_load 0.025 [all_outputs]

# Multicycle Path & False Path Constraints
set_false_path -from [get_ports rst_n]
