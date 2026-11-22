# ==============================================================================
# Synthesis Script - Synopsys Design Compiler / Yosys
# Module: mipi_csi2_receiver_top
# ==============================================================================

set DESIGN_NAME "mipi_csi2_receiver_top"
set RTL_DIR "../rtl"
set REPORT_DIR "./reports"
file mkdir $REPORT_DIR

# Read RTL files
set rtl_files [list \
    "$RTL_DIR/dphy/mipi_dphy_lane_rx.v" \
    "$RTL_DIR/packet/csi2_packet_handler.v" \
    "$RTL_DIR/depacker/raw10_depacker.v" \
    "$RTL_DIR/isp/isp_sliding_window_3x3.v" \
    "$RTL_DIR/isp/isp_gaussian_filter.v" \
    "$RTL_DIR/isp/isp_sobel_edge.v" \
    "$RTL_DIR/axi/axis_video_out.v" \
    "$RTL_DIR/mipi_csi2_receiver_top.v" \
]

foreach file $rtl_files {
    analyze -format verilog $file
}

elaborate $DESIGN_NAME
current_design $DESIGN_NAME

# Load Constraints
read_sdc ./csi2_timing.sdc

# Compile with high effort
compile_ultra -gate_clock

# Generate Reports
report_timing -delay_type max -nworst 10 -max_paths 10 > $REPORT_DIR/timing_report.txt
report_area -hierarchy > $REPORT_DIR/area_report.txt
report_power -hierarchy > $REPORT_DIR/power_report.txt
report_qor > $REPORT_DIR/qor_report.txt

write_file -format verilog -hierarchy -output ./netlist/${DESIGN_NAME}_syn.v
write_file -format ddc -hierarchy -output ./netlist/${DESIGN_NAME}_syn.ddc
