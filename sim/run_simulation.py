#!/usr/bin/env python3
"""
Simulation Automation Script for MIPI CSI-2 v2.0 Camera Receiver ASIC
Compiles synthesizable RTL and self-checking testbench using Icarus Verilog and executes VVP.
"""

import subprocess
import sys
import os

def run():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sim_dir = os.path.join(repo_root, "sim")
    waves_dir = os.path.join(sim_dir, "waves")
    os.makedirs(waves_dir, exist_ok=True)

    rtl_files = [
        os.path.join(repo_root, "rtl", "dphy", "mipi_dphy_lane_rx.v"),
        os.path.join(repo_root, "rtl", "packet", "csi2_packet_handler.v"),
        os.path.join(repo_root, "rtl", "depacker", "raw10_depacker.v"),
        os.path.join(repo_root, "rtl", "isp", "isp_sliding_window_3x3.v"),
        os.path.join(repo_root, "rtl", "isp", "isp_gaussian_filter.v"),
        os.path.join(repo_root, "rtl", "isp", "isp_sobel_edge.v"),
        os.path.join(repo_root, "rtl", "axi", "axis_video_out.v"),
        os.path.join(repo_root, "rtl", "mipi_csi2_receiver_top.v"),
    ]

    tb_file = os.path.join(repo_root, "tb", "tb_mipi_csi2_receiver.v")
    out_vvp = os.path.join(sim_dir, "csi2_sim.vvp")

    print("[INFO] Compiling MIPI CSI-2 Camera Receiver ASIC RTL and Testbench...")
    cmd_compile = ["iverilog", "-g2012", "-o", out_vvp, "-I", os.path.join(repo_root, "rtl")] + rtl_files + [tb_file]
    res_compile = subprocess.run(cmd_compile, capture_output=True, text=True)

    if res_compile.returncode != 0:
        print("[ERROR] Compilation failed:")
        print(res_compile.stderr)
        sys.exit(1)
    print("[INFO] Compilation successful!")

    print("[INFO] Running VVP simulation...")
    cmd_run = ["vvp", out_vvp]
    res_run = subprocess.run(cmd_run, cwd=repo_root, capture_output=True, text=True)

    print(res_run.stdout)
    if res_run.stderr:
        print("[STDERR]:", res_run.stderr)

    if "ALL 10 VERIFICATION TESTS COMPLETED WITH 100% PASS RATE" in res_run.stdout:
        print("[SUCCESS] All 10 MIPI CSI-2 test cases PASSED!")
        sys.exit(0)
    else:
        print("[FAILURE] Simulation failed assertion checks.")
        sys.exit(1)

if __name__ == "__main__":
    run()
