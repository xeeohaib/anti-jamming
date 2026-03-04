# =============================================================================
# build_project.tcl
# Vivado TCL build script for the Anti-Jamming Null Steering System
# Target: Xilinx ZC702 (Zynq-7000 XC7Z020)
#
# Usage:
#   vivado -mode batch -source scripts/build_project.tcl
#   or interactively: source scripts/build_project.tcl
# =============================================================================

set project_name "null_steering"
set project_dir  "[file normalize "../vivado_project"]"
set rtl_dir      "[file normalize "../rtl"]"
set tb_dir       "[file normalize "../tb"]"
set const_dir    "[file normalize "../constraints"]"

# Target device
set part_name    "xc7z020clg484-1"
set board_part   "xilinx.com:zc702:part0:1.4"

# =============================================================================
# Create project
# =============================================================================
puts "Creating Vivado project: $project_name"
create_project $project_name $project_dir -part $part_name -force

# Set board
set_property board_part $board_part [current_project]

# Set simulator
set_property target_simulator XSim [current_project]

# =============================================================================
# Add RTL source files
# =============================================================================
puts "Adding RTL sources..."
set rtl_files [list \
    "$rtl_dir/signal_acquisition.v"   \
    "$rtl_dir/covariance_matrix.v"    \
    "$rtl_dir/matrix_inverse.v"       \
    "$rtl_dir/weight_compute.v"       \
    "$rtl_dir/beam_apply.v"           \
    "$rtl_dir/adar1000_spi.v"         \
    "$rtl_dir/axi_lite_slave.v"       \
    "$rtl_dir/mvdr_beamformer.v"      \
    "$rtl_dir/top_null_steering.v"    \
]

add_files -norecurse $rtl_files
set_property file_type {Verilog} [get_files *.v]

# =============================================================================
# Add testbench files (simulation only)
# =============================================================================
puts "Adding testbench files..."
set tb_files [list \
    "$tb_dir/tb_covariance_matrix.v"  \
    "$tb_dir/tb_mvdr_beamformer.v"    \
    "$tb_dir/tb_adar1000_spi.v"       \
    "$tb_dir/tb_top_null_steering.v"  \
]

add_files -fileset sim_1 -norecurse $tb_files

# Set top-level simulation file
set_property top tb_top_null_steering [get_filesets sim_1]
set_property top_lib {} [get_filesets sim_1]

# =============================================================================
# Add constraints
# =============================================================================
puts "Adding constraints..."
add_files -fileset constrs_1 -norecurse "$const_dir/zc702_pins.xdc"

# =============================================================================
# Create Block Design for Zynq PS
# =============================================================================
puts "Creating Zynq PS block design..."

create_bd_design "zynq_ps_bd"

# Add Zynq Processing System IP
set zynq_ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 processing_system7_0]

# Configure Zynq PS for ZC702
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" } \
    [get_bd_cells processing_system7_0]

# Enable AXI Master port (M_AXI_GP0)
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0 {1}          \
    CONFIG.PCW_M_AXI_GP0_ENABLE_STATIC_REMAP {0} \
] $zynq_ps

# Enable FCLK_CLK0 at 100 MHz
set_property -dict [list \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
    CONFIG.PCW_EN_CLK0_PORT {1}               \
] $zynq_ps

# Enable SPI0 (used for ADAR1000 if driven directly from PS)
set_property -dict [list CONFIG.PCW_SPI0_PERIPHERAL_ENABLE {1} ] $zynq_ps

# Add AXI Interconnect
set axi_ic [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_interconnect_0]
set_property -dict [list CONFIG.NUM_SI {1} CONFIG.NUM_MI {1}] $axi_ic

# Add null steering RTL as a module reference
# Note: This creates a wrapper - in practice, use IP packager for the PL RTL
# For synthesis-only flow, the top_null_steering.v is instantiated directly

# Connect clocks and resets
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
               [get_bd_pins axi_interconnect_0/ACLK]
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
               [get_bd_pins processing_system7_0/M_AXI_GP0_ACLK]

# Make external ports
create_bd_port -dir O -type clk FCLK_CLK0
connect_bd_net [get_bd_pins processing_system7_0/FCLK_CLK0] \
               [get_bd_ports FCLK_CLK0]

# Validate and save block design
validate_bd_design
save_bd_design

# Generate block design wrapper
make_wrapper -files [get_files zynq_ps_bd.bd] -top
add_files -norecurse "$project_dir/${project_name}.srcs/sources_1/bd/zynq_ps_bd/hdl/zynq_ps_bd_wrapper.v"

# =============================================================================
# Set top-level synthesis module
# =============================================================================
set_property top top_null_steering [current_fileset]

# =============================================================================
# Synthesis settings
# =============================================================================
set_property strategy "Vivado Synthesis Defaults" [get_runs synth_1]
set_property -name {STEPS.SYNTH_DESIGN.ARGS.MORE OPTIONS} \
             -value {-retiming} \
             -objects [get_runs synth_1]

# Implementation settings
set_property strategy "Performance_ExplorePostRoutePhysOpt" [get_runs impl_1]

# =============================================================================
# Run synthesis and implementation
# =============================================================================
puts "Starting synthesis..."
launch_runs synth_1 -jobs 4
wait_on_run synth_1

if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    error "Synthesis failed!"
}
puts "Synthesis complete."

puts "Starting implementation..."
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    error "Implementation failed!"
}
puts "Implementation complete."

# =============================================================================
# Generate reports
# =============================================================================
puts "Generating reports..."
open_run impl_1

report_timing_summary -file "$project_dir/timing_summary.rpt"
report_utilization    -file "$project_dir/utilization.rpt"
report_power          -file "$project_dir/power.rpt"

puts "Reports written to $project_dir/"
puts "Build complete! Bitstream: $project_dir/${project_name}.runs/impl_1/top_null_steering.bit"
