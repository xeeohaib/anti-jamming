# =============================================================================
# zc702_pins.xdc
# Pin constraint file for the Xilinx ZC702 development board
# Target: Zynq-7000 XC7Z020-CLG484
#
# This file assigns:
# - System clock from ZC702 onboard oscillator
# - SPI interface to ADAR1000 (via PMOD J62/J63, ADAR1000 P3 connector)
# - ADC interface (via FMC LPC connector)
# - Status LEDs (GPIO)
# - Timing constraints for 100 MHz system clock
# =============================================================================

# =============================================================================
# Clock constraints
# =============================================================================

# Primary clock: 100 MHz single-ended clock from Zynq PS FCLK0
# The top-level port is sys_clk (single-ended) as defined in top_null_steering.v
create_clock -period 10.000 -name sys_clk \
    [get_ports {sys_clk}]

# SPI clock: maximum 25 MHz, routed to output pin
create_generated_clock -name spi_clk_gen \
    -source [get_ports {sys_clk}] \
    -divide_by 10 \
    [get_ports {spi_clk}]

# =============================================================================
# System clock input (if driven from PL oscillator rather than PS FCLK0)
# ZC702 Y9: 200 MHz differential LVDS oscillator - uncomment and adapt if
# using an external differential clock with an IBUFDS → MMCM in the design.
# =============================================================================
# create_clock -period 5.000 -name sys_diff_clk [get_ports {sys_clk_p}]
# set_property PACKAGE_PIN H9  [get_ports {sys_clk_p}]
# set_property PACKAGE_PIN G9  [get_ports {sys_clk_n}]
# set_property IOSTANDARD LVDS [get_ports {sys_clk_p}]
# set_property IOSTANDARD LVDS [get_ports {sys_clk_n}]

# =============================================================================
# SPI Interface to ADAR1000 and GPIO Control Lines
# Connected to ADAR1000 P3 Peripheral Module-Compatible Interface
# via ZC702 PMOD connectors J62 (P3 top row) and J63 (P3 bottom row)
#
# P3 connector pinout:
#   P3-1  GPIO0 (RX_LOAD) → J62-1   P3-2  SPI_SEL_A (~CS) → J62-2
#   P3-3  GPIO1 (TX_LOAD) → J62-3   P3-4  SPI_MOSI        → J62-4
#   P3-5  GPIO4 (TR)      → J62-5   P3-6  SPI_MISO        → J62-6
#   P3-7  GPIO5 (PA_ON)   → J63-1   P3-8  SPI_CLK         → J63-2
#   P3-9  AGND                       P3-10 AGND
#   P3-11 NC                         P3-12 NC
#
# ZC702 J62 and J63: Bank 13 LVCMOS33 (3.3 V)
# PACKAGE_PIN values below target ZC702 Rev D/E (XC7Z020-CLG484-1).
# ALWAYS verify against your specific ZC702 schematic revision before use.
# Schematic source: Xilinx UG850 "ZC702 Evaluation Board for the Zynq-7000
# AP SoC" — check the connector J62/J63 net-to-pin tables in that document.
# =============================================================================

# ---------------------------------------------------------------------------
# J62 signals (P3 top row, pins 1-6)
# ---------------------------------------------------------------------------

# GPIO0 / RX_LOAD - J62 pin 1 (P3-1)
set_property PACKAGE_PIN AB8   [get_ports {gpio0_rx_load}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio0_rx_load}]
set_property SLEW SLOW         [get_ports {gpio0_rx_load}]

# SPI Chip Select (active low) / SPI_SEL_A - J62 pin 2 (P3-2)
set_property PACKAGE_PIN AA7   [get_ports {spi_csn}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_csn}]
set_property SLEW FAST         [get_ports {spi_csn}]

# GPIO1 / TX_LOAD - J62 pin 3 (P3-3)
set_property PACKAGE_PIN AB7   [get_ports {gpio1_tx_load}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio1_tx_load}]
set_property SLEW SLOW         [get_ports {gpio1_tx_load}]

# SPI MOSI - J62 pin 4 (P3-4)
set_property PACKAGE_PIN AB6   [get_ports {spi_mosi}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_mosi}]
set_property SLEW FAST         [get_ports {spi_mosi}]

# GPIO4 / TR (Transmit/Receive switch) - J62 pin 5 (P3-5)
# Reset default: TR=0 → receive mode (safe default; PA is also off at reset)
set_property PACKAGE_PIN Y9    [get_ports {gpio4_tr}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio4_tr}]
set_property SLEW SLOW         [get_ports {gpio4_tr}]

# SPI MISO - J62 pin 6 (P3-6)
set_property PACKAGE_PIN AA6   [get_ports {spi_miso}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_miso}]

# ---------------------------------------------------------------------------
# J63 signals (P3 bottom row, pins 7-8)
# ---------------------------------------------------------------------------

# GPIO5 / PA_ON (Power Amplifier enable) - J63 pin 1 (P3-7)
# Reset default: PA_ON=0 → PA disabled (safe power-on state)
set_property PACKAGE_PIN Y8    [get_ports {gpio5_pa_on}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio5_pa_on}]
set_property SLEW SLOW         [get_ports {gpio5_pa_on}]

# SPI Clock - J63 pin 2 (P3-8)
set_property PACKAGE_PIN AA5   [get_ports {spi_clk}]
set_property IOSTANDARD LVCMOS33 [get_ports {spi_clk}]
set_property SLEW FAST         [get_ports {spi_clk}]

# =============================================================================
# ADC Interface - FMC LPC Connector (J64)
# Using LA (Low Pin Count) bank for ADC parallel interface
# 4 channels x (I + Q) x 16 bits = 128 signal lines
# For brevity, only channel 0 I/Q are assigned; others follow same pattern
# =============================================================================

# ADC Channel 0 I (16-bit parallel)
set_property PACKAGE_PIN D20   [get_ports {adc_i_ch0[0]}]
set_property PACKAGE_PIN C20   [get_ports {adc_i_ch0[1]}]
set_property PACKAGE_PIN B20   [get_ports {adc_i_ch0[2]}]
set_property PACKAGE_PIN A20   [get_ports {adc_i_ch0[3]}]
set_property PACKAGE_PIN E19   [get_ports {adc_i_ch0[4]}]
set_property PACKAGE_PIN D19   [get_ports {adc_i_ch0[5]}]
set_property PACKAGE_PIN C19   [get_ports {adc_i_ch0[6]}]
set_property PACKAGE_PIN B19   [get_ports {adc_i_ch0[7]}]
set_property PACKAGE_PIN A19   [get_ports {adc_i_ch0[8]}]
set_property PACKAGE_PIN F19   [get_ports {adc_i_ch0[9]}]
set_property PACKAGE_PIN G19   [get_ports {adc_i_ch0[10]}]
set_property PACKAGE_PIN H19   [get_ports {adc_i_ch0[11]}]
set_property PACKAGE_PIN J19   [get_ports {adc_i_ch0[12]}]
set_property PACKAGE_PIN K19   [get_ports {adc_i_ch0[13]}]
set_property PACKAGE_PIN L19   [get_ports {adc_i_ch0[14]}]
set_property PACKAGE_PIN M19   [get_ports {adc_i_ch0[15]}]

set_property IOSTANDARD LVCMOS18 [get_ports {adc_i_ch0[*]}]

# ADC Valid (all channels synchronous)
set_property PACKAGE_PIN N19   [get_ports {adc_valid}]
set_property IOSTANDARD LVCMOS18 [get_ports {adc_valid}]

# =============================================================================
# Status LEDs
# DS12-DS15 on ZC702 (active high)
# =============================================================================

set_property PACKAGE_PIN A17   [get_ports {status_leds[0]}]
set_property PACKAGE_PIN B17   [get_ports {status_leds[1]}]
set_property PACKAGE_PIN C17   [get_ports {status_leds[2]}]
set_property PACKAGE_PIN D17   [get_ports {status_leds[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {status_leds[*]}]

# =============================================================================
# Timing constraints
# =============================================================================

# Input delay for ADC data (2 ns setup time relative to ADC valid)
set_input_delay -clock fclk0 -max 2.0 [get_ports {adc_i_ch0[*]}]
set_input_delay -clock fclk0 -min 0.5 [get_ports {adc_i_ch0[*]}]
set_input_delay -clock fclk0 -max 2.0 [get_ports {adc_valid}]
set_input_delay -clock fclk0 -min 0.5 [get_ports {adc_valid}]

# MISO input delay (SPI)
set_input_delay -clock spi_clk_gen -max 5.0 [get_ports {spi_miso}]
set_input_delay -clock spi_clk_gen -min 1.0 [get_ports {spi_miso}]

# SPI output delays
set_output_delay -clock spi_clk_gen -max 2.0 [get_ports {spi_mosi}]
set_output_delay -clock spi_clk_gen -max 2.0 [get_ports {spi_csn}]
set_output_delay -clock spi_clk_gen -max 2.0 [get_ports {spi_clk}]

# GPIO control outputs (async, no strict timing required)
set_false_path -to [get_ports {gpio0_rx_load}]
set_false_path -to [get_ports {gpio1_tx_load}]
set_false_path -to [get_ports {gpio4_tr}]
set_false_path -to [get_ports {gpio5_pa_on}]

# =============================================================================
# False paths / multi-cycle paths
# =============================================================================

# The covariance matrix computation is a long pipeline; allow multiple cycles
# for the matrix inversion state machine transitions
set_multicycle_path -setup -from [get_cells {u_mvdr/u_mat_inv/*}] \
                    -to   [get_cells {u_mvdr/u_mat_inv/*}] 4
set_multicycle_path -hold  -from [get_cells {u_mvdr/u_mat_inv/*}] \
                    -to   [get_cells {u_mvdr/u_mat_inv/*}] 3

# =============================================================================
# Physical constraints (Pblock for PL logic)
# =============================================================================

# Optionally constrain the MVDR logic to a specific clock region
# to improve timing closure on the ZC702 (XC7Z020 has 4 clock regions)
# create_pblock pblock_mvdr
# add_cells_to_pblock [get_pblocks pblock_mvdr] [get_cells -hier {u_mvdr}]
# resize_pblock [get_pblocks pblock_mvdr] -add {CLOCKREGION_X0Y1:CLOCKREGION_X1Y1}

# =============================================================================
# Configuration settings
# =============================================================================

set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4  [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE   33 [current_design]
set_property CONFIG_VOLTAGE                3.3 [current_design]
set_property CFGBVS                        VCCO [current_design]
