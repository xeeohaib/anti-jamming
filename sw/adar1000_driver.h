/**
 * @file adar1000_driver.h
 * @brief ADAR1000 beamformer IC driver for Zynq PS (ARM Cortex-A9)
 *
 * Provides register map definitions, initialization, and control functions
 * for the Analog Devices ADAR1000 4-channel X/Ka-band beamformer IC.
 *
 * Target: Xilinx ZC702 (Zynq-7000)
 */

#ifndef ADAR1000_DRIVER_H
#define ADAR1000_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * ADAR1000 Register Map
 * ============================================================================ */

/* Interface control */
#define ADAR1000_INTERFACE_CFG_A    0x000
#define ADAR1000_INTERFACE_CFG_B    0x001
#define ADAR1000_DEV_CFG            0x002
#define ADAR1000_CHIP_TYPE          0x003
#define ADAR1000_PRODUCT_ID_H       0x004
#define ADAR1000_PRODUCT_ID_L       0x005
#define ADAR1000_SCRATCH_PAD        0x00A
#define ADAR1000_SPI_REV            0x00B
#define ADAR1000_VENDOR_ID_H        0x00C
#define ADAR1000_VENDOR_ID_L        0x00D
#define ADAR1000_INTERFACE_CFG_C    0x010

/* Transmit channel registers (4 channels: 0-3) */
#define ADAR1000_CH_TX_PHASE_I(ch)  (0x010 + (ch) * 0x10)  /* I phase adjust */
#define ADAR1000_CH_TX_PHASE_Q(ch)  (0x011 + (ch) * 0x10)  /* Q phase adjust */
#define ADAR1000_CH_TX_GAIN(ch)     (0x012 + (ch) * 0x10)  /* Gain */
#define ADAR1000_CH_TX_VM_I(ch)     (0x013 + (ch) * 0x10)  /* Vector mod I */
#define ADAR1000_CH_TX_VM_Q(ch)     (0x014 + (ch) * 0x10)  /* Vector mod Q */

/* Receive channel registers (4 channels: 0-3) */
#define ADAR1000_CH_RX_PHASE_I(ch)  (0x050 + (ch) * 0x10)  /* I phase adjust */
#define ADAR1000_CH_RX_PHASE_Q(ch)  (0x051 + (ch) * 0x10)  /* Q phase adjust */
#define ADAR1000_CH_RX_GAIN(ch)     (0x052 + (ch) * 0x10)  /* Gain */
#define ADAR1000_CH_RX_VM_I(ch)     (0x053 + (ch) * 0x10)  /* Vector mod I */
#define ADAR1000_CH_RX_VM_Q(ch)     (0x054 + (ch) * 0x10)  /* Vector mod Q */

/* Beam memory registers */
#define ADAR1000_CH1_RX_BEAM_POS_0  0x100
#define ADAR1000_CH2_RX_BEAM_POS_0  0x120
#define ADAR1000_CH3_RX_BEAM_POS_0  0x140
#define ADAR1000_CH4_RX_BEAM_POS_0  0x160

/* Control registers */
#define ADAR1000_LD_WRK_REGS        0x028  /* Load working registers from beam memory */
#define ADAR1000_TX_ENABLES         0x030  /* TX channel enables */
#define ADAR1000_RX_ENABLES         0x031  /* RX channel enables */
#define ADAR1000_TX_CTRL            0x038  /* TX control */
#define ADAR1000_RX_CTRL            0x039  /* RX control */
#define ADAR1000_MISC_ENABLES       0x03F  /* Miscellaneous enables */
#define ADAR1000_SW_CTRL            0x042  /* Software control */
#define ADAR1000_ADC_CTRL           0x043  /* ADC control */
#define ADAR1000_ADC_OUTPUT         0x044  /* ADC output */

/* ============================================================================
 * Constants
 * ============================================================================ */

#define ADAR1000_NUM_CHANNELS       4
#define ADAR1000_PHASE_STEP_DEG     2.8125f   /* Phase resolution in degrees (128 steps / 360) */
#define ADAR1000_PHASE_STEPS        128        /* 7-bit phase (0-127 maps to 0-360 degrees) */
#define ADAR1000_GAIN_STEP_DB       0.5f       /* Gain step in dB */
#define ADAR1000_GAIN_MAX_STEPS     127        /* Maximum gain steps */
#define ADAR1000_MAX_GAIN_DB        31.5f      /* Maximum gain in dB (127 * 0.5 - offset) */

/* SPI address offsets for ADAR1000 devices (up to 4 devices on one SPI bus) */
#define ADAR1000_DEV_ADDR_SHIFT     5          /* Address bits [6:5] select device */
#define ADAR1000_MAX_DEVICES        4

/* ============================================================================
 * Data structures
 * ============================================================================ */

/** ADAR1000 channel settings */
typedef struct {
    float   phase_deg;      /**< Phase shift in degrees (0-360) */
    float   gain_db;        /**< Gain in dB (0 to MAX_GAIN_DB) */
    bool    enable;         /**< Channel enable */
} adar1000_channel_t;

/** ADAR1000 device handle */
typedef struct {
    uint32_t    base_addr;              /**< Base AXI address for SPI controller */
    uint8_t     device_index;           /**< Device index (0-3) for multi-device bus */
    adar1000_channel_t rx_ch[4];        /**< RX channel settings */
    adar1000_channel_t tx_ch[4];        /**< TX channel settings */
    bool        rx_mode;                /**< True = RX mode, False = TX mode */
    bool        initialized;
} adar1000_dev_t;

/* ============================================================================
 * SPI transaction callback type
 * ============================================================================ */

/**
 * SPI write/read function prototype.
 * Implementations may use Xilinx SPI driver or custom PL SPI controller.
 *
 * @param dev_addr  7-bit SPI address (includes device select bits)
 * @param wdata     8-bit write data
 * @param rdata     Pointer to 8-bit read data buffer (may be NULL for write-only)
 * @return          0 on success, negative on error
 */
typedef int (*adar1000_spi_fn_t)(uint8_t dev_addr, uint8_t wdata, uint8_t *rdata);

/* ============================================================================
 * Function prototypes
 * ============================================================================ */

/**
 * @brief Initialize the ADAR1000 driver and hardware.
 *
 * Performs hardware reset, sets default register values, and verifies
 * device communication via product ID readback.
 *
 * @param dev       Pointer to device handle (must be allocated by caller)
 * @param spi_fn    SPI transaction callback function
 * @param dev_idx   Device index on the SPI bus (0-3)
 * @return          0 on success, -1 on error
 */
int adar1000_init(adar1000_dev_t *dev, adar1000_spi_fn_t spi_fn, uint8_t dev_idx);

/**
 * @brief Write a register on the ADAR1000.
 *
 * @param dev       Device handle
 * @param reg_addr  15-bit register address
 * @param value     8-bit value to write
 * @return          0 on success, negative on error
 */
int adar1000_write_reg(adar1000_dev_t *dev, uint16_t reg_addr, uint8_t value);

/**
 * @brief Read a register from the ADAR1000.
 *
 * @param dev       Device handle
 * @param reg_addr  15-bit register address
 * @param value     Pointer to 8-bit read data
 * @return          0 on success, negative on error
 */
int adar1000_read_reg(adar1000_dev_t *dev, uint16_t reg_addr, uint8_t *value);

/**
 * @brief Set the RX phase for a channel.
 *
 * @param dev       Device handle
 * @param channel   Channel index (0-3)
 * @param phase_deg Phase in degrees (0.0 to 360.0)
 * @return          0 on success, negative on error
 */
int adar1000_set_rx_phase(adar1000_dev_t *dev, uint8_t channel, float phase_deg);

/**
 * @brief Set the RX gain for a channel.
 *
 * @param dev       Device handle
 * @param channel   Channel index (0-3)
 * @param gain_db   Gain in dB (0.0 to ADAR1000_MAX_GAIN_DB)
 * @return          0 on success, negative on error
 */
int adar1000_set_rx_gain(adar1000_dev_t *dev, uint8_t channel, float gain_db);

/**
 * @brief Set RX phase and gain for all 4 channels simultaneously.
 *
 * @param dev         Device handle
 * @param phase_deg   Array of 4 phase values in degrees
 * @param gain_db     Array of 4 gain values in dB
 * @return            0 on success, negative on error
 */
int adar1000_set_beam_weights(adar1000_dev_t *dev,
                               const float phase_deg[4],
                               const float gain_db[4]);

/**
 * @brief Convert complex weight to ADAR1000 phase/gain settings.
 *
 * Converts a complex beamforming weight (amplitude + phase) to the
 * ADAR1000's 7-bit phase and gain register values.
 *
 * @param weight_re   Real part of complex weight
 * @param weight_im   Imaginary part of complex weight
 * @param phase_deg   Output: phase in degrees
 * @param gain_db     Output: gain in dB (relative to max)
 */
void adar1000_weight_to_phase_gain(float weight_re, float weight_im,
                                    float *phase_deg, float *gain_db);

/**
 * @brief Apply MVDR weights to all 4 channels.
 *
 * Converts complex MVDR weights to ADAR1000 phase/gain register values
 * and programs the device.
 *
 * @param dev         Device handle
 * @param weights_re  Real parts of MVDR weights [4]
 * @param weights_im  Imaginary parts of MVDR weights [4]
 * @return            0 on success, negative on error
 */
int adar1000_apply_mvdr_weights(adar1000_dev_t *dev,
                                 const float weights_re[4],
                                 const float weights_im[4]);

/**
 * @brief Enable/disable a specific RX channel.
 *
 * @param dev       Device handle
 * @param channel   Channel index (0-3)
 * @param enable    true = enable, false = disable
 * @return          0 on success, negative on error
 */
int adar1000_set_rx_enable(adar1000_dev_t *dev, uint8_t channel, bool enable);

/**
 * @brief Load working registers from beam memory position.
 *
 * @param dev       Device handle
 * @param position  Beam memory position (0-120)
 * @return          0 on success, negative on error
 */
int adar1000_load_beam_position(adar1000_dev_t *dev, uint8_t position);

/**
 * @brief Reset the ADAR1000 device to default state.
 *
 * @param dev       Device handle
 * @return          0 on success, negative on error
 */
int adar1000_reset(adar1000_dev_t *dev);

#endif /* ADAR1000_DRIVER_H */
