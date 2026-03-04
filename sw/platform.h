/**
 * @file platform.h
 * @brief ZC702 platform initialization and utility functions.
 *
 * Provides Zynq PS initialization, AXI register access helpers,
 * timer setup, and interrupt configuration for the ZC702 board.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * ZC702 Platform Memory Map
 * ============================================================================ */

/** Base address of null steering PL AXI-Lite peripheral */
#define NULL_STEERING_BASEADDR      0x43C00000UL

/** AXI register offsets (matching axi_lite_slave.v register map) */
#define REG_CTRL                    0x00
#define REG_STATUS                  0x04
#define REG_SNAP_COUNT              0x08
#define REG_DIAG_LOAD               0x0C
#define REG_STEER_ANGLE             0x10
#define REG_SPI_CTRL                0x14
#define REG_SPI_STATUS              0x18
#define REG_SPI_RDATA               0x1C
#define REG_W0_RE                   0x20
#define REG_W0_IM                   0x24
#define REG_W1_RE                   0x28
#define REG_W1_IM                   0x2C
#define REG_W2_RE                   0x30
#define REG_W2_IM                   0x34
#define REG_W3_RE                   0x38
#define REG_W3_IM                   0x3C
#define REG_VERSION                 0x40
/** GPIO control register — ADAR1000 P3 connector GPIO lines */
#define REG_GPIO_CTRL               0x44

/** Control register bit positions */
#define CTRL_START_CAPTURE          (1 << 0)
#define CTRL_START_COV              (1 << 1)
#define CTRL_START_INVERSION        (1 << 2)
#define CTRL_START_WEIGHTS          (1 << 3)
#define CTRL_APPLY_BEAM             (1 << 4)

/** Status register bit positions */
#define STATUS_CAPTURE_DONE         (1 << 0)
#define STATUS_COV_VALID            (1 << 1)
#define STATUS_INV_VALID            (1 << 2)
#define STATUS_WEIGHTS_VALID        (1 << 3)
#define STATUS_BUSY                 (1 << 4)
#define STATUS_SINGULAR             (1 << 5)

/** GPIO_CTRL register bit positions (AXI offset 0x44, ADAR1000 P3 connector) */
#define GPIO_RX_LOAD                (1 << 0)   /**< P3-1 GPIO0: load RX beam registers */
#define GPIO_TX_LOAD                (1 << 1)   /**< P3-3 GPIO1: load TX beam registers */
#define GPIO_TR                     (1 << 2)   /**< P3-5 GPIO4: 1=transmit, 0=receive */
#define GPIO_PA_ON                  (1 << 3)   /**< P3-7 GPIO5: power amplifier enable */

/** SPI control register encoding */
#define SPI_CTRL_START              (1 << 0)
#define SPI_CTRL_RW_BIT             8
#define SPI_CTRL_ADDR_SHIFT         9
#define SPI_CTRL_WDATA_SHIFT        16

/** ZC702 timer base address */
#define XPAR_XTTCPS_0_BASEADDR      0xF8001000UL
/** ZC702 UART base address */
#define XPAR_XUARTPS_0_BASEADDR     0xE0000000UL
/** Zynq system clock frequency */
#define XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ  666666667UL  /* 2/3 GHz = 666.667 MHz */

/* ============================================================================
 * Timer / delay
 * ============================================================================ */

/** Milliseconds to loop count (approximate for ARM Cortex-A9 at 667 MHz) */
#define MS_TO_LOOPS(ms)   ((ms) * 100000UL)

/* ============================================================================
 * Function prototypes
 * ============================================================================ */

/**
 * @brief Initialize the ZC702 platform.
 *
 * Performs:
 * - Zynq PS MMU and cache configuration
 * - UART console initialization (115200 baud)
 * - Timer initialization
 * - Interrupt controller initialization
 * - PL fabric clock enable
 *
 * @return 0 on success, negative on error
 */
int platform_init(void);

/**
 * @brief De-initialize the platform (clean shutdown).
 */
void platform_deinit(void);

/**
 * @brief Write a 32-bit value to an AXI-mapped register.
 *
 * @param base_addr   Base address of AXI peripheral
 * @param offset      Register byte offset
 * @param value       32-bit value to write
 */
void axi_write(uint32_t base_addr, uint32_t offset, uint32_t value);

/**
 * @brief Read a 32-bit value from an AXI-mapped register.
 *
 * @param base_addr   Base address of AXI peripheral
 * @param offset      Register byte offset
 * @return            32-bit register value
 */
uint32_t axi_read(uint32_t base_addr, uint32_t offset);

/**
 * @brief Poll a status register bit until set or timeout.
 *
 * @param base_addr   AXI peripheral base address
 * @param offset      Register offset
 * @param mask        Bit mask to test
 * @param timeout_ms  Timeout in milliseconds
 * @return            0 if bit was set, -1 on timeout
 */
int axi_poll_until_set(uint32_t base_addr, uint32_t offset,
                        uint32_t mask, uint32_t timeout_ms);

/**
 * @brief Trigger PL capture and wait for completion.
 *
 * Writes CTRL_START_CAPTURE and polls STATUS_CAPTURE_DONE.
 *
 * @param timeout_ms  Timeout in milliseconds
 * @return            0 on success, -1 on timeout
 */
int platform_trigger_capture(uint32_t timeout_ms);

/**
 * @brief Trigger PL covariance computation and wait.
 *
 * @param timeout_ms  Timeout in milliseconds
 * @return            0 on success, -1 on timeout
 */
int platform_trigger_covariance(uint32_t timeout_ms);

/**
 * @brief Wait for MVDR weight computation to complete.
 *
 * @param timeout_ms  Timeout in milliseconds
 * @return            0 on success, -1 on timeout or error
 */
int platform_wait_for_weights(uint32_t timeout_ms);

/**
 * @brief Read computed MVDR weights from PL.
 *
 * @param weights_re  Output: real parts [4]
 * @param weights_im  Output: imaginary parts [4]
 */
void platform_read_weights(float weights_re[4], float weights_im[4]);

/**
 * @brief Configure PL diagonal loading and snapshot count.
 *
 * @param diag_load_q16  Diagonal loading in Q16 format
 * @param n_snapshots    Number of snapshots
 */
void platform_configure_mvdr(uint32_t diag_load_q16, uint32_t n_snapshots);

/**
 * @brief Perform an SPI write transaction to ADAR1000 via PL.
 *
 * @param addr    7-bit register address
 * @param data    8-bit write data
 * @return        0 on success, -1 on timeout
 */
int platform_spi_write(uint8_t addr, uint8_t data);

/**
 * @brief Perform an SPI read transaction from ADAR1000 via PL.
 *
 * @param addr    7-bit register address
 * @param data    Pointer to 8-bit read data
 * @return        0 on success, -1 on timeout
 */
int platform_spi_read(uint8_t addr, uint8_t *data);

/**
 * @brief Set ADAR1000 P3 GPIO control signals.
 *
 * Controls the four GPIO lines on the ADAR1000 P3 connector:
 *   - GPIO0/RX_LOAD (bit 0): pulse high to latch RX beam registers
 *   - GPIO1/TX_LOAD (bit 1): pulse high to latch TX beam registers
 *   - GPIO4/TR      (bit 2): 1 = transmit mode, 0 = receive mode
 *   - GPIO5/PA_ON   (bit 3): 1 = power amplifier enabled
 *
 * Use the GPIO_RX_LOAD, GPIO_TX_LOAD, GPIO_TR, GPIO_PA_ON masks.
 *
 * @param mask  Bitmask of GPIO signals to assert (GPIO_* constants)
 */
void platform_gpio_ctrl(uint32_t mask);

/**
 * @brief Millisecond delay.
 *
 * @param ms  Delay in milliseconds
 */
void platform_delay_ms(uint32_t ms);

/**
 * @brief Print a message to UART console.
 *
 * @param fmt   printf-style format string
 * @param ...   Arguments
 */
void platform_print(const char *fmt, ...);

/**
 * @brief Get current timestamp in milliseconds (from timer).
 *
 * @return  Current time in milliseconds
 */
uint32_t platform_get_time_ms(void);

#endif /* PLATFORM_H */
