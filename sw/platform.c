/**
 * @file platform.c
 * @brief ZC702 platform implementation.
 *
 * Implements AXI register access, SPI control, and timing utilities.
 * Uses Xilinx xil_io.h macros for memory-mapped I/O.
 */

#include "platform.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ============================================================================
 * Xilinx HAL abstraction
 * When building for ZC702 with Xilinx SDK, replace with proper xil_io.h usage.
 * For host simulation, use standard memory-mapped I/O.
 * ============================================================================ */

#ifdef XILINX_ZYNQ
#include "xil_io.h"
#include "xil_printf.h"
#include "xtime_l.h"
#define MEM_WRITE32(addr, val)  Xil_Out32((addr), (val))
#define MEM_READ32(addr)        Xil_In32((addr))
#else
/* Host simulation fallback: use volatile pointer */
#include <stdint.h>
#define MEM_WRITE32(addr, val)  (*(volatile uint32_t *)(uintptr_t)(addr)) = (val)
#define MEM_READ32(addr)        (*(volatile uint32_t *)(uintptr_t)(addr))
#endif

/* ============================================================================
 * Private state
 * ============================================================================ */

static bool g_platform_initialized = false;
static uint32_t g_timer_base = 0;

/* ============================================================================
 * Public API
 * ============================================================================ */

int platform_init(void)
{
    if (g_platform_initialized) return 0;

#ifdef XILINX_ZYNQ
    /* Initialize Xilinx standalone BSP */
    Xil_ICacheEnable();
    Xil_DCacheEnable();
#endif

    g_timer_base = XPAR_XTTCPS_0_BASEADDR;
    g_platform_initialized = true;

    platform_print("Platform initialized (ZC702 Zynq-7000)\n");
    platform_print("Null steering PL base: 0x%08X\n", NULL_STEERING_BASEADDR);

    /* Read and print PL version */
    uint32_t version = axi_read(NULL_STEERING_BASEADDR, REG_VERSION);
    platform_print("PL Version: %d.%d\n",
                   (version >> 16) & 0xFFFF, version & 0xFFFF);

    return 0;
}

void platform_deinit(void)
{
#ifdef XILINX_ZYNQ
    Xil_DCacheDisable();
    Xil_ICacheDisable();
#endif
    g_platform_initialized = false;
}

void axi_write(uint32_t base_addr, uint32_t offset, uint32_t value)
{
    MEM_WRITE32(base_addr + offset, value);
}

uint32_t axi_read(uint32_t base_addr, uint32_t offset)
{
    return MEM_READ32(base_addr + offset);
}

int axi_poll_until_set(uint32_t base_addr, uint32_t offset,
                        uint32_t mask, uint32_t timeout_ms)
{
    uint32_t deadline = platform_get_time_ms() + timeout_ms;
    while (!(axi_read(base_addr, offset) & mask)) {
        if (platform_get_time_ms() >= deadline) {
            platform_print("Timeout polling 0x%08X[0x%02X] mask=0x%08X\n",
                           base_addr, offset, mask);
            return -1;
        }
    }
    return 0;
}

int platform_trigger_capture(uint32_t timeout_ms)
{
    /* Pulse START_CAPTURE */
    axi_write(NULL_STEERING_BASEADDR, REG_CTRL, CTRL_START_CAPTURE);
    axi_write(NULL_STEERING_BASEADDR, REG_CTRL, 0);

    return axi_poll_until_set(NULL_STEERING_BASEADDR, REG_STATUS,
                               STATUS_CAPTURE_DONE, timeout_ms);
}

int platform_trigger_covariance(uint32_t timeout_ms)
{
    /* Pulse START_COV */
    axi_write(NULL_STEERING_BASEADDR, REG_CTRL, CTRL_START_COV);
    axi_write(NULL_STEERING_BASEADDR, REG_CTRL, 0);

    return axi_poll_until_set(NULL_STEERING_BASEADDR, REG_STATUS,
                               STATUS_COV_VALID, timeout_ms);
}

int platform_wait_for_weights(uint32_t timeout_ms)
{
    /* Check for singular error first */
    if (axi_read(NULL_STEERING_BASEADDR, REG_STATUS) & STATUS_SINGULAR) {
        platform_print("Warning: Singular matrix detected in PL\n");
    }

    return axi_poll_until_set(NULL_STEERING_BASEADDR, REG_STATUS,
                               STATUS_WEIGHTS_VALID, timeout_ms);
}

void platform_read_weights(float weights_re[4], float weights_im[4])
{
    /* Read Q16 fixed-point weights and convert to float */
    static const float Q16_SCALE = 1.0f / 65536.0f;

    uint32_t w0_re = axi_read(NULL_STEERING_BASEADDR, REG_W0_RE);
    uint32_t w0_im = axi_read(NULL_STEERING_BASEADDR, REG_W0_IM);
    uint32_t w1_re = axi_read(NULL_STEERING_BASEADDR, REG_W1_RE);
    uint32_t w1_im = axi_read(NULL_STEERING_BASEADDR, REG_W1_IM);
    uint32_t w2_re = axi_read(NULL_STEERING_BASEADDR, REG_W2_RE);
    uint32_t w2_im = axi_read(NULL_STEERING_BASEADDR, REG_W2_IM);
    uint32_t w3_re = axi_read(NULL_STEERING_BASEADDR, REG_W3_RE);
    uint32_t w3_im = axi_read(NULL_STEERING_BASEADDR, REG_W3_IM);

    weights_re[0] = (float)(int32_t)w0_re * Q16_SCALE;
    weights_im[0] = (float)(int32_t)w0_im * Q16_SCALE;
    weights_re[1] = (float)(int32_t)w1_re * Q16_SCALE;
    weights_im[1] = (float)(int32_t)w1_im * Q16_SCALE;
    weights_re[2] = (float)(int32_t)w2_re * Q16_SCALE;
    weights_im[2] = (float)(int32_t)w2_im * Q16_SCALE;
    weights_re[3] = (float)(int32_t)w3_re * Q16_SCALE;
    weights_im[3] = (float)(int32_t)w3_im * Q16_SCALE;
}

void platform_configure_mvdr(uint32_t diag_load_q16, uint32_t n_snapshots)
{
    axi_write(NULL_STEERING_BASEADDR, REG_DIAG_LOAD,  diag_load_q16);
    axi_write(NULL_STEERING_BASEADDR, REG_SNAP_COUNT, n_snapshots);
}

int platform_spi_write(uint8_t addr, uint8_t data)
{
    /* Build SPI_CTRL word: [0]=start, [8]=rw=0, [15:9]=addr, [23:16]=data */
    uint32_t ctrl = SPI_CTRL_START
                  | (0 << SPI_CTRL_RW_BIT)
                  | ((uint32_t)(addr & 0x7F) << SPI_CTRL_ADDR_SHIFT)
                  | ((uint32_t)data << SPI_CTRL_WDATA_SHIFT);

    axi_write(NULL_STEERING_BASEADDR, REG_SPI_CTRL, ctrl);

    /* Wait for done */
    return axi_poll_until_set(NULL_STEERING_BASEADDR, REG_SPI_STATUS,
                               (1 << 1), 10);  /* bit 1 = spi_done */
}

int platform_spi_read(uint8_t addr, uint8_t *data)
{
    if (!data) return -1;

    uint32_t ctrl = SPI_CTRL_START
                  | (1 << SPI_CTRL_RW_BIT)
                  | ((uint32_t)(addr & 0x7F) << SPI_CTRL_ADDR_SHIFT);

    axi_write(NULL_STEERING_BASEADDR, REG_SPI_CTRL, ctrl);

    if (axi_poll_until_set(NULL_STEERING_BASEADDR, REG_SPI_STATUS,
                            (1 << 1), 10) != 0) {
        return -1;
    }

    *data = (uint8_t)(axi_read(NULL_STEERING_BASEADDR, REG_SPI_RDATA) & 0xFF);
    return 0;
}

void platform_delay_ms(uint32_t ms)
{
#ifdef XILINX_ZYNQ
    usleep(ms * 1000UL);
#else
    /* Host simulation: busy loop */
    volatile uint32_t count = ms * 10000UL;
    while (count-- > 0) {}
#endif
}

void platform_print(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
#ifdef XILINX_ZYNQ
    /* Xilinx SDK uses xil_printf; for formatted output use vprintf to UART */
    vprintf(fmt, args);
#else
    vprintf(fmt, args);
#endif
    va_end(args);
}

uint32_t platform_get_time_ms(void)
{
#ifdef XILINX_ZYNQ
    XTime ticks;
    XTime_GetTime(&ticks);
    return (uint32_t)(ticks / (XPAR_CPU_CORTEXA9_CORE_CLOCK_FREQ_HZ / 1000UL));
#else
    /* Host simulation: use a simple counter */
    static uint32_t sim_time = 0;
    return sim_time++;
#endif
}
