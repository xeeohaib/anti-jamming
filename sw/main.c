/**
 * @file main.c
 * @brief Main application for the anti-jamming null steering system.
 *
 * Implements the main control loop for the Zynq ZC702 platform:
 *   1. Initialize platform and ADAR1000
 *   2. Continuously: capture samples -> detect jammer -> estimate DOA
 *   3. Compute MVDR weights (HW or SW) -> program ADAR1000
 *   4. Report status via UART
 *
 * Target: Xilinx ZC702 (Zynq-7000 XC7Z020)
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "platform.h"
#include "adar1000_driver.h"
#include "mvdr_algorithm.h"
#include "jammer_detect.h"
#include "doa_estimation.h"

/* ============================================================================
 * Configuration parameters
 * ============================================================================ */

/** Number of snapshots for covariance estimation */
#define CFG_N_SNAPSHOTS         256

/** MVDR diagonal loading factor (fraction of trace(Rxx)) */
#define CFG_DIAG_LOAD_FACTOR    0.01f

/** Desired look direction (degrees, 0 = broadside) */
#define CFG_LOOK_ANGLE_DEG      0.0f

/** Carrier frequency (Hz) */
#define CFG_CARRIER_FREQ_HZ     10.0e9f

/** Element spacing (m) - half wavelength at 10 GHz */
#define CFG_ELEMENT_SPACING_M   0.015f

/** Update rate: recompute every N processing cycles */
#define CFG_UPDATE_INTERVAL     1

/** Jammer detection JSR threshold (dB) */
#define CFG_JSR_THRESHOLD_DB    10.0f

/** Whether to use hardware MVDR (PL) or software MVDR (PS fallback) */
#define CFG_USE_HW_MVDR         1

/** Maximum simulation snapshots for software buffer */
#define SIM_SNAPSHOT_BUFFER     256

/* ============================================================================
 * Global state
 * ============================================================================ */

static adar1000_dev_t       g_adar1000;
static mvdr_state_t         g_mvdr;
static jammer_detect_state_t g_jammer_detect;
static doa_state_t          g_doa;

/** Software snapshot buffers (used in SW MVDR mode) */
static float g_snap_re[SIM_SNAPSHOT_BUFFER][4];
static float g_snap_im[SIM_SNAPSHOT_BUFFER][4];

/* ============================================================================
 * SPI callback (wraps platform SPI) for ADAR1000 driver
 * ============================================================================ */

static int spi_callback(uint8_t dev_addr, uint8_t wdata, uint8_t *rdata)
{
    if (rdata) {
        return platform_spi_read(dev_addr, rdata);
    } else {
        return platform_spi_write(dev_addr, wdata);
    }
}

/* ============================================================================
 * Helper: capture samples from PL and fill software buffer
 * ============================================================================ */

static int capture_snapshots(int n_snap)
{
    /* Configure PL snapshot count */
    platform_configure_mvdr(
        (uint32_t)(CFG_DIAG_LOAD_FACTOR * 65536.0f),  /* Q16 */
        (uint32_t)n_snap
    );

    /* Trigger PL capture */
    if (platform_trigger_capture(1000) != 0) {
        platform_print("ERROR: Capture timeout\n");
        return -1;
    }

    /*
     * In a real system, the PS would DMA the ADC data from a shared memory
     * buffer. Here we fill with placeholder values for the software MVDR path.
     * The PL MVDR path reads directly from its internal buffers.
     */
    for (int k = 0; k < n_snap && k < SIM_SNAPSHOT_BUFFER; k++) {
        for (int ch = 0; ch < 4; ch++) {
            g_snap_re[k][ch] = 0.0f;  /* Replaced by DMA read in real system */
            g_snap_im[k][ch] = 0.0f;
        }
    }
    return 0;
}

/* ============================================================================
 * Helper: run hardware MVDR pipeline (PL)
 * ============================================================================ */

static int run_hw_mvdr(float weights_re[4], float weights_im[4])
{
    /* Trigger covariance computation */
    if (platform_trigger_covariance(5000) != 0) {
        platform_print("ERROR: Covariance timeout\n");
        return -1;
    }

    /* Wait for MVDR weights */
    if (platform_wait_for_weights(5000) != 0) {
        platform_print("ERROR: Weight computation timeout\n");
        return -1;
    }

    /* Read weights from PL registers */
    platform_read_weights(weights_re, weights_im);
    return 0;
}

/* ============================================================================
 * Helper: run software MVDR on PS (fallback)
 * ============================================================================ */

static int run_sw_mvdr(float weights_re[4], float weights_im[4],
                        int n_snap, float look_angle_deg)
{
    int ret = mvdr_update(&g_mvdr, g_snap_re, g_snap_im, n_snap, look_angle_deg);
    if (ret != 0) return ret;

    for (int i = 0; i < 4; i++) {
        weights_re[i] = g_mvdr.weights.v[i].re;
        weights_im[i] = g_mvdr.weights.v[i].im;
    }
    return 0;
}

/* ============================================================================
 * Main application
 * ============================================================================ */

int main(void)
{
    int ret;
    uint32_t cycle = 0;
    float weights_re[4] = {1.0f, 1.0f, 1.0f, 1.0f};
    float weights_im[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float look_angle    = CFG_LOOK_ANGLE_DEG;

    /* -----------------------------------------------------------------------
     * 1. Platform initialization
     * ----------------------------------------------------------------------- */
    ret = platform_init();
    if (ret != 0) {
        printf("FATAL: Platform init failed (%d)\n", ret);
        return -1;
    }

    /* -----------------------------------------------------------------------
     * 2. ADAR1000 initialization
     * ----------------------------------------------------------------------- */
    ret = adar1000_init(&g_adar1000, spi_callback, 0);
    if (ret != 0) {
        platform_print("WARNING: ADAR1000 init failed, continuing without HW beamformer\n");
    }

    /* -----------------------------------------------------------------------
     * 3. Algorithm initialization
     * ----------------------------------------------------------------------- */
    mvdr_params_t mvdr_params = {
        .diag_load_factor  = CFG_DIAG_LOAD_FACTOR,
        .n_snapshots       = CFG_N_SNAPSHOTS,
        .element_spacing_m = CFG_ELEMENT_SPACING_M,
        .carrier_freq_hz   = CFG_CARRIER_FREQ_HZ,
        .look_angle_deg    = CFG_LOOK_ANGLE_DEG
    };
    mvdr_init(&g_mvdr, &mvdr_params);
    jammer_detect_init(&g_jammer_detect, CFG_JSR_THRESHOLD_DB);
    doa_init(&g_doa, NULL);

    /* Set initial uniform weights on ADAR1000 */
    float init_phase[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float init_gain[4]  = {20.0f, 20.0f, 20.0f, 20.0f};
    adar1000_set_beam_weights(&g_adar1000, init_phase, init_gain);

    /* Enable PL beam application */
    axi_write(NULL_STEERING_BASEADDR, REG_CTRL, CTRL_APPLY_BEAM);

    platform_print("\n=== Anti-Jamming Null Steering System Started ===\n");
    platform_print("Look angle: %.1f deg, N_snap: %d\n",
                   CFG_LOOK_ANGLE_DEG, CFG_N_SNAPSHOTS);

    /* -----------------------------------------------------------------------
     * 4. Main control loop
     * ----------------------------------------------------------------------- */
    while (1) {
        cycle++;
        platform_print("\n--- Cycle %u ---\n", cycle);

        /* Step 1: Capture samples */
        if (capture_snapshots(CFG_N_SNAPSHOTS) != 0) {
            platform_delay_ms(100);
            continue;
        }

        /* Step 2: Jammer detection (uses software samples in SW mode) */
        jammer_estimate_power(&g_jammer_detect, g_snap_re, g_snap_im, CFG_N_SNAPSHOTS);
        int n_jammers = jammer_detect(&g_jammer_detect);

        if (g_jammer_detect.jammer_present) {
            jammer_print_status(&g_jammer_detect);

            /* Step 3: DOA estimation */
            mvdr_estimate_covariance(&g_mvdr, g_snap_re, g_snap_im, CFG_N_SNAPSHOTS);
            int n_doa = doa_estimate_music(&g_doa, &g_mvdr.Rxx, 1);
            doa_print_results(&g_doa);

            if (n_doa > 0 && g_doa.results[0].valid) {
                /* Use first DOA result as jammer direction to null */
                float jammer_angle = g_doa.results[0].angle_deg;
                platform_print("Jammer detected at %.1f deg, nulling...\n", jammer_angle);
            }

            /* Step 4: Compute MVDR weights */
#if CFG_USE_HW_MVDR
            ret = run_hw_mvdr(weights_re, weights_im);
#else
            ret = run_sw_mvdr(weights_re, weights_im, CFG_N_SNAPSHOTS, look_angle);
#endif
            if (ret != 0) {
                platform_print("WARNING: MVDR failed, using previous weights\n");
            }

            /* Step 5: Program ADAR1000 with new weights */
            if (g_adar1000.initialized) {
                ret = adar1000_apply_mvdr_weights(&g_adar1000, weights_re, weights_im);
                if (ret != 0) {
                    platform_print("WARNING: ADAR1000 programming failed\n");
                }
            }
        } else {
            platform_print("No jammer detected (cycle %u)\n", cycle);
        }

        /* Report status */
        uint32_t status = axi_read(NULL_STEERING_BASEADDR, REG_STATUS);
        platform_print("PL Status: 0x%08X (caps=%d, cov=%d, inv=%d, w=%d, busy=%d, sing=%d)\n",
                       status,
                       (status >> 0) & 1,
                       (status >> 1) & 1,
                       (status >> 2) & 1,
                       (status >> 3) & 1,
                       (status >> 4) & 1,
                       (status >> 5) & 1);

        platform_delay_ms(100);  /* 10 Hz update rate */
    }

    platform_deinit();
    return 0;
}
