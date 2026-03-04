/**
 * @file adar1000_driver.c
 * @brief ADAR1000 beamformer IC driver implementation.
 *
 * Implements register access, phase/gain control, and MVDR weight application
 * for the Analog Devices ADAR1000 4-channel beamformer IC.
 */

#include "adar1000_driver.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * Private data
 * ============================================================================ */

static adar1000_spi_fn_t g_spi_fn = NULL;

/* Expected product ID bytes for ADAR1000 */
#define ADAR1000_PRODUCT_ID_H_VAL   0x00
#define ADAR1000_PRODUCT_ID_L_VAL   0x8A

/* Device address bits shift within 7-bit SPI address */
/* ADAR1000: address [14:8] = register, [1:0] of addr byte select device */
#define ADAR1000_SPI_ADDR(dev_idx, reg) \
    (uint8_t)(((reg) & 0x7F) | (((dev_idx) & 0x03) << 5))

/* ============================================================================
 * Private helper functions
 * ============================================================================ */

/**
 * Build 24-bit SPI transaction word and perform write.
 * ADAR1000 SPI format: [23] = R/W#, [22:8] = address, [7:0] = data
 */
static int spi_write(adar1000_dev_t *dev, uint16_t addr, uint8_t data)
{
    if (!g_spi_fn) return -1;
    /* Encode device address into upper address bits */
    uint8_t spi_addr = (uint8_t)((addr & 0x7F) | ((dev->device_index & 0x03) << 5));
    return g_spi_fn(spi_addr, data, NULL);
}

static int spi_read(adar1000_dev_t *dev, uint16_t addr, uint8_t *data)
{
    if (!g_spi_fn || !data) return -1;
    uint8_t spi_addr = (uint8_t)(0x80 | (addr & 0x7F) | ((dev->device_index & 0x03) << 5));
    return g_spi_fn(spi_addr, 0x00, data);
}

/* ============================================================================
 * Public API implementation
 * ============================================================================ */

int adar1000_init(adar1000_dev_t *dev, adar1000_spi_fn_t spi_fn, uint8_t dev_idx)
{
    if (!dev || !spi_fn) return -1;

    memset(dev, 0, sizeof(adar1000_dev_t));
    dev->device_index = dev_idx & 0x03;
    g_spi_fn = spi_fn;

    /* Reset device via software reset bit */
    if (adar1000_reset(dev) != 0) {
        printf("ADAR1000[%d]: Reset failed\n", dev_idx);
        return -1;
    }

    /* Verify product ID */
    uint8_t pid_h = 0, pid_l = 0;
    spi_read(dev, ADAR1000_PRODUCT_ID_H, &pid_h);
    spi_read(dev, ADAR1000_PRODUCT_ID_L, &pid_l);
    if (pid_l != ADAR1000_PRODUCT_ID_L_VAL) {
        printf("ADAR1000[%d]: Product ID mismatch (got 0x%02X, expected 0x%02X)\n",
               dev_idx, pid_l, ADAR1000_PRODUCT_ID_L_VAL);
        /* Continue anyway - may be a compatible variant */
    }

    /* Configure interface: MSB first, SPI mode 0 */
    spi_write(dev, ADAR1000_INTERFACE_CFG_A, 0x00);

    /* Enable all RX channels by default */
    spi_write(dev, ADAR1000_RX_ENABLES, 0x0F);

    /* Set default gain (mid-scale) for all channels */
    for (uint8_t ch = 0; ch < ADAR1000_NUM_CHANNELS; ch++) {
        spi_write(dev, ADAR1000_CH_RX_GAIN(ch), 0x7F);
        spi_write(dev, ADAR1000_CH_RX_PHASE_I(ch), 0x00);
        spi_write(dev, ADAR1000_CH_RX_PHASE_Q(ch), 0x00);
        dev->rx_ch[ch].gain_db  = 31.5f;  /* mid-scale */
        dev->rx_ch[ch].phase_deg = 0.0f;
        dev->rx_ch[ch].enable   = true;
    }

    /* Load working registers */
    spi_write(dev, ADAR1000_LD_WRK_REGS, 0x01);

    dev->rx_mode   = true;
    dev->initialized = true;

    printf("ADAR1000[%d]: Initialized (PID H=0x%02X L=0x%02X)\n", dev_idx, pid_h, pid_l);
    return 0;
}

int adar1000_write_reg(adar1000_dev_t *dev, uint16_t reg_addr, uint8_t value)
{
    if (!dev || !dev->initialized) return -1;
    return spi_write(dev, reg_addr, value);
}

int adar1000_read_reg(adar1000_dev_t *dev, uint16_t reg_addr, uint8_t *value)
{
    if (!dev || !dev->initialized || !value) return -1;
    return spi_read(dev, reg_addr, value);
}

int adar1000_set_rx_phase(adar1000_dev_t *dev, uint8_t channel, float phase_deg)
{
    if (!dev || !dev->initialized || channel >= ADAR1000_NUM_CHANNELS) return -1;

    /* Normalize phase to [0, 360) */
    while (phase_deg < 0.0f)   phase_deg += 360.0f;
    while (phase_deg >= 360.0f) phase_deg -= 360.0f;

    /* Convert to 7-bit register value */
    uint8_t phase_reg = (uint8_t)roundf(phase_deg / ADAR1000_PHASE_STEP_DEG) & 0x7F;

    /* ADAR1000 uses I/Q vector modulator: phase is encoded as I/Q components.
     * For simplicity, write phase_reg to both I and Q phase registers.
     * A full implementation would compute I = cos(phase) and Q = sin(phase)
     * as 7-bit signed values.
     */
    float rad = phase_deg * (float)M_PI / 180.0f;
    int8_t i_val = (int8_t)roundf(63.0f * cosf(rad));
    int8_t q_val = (int8_t)roundf(63.0f * sinf(rad));

    int ret = 0;
    ret |= spi_write(dev, ADAR1000_CH_RX_PHASE_I(channel), (uint8_t)i_val);
    ret |= spi_write(dev, ADAR1000_CH_RX_PHASE_Q(channel), (uint8_t)q_val);

    if (ret == 0) {
        dev->rx_ch[channel].phase_deg = phase_deg;
    }
    return ret;
}

int adar1000_set_rx_gain(adar1000_dev_t *dev, uint8_t channel, float gain_db)
{
    if (!dev || !dev->initialized || channel >= ADAR1000_NUM_CHANNELS) return -1;

    /* Clamp gain */
    if (gain_db < 0.0f)                    gain_db = 0.0f;
    if (gain_db > ADAR1000_MAX_GAIN_DB)    gain_db = ADAR1000_MAX_GAIN_DB;

    /* Convert to 7-bit gain step: 0 dB = step 0, max = step 127 */
    uint8_t gain_reg = (uint8_t)roundf(gain_db / ADAR1000_GAIN_STEP_DB) & 0x7F;

    int ret = spi_write(dev, ADAR1000_CH_RX_GAIN(channel), gain_reg);
    if (ret == 0) {
        dev->rx_ch[channel].gain_db = gain_db;
    }
    return ret;
}

int adar1000_set_beam_weights(adar1000_dev_t *dev,
                               const float phase_deg[4],
                               const float gain_db[4])
{
    if (!dev || !dev->initialized || !phase_deg || !gain_db) return -1;

    int ret = 0;
    for (uint8_t ch = 0; ch < ADAR1000_NUM_CHANNELS; ch++) {
        ret |= adar1000_set_rx_phase(dev, ch, phase_deg[ch]);
        ret |= adar1000_set_rx_gain(dev, ch, gain_db[ch]);
    }

    /* Trigger load of working registers */
    if (ret == 0) {
        ret |= spi_write(dev, ADAR1000_LD_WRK_REGS, 0x01);
    }
    return ret;
}

void adar1000_weight_to_phase_gain(float weight_re, float weight_im,
                                    float *phase_deg, float *gain_db)
{
    if (!phase_deg || !gain_db) return;

    /* Phase = atan2(im, re) in degrees */
    *phase_deg = atan2f(weight_im, weight_re) * 180.0f / (float)M_PI;
    if (*phase_deg < 0.0f) *phase_deg += 360.0f;

    /* Amplitude = sqrt(re^2 + im^2), convert to dB */
    float amplitude = sqrtf(weight_re * weight_re + weight_im * weight_im);
    if (amplitude <= 0.0f) amplitude = 1e-10f;
    *gain_db = 20.0f * log10f(amplitude);

    /* Clamp to valid range */
    if (*gain_db < 0.0f)                  *gain_db = 0.0f;
    if (*gain_db > ADAR1000_MAX_GAIN_DB)  *gain_db = ADAR1000_MAX_GAIN_DB;
}

int adar1000_apply_mvdr_weights(adar1000_dev_t *dev,
                                 const float weights_re[4],
                                 const float weights_im[4])
{
    if (!dev || !dev->initialized || !weights_re || !weights_im) return -1;

    float phase_deg[4];
    float gain_db[4];

    /* Find maximum amplitude for normalization */
    float max_amp = 0.0f;
    for (int ch = 0; ch < 4; ch++) {
        float amp = sqrtf(weights_re[ch]*weights_re[ch] + weights_im[ch]*weights_im[ch]);
        if (amp > max_amp) max_amp = amp;
    }
    if (max_amp <= 0.0f) max_amp = 1.0f;

    /* Convert each weight to phase and relative gain.
     * Normalize amplitudes to [0, 1] relative to the strongest channel.
     * Phase comes from atan2(im, re).
     * Gain is mapped linearly: amplitude 1.0 → MAX_GAIN_DB, 0.0 → 0 dB.
     * Using linear (not log) mapping avoids −inf for near-zero weights and
     * keeps gain in the valid ADAR1000 range [0, MAX_GAIN_DB]. */
    for (int ch = 0; ch < 4; ch++) {
        float re_norm = weights_re[ch] / max_amp;
        float im_norm = weights_im[ch] / max_amp;

        /* Phase */
        phase_deg[ch] = atan2f(im_norm, re_norm) * 180.0f / (float)M_PI;
        if (phase_deg[ch] < 0.0f) phase_deg[ch] += 360.0f;

        /* Gain: linear amplitude fraction × full-scale dB range */
        float amp_norm = sqrtf(re_norm*re_norm + im_norm*im_norm);
        gain_db[ch] = amp_norm * ADAR1000_MAX_GAIN_DB;
        if (gain_db[ch] < 0.0f)                 gain_db[ch] = 0.0f;
        if (gain_db[ch] > ADAR1000_MAX_GAIN_DB) gain_db[ch] = ADAR1000_MAX_GAIN_DB;
    }

    printf("ADAR1000: Applying weights:\n");
    for (int ch = 0; ch < 4; ch++) {
        printf("  CH%d: phase=%.2f deg, gain=%.2f dB (re=%.4f, im=%.4f)\n",
               ch, phase_deg[ch], gain_db[ch], weights_re[ch], weights_im[ch]);
    }

    return adar1000_set_beam_weights(dev, phase_deg, gain_db);
}

int adar1000_set_rx_enable(adar1000_dev_t *dev, uint8_t channel, bool enable)
{
    if (!dev || !dev->initialized || channel >= ADAR1000_NUM_CHANNELS) return -1;

    uint8_t enables = 0;
    /* Read current enables */
    spi_read(dev, ADAR1000_RX_ENABLES, &enables);

    if (enable)
        enables |=  (1 << channel);
    else
        enables &= ~(1 << channel);

    dev->rx_ch[channel].enable = enable;
    return spi_write(dev, ADAR1000_RX_ENABLES, enables);
}

int adar1000_load_beam_position(adar1000_dev_t *dev, uint8_t position)
{
    if (!dev || !dev->initialized) return -1;
    return spi_write(dev, ADAR1000_LD_WRK_REGS, position);
}

int adar1000_reset(adar1000_dev_t *dev)
{
    if (!dev) return -1;
    /* ADAR1000 software reset: write 0x81 to INTERFACE_CFG_A */
    int ret = spi_write(dev, ADAR1000_INTERFACE_CFG_A, 0x81);
    /* Wait for reset to complete (typically < 1 ms) */
    /* In bare-metal, a busy-wait loop would be used here */
    volatile int i;
    for (i = 0; i < 10000; i++) { /* ~100 us at 100MHz */ }
    return ret;
}
