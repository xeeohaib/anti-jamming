/**
 * @file jammer_detect.c
 * @brief Jammer detection implementation.
 */

#include "jammer_detect.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * Private helpers
 * ============================================================================ */

static float watts_to_db(float power_w)
{
    if (power_w <= 0.0f) return -200.0f;
    return 10.0f * log10f(power_w);
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void jammer_detect_init(jammer_detect_state_t *state, float threshold_db)
{
    memset(state, 0, sizeof(jammer_detect_state_t));
    state->threshold_db = threshold_db;
    state->signal_power_db = -100.0f;  /* Unknown initially */
}

void jammer_estimate_power(jammer_detect_state_t *state,
                            const float samples_re[][4],
                            const float samples_im[][4],
                            int n_samples)
{
    if (!state || n_samples <= 0) return;

    float power_sum[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    for (int k = 0; k < n_samples; k++) {
        for (int ch = 0; ch < 4; ch++) {
            power_sum[ch] += samples_re[k][ch]*samples_re[k][ch]
                           + samples_im[k][ch]*samples_im[k][ch];
        }
    }

    float inv_n = 1.0f / (float)n_samples;
    float total = 0.0f;
    for (int ch = 0; ch < 4; ch++) {
        power_sum[ch] *= inv_n;
        state->element_power_db[ch] = watts_to_db(power_sum[ch]);
        total += power_sum[ch];
    }

    state->mean_power_db = watts_to_db(total / 4.0f);
}

int jammer_detect(jammer_detect_state_t *state)
{
    if (!state) return 0;

    int n_detected = 0;
    state->jammer_present = false;

    /* Compare each element's power against the mean */
    /* Elements with power > mean + threshold indicate jammer direction */
    for (int ch = 0; ch < 4; ch++) {
        float jsr = state->element_power_db[ch] - state->mean_power_db;
        state->jammer[ch].jsr_db    = jsr;
        state->jammer[ch].power_db  = state->element_power_db[ch];

        if (jsr > state->threshold_db) {
            state->jammer[ch].detected = true;
            n_detected++;
        } else {
            state->jammer[ch].detected = false;
        }
    }

    state->n_detected = n_detected;
    if (n_detected > 0) {
        state->jammer_present = true;
        state->detection_count++;
    }

    return n_detected;
}

float jammer_estimate_jsr(jammer_detect_state_t *state,
                           const mat4x4_c *Rxx,
                           float *noise_power,
                           float *jammer_power)
{
    if (!state || !Rxx) return 0.0f;

    /* Estimate noise power from the minimum diagonal element */
    float min_diag = Rxx->m[0][0].re;
    float max_diag = Rxx->m[0][0].re;
    float sum_diag = 0.0f;

    for (int i = 0; i < 4; i++) {
        float d = Rxx->m[i][i].re;
        sum_diag += d;
        if (d < min_diag) min_diag = d;
        if (d > max_diag) max_diag = d;
    }

    /* Noise floor estimate: minimum diagonal element */
    float noise_est = min_diag;
    /* Jammer power estimate: excess above noise floor */
    float jammer_est = (sum_diag / 4.0f) - noise_est;
    if (jammer_est < 0.0f) jammer_est = 0.0f;

    if (noise_power)  *noise_power  = noise_est;
    if (jammer_power) *jammer_power = jammer_est;

    if (noise_est <= 0.0f) return 0.0f;
    float jsr_linear = jammer_est / noise_est;
    return 10.0f * log10f(jsr_linear + 1e-10f);
}

void jammer_print_status(const jammer_detect_state_t *state)
{
    if (!state) return;

    printf("=== Jammer Detection Status ===\n");
    printf("  Jammer present: %s (%d detected)\n",
           state->jammer_present ? "YES" : "NO", state->n_detected);
    printf("  Mean power: %.2f dBFS\n", state->mean_power_db);
    printf("  Detection count: %u\n", state->detection_count);
    for (int ch = 0; ch < 4; ch++) {
        printf("  CH%d: power=%.2f dBFS, JSR=%.2f dB %s\n",
               ch,
               state->jammer[ch].power_db,
               state->jammer[ch].jsr_db,
               state->jammer[ch].detected ? "[JAMMER]" : "");
    }
    printf("===============================\n");
}

bool jammer_update_needed(const jammer_detect_state_t *state,
                           uint32_t update_interval,
                           uint32_t sample_counter)
{
    if (!state) return false;
    /* Update if jammer detected and at the right interval */
    return state->jammer_present && ((sample_counter % update_interval) == 0);
}
