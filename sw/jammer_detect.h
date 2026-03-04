/**
 * @file jammer_detect.h
 * @brief Jammer detection for the anti-jamming null steering system.
 *
 * Provides power spectral analysis and threshold-based jammer detection
 * using received antenna array data.
 */

#ifndef JAMMER_DETECT_H
#define JAMMER_DETECT_H

#include <stdint.h>
#include <stdbool.h>
#include "mvdr_algorithm.h"

/* ============================================================================
 * Constants
 * ============================================================================ */

#define JAMMER_MAX_JAMMERS          4    /**< Maximum detectable jammers */
#define JAMMER_DETECT_WINDOW        64   /**< Power estimation window (samples) */
#define JAMMER_DEFAULT_THRESHOLD_DB 10.0f /**< Default jammer-to-signal threshold (dB) */

/* ============================================================================
 * Data structures
 * ============================================================================ */

/** Single jammer detection result */
typedef struct {
    float   power_db;       /**< Estimated jammer power in dB */
    float   jsr_db;         /**< Jammer-to-signal ratio in dB */
    bool    detected;       /**< True if jammer is active */
} jammer_info_t;

/** Jammer detector state */
typedef struct {
    float           threshold_db;               /**< JSR detection threshold */
    float           signal_power_db;            /**< Estimated desired signal power */
    jammer_info_t   jammer[JAMMER_MAX_JAMMERS]; /**< Per-element (channel) jammer info */
    float           element_power_db[4];        /**< Power per antenna element (dB) */
    float           mean_power_db;              /**< Mean power across elements */
    int             n_detected;                 /**< Number of detected jammers */
    bool            jammer_present;             /**< True if any jammer detected */
    uint32_t        detection_count;            /**< Detection event counter */
} jammer_detect_state_t;

/* ============================================================================
 * Function prototypes
 * ============================================================================ */

/**
 * @brief Initialize the jammer detector.
 *
 * @param state         Detector state
 * @param threshold_db  JSR threshold for detection (dB), e.g., 10.0
 */
void jammer_detect_init(jammer_detect_state_t *state, float threshold_db);

/**
 * @brief Estimate received power per antenna element.
 *
 * Computes the average power across a block of samples for each element:
 *   P[ch] = (1/N) * sum_k (|i[k]|^2 + |q[k]|^2)
 *
 * @param state         Detector state (updates element_power_db[])
 * @param samples_re    Real (I) samples [N_SAMPLES][4]
 * @param samples_im    Imaginary (Q) samples [N_SAMPLES][4]
 * @param n_samples     Number of samples
 */
void jammer_estimate_power(jammer_detect_state_t *state,
                            const float samples_re[][4],
                            const float samples_im[][4],
                            int n_samples);

/**
 * @brief Detect jammers based on per-element power estimates.
 *
 * Compares each channel's power to the mean. Elements with power significantly
 * above the mean are flagged as potential jammer directions.
 *
 * @param state     Detector state (updated with jammer detections)
 * @return          Number of detected jammers
 */
int jammer_detect(jammer_detect_state_t *state);

/**
 * @brief Compute jammer-to-signal ratio from covariance matrix eigenstructure.
 *
 * Uses the diagonal of the covariance matrix to identify the noise subspace
 * power and jammer subspace power.
 *
 * @param state         Detector state
 * @param Rxx           Estimated covariance matrix
 * @param noise_power   Output: estimated noise power
 * @param jammer_power  Output: estimated jammer power
 * @return              JSR in dB
 */
float jammer_estimate_jsr(jammer_detect_state_t *state,
                           const mat4x4_c *Rxx,
                           float *noise_power,
                           float *jammer_power);

/**
 * @brief Print jammer detection status to console.
 *
 * @param state     Detector state
 */
void jammer_print_status(const jammer_detect_state_t *state);

/**
 * @brief Check if jammer update should be triggered.
 *
 * Returns true if a jammer is detected and enough time has passed
 * since the last update.
 *
 * @param state             Detector state
 * @param update_interval   Minimum samples between updates
 * @param sample_counter    Current sample counter
 * @return                  true if update needed
 */
bool jammer_update_needed(const jammer_detect_state_t *state,
                           uint32_t update_interval,
                           uint32_t sample_counter);

#endif /* JAMMER_DETECT_H */
