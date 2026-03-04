/**
 * @file doa_estimation.h
 * @brief Direction-of-Arrival (DOA) estimation for the null steering system.
 *
 * Implements MUSIC (Multiple Signal Classification) and spatial spectral
 * search algorithms to identify jammer directions from covariance matrix
 * eigenstructure.
 */

#ifndef DOA_ESTIMATION_H
#define DOA_ESTIMATION_H

#include <stdint.h>
#include <stdbool.h>
#include "mvdr_algorithm.h"

/* ============================================================================
 * Constants
 * ============================================================================ */

#define DOA_MAX_SOURCES         3     /**< Maximum number of DOA estimates */
#define DOA_ANGLE_RANGE_DEG     180   /**< Total angular search range (degrees) */
#define DOA_ANGLE_STEP_DEG      1.0f  /**< Angular search step (degrees) */
#define DOA_N_ANGLE_BINS        180   /**< Number of angle bins (-90 to +90) */

/* ============================================================================
 * Data structures
 * ============================================================================ */

/** DOA estimation result */
typedef struct {
    float   angle_deg;      /**< Estimated DOA angle (degrees, -90 to +90) */
    float   power_db;       /**< Spectral peak power (dB) */
    bool    valid;          /**< True if this entry contains a valid estimate */
} doa_result_t;

/** DOA estimator configuration */
typedef struct {
    float   element_spacing_m;      /**< Array element spacing in meters */
    float   carrier_freq_hz;        /**< Carrier frequency in Hz */
    int     n_sources;              /**< Number of sources to find */
    float   angle_min_deg;          /**< Minimum search angle */
    float   angle_max_deg;          /**< Maximum search angle */
    float   angle_step_deg;         /**< Angular search step */
    int     n_signal_subspace;      /**< Number of signal subspace vectors */
} doa_config_t;

/** DOA estimator state */
typedef struct {
    doa_config_t    config;
    doa_result_t    results[DOA_MAX_SOURCES];
    int             n_results;
    float           spectrum[DOA_N_ANGLE_BINS];     /**< MUSIC pseudospectrum */
    float           angles[DOA_N_ANGLE_BINS];       /**< Angle grid */
    bool            valid;
} doa_state_t;

/* ============================================================================
 * Function prototypes
 * ============================================================================ */

/**
 * @brief Initialize DOA estimator.
 *
 * @param state     DOA estimator state
 * @param config    Configuration parameters (NULL for defaults)
 */
void doa_init(doa_state_t *state, const doa_config_t *config);

/**
 * @brief Run DOA estimation using simple spatial spectral search.
 *
 * Scans the covariance matrix diagonal-based spatial spectrum to find peaks
 * corresponding to signal sources (simpler than full MUSIC).
 *
 * @param state     DOA state
 * @param Rxx       Estimated covariance matrix
 * @return          Number of DOA estimates found
 */
int doa_estimate_spectral(doa_state_t *state, const mat4x4_c *Rxx);

/**
 * @brief Compute eigendecomposition of 4x4 Hermitian matrix.
 *
 * Uses power iteration to compute eigenvectors/values.
 * Note: For a 4x4 matrix, we use a simplified approach suitable for
 * real-time embedded implementation.
 *
 * @param A             Input 4x4 Hermitian matrix
 * @param eigenvalues   Output eigenvalues (real, sorted descending)
 * @param eigenvectors  Output eigenvectors (columns), 4x4 matrix
 * @return              0 on success, -1 on failure
 */
int doa_eigendecompose(const mat4x4_c *A,
                        float eigenvalues[4],
                        mat4x4_c *eigenvectors);

/**
 * @brief Run MUSIC DOA estimation.
 *
 * Uses noise subspace eigenvectors to compute MUSIC pseudospectrum.
 * P_MUSIC(theta) = 1 / (a^H(theta) * En * En^H * a(theta))
 * where En is the noise subspace matrix.
 *
 * @param state     DOA state
 * @param Rxx       Estimated covariance matrix
 * @param n_sources Number of signal sources (determines signal subspace dim)
 * @return          Number of DOA estimates
 */
int doa_estimate_music(doa_state_t *state, const mat4x4_c *Rxx, int n_sources);

/**
 * @brief Find peaks in DOA spectrum.
 *
 * Identifies local maxima in the computed spectrum above a threshold.
 *
 * @param spectrum      Input spectrum array
 * @param angles        Corresponding angle values
 * @param n_bins        Number of spectrum bins
 * @param n_peaks       Maximum number of peaks to find
 * @param results       Output DOA results
 * @return              Number of peaks found
 */
int doa_find_peaks(const float *spectrum, const float *angles,
                    int n_bins, int n_peaks,
                    doa_result_t *results);

/**
 * @brief Print DOA estimation results.
 *
 * @param state     DOA state
 */
void doa_print_results(const doa_state_t *state);

#endif /* DOA_ESTIMATION_H */
