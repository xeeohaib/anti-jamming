/**
 * @file mvdr_algorithm.h
 * @brief Software MVDR (Minimum Variance Distortionless Response) algorithm.
 *
 * Provides a complete software implementation of the MVDR beamforming algorithm
 * for verification, simulation, and ARM Cortex-A9 fallback execution.
 *
 * MVDR weights: w = (Rxx^-1 * a) / (a^H * Rxx^-1 * a)
 * where Rxx is the spatial covariance matrix and a is the steering vector.
 */

#ifndef MVDR_ALGORITHM_H
#define MVDR_ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ============================================================================
 * Constants
 * ============================================================================ */

#define MVDR_NUM_ELEMENTS   4           /**< Number of antenna array elements */
#define MVDR_MAX_SNAPSHOTS  1024  /**< Maximum allowed snapshot buffer (hard limit for
                                   *   stack/heap allocation). Default run-time count
                                   *   (MVDR_DEFAULT_N_SNAPSHOTS / CFG_N_SNAPSHOTS) is
                                   *   256; increase this only if the buffer allocation
                                   *   in mvdr_algorithm.c is updated accordingly. */
#define MVDR_DEFAULT_DIAG_LOAD 0.01f   /**< Default diagonal loading factor */

/* ============================================================================
 * Data structures
 * ============================================================================ */

/** Complex number type (single precision) */
typedef struct {
    float re;   /**< Real part */
    float im;   /**< Imaginary part */
} complex_f;

/** 4x4 complex matrix */
typedef struct {
    complex_f m[4][4];
} mat4x4_c;

/** 4-element complex vector */
typedef struct {
    complex_f v[4];
} vec4_c;

/** MVDR algorithm parameters */
typedef struct {
    float   diag_load_factor;   /**< Diagonal loading: delta = factor * trace(Rxx) */
    int     n_snapshots;        /**< Number of snapshots for covariance estimation */
    float   element_spacing_m;  /**< Array element spacing in meters */
    float   carrier_freq_hz;    /**< Carrier frequency in Hz */
    float   look_angle_deg;     /**< Desired look direction in degrees (0 = broadside) */
} mvdr_params_t;

/** MVDR algorithm state */
typedef struct {
    mvdr_params_t   params;
    mat4x4_c        Rxx;            /**< Spatial covariance matrix */
    mat4x4_c        Rxx_inv;        /**< Inverse covariance matrix */
    vec4_c          steering_vec;   /**< Steering vector for look direction */
    vec4_c          weights;        /**< Computed MVDR weights */
    float           condition_num;  /**< Condition number (for diagnostics) */
    bool            weights_valid;
} mvdr_state_t;

/* ============================================================================
 * Function prototypes
 * ============================================================================ */

/**
 * @brief Initialize MVDR algorithm state.
 *
 * @param state     Pointer to MVDR state structure
 * @param params    Algorithm parameters
 */
void mvdr_init(mvdr_state_t *state, const mvdr_params_t *params);

/**
 * @brief Estimate spatial covariance matrix from snapshots.
 *
 * Rxx = (1/N) * sum_k x(k) * x^H(k)
 *
 * @param state         MVDR state
 * @param snapshots_re  Real parts of snapshots [N_SNAP][4]
 * @param snapshots_im  Imaginary parts of snapshots [N_SNAP][4]
 * @param n_snap        Number of snapshots
 */
void mvdr_estimate_covariance(mvdr_state_t *state,
                               const float snapshots_re[][4],
                               const float snapshots_im[][4],
                               int n_snap);

/**
 * @brief Apply diagonal loading to covariance matrix for robustness.
 *
 * Rxx_loaded = Rxx + delta * I
 * where delta = diag_load_factor * trace(Rxx)
 *
 * @param state     MVDR state (modifies Rxx in-place)
 */
void mvdr_apply_diagonal_loading(mvdr_state_t *state);

/**
 * @brief Compute 4x4 complex matrix inverse using Gauss-Jordan elimination.
 *
 * @param A         Input matrix (4x4 complex)
 * @param A_inv     Output inverse matrix
 * @param cond_num  Output condition number estimate
 * @return          0 on success, -1 if matrix is singular
 */
int mvdr_matrix_inverse(const mat4x4_c *A, mat4x4_c *A_inv, float *cond_num);

/**
 * @brief Generate steering vector for given angle.
 *
 * For a uniform linear array (ULA) with half-wavelength spacing:
 *   a[n] = exp(j * pi * n * sin(theta)), n = 0,...,N-1
 *
 * @param state         MVDR state (updates steering_vec)
 * @param angle_deg     Steering angle in degrees
 */
void mvdr_compute_steering_vector(mvdr_state_t *state, float angle_deg);

/**
 * @brief Compute MVDR beamforming weights.
 *
 * w = (Rxx^-1 * a) / (a^H * Rxx^-1 * a)
 *
 * @param state     MVDR state (reads Rxx_inv and steering_vec, writes weights)
 * @return          0 on success, -1 if denominator is zero
 */
int mvdr_compute_weights(mvdr_state_t *state);

/**
 * @brief Full MVDR update: covariance -> inversion -> weights.
 *
 * @param state         MVDR state
 * @param snapshots_re  Real parts of input snapshots [N][4]
 * @param snapshots_im  Imaginary parts of input snapshots [N][4]
 * @param n_snap        Number of snapshots
 * @param angle_deg     Desired look angle in degrees
 * @return              0 on success, negative on error
 */
int mvdr_update(mvdr_state_t *state,
                const float snapshots_re[][4],
                const float snapshots_im[][4],
                int n_snap,
                float angle_deg);

/**
 * @brief Apply beamforming weights to a single sample snapshot.
 *
 * y = w^H * x
 *
 * @param weights   Beamforming weights [4]
 * @param x_re      Input sample real parts [4]
 * @param x_im      Input sample imaginary parts [4]
 * @param y_re      Output beamformed sample real part
 * @param y_im      Output beamformed sample imaginary part
 */
void mvdr_apply_weights(const vec4_c *weights,
                        const float x_re[4], const float x_im[4],
                        float *y_re, float *y_im);

/**
 * @brief Compute array pattern (gain vs angle) for current weights.
 *
 * Useful for verification: computes |w^H * a(theta)|^2 for angles -90..90 deg.
 *
 * @param state         MVDR state
 * @param angles_deg    Array of angles to evaluate
 * @param n_angles      Number of angles
 * @param pattern       Output gain values [n_angles]
 */
void mvdr_compute_pattern(const mvdr_state_t *state,
                           const float *angles_deg, int n_angles,
                           float *pattern);

/* ============================================================================
 * Complex arithmetic helpers (inline)
 * ============================================================================ */

static inline complex_f cadd(complex_f a, complex_f b) {
    return (complex_f){a.re + b.re, a.im + b.im};
}

static inline complex_f csub(complex_f a, complex_f b) {
    return (complex_f){a.re - b.re, a.im - b.im};
}

static inline complex_f cmul(complex_f a, complex_f b) {
    return (complex_f){a.re*b.re - a.im*b.im, a.re*b.im + a.im*b.re};
}

static inline complex_f cdiv(complex_f a, complex_f b) {
    float denom = b.re*b.re + b.im*b.im;
    if (denom == 0.0f) return (complex_f){0.0f, 0.0f};
    return (complex_f){(a.re*b.re + a.im*b.im)/denom,
                       (a.im*b.re - a.re*b.im)/denom};
}

static inline complex_f cconj(complex_f a) {
    return (complex_f){a.re, -a.im};
}

static inline float cabs2(complex_f a) {
    return a.re*a.re + a.im*a.im;
}

static inline float cabs_f(complex_f a) {
    return sqrtf(a.re*a.re + a.im*a.im);
}

static inline complex_f cscale(complex_f a, float s) {
    return (complex_f){a.re*s, a.im*s};
}

#endif /* MVDR_ALGORITHM_H */
