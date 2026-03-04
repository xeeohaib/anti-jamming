/**
 * @file doa_estimation.c
 * @brief DOA estimation implementation using spatial spectral methods and MUSIC.
 */

#include "doa_estimation.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <float.h>

/* ============================================================================
 * Private helpers
 * ============================================================================ */

/** Compute steering vector a(theta) for ULA */
static void compute_steering(const doa_state_t *state, float angle_deg,
                              complex_f a[4])
{
    float c = 3e8f;
    float lambda = c / state->config.carrier_freq_hz;
    float d = state->config.element_spacing_m;
    float sin_theta = sinf(angle_deg * (float)M_PI / 180.0f);
    float dphi = 2.0f * (float)M_PI * d * sin_theta / lambda;

    for (int n = 0; n < 4; n++) {
        float phase = (float)n * dphi;
        a[n].re = cosf(phase);
        a[n].im = sinf(phase);
    }
}

/** Matrix-vector multiply: y = A * x (4x4 * 4x1) */
static void mat4_vec_mul(const mat4x4_c *A, const complex_f x[4], complex_f y[4])
{
    for (int i = 0; i < 4; i++) {
        y[i].re = 0.0f;
        y[i].im = 0.0f;
        for (int j = 0; j < 4; j++) {
            complex_f p = cmul(A->m[i][j], x[j]);
            y[i].re += p.re;
            y[i].im += p.im;
        }
    }
}

/** Hermitian inner product: a^H * b */
static complex_f inner_h(const complex_f a[4], const complex_f b[4])
{
    complex_f result = {0.0f, 0.0f};
    for (int i = 0; i < 4; i++) {
        complex_f ca = cconj(a[i]);
        complex_f p  = cmul(ca, b[i]);
        result.re += p.re;
        result.im += p.im;
    }
    return result;
}

/** Norm squared of a vector */
static float vec_norm2(const complex_f v[4])
{
    float s = 0.0f;
    for (int i = 0; i < 4; i++) s += cabs2(v[i]);
    return s;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void doa_init(doa_state_t *state, const doa_config_t *config)
{
    memset(state, 0, sizeof(doa_state_t));

    if (config) {
        state->config = *config;
    } else {
        state->config.element_spacing_m  = 0.015f;
        state->config.carrier_freq_hz    = 10.0e9f;
        state->config.n_sources          = 1;
        state->config.angle_min_deg      = -90.0f;
        state->config.angle_max_deg      = 90.0f;
        state->config.angle_step_deg     = DOA_ANGLE_STEP_DEG;
        state->config.n_signal_subspace  = 1;
    }

    /* Initialize angle grid */
    float step = state->config.angle_step_deg;
    float a = state->config.angle_min_deg;
    for (int i = 0; i < DOA_N_ANGLE_BINS && a <= state->config.angle_max_deg; i++) {
        state->angles[i] = a;
        a += step;
    }
}

int doa_estimate_spectral(doa_state_t *state, const mat4x4_c *Rxx)
{
    if (!state || !Rxx) return 0;

    /* Compute Bartlett beamformer spectrum: P(theta) = a^H * Rxx * a */
    for (int i = 0; i < DOA_N_ANGLE_BINS; i++) {
        float angle = state->angles[i];
        complex_f a[4];
        complex_f Ra[4];
        compute_steering(state, angle, a);
        mat4_vec_mul(Rxx, a, Ra);
        complex_f p = inner_h(a, Ra);
        state->spectrum[i] = p.re;  /* Power (real for Hermitian Rxx) */
    }

    state->n_results = doa_find_peaks(state->spectrum, state->angles,
                                       DOA_N_ANGLE_BINS,
                                       DOA_MAX_SOURCES, state->results);
    state->valid = (state->n_results > 0);
    return state->n_results;
}

int doa_eigendecompose(const mat4x4_c *A,
                        float eigenvalues[4],
                        mat4x4_c *eigenvectors)
{
    /* Power iteration to compute dominant eigenvectors of Hermitian matrix A.
     * For a 4x4 matrix we use sequential deflation. */

    mat4x4_c B;
    memcpy(&B, A, sizeof(mat4x4_c));

    for (int ev_idx = 0; ev_idx < 4; ev_idx++) {
        /* Initialize random vector */
        complex_f q[4];
        for (int i = 0; i < 4; i++) {
            q[i].re = (i == ev_idx) ? 1.0f : 0.1f;
            q[i].im = 0.0f;
        }

        /* Power iteration */
        for (int iter = 0; iter < 100; iter++) {
            complex_f Bq[4];
            mat4_vec_mul(&B, q, Bq);

            /* Normalize */
            float norm = sqrtf(vec_norm2(Bq));
            if (norm < 1e-30f) break;
            for (int i = 0; i < 4; i++) {
                q[i].re = Bq[i].re / norm;
                q[i].im = Bq[i].im / norm;
            }
        }

        /* Rayleigh quotient = eigenvalue estimate */
        complex_f Bq[4];
        mat4_vec_mul(&B, q, Bq);
        complex_f rq = inner_h(q, Bq);
        eigenvalues[ev_idx] = rq.re;

        /* Store eigenvector */
        for (int i = 0; i < 4; i++) {
            eigenvectors->m[i][ev_idx] = q[i];
        }

        /* Deflate: B = B - lambda * q * q^H */
        float lam = rq.re;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                complex_f outer = cmul(q[i], cconj(q[j]));
                B.m[i][j].re -= lam * outer.re;
                B.m[i][j].im -= lam * outer.im;
            }
        }
    }

    /* Simple bubble sort eigenvalues descending */
    for (int i = 0; i < 3; i++) {
        for (int j = i+1; j < 4; j++) {
            if (eigenvalues[j] > eigenvalues[i]) {
                float tmp = eigenvalues[i];
                eigenvalues[i] = eigenvalues[j];
                eigenvalues[j] = tmp;
                /* Swap eigenvector columns */
                for (int r = 0; r < 4; r++) {
                    complex_f tmp_v = eigenvectors->m[r][i];
                    eigenvectors->m[r][i] = eigenvectors->m[r][j];
                    eigenvectors->m[r][j] = tmp_v;
                }
            }
        }
    }

    return 0;
}

int doa_estimate_music(doa_state_t *state, const mat4x4_c *Rxx, int n_sources)
{
    if (!state || !Rxx || n_sources <= 0 || n_sources >= 4) return 0;

    /* Step 1: Eigendecompose Rxx */
    float eigenvalues[4];
    mat4x4_c eigenvectors;
    doa_eigendecompose(Rxx, eigenvalues, &eigenvectors);

    /* Noise subspace: eigenvectors corresponding to smallest eigenvalues */
    /* Signal subspace dimension = n_sources, noise = 4 - n_sources */
    int n_noise = 4 - n_sources;

    /* Step 2: Compute MUSIC spectrum */
    for (int i = 0; i < DOA_N_ANGLE_BINS; i++) {
        complex_f a[4];
        compute_steering(state, state->angles[i], a);

        /* P_MUSIC = 1 / sum_k |a^H * e_noise_k|^2 */
        float denom = 0.0f;
        for (int k = n_sources; k < 4; k++) {
            /* Extract k-th noise eigenvector */
            complex_f en[4];
            for (int r = 0; r < 4; r++) {
                en[r] = eigenvectors.m[r][k];
            }
            complex_f proj = inner_h(a, en);
            denom += cabs2(proj);
        }

        state->spectrum[i] = (denom > 1e-30f) ? (1.0f / denom) : 1e30f;
    }

    /* Step 3: Find peaks */
    state->n_results = doa_find_peaks(state->spectrum, state->angles,
                                       DOA_N_ANGLE_BINS,
                                       n_sources, state->results);
    state->valid = (state->n_results > 0);
    return state->n_results;
}

int doa_find_peaks(const float *spectrum, const float *angles,
                    int n_bins, int n_peaks,
                    doa_result_t *results)
{
    if (!spectrum || !angles || !results || n_bins < 3) return 0;

    /* Convert to dB for peak finding */
    float spec_db[DOA_N_ANGLE_BINS];
    float max_val = 0.0f;
    for (int i = 0; i < n_bins; i++) {
        if (spectrum[i] > max_val) max_val = spectrum[i];
    }
    for (int i = 0; i < n_bins; i++) {
        float val = spectrum[i] / (max_val + 1e-30f);
        spec_db[i] = (val > 1e-30f) ? (10.0f * log10f(val)) : -300.0f;
    }

    /* Find local maxima */
    int n_found = 0;
    for (int i = 0; i < n_peaks; i++) {
        results[i].valid = false;
        results[i].angle_deg  = 0.0f;
        results[i].power_db   = -300.0f;
    }

    /* Mark all local maxima */
    for (int i = 1; i < n_bins - 1; i++) {
        if (spec_db[i] > spec_db[i-1] && spec_db[i] > spec_db[i+1]) {
            /* Local maximum: insert into sorted results */
            if (n_found < n_peaks) {
                results[n_found].angle_deg = angles[i];
                results[n_found].power_db  = spec_db[i];
                results[n_found].valid     = true;
                n_found++;
            } else {
                /* Replace smallest */
                int min_idx = 0;
                for (int k = 1; k < n_peaks; k++) {
                    if (results[k].power_db < results[min_idx].power_db)
                        min_idx = k;
                }
                if (spec_db[i] > results[min_idx].power_db) {
                    results[min_idx].angle_deg = angles[i];
                    results[min_idx].power_db  = spec_db[i];
                }
            }
        }
    }

    /* Sort results by descending power */
    for (int i = 0; i < n_found - 1; i++) {
        for (int j = i+1; j < n_found; j++) {
            if (results[j].power_db > results[i].power_db) {
                doa_result_t tmp = results[i];
                results[i] = results[j];
                results[j] = tmp;
            }
        }
    }

    return n_found;
}

void doa_print_results(const doa_state_t *state)
{
    if (!state) return;
    printf("=== DOA Estimation Results (%d found) ===\n", state->n_results);
    for (int i = 0; i < state->n_results; i++) {
        if (state->results[i].valid) {
            printf("  Source %d: angle=%.1f deg, power=%.2f dB\n",
                   i+1, state->results[i].angle_deg, state->results[i].power_db);
        }
    }
    printf("==========================================\n");
}
