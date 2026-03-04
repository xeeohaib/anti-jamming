/**
 * @file mvdr_algorithm.c
 * @brief Software MVDR beamforming algorithm implementation.
 */

#include "mvdr_algorithm.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <float.h>

/* ============================================================================
 * Private helper functions
 * ============================================================================ */

/** Matrix-vector multiply: y = A * x */
static void mat_vec_mul(const mat4x4_c *A, const vec4_c *x, vec4_c *y)
{
    for (int i = 0; i < 4; i++) {
        y->v[i].re = 0.0f;
        y->v[i].im = 0.0f;
        for (int j = 0; j < 4; j++) {
            complex_f prod = cmul(A->m[i][j], x->v[j]);
            y->v[i].re += prod.re;
            y->v[i].im += prod.im;
        }
    }
}

/** Inner product: result = a^H * b */
static complex_f inner_product_conj(const vec4_c *a, const vec4_c *b)
{
    complex_f result = {0.0f, 0.0f};
    for (int i = 0; i < 4; i++) {
        complex_f conj_a = cconj(a->v[i]);
        complex_f prod   = cmul(conj_a, b->v[i]);
        result.re += prod.re;
        result.im += prod.im;
    }
    return result;
}

/** Compute trace of a 4x4 complex matrix (sum of diagonal real parts) */
static float mat_trace_re(const mat4x4_c *A)
{
    return A->m[0][0].re + A->m[1][1].re + A->m[2][2].re + A->m[3][3].re;
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void mvdr_init(mvdr_state_t *state, const mvdr_params_t *params)
{
    memset(state, 0, sizeof(mvdr_state_t));
    if (params) {
        state->params = *params;
    } else {
        state->params.diag_load_factor  = MVDR_DEFAULT_DIAG_LOAD;
        state->params.n_snapshots       = 256;
        state->params.element_spacing_m = 0.015f;   /* ~lambda/2 at 10 GHz */
        state->params.carrier_freq_hz   = 10.0e9f;  /* 10 GHz (X-band) */
        state->params.look_angle_deg    = 0.0f;     /* broadside */
    }
    state->weights_valid = false;
}

void mvdr_estimate_covariance(mvdr_state_t *state,
                               const float snapshots_re[][4],
                               const float snapshots_im[][4],
                               int n_snap)
{
    /* Clear covariance matrix */
    memset(&state->Rxx, 0, sizeof(mat4x4_c));

    if (n_snap <= 0) return;

    /* Accumulate outer products: Rxx += x * x^H */
    for (int k = 0; k < n_snap; k++) {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                /* x_i * conj(x_j) */
                float re_ij = snapshots_re[k][i]*snapshots_re[k][j]
                            + snapshots_im[k][i]*snapshots_im[k][j];
                float im_ij = snapshots_im[k][i]*snapshots_re[k][j]
                            - snapshots_re[k][i]*snapshots_im[k][j];
                state->Rxx.m[i][j].re += re_ij;
                state->Rxx.m[i][j].im += im_ij;
            }
        }
    }

    /* Normalize by N */
    float inv_n = 1.0f / (float)n_snap;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            state->Rxx.m[i][j].re *= inv_n;
            state->Rxx.m[i][j].im *= inv_n;
        }
    }
}

void mvdr_apply_diagonal_loading(mvdr_state_t *state)
{
    float trace = mat_trace_re(&state->Rxx);
    float delta = state->params.diag_load_factor * trace;

    /* Add delta to real diagonal */
    for (int i = 0; i < 4; i++) {
        state->Rxx.m[i][i].re += delta;
    }
}

int mvdr_matrix_inverse(const mat4x4_c *A, mat4x4_c *A_inv, float *cond_num)
{
    /* Gauss-Jordan elimination on augmented matrix [A | I] */
    complex_f aug[4][8];
    int i, j, k;

    /* Initialize augmented matrix */
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            aug[i][j]   = A->m[i][j];
            aug[i][j+4] = (i == j) ? (complex_f){1.0f, 0.0f} : (complex_f){0.0f, 0.0f};
        }
    }

    float max_pivot = 0.0f;
    float min_pivot = FLT_MAX;

    /* Forward elimination with partial pivoting */
    for (k = 0; k < 4; k++) {
        /* Find pivot */
        int pivot_row = k;
        float max_val = cabs_f(aug[k][k]);
        for (i = k+1; i < 4; i++) {
            float val = cabs_f(aug[i][k]);
            if (val > max_val) {
                max_val   = val;
                pivot_row = i;
            }
        }

        /* Swap rows */
        if (pivot_row != k) {
            for (j = 0; j < 8; j++) {
                complex_f tmp   = aug[k][j];
                aug[k][j]       = aug[pivot_row][j];
                aug[pivot_row][j] = tmp;
            }
        }

        /* Check for singularity */
        if (cabs_f(aug[k][k]) < 1e-30f) {
            printf("MVDR: Matrix is singular at pivot %d\n", k);
            return -1;
        }

        /* Track pivot magnitudes for condition number estimate */
        float pv = cabs_f(aug[k][k]);
        if (pv > max_pivot) max_pivot = pv;
        if (pv < min_pivot) min_pivot = pv;

        /* Scale pivot row */
        complex_f inv_pivot = cdiv((complex_f){1.0f, 0.0f}, aug[k][k]);
        for (j = 0; j < 8; j++) {
            aug[k][j] = cmul(aug[k][j], inv_pivot);
        }

        /* Eliminate column */
        for (i = 0; i < 4; i++) {
            if (i == k) continue;
            complex_f factor = aug[i][k];
            for (j = 0; j < 8; j++) {
                aug[i][j] = csub(aug[i][j], cmul(factor, aug[k][j]));
            }
        }
    }

    /* Extract inverse */
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            A_inv->m[i][j] = aug[i][j+4];
        }
    }

    /* Condition number estimate */
    if (cond_num) {
        *cond_num = (min_pivot > 0.0f) ? (max_pivot / min_pivot) : FLT_MAX;
    }

    return 0;
}

void mvdr_compute_steering_vector(mvdr_state_t *state, float angle_deg)
{
    float c = 3e8f;  /* Speed of light */
    float lambda = c / state->params.carrier_freq_hz;
    float d = state->params.element_spacing_m;
    float sin_theta = sinf(angle_deg * (float)M_PI / 180.0f);

    /* Phase increment between adjacent elements */
    float dphi = 2.0f * (float)M_PI * d * sin_theta / lambda;

    for (int n = 0; n < 4; n++) {
        float phase = (float)n * dphi;
        state->steering_vec.v[n].re = cosf(phase);
        state->steering_vec.v[n].im = sinf(phase);
    }

    state->params.look_angle_deg = angle_deg;
}

int mvdr_compute_weights(mvdr_state_t *state)
{
    /* Step 1: v = Rxx^-1 * a */
    vec4_c v;
    mat_vec_mul(&state->Rxx_inv, &state->steering_vec, &v);

    /* Step 2: denom = a^H * v (should be real and positive for valid Rxx) */
    complex_f denom = inner_product_conj(&state->steering_vec, &v);

    if (cabs2(denom) < 1e-30f) {
        printf("MVDR: Denominator near zero, cannot compute weights\n");
        return -1;
    }

    /* Step 3: w = v / denom */
    for (int i = 0; i < 4; i++) {
        state->weights.v[i] = cdiv(v.v[i], denom);
    }

    state->weights_valid = true;
    return 0;
}

int mvdr_update(mvdr_state_t *state,
                const float snapshots_re[][4],
                const float snapshots_im[][4],
                int n_snap,
                float angle_deg)
{
    /* 1. Estimate covariance */
    mvdr_estimate_covariance(state, snapshots_re, snapshots_im, n_snap);

    /* 2. Apply diagonal loading */
    mvdr_apply_diagonal_loading(state);

    /* 3. Compute inverse */
    int ret = mvdr_matrix_inverse(&state->Rxx, &state->Rxx_inv, &state->condition_num);
    if (ret != 0) {
        printf("MVDR: Matrix inversion failed (condition number: %.2e)\n",
               state->condition_num);
        return -1;
    }

    /* 4. Compute steering vector */
    mvdr_compute_steering_vector(state, angle_deg);

    /* 5. Compute weights */
    ret = mvdr_compute_weights(state);
    if (ret != 0) return -1;

    printf("MVDR: Update complete. Look angle=%.1f deg, condition=%.2e\n",
           angle_deg, state->condition_num);
    for (int i = 0; i < 4; i++) {
        printf("  w[%d] = %.6f + j%.6f (mag=%.6f, phase=%.2f deg)\n", i,
               state->weights.v[i].re, state->weights.v[i].im,
               sqrtf(cabs2(state->weights.v[i])),
               atan2f(state->weights.v[i].im, state->weights.v[i].re) * 180.0f / (float)M_PI);
    }

    return 0;
}

void mvdr_apply_weights(const vec4_c *weights,
                        const float x_re[4], const float x_im[4],
                        float *y_re, float *y_im)
{
    complex_f y = {0.0f, 0.0f};
    for (int i = 0; i < 4; i++) {
        /* y += conj(w[i]) * x[i] */
        complex_f w_conj = cconj(weights->v[i]);
        complex_f x_i = {x_re[i], x_im[i]};
        complex_f prod = cmul(w_conj, x_i);
        y.re += prod.re;
        y.im += prod.im;
    }
    *y_re = y.re;
    *y_im = y.im;
}

void mvdr_compute_pattern(const mvdr_state_t *state,
                           const float *angles_deg, int n_angles,
                           float *pattern)
{
    float c = 3e8f;
    float lambda = c / state->params.carrier_freq_hz;
    float d = state->params.element_spacing_m;

    for (int ai = 0; ai < n_angles; ai++) {
        float sin_theta = sinf(angles_deg[ai] * (float)M_PI / 180.0f);
        float dphi = 2.0f * (float)M_PI * d * sin_theta / lambda;

        /* Compute a^H * w for this angle */
        complex_f response = {0.0f, 0.0f};
        for (int n = 0; n < 4; n++) {
            float phase = (float)n * dphi;
            complex_f a_n = {cosf(phase), sinf(phase)};
            complex_f w_conj = cconj(state->weights.v[n]);
            complex_f prod = cmul(w_conj, a_n);
            response.re += prod.re;
            response.im += prod.im;
        }
        pattern[ai] = cabs2(response);
    }
}
