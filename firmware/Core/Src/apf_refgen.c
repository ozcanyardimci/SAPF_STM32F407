/* USER CODE BEGIN Header */
/**
 * @file    apf_refgen.c
 * @brief   Reference current generator (outer control loop) for the
 *          Single-Phase Shunt Active Power Filter (SAPF) on STM32F407VGT6.
 *
 * Implements the APF_RefGen algorithm: seven steps of instantaneous power
 * theory that compute the harmonic compensation current reference.
 *
 * Physical principle:
 *   A nonlinear load draws: I_L1 = I_fundamental + I_harmonics
 *   The APF must inject:    I_APF = −I_harmonics
 *   Result at the grid:     I_grid = I_L1 + I_APF = I_fundamental only
 *
 * How this module produces −I_harmonics:
 *   After filter convergence, i_s_ref ≈ I_fund_peak × sin(ωt)
 *   I_APF_ref = i_s_ref − I_L1
 *             = I_fund × sin(ωt) − [I_fund × sin(ωt) + I_harmonics(t)]
 *             = −I_harmonics(t)
 *   APF_HystCtrl forces I_APF to track I_APF_ref, completing the loop.
 *
 * Called from:  TIM1 update ISR (stm32f4xx_it.c) at 20kHz (Ts = 50µs)
 * Init called:  main.c USER CODE BEGIN 2, before DMA and timer start
 */
/* USER CODE END Header */

#include "main.h"        /* stm32f4xx_hal.h, project-wide defines          */
#include <math.h>        /* sqrtf()                                         */
#include "apf_refgen.h"  /* PID_t typedef, function prototypes              */

/* =========================================================================
 * ALGORITHM CONSTANTS
 * ========================================================================= */

/**
 * TS  —  0.00005 seconds (50 microseconds)
 *
 * ISR sample period.  TIM1 fires at 168MHz / (8399+1) = exactly 20000Hz,
 * so every call to APF_RefGen_Update represents 50µs of real time.
 * Used in the discrete PID: integral += Ki × error × TS.
 * Used in the derivative:   derivative = Kd × (e−e_prev) / TS.
 */
#define TS      0.00005f

/**
 * TAU  —  0.0318 seconds
 *
 * Time constant of the first-order IIR low-pass filter.
 *   tau = 1 / (2π × fc) = 1 / (2π × 5Hz) ≈ 0.0318s
 *
 * The 5Hz cutoff was chosen to reject all AC power components:
 *   - Fundamental power oscillates at 100Hz  (2 × 50Hz grid)
 *   - Harmonic cross-products at 200Hz, 300Hz, ...
 * All of these are far above 5Hz and are attenuated by the filter.
 * Only the slowly varying average active power P_avg passes through.
 * 5Hz is a standard choice in published shunt APF implementations.
 */
#define TAU     0.0318f

/**
 * ALPHA  —  TS / (TAU + TS)  = 0.001570  (compile-time constant)
 *
 * First-order IIR filter coefficient, derived from forward Euler
 * discretisation of the continuous-time single-pole LPF H(s) = 1/(τs+1):
 *   y[k] = y[k-1] + ALPHA × (x[k] − y[k-1])
 *
 * Applied to both the P_avg and V_S_sq_avg filters identically.
 * Numerical value: 50e-6 / (0.0318 + 50e-6) ≈ 0.001570.
 */
#define ALPHA   (TS / (TAU + TS))

/**
 * VDC_REF  —  90.0 volts
 *
 * DC link voltage setpoint.
 *
 * The DC link must exceed the grid peak (67.9V) so the H-bridge can
 * force current through the APF inductor in either direction.  The
 * minimum required headroom depends on the maximum current slew rate:
 *   ΔV_min = L × (ΔI / Ts) = 0.00421H × (1A / 50µs) ≈ 84V worst case
 * 90V provides 22.1V margin above the 67.9V grid peak, which is
 * sufficient for rated operating conditions.
 */
#define VDC_REF  90.0f

/**
 * PID_OUT_LIMIT  —  1.0 ampere
 *
 * Maximum magnitude of the DC link PID output (delta_I).
 *
 * delta_I is a small correction current superimposed on the harmonic
 * compensation reference.  It must remain small so it does not distort
 * the ideal i_s_ref waveform and degrade APF THD performance.
 * ±1.0A is sufficient to regulate a DC link capacitor while keeping
 * the waveform distortion negligible.
 */
#define PID_OUT_LIMIT  1.0f

/**
 * PID_INT_LIMIT  —  0.5 ampere
 *
 * Maximum magnitude of the PID integral accumulator (anti-windup clamp).
 *
 * The integral is clamped BEFORE the proportional and derivative terms
 * are added (not after).  This is intentional:
 *   - Clamping only the total output allows the integral to wind up to
 *     an arbitrarily large value while the output is saturated.  When
 *     saturation ends, the large integral causes overshoot.
 *   - Clamping the integral itself keeps it bounded at all times.
 *     0.5A leaves headroom for the proportional term within the ±1.0A
 *     output limit, so both limits are independently enforced.
 */
#define PID_INT_LIMIT  0.5f

/* =========================================================================
 * FILE-SCOPE STATE VARIABLES
 * ========================================================================= */

/**
 * P_avg  —  running filtered average of instantaneous active power [W]
 *
 * Updated every ISR call by the 5Hz IIR LPF in Step 2.
 * Initialised to 0W; converges to the true average power in ~3×TAU ≈ 95ms.
 *
 * volatile: the ISR writes this every 50µs; the debugger and CubeMonitor
 * read it asynchronously.  volatile prevents the compiler from caching the
 * value in a register and ensures every read fetches from memory.
 */
static volatile float P_avg = 0.0f;

/**
 * V_S_sq_avg  —  running filtered mean of V_S² [V²]
 *
 * Updated every ISR call by the 5Hz IIR LPF in Step 3.
 * Its square root gives V_S_rms without storing a complete 50Hz cycle
 * (which would require 400 samples at 20kHz).
 *
 * Initial value: 2449.7 V² = (V_S_FULL_SCALE / √2)²
 *              = (70.0V / 1.41421)² = 49.497² ≈ 2449.7
 *
 * Why not start at 0?
 *   If V_S_sq_avg = 0 at power-up, V_S_rms = 0 → clamped to 1V.
 *   Then V_S_peak ≈ 1.4V, and u_t = V_S / 1.4V ≈ ±35 (should be ±1).
 *   I_APF_ref spikes to ±35× the intended value for the ~95ms
 *   convergence transient — dangerous on real hardware.
 *   Pre-seeding with the expected steady-state value eliminates this
 *   spike entirely.  (Bug 5, CLAUDE.md Section 13.)
 *
 * volatile: same reasoning as P_avg.
 */
static volatile float V_S_sq_avg = 2449.7f;

/**
 * vdc_pid  —  DC link voltage PID controller instance.
 *
 * Gains are set in APF_RefGen_Init() to Kp=0.02, Ki=0.5, Kd=0.0.
 * These give ~8Hz closed-loop bandwidth, safely below the 50Hz grid.
 * Simulink used Kp=0.1, Ki=50 (73Hz bandwidth) — integral windup on
 * every startup transient on real hardware.  Reduced gains are stable.
 */
static PID_t vdc_pid;

/* =========================================================================
 * STATIC (PRIVATE) FUNCTIONS
 * ========================================================================= */

/**
 * pid_update
 * ----------
 * One step of a discrete PID controller with independent anti-windup.
 *
 * Discretisation: forward Euler (same method used for P_avg filter).
 *   proportional = Kp × e(k)
 *   integral    += Ki × e(k) × TS          [clamped to ±PID_INT_LIMIT]
 *   derivative   = Kd × (e(k) − e(k-1)) / TS
 *   output       = P + I + D               [clamped to ±PID_OUT_LIMIT]
 *
 * Anti-windup strategy:
 *   The integral is clamped BEFORE the final sum.  This ensures both
 *   the integral and the output limits are independently enforced at all
 *   times.  If only the output were clamped, a wound-up integral could
 *   cause overshoot when the output comes out of saturation.
 *
 * Parameters:
 *   pid    pointer to PID_t holding gains and persistent state
 *   error  [V]   current control error = setpoint − measurement
 *
 * Returns:
 *   output  [A]  PID output clamped to ±PID_OUT_LIMIT
 */
static float pid_update(PID_t *pid, float error)
{
    /* Proportional term */
    float proportional = pid->Kp * error;

    /* Integral term — accumulate, then clamp (anti-windup) */
    pid->integral += pid->Ki * error * TS;
    if (pid->integral >  PID_INT_LIMIT) { pid->integral =  PID_INT_LIMIT; }
    if (pid->integral < -PID_INT_LIMIT) { pid->integral = -PID_INT_LIMIT; }

    /* Derivative term — backward difference approximation
     * Kd = 0.0f so this evaluates to zero; kept for completeness     */
    float derivative = pid->Kd * (error - pid->prev_error) / TS;
    pid->prev_error = error;   /* store e(k) as e(k-1) for next call */

    /* Sum and clamp total output */
    float output = proportional + pid->integral + derivative;
    if (output >  PID_OUT_LIMIT) { output =  PID_OUT_LIMIT; }
    if (output < -PID_OUT_LIMIT) { output = -PID_OUT_LIMIT; }

    return output;
}

/* =========================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================= */

/**
 * APF_RefGen_Init
 * ---------------
 * Resets all persistent algorithm state to known safe initial values.
 *
 * V_S_sq_avg is pre-loaded to 2449.7 V² = (V_S_FULL_SCALE / √2)²
 * = (70.0V / 1.41421)² to prevent the startup transient described in
 * Bug 5 (CLAUDE.md Section 13).  Update this value after confirming the
 * real sensor gain and voltage divider ratios on hardware.
 *
 * Must be called as step 1 of the startup sequence (CLAUDE.md Section 10),
 * before HAL_ADC_Start_DMA() and before any TIM1 start call.
 * Also safe to call on warm restart (watchdog or debugger).
 *
 * Inputs:  none
 * Outputs: P_avg = 0, V_S_sq_avg = 2449.7, vdc_pid initialised
 */
void APF_RefGen_Init(void)
{
    P_avg = 0.0f;

    /* Initialized to (V_S_FULL_SCALE / √2)²
     * = (70.0f / √2)² = 70² / 2 = 4900 / 2 = 2450.0f exactly
     * Prevents startup transient where V_S_rms clamps to 1.0V
     * causing u_t to reach ±35 instead of ±1.
     * Update this value when V_S_FULL_SCALE is calibrated
     * against real sensor measurements.                     */
    V_S_sq_avg = 2450.0f;

    /* DC link voltage PID gains.
     * Kp=0.02, Ki=0.5  →  ~8Hz bandwidth, stable on real hardware.
     * Kd=0.0            →  derivative disabled to avoid noise amplification. */
    vdc_pid.Kp         = 0.02f;
    vdc_pid.Ki         = 0.5f;
    vdc_pid.Kd         = 0.0f;
    vdc_pid.integral   = 0.0f;
    vdc_pid.prev_error = 0.0f;
}

/**
 * APF_RefGen_Update
 * ------------------
 * Runs the seven-step reference current generator.
 *
 * Called from the TIM1 update ISR at 20kHz (every 50µs).
 *
 * After filter convergence (~95ms), the output I_APF_ref ≈ −I_harmonics:
 *   i_s_ref  = (I_active + delta_I) × u_t  ≈  I_fund_peak × sin(ωt)
 *   I_APF_ref = i_s_ref − I_L1
 *             = I_fund × sin(ωt) − [I_fund × sin(ωt) + I_harmonics(t)]
 *             = −I_harmonics(t)
 * APF_HystCtrl then forces the real APF current to track I_APF_ref.
 *
 * Parameters:
 *   V_S   [V]  Scaled grid voltage from ADC    (bipolar ±70V range)
 *   I_L1  [A]  Scaled load current from ADC   (bipolar ±5A range)
 *   V_dc  [V]  Scaled DC link voltage from ADC (unipolar 0–100V range)
 *
 * Returns:
 *   I_APF_ref  [A]  Reference compensation current for APF_HystCtrl_Update
 */
float APF_RefGen_Update(float V_S, float I_L1, float V_dc)
{
    /* ------------------------------------------------------------------
     * Step 1 — Instantaneous power  [W]
     *
     * p = V_S × I_L1 decomposes into:
     *   DC term:  V_rms × I_rms × cos(φ) = P_average  (active power)
     *   AC terms: oscillating at 2ω, 4ω, 6ω, ...      (harmonic powers)
     * Only the DC term is needed; the 5Hz IIR in Step 2 rejects the AC.
     * ------------------------------------------------------------------ */
    float p = V_S * I_L1;

    /* ------------------------------------------------------------------
     * Step 2 — Average active power IIR filter  [W]
     *
     * Single-pole IIR, cutoff 5Hz, coefficient ALPHA = TS/(TAU+TS).
     * Rejects the 100Hz and higher harmonic power oscillations.
     * Converges to steady state in approximately 3 × TAU ≈ 95ms.
     * ------------------------------------------------------------------ */
    P_avg = P_avg + ALPHA * (p - P_avg);

    /* ------------------------------------------------------------------
     * Step 3 — Running RMS estimate  [V]
     *
     * Filter V_S² to obtain its running mean, then take sqrt for RMS.
     * This avoids buffering a full 50Hz grid cycle (400 samples at 20kHz).
     *
     * Guard: clamp V_S_rms to 1V minimum to prevent division by zero
     * during startup or grid loss.
     * ------------------------------------------------------------------ */
    V_S_sq_avg = V_S_sq_avg + ALPHA * (V_S * V_S - V_S_sq_avg);
    float V_S_rms = sqrtf((float)V_S_sq_avg);
    if (V_S_rms < 1.0f) { V_S_rms = 1.0f; }

    /* ------------------------------------------------------------------
     * Step 4 — Active current peak amplitude  [A]
     *
     * Derivation:
     *   For a purely active load: P = V_rms × I_rms
     *   I_rms = P_avg / V_S_rms
     *   I_peak = I_rms × √2 = P_avg × √2 / V_S_rms
     *
     * Guard: clamp to zero if P_avg is negative.
     *   Negative P_avg means the load is sourcing power (e.g. regeneration).
     *   This APF does not support grid-feed operation; clamping prevents
     *   the reference from inverting and destabilising the system.
     * ------------------------------------------------------------------ */
    float I_active = P_avg * sqrtf(2.0f) / V_S_rms;
    if (I_active < 0.0f) { I_active = 0.0f; }

    /* ------------------------------------------------------------------
     * Step 5 — DC link PID correction  [A]
     *
     * Computes a small correction current delta_I to maintain the DC
     * link capacitor at VDC_REF = 90V.
     *   Positive error (V_dc too low) → positive delta_I → APF draws
     *   slightly more energy from the grid, charging the capacitor.
     *   Negative error (V_dc too high, unlikely) → negative delta_I.
     * Bounded to ±PID_OUT_LIMIT = ±1A to protect harmonic compensation.
     * ------------------------------------------------------------------ */
    float e_Vdc  = VDC_REF - V_dc;
    float delta_I = pid_update(&vdc_pid, e_Vdc);

    /* ------------------------------------------------------------------
     * Step 6 — Unit sine template  [dimensionless, nominal range ±1]
     *
     * u_t = V_S / V_S_peak normalises the measured grid voltage to a
     * unit sine wave.  This provides implicit grid synchronisation
     * without a phase-locked loop (PLL), and automatically tracks
     * grid frequency and phase variations.
     *
     * Guard: clamp V_S_peak to 1V minimum — same reason as V_S_rms guard.
     * ------------------------------------------------------------------ */
    float V_S_peak = sqrtf(2.0f) * V_S_rms;
    if (V_S_peak < 1.0f) { V_S_peak = 1.0f; }
    float u_t = V_S / V_S_peak;

    /* ------------------------------------------------------------------
     * Step 7 — Reference compensation current  [A]
     *
     * i_s_ref   = desired sinusoidal source current the grid should supply
     * I_APF_ref = correction current APF must inject into the AC line
     *
     * I_APF_ref = i_s_ref − I_L1
     *
     * After convergence: i_s_ref ≈ I_fund × sin(ωt), so
     *   I_APF_ref ≈ I_fund × sin(ωt) − (I_fund × sin(ωt) + I_harmonics)
     *             = −I_harmonics(t)
     * APF_HystCtrl forces I_APF to track this, cancelling harmonics.
     * ------------------------------------------------------------------ */
    float I_tot     = I_active + delta_I;
    float i_s_ref   = I_tot * u_t;
    float I_APF_ref = i_s_ref - I_L1;

    return I_APF_ref;
}

/**
 * APF_RefGen_GetPavg
 * -------------------
 * Returns the current filtered average active power.
 *
 * P_avg is static and not visible outside this translation unit.
 * This getter exposes it for assignment to debug_P_avg in main.c,
 * which is then observable via STM32CubeMonitor during live operation.
 *
 * Inputs:  none
 * Returns: P_avg  [W]
 */
float APF_RefGen_GetPavg(void)
{
    return (float)P_avg;
}
