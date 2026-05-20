/* USER CODE BEGIN Header */
/**
 * @file    apf_refgen.h
 * @brief   Reference current generator (outer control loop) for the
 *          Single-Phase Shunt Active Power Filter (SAPF) on STM32F407VGT6.
 *
 * APF_RefGen uses instantaneous power theory to compute the harmonic
 * compensation current reference at 20kHz.  Its output feeds APF_HystCtrl
 * (the inner loop), which forces the real APF inductor current to track it.
 *
 * Control hierarchy:
 *   APF_RefGen_Update  →  I_APF_ref  →  APF_HystCtrl_Update  →  TIM1 CCR1
 *
 * Call order (see CLAUDE.md Section 10):
 *   1. APF_RefGen_Init()          -- before DMA and timer start
 *   2. APF_RefGen_Update(...)     -- called from TIM1 update ISR at 20kHz
 *   3. APF_RefGen_GetPavg()       -- called any time for debug monitoring
 */
/* USER CODE END Header */

#ifndef APF_REFGEN_H
#define APF_REFGEN_H

#include "main.h"   /* pulls in stm32f4xx_hal.h and project-wide defines */

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * PID CONTROLLER STRUCTURE
 *
 * Stores both the tuning parameters and the persistent state that must
 * survive between ISR calls.  A single PID_t instance (vdc_pid, defined
 * in apf_refgen.c) regulates the DC link voltage.
 * ========================================================================= */

/**
 * PID_t — discrete PID controller state and configuration.
 */
typedef struct
{
    /**
     * Kp  —  proportional gain [dimensionless]
     * Set to 0.02 for the DC link loop.  Targets ~8Hz closed-loop bandwidth,
     * safely below the 50Hz grid frequency.
     */
    float Kp;

    /**
     * Ki  —  integral gain [1/s]
     * Combined with TS inside pid_update: integral += Ki × error × TS.
     * Set to 0.5/s.  Simulink used 50/s (73Hz bandwidth) — too fast for
     * real hardware: caused integral windup on every startup transient.
     */
    float Ki;

    /**
     * Kd  —  derivative gain [s]
     * Set to 0.0 (derivative disabled).  DC link voltage is a smooth
     * signal; enabling the derivative would amplify ACS712 and ADC
     * quantisation noise, destabilising the outer loop.
     */
    float Kd;

    /**
     * integral  —  accumulated integral state [A]
     * Persistent state between ISR calls.  Clamped to ±PID_INT_LIMIT
     * each call (anti-windup) before being added to the other terms.
     * Reset to 0 by APF_RefGen_Init().
     */
    float integral;

    /**
     * prev_error  —  previous error value [V]
     * Persistent state for the backward-difference derivative term:
     *   derivative = Kd × (e(k) − e(k-1)) / TS
     * Reset to 0 by APF_RefGen_Init().
     */
    float prev_error;

} PID_t;

/* =========================================================================
 * FUNCTION PROTOTYPES
 * ========================================================================= */

/**
 * APF_RefGen_Init
 * ---------------
 * Resets all persistent state to safe known initial values.
 *
 * V_S_sq_avg is pre-loaded to 2449.7 V² = (V_S_FULL_SCALE / √2)²
 * = (70.0 / 1.41421)².  Without this pre-seed, V_S_sq_avg starts at
 * zero, V_S_rms clamps to 1V, and u_t reaches ±35 for ~95ms, producing
 * dangerous I_APF_ref spikes on real hardware (Bug 5, CLAUDE.md §13).
 *
 * Must be called before HAL_ADC_Start_DMA() and before any TIM1 start
 * function, following the startup sequence in CLAUDE.md Section 10.
 * Safe to call on warm restart (watchdog reset or debugger restart).
 *
 * Inputs:  none
 * Outputs: P_avg = 0, V_S_sq_avg = 2449.7, vdc_pid fully initialised
 */
void APF_RefGen_Init(void);

/**
 * APF_RefGen_Update
 * ------------------
 * Runs the seven-step reference current generator algorithm.
 *
 * Called from the TIM1 update ISR at 20kHz (every 50µs).
 *
 * Algorithm overview (see CLAUDE.md Section 8 for full specification):
 *   1. p = V_S × I_L1                     instantaneous power
 *   2. P_avg  ← IIR(p,  5Hz)              average active power
 *   3. V_S_rms ← sqrt(IIR(V_S², 5Hz))    running RMS estimate
 *   4. I_active = P_avg × √2 / V_S_rms   active current peak
 *   5. delta_I = PID(90V − V_dc)          DC link correction
 *   6. u_t = V_S / (√2 × V_S_rms)        unit sine template
 *   7. I_APF_ref = (I_active+delta_I)×u_t − I_L1   compensation ref
 *
 * Parameters:
 *   V_S   [V]  Scaled grid voltage from ADC    (bipolar ±70V range)
 *   I_L1  [A]  Scaled load current from ADC   (bipolar ±5A range)
 *   V_dc  [V]  Scaled DC link voltage from ADC (unipolar 0–100V range)
 *
 * Returns:
 *   I_APF_ref  [A]  Reference compensation current for APF_HystCtrl
 */
float APF_RefGen_Update(float V_S, float I_L1, float V_dc);

/**
 * APF_RefGen_GetPavg
 * -------------------
 * Returns the current filtered average active power estimate.
 *
 * P_avg is a static variable inside apf_refgen.c and is not directly
 * accessible from main.c.  This getter exposes it safely for assignment
 * to debug_P_avg, which is monitored via STM32CubeMonitor.
 *
 * Inputs:  none
 * Returns: P_avg  [W]  current filtered average active power
 */
float APF_RefGen_GetPavg(void);

#ifdef __cplusplus
}
#endif

#endif /* APF_REFGEN_H */
