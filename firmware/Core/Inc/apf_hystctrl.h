/* USER CODE BEGIN Header */
/**
 * @file    apf_hystctrl.h
 * @brief   Hysteresis current controller for the Single-Phase Shunt
 *          Active Power Filter (SAPF) on STM32F407VGT6 Discovery.
 *
 * This module implements the inner control loop of the APF.
 * It drives the H-bridge gate through TIM1 CH1 / CH1N (PA8 / PB13)
 * using a fixed-band hysteresis algorithm.
 *
 * Call order (see CLAUDE.md Section 10):
 *   1. APF_HystCtrl_Init()          -- before any timer or DMA start
 *   2. APF_HystCtrl_Update(...)     -- called from TIM1 update ISR at 20kHz
 */
/* USER CODE END Header */

#ifndef APF_HYSTCTRL_H
#define APF_HYSTCTRL_H

#include "main.h"   /* pulls in stm32f4xx_hal.h and project-wide defines */

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * ADC FULL-SCALE CONSTANTS
 *
 * These constants define the physical range that maps to the 12-bit ADC
 * span (0 to 4095 counts).  They are used in main.c to convert raw DMA
 * buffer values to volts or amps before passing them to the algorithms.
 *
 * Bipolar signals (current sensors, grid voltage):
 *   physical = ((float)raw - 2048.0f) / 2047.5f * FULL_SCALE
 *
 * Unipolar signals (DC link voltage):
 *   physical = ((float)raw / 4095.0f) * FULL_SCALE
 *
 * NOTE: These constants assume correctly designed sensor output dividers.
 * Verify against actual hardware measurements before finalising.
 * ========================================================================= */

/**
 * I_APF_FULL_SCALE  —  5.0 A
 *
 * The ACS712-05B current sensor has a measurement range of ±5A.
 * The APF inductor (4.21mH) and test load are sized so the APF current
 * stays within this range.  Bipolar: ADC mid-scale (2048) = 0A.
 */
#define I_APF_FULL_SCALE   5.0f

/**
 * V_DC_FULL_SCALE  —  100.0 V
 *
 * The DC link setpoint is 90V.  100V gives a 10V (11%) margin so the
 * DC link capacitor voltage is always representable at full ADC resolution.
 * Unipolar: ADC zero (0) = 0V, ADC full-scale (4095) = 100V.
 */
#define V_DC_FULL_SCALE    100.0f

/**
 * V_S_FULL_SCALE  —  70.0 V
 *
 * The grid is 48V RMS, giving a peak of 48 × √2 = 67.9V.  70V provides
 * approximately 3% headroom above the grid peak.
 * Bipolar: ADC mid-scale (2048) = 0V.
 */
#define V_S_FULL_SCALE     70.0f

/**
 * I_L1_FULL_SCALE  —  5.0 A
 *
 * The ACS712-05B current sensor has a measurement range of ±5A.
 * The test load (diode bridge + resistor) is sized to stay within this
 * range.  Bipolar: ADC mid-scale (2048) = 0A.
 */
#define I_L1_FULL_SCALE    5.0f

/* =========================================================================
 * HYSTERESIS BAND
 * ========================================================================= */

/**
 * HYST_BAND  —  0.5 A
 *
 * The half-width of the hysteresis band in amperes.  The H-bridge gate
 * state changes only when the tracking error leaves the range
 * [−HYST_BAND, +HYST_BAND].
 *
 * Why 0.5A and not 0.1A (the Simulink model value):
 *   Real hardware introduces two noise sources that are absent in Simulink:
 *   (a) ACS712-05B output noise (~10–20mV, equivalent to ~54–108mA)
 *   (b) ADC quantisation: 1 LSB = 5A / 2048 ≈ 2.4mA, but ACS712 output
 *       noise dominates, so the effective noise floor is well above 0.1A.
 *   With a 0.1A band the controller would chatter continuously at the
 *   band boundary without actually reducing THD.  0.5A keeps switching
 *   events meaningful while still allowing fast (20kHz) current tracking.
 */
#define HYST_BAND          0.5f

/* =========================================================================
 * FUNCTION PROTOTYPES
 * ========================================================================= */

/**
 * APF_HystCtrl_Init
 * -----------------
 * Initialises the hysteresis controller to the safe power-up state.
 *
 * Sets the internal gate variable to 0 (negative injection leg active)
 * and pre-loads TIM1->CCR1 to the gate-off value before the PWM timer
 * starts.  This prevents an uncontrolled injection transient at startup
 * and also provides a clean state for warm restarts (watchdog or debugger
 * reset without hardware power cycle).
 *
 * Must be called before HAL_TIM_PWM_Start() and before DMA is armed,
 * following the startup sequence defined in CLAUDE.md Section 10.
 *
 * Inputs:  none
 * Outputs: gate = 0,  TIM1->CCR1 = CCR1_GATE_OFF (20 counts = 119ns)
 */
void APF_HystCtrl_Init(void);

/**
 * APF_HystCtrl_Update
 * --------------------
 * Runs the hysteresis current controller.
 *
 * Called from the TIM1 update ISR at 20kHz (every 50µs).  Computes the
 * APF current tracking error and decides whether to change the H-bridge
 * gate state.  Inside the ±HYST_BAND region the gate state is frozen
 * (hysteresis memory) to prevent chattering.
 *
 * Gate state to H-bridge mapping:
 *   gate = 1 → Q1+Q4 conduct → positive current injected into the AC line
 *   gate = 0 → Q2+Q3 conduct → negative current injected into the AC line
 *
 * Parameters:
 *   I_APF_ref  [A]  Reference compensation current from APF_RefGen_Update
 *   I_APF      [A]  Actual APF inductor current, scaled to amps (from ACS712)
 *
 * Returns: nothing (result written directly to TIM1->CCR1)
 */
void APF_HystCtrl_Update(float I_APF_ref, float I_APF);

#ifdef __cplusplus
}
#endif

#endif /* APF_HYSTCTRL_H */
