/* USER CODE BEGIN Header */
/**
 * @file    apf_hystctrl.c
 * @brief   Hysteresis current controller for the Single-Phase Shunt
 *          Active Power Filter (SAPF) on STM32F407VGT6 Discovery.
 *
 * This is the inner control loop of the SAPF.  It receives a reference
 * current from APF_RefGen (the outer loop) and switches the full H-bridge
 * to force the actual APF inductor current to track the reference.
 *
 * Hardware driven by this module:
 *   TIM1 CH1  (PA8)  → IR2103 IN pin → Q1+Q4 (high-side / positive leg)
 *   TIM1 CH1N (PB13) → IR2103 SD pin → Q2+Q3 (low-side / negative leg)
 *   Deadtime: 84 TIM1 counts = 500ns (hardware, prevents shoot-through)
 *
 * Called from:  TIM1 update ISR (stm32f4xx_it.c) at 20kHz (Ts = 50µs)
 * Init called:  main.c USER CODE BEGIN 2, before any timer or DMA start
 */
/* USER CODE END Header */

#include "main.h"         /* stm32f4xx_hal.h, TIM1 register access         */
#include "tim.h"          /* htim1 handle, TIM1->ARR value                  */
#include "apf_hystctrl.h" /* HYST_BAND, function prototypes                 */

/* =========================================================================
 * FILE-SCOPE STATE VARIABLE
 * ========================================================================= */

/**
 * gate  —  current H-bridge switching state (0 or 1).
 *
 * Physical meaning:
 *   0 → Q2+Q3 conducting  → negative current injected into the AC line
 *   1 → Q1+Q4 conducting  → positive current injected into the AC line
 *
 * Why file scope (not local to APF_HystCtrl_Update):
 *   The hysteresis algorithm requires memory between ISR calls.  When the
 *   tracking error is inside the ±HYST_BAND region the controller must
 *   hold the previous gate state without switching.  A local variable
 *   would be zero-initialised on every call, making hysteresis impossible.
 *   File scope persists the value across calls.
 *
 * Why static (not extern / global):
 *   Only APF_HystCtrl_Init and APF_HystCtrl_Update need this variable.
 *   Declaring it static limits visibility to this translation unit and
 *   prevents accidental modification from other modules.
 *
 * Warm restart support:
 *   APF_HystCtrl_Init() explicitly resets gate to 0 before the PWM timer
 *   starts.  This ensures a known safe state after a watchdog reset or
 *   debugger restart without a hardware power cycle.
 */
static uint8_t gate = 0;

/* =========================================================================
 * PRIVATE CONSTANT
 * ========================================================================= */

/**
 * CCR1_GATE_OFF  —  20 timer counts
 *
 * The TIM1 CCR1 value that represents the H-bridge gate-off condition
 * for the negative-injection leg (Q2+Q3 conducting, Q1+Q4 blocked).
 *
 * Pulse width: 20 counts × (1 / 168MHz) = 119ns
 *
 * Why not CCR1 = 0?
 *   Setting CCR1 = 0 forces OC1REF permanently LOW regardless of the
 *   counter value.  This is a known STM32 TIM1 behaviour documented in
 *   the reference manual.  A permanently LOW OC1REF means:
 *     (a) CH1 output is always LOW  → Q1+Q4 always off (good)
 *     (b) CH1N output is always HIGH → Q2+Q3 always on  (intended)
 *   However, with OC1REF stuck LOW the deadtime generator no longer
 *   produces complementary transitions, which can cause unexpected
 *   behaviour in some IR2103 bootstrap charge scenarios.  Using
 *   CCR1 = 20 avoids this edge case entirely.
 *
 * Why 20 counts (119ns) specifically?
 *   The hardware deadtime is 84 TIM1 counts = 500ns.
 *   119ns < 500ns  → the deadtime generator blocks the 119ns OC1REF
 *   pulse from reaching the CH1 output, so Q1+Q4 never see a turn-on
 *   signal.  From the H-bridge perspective the high side is fully off.
 *
 *   119ns > 95ns  → satisfies the minimum OC1REF pulse width that the
 *   timer output stage requires to guarantee a valid transition.  This
 *   prevents the timer from entering an undefined state at the boundary.
 *
 *   Historical note: CCR1 = 20 was originally chosen because 119ns also
 *   exceeds the 95ns minimum ADC trigger detection threshold.  The ADC
 *   trigger has since been moved to TIM1 CH2 (CCR2 = 4199) permanently,
 *   so that constraint no longer applies here — but the value satisfies
 *   all remaining timing constraints and is kept as the safe default.
 */
#define CCR1_GATE_OFF  20u

/* =========================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================= */

/**
 * APF_HystCtrl_Init
 * -----------------
 * Initialises the hysteresis controller to the safe power-up state.
 *
 * Resets the gate variable to 0 (negative injection leg) and pre-loads
 * TIM1->CCR1 to CCR1_GATE_OFF so the H-bridge high-side (Q1+Q4) is
 * guaranteed off when the PWM timer starts.
 *
 * Must be called as the first step in the startup sequence (CLAUDE.md
 * Section 10), before HAL_ADC_Start_DMA() and before any TIM1 start
 * function.  Also safe to call on warm restart.
 *
 * Inputs:  none
 * Outputs: gate = 0,  TIM1->CCR1 = CCR1_GATE_OFF
 */
void APF_HystCtrl_Init(void)
{
    gate = 0;                    /* safe initial state: negative injection leg */
    TIM1->CCR1 = CCR1_GATE_OFF; /* pre-load CCR1 before the timer starts      */
}

/**
 * APF_HystCtrl_Update
 * --------------------
 * Runs the hysteresis current controller.  Called from the TIM1 update
 * ISR at 20kHz (every 50µs).
 *
 * Computes the tracking error (reference minus actual APF current) and
 * applies two-level hysteresis to decide the H-bridge gate state:
 *
 *   error > +HYST_BAND:  gate = 1, positive injection (Q1+Q4 ON)
 *   error < -HYST_BAND:  gate = 0, negative injection (Q2+Q3 ON)
 *   otherwise:           gate unchanged (hysteresis memory, no switching)
 *
 * Hysteresis memory prevents chattering: the gate state is frozen while
 * the error remains inside the band, so a single noise spike cannot cause
 * repeated toggling.
 *
 * Parameters:
 *   I_APF_ref  [A]  Reference compensation current from APF_RefGen_Update
 *   I_APF      [A]  Actual APF inductor current, scaled to amps
 *
 * Returns: nothing (result written directly to TIM1->CCR1)
 */
void APF_HystCtrl_Update(float I_APF_ref, float I_APF)
{
    /* Compute tracking error in amperes.
     * Positive error means the APF is injecting too little current.
     * Negative error means the APF is injecting too much current.    */
    float error = I_APF_ref - I_APF;

    if (error > +HYST_BAND)
    {
        /* APF current is below reference by more than the band.
         * Switch Q1+Q4 ON to drive current up (positive injection).
         * CCR1 = ARR (8399) makes OC1REF HIGH for the full PWM period:
         *   CH1  → HIGH  (after deadtime)  → Q1+Q4 gate driven HIGH
         *   CH1N → LOW   (after deadtime)  → Q2+Q3 gate driven LOW  */
        gate = 1;
        TIM1->CCR1 = TIM1->ARR;
    }
    else if (error < -HYST_BAND)
    {
        /* APF current is above reference by more than the band.
         * Switch Q2+Q3 ON to drive current down (negative injection).
         * CCR1 = 20 makes OC1REF HIGH for only 119ns:
         *   CH1  → blocked by 500ns deadtime → Q1+Q4 gate stays LOW
         *   CH1N → HIGH (after deadtime)     → Q2+Q3 gate driven HIGH */
        gate = 0;
        TIM1->CCR1 = CCR1_GATE_OFF;
    }
    else
    {
        /* Error is within ±HYST_BAND.
         * Do not change the gate state or CCR1.
         * The previous gate decision remains active.  This is the
         * hysteresis memory: the controller will not switch again until
         * the error grows large enough to cross a band boundary.       */
    }
}
