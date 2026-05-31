# ANTIGRAVITY.md — Antigravity CLI Context File

# SAPF STM32F407 — Single-Phase Active Power Filter

# Read this completely at the start of every Antigravity CLI session.

═══════════════════════════════════════════════════════
SECTION 1 — PROJECT IDENTITY
═══════════════════════════════════════════════════════

Project:    Single-Phase Shunt Active Power Filter (SAPF)
Authors:    Özcan YARDIMCI, Yusuf Cafer TOK
University: Sakarya University, Turkey
Purpose:    Undergraduate thesis
Board:      STM32F407VGT6 Discovery, 168MHz HSE

═══════════════════════════════════════════════════════
SECTION 2 — WHAT THIS PROJECT DOES
═══════════════════════════════════════════════════════

A shunt APF connects in parallel with a nonlinear load on a
single-phase 48V AC grid. The nonlinear load (diode bridge +
resistor) draws distorted current containing harmonics.

The APF measures load current and grid voltage, computes the
harmonic content, and injects equal and opposite harmonic
current through an H-bridge inverter and series inductor.
The result is clean sinusoidal current drawn from the grid.

Two nested control loops run at 20kHz on STM32F407:
Outer: APF\_RefGen computes reference compensation current
Inner: APF\_HystCtrl forces actual current to track reference

═══════════════════════════════════════════════════════
SECTION 3 — CONFIRMED HARDWARE
═══════════════════════════════════════════════════════

Grid:             48V RMS (transformer secondary, 50Hz)
Grid peak:        67.9V (48 × √2)
DC link:          90V setpoint

APF inductor:     4.21mH air-core (measured with LCR meter)
Simulink used 5mH — acceptable difference

Current sensors:  2× ACS712-05B
Range: ±5A
Sensitivity: 185mV/A
Output: 0-5V centered at 2.5V
Supply: 5V from Discovery board
Load must stay within ±5A

Gate drivers:     2× IR2103 half-bridge
MOSFETs:          4× IRF3710 N-channel
Snubber cap:      4.7µF 400V (across H-bridge VCC-GND)
Test load:        Diode bridge + resistor
System works with any nonlinear load

═══════════════════════════════════════════════════════
SECTION 4 — PENDING HARDWARE
═══════════════════════════════════════════════════════

These are not yet confirmed. Never assume values.
Flag any code depending on these as needing confirmation.

DC link capacitor:    TBD (must be rated above 90V)
V\_S voltage divider:  TBD
V\_dc voltage divider: TBD
ACS712 output divider:TBD (5V → 3.3V scaling required)
Gate resistors:       TBD
Bootstrap capacitors: TBD
Diode bridge rating:  TBD

═══════════════════════════════════════════════════════
SECTION 5 — CRITICAL ARCHITECTURAL DECISION
═══════════════════════════════════════════════════════

TIM1\_CH1 (CCR1): gate control only — variable, written by
APF\_HystCtrl every ISR call

TIM1\_CH2 (CCR2): ADC trigger only — FIXED at 4199
Never written by any algorithm
Fires at counter midpoint (midpoint sampling)

This separation is permanent and must never change.
Previous iteration used CH1 for both gate and ADC trigger.
This caused ADC starvation bug when CCR1 was set to zero.

Midpoint sampling explanation:
CCR2 = 4199 = ARR/2 = midpoint of 20kHz period
ADC samples exactly between two switching edges
Switching noise is minimal at midpoint
This is standard professional practice for power electronics

STARTUP SEQUENCE (implemented in main.c USER CODE BEGIN 2):
Full sequence documented in CLAUDE.md Section 10.
Summary:

1. APF\_RefGen\_Init()                reset algorithm state
2. APF\_HystCtrl\_Init()              reset gate and CCR1
3. HAL\_ADC\_Start\_DMA()              arm DMA before timer
4. HAL\_TIM\_PWM\_Start() CH1          activate gate outputs
5. HAL\_TIMEx\_PWMN\_Start() CH1       activate complementary
6. HAL\_TIM\_PWM\_Start() CH2          activate ADC trigger
7. \_\_HAL\_TIM\_ENABLE\_IT()            enable ISR last

═══════════════════════════════════════════════════════
SECTION 6 — COMPLETE ALGORITHM
═══════════════════════════════════════════════════════

OUTER LOOP — APF\_RefGen:

Constants:
TS      = 0.00005f     (20kHz period)
ALPHA   = 0.001570f    (IIR coefficient, fc=5Hz)
VDC\_REF = 90.0f        (DC link setpoint)

Step 1: p = V\_S × I\_L1
Step 2: P\_avg = P\_avg + ALPHA × (p - P\_avg)
Step 3: V\_S\_sq\_avg = V\_S\_sq\_avg + ALPHA × (V\_S² - V\_S\_sq\_avg)
V\_S\_rms = sqrt(V\_S\_sq\_avg)
guard: V\_S\_rms minimum 1.0V
Step 4: I\_active = P\_avg × sqrt(2) / V\_S\_rms
Step 5: delta\_I = PID(90 - V\_dc)
Kp=0.02, Ki=0.5, Kd=0.0
Integral clamp ±0.5A, output clamp ±1.0A
Step 6: u\_t = V\_S / (sqrt(2) × V\_S\_rms)
guard: V\_S\_peak minimum 1.0V
Step 7: I\_APF\_ref = (I\_active + delta\_I) × u\_t - I\_L1

INNER LOOP — APF\_HystCtrl:

error = I\_APF\_ref - I\_APF

if error > +0.5A:
gate = 1
TIM1->CCR1 = TIM1->ARR

else if error < -0.5A:
gate = 0
TIM1->CCR1 = 20

else:
gate unchanged (hysteresis memory)

═══════════════════════════════════════════════════════
SECTION 7 — ADC CHANNEL MAP AND SCALING
═══════════════════════════════════════════════════════

adc\_buf\[0]: PA1, CH1  → I\_APF  bipolar ±5A
adc\_buf\[1]: PA2, CH2  → V\_dc   unipolar 0-100V
adc\_buf\[2]: PA3, CH3  → V\_S    bipolar ±70V
adc\_buf\[3]: PC1, CH11 → I\_L1   bipolar ±5A

Bipolar:  ((float)raw - 2048.0f) / 2047.5f × FULL\_SCALE
Unipolar: ((float)raw / 4095.0f) × FULL\_SCALE

═══════════════════════════════════════════════════════
SECTION 8 — YOUR ROLE AND RULES
═══════════════════════════════════════════════════════

YOU MAY:
Read all project files
Find bugs and inconsistencies
Make recommendations with clear reasoning
Verify algorithm against published APF literature
Analyze datasheets and extract parameters
Calculate hardware values when asked
Review code for clarity and correctness

YOU MAY NOT:
Modify any file under any circumstances
Assume values for pending hardware items
Make decisions without flagging to user

When you find a bug:
State file name and line number
Explain the problem clearly
Suggest the fix with reasoning
Wait for user confirmation

═══════════════════════════════════════════════════════
SECTION 9 — KNOWN FIXED BUGS
═══════════════════════════════════════════════════════

These are already fixed. Do not flag as new bugs.

1. CCR1=0 kills ADC trigger → fixed by TIM1\_CH2 separation
2. CCR1 pulse too narrow → fixed by TIM1\_CH2
3. HAL\_TIM\_Base\_Start\_IT fails → fixed by \_\_HAL\_TIM\_ENABLE\_IT
4. Gate variable in function scope → fixed to file scope
5. V\_S\_sq\_avg zero init → fixed to (V\_S\_FULL\_SCALE/√2)²
6. Wrong TIM1 pins PE9/PA7 → fixed to PA8/PB13
7. Wrong I\_active formula → fixed to P\_avg×sqrt(2)/V\_S\_rms

═══════════════════════════════════════════════════════
SECTION 10 — PROJECT STATUS
═══════════════════════════════════════════════════════

\[ ] CubeMX configuration
\[x] apf\_refgen.c and apf\_refgen.h — written, verified
\[x] apf\_hystctrl.c and apf\_hystctrl.h — written, verified, builds cleanly
\[x] main.c ISR and startup — complete
\[x] Multi-agent review complete — ready for
software-in-loop test
\[ ] Software-in-loop test
\[ ] Hardware calculations
\[ ] Real hardware integration
\[ ] THD measurement
\[ ] Publication

