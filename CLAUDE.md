# CLAUDE.md — Claude Code Context File
# SAPF STM32F407 — Single-Phase Active Power Filter
# Read this file completely at the start of every Claude Code session.
# This file is the ground truth for all firmware decisions.
# Never assume values for pending items — always ask the user first.

═══════════════════════════════════════════════════════
SECTION 1 — PROJECT IDENTITY
═══════════════════════════════════════════════════════

Project:    Single-Phase Shunt Active Power Filter (SAPF)
Authors:    Özcan YARDIMCI, Yusuf Cafer TOK
University: Sakarya University, Turkey
Purpose:    Undergraduate thesis
Board:      STM32F407VGT6 Discovery
IDE:        STM32CubeIDE

═══════════════════════════════════════════════════════
SECTION 2 — CONFIRMED HARDWARE
═══════════════════════════════════════════════════════

Grid voltage:         48V RMS (transformer secondary)
Grid peak:            48 × √2 = 67.9V
Grid frequency:       50Hz

DC link setpoint:     90V
APF inductor:         4.21mH air-core (measured with LCR meter)
                      Simulink model used 5mH — acceptable difference

Current sensors:      2× ACS712-05B
                      Measurement range: ±5A
                      Output: 0-5V centered at 2.5V (at VCC/2)
                      Sensitivity: 185mV/A
                      Supply: 5V from Discovery board USB pin
                      Load must stay within ±5A range

Gate drivers:         2× IR2103 half-bridge driver
MOSFETs:              4× IRF3710 N-channel
Snubber capacitor:    4.7µF 400V (placed across H-bridge VCC-GND)
Test load:            Diode bridge + resistor
                      System designed for any nonlinear load

═══════════════════════════════════════════════════════
SECTION 3 — PENDING HARDWARE
═══════════════════════════════════════════════════════

These items are not yet specified.
Never write code that assumes values for these.
Always ask the user before using any value here.

DC link capacitor:    value and rating TBD
V_S voltage divider:  resistor values TBD
V_dc voltage divider: resistor values TBD
ACS712 output divider:5V output → 3.3V ADC input, values TBD
Gate resistors:       TBD
Bootstrap capacitors: TBD (required for IR2103 high-side drive)
Diode bridge rating:  TBD

═══════════════════════════════════════════════════════
SECTION 4 — CLOCK CONFIGURATION
═══════════════════════════════════════════════════════

HSE crystal:      8MHz (onboard Discovery board)
PLL:              M=4, N=168, P=2
HCLK:             168MHz
APB1 (PCLK1):     42MHz
APB2 (PCLK2):     84MHz
TIM1 clock:       168MHz (APB2 × 2, advanced timer)
ADC clock:        21MHz  (PCLK2 / 4)

ADC clock maximum: 36MHz
21MHz is safely within specification.

═══════════════════════════════════════════════════════
SECTION 5 — TIM1 CONFIGURATION
═══════════════════════════════════════════════════════

Mode:             Up-counting PWM
Prescaler:        0
ARR:              8399
Frequency:        168MHz / (8399+1) = exactly 20000Hz
Period:           50 microseconds

CH1 (CCR1):       Gate control — variable
                  Written by APF_HystCtrl every ISR call
                  PA8  → TIM1_CH1  → positive leg (Q1+Q4)
                  PB13 → TIM1_CH1N → negative leg (Q2+Q3)
                  Deadtime: 84 counts = 500ns

CH2 (CCR2):       ADC trigger — FIXED at 4199
                  Never written by any algorithm
                  Fires at exactly mid-period (midpoint sampling)
                  Separates gate control from ADC trigger permanently

CRITICAL RULE:
  CH1 = gate control only
  CH2 = ADC trigger only
  These must never be swapped or combined.

Deadtime:         84 counts = 500ns
                  Hardware-generated, prevents shoot-through
                  Applied to CH1/CH1N outputs only

NVIC:             TIM1_UP_TIM10_IRQn, priority 0

═══════════════════════════════════════════════════════
SECTION 6 — ADC CONFIGURATION
═══════════════════════════════════════════════════════

Instance:         ADC1
Clock:            21MHz (PCLK2/4)
Resolution:       12-bit (0 to 4095)
Scan mode:        Enabled
Trigger source:   TIM1_CC2 rising edge (CH2, CCR2=4199)
DMA:              DMA2 Stream0, circular mode

Scan sequence:
  Rank 1: CH1  → PA1 → I_APF  (ACS712-05B, bipolar ±5A)
  Rank 2: CH2  → PA2 → V_dc   (voltage divider, unipolar 0-100V)
  Rank 3: CH3  → PA3 → V_S    (voltage divider, bipolar ±70V)
  Rank 4: CH11 → PC1 → I_L1   (ACS712-05B, bipolar ±5A)

DMA buffer:
  volatile uint16_t adc_buf[4]
  adc_buf[0] = I_APF raw ADC value
  adc_buf[1] = V_dc raw ADC value
  adc_buf[2] = V_S raw ADC value
  adc_buf[3] = I_L1 raw ADC value

Sampling time:    84 cycles per channel
Total scan time:  4 × (84+12) / 21MHz = 18.3µs
                  Completes well within 50µs ISR period

═══════════════════════════════════════════════════════
SECTION 7 — ADC SCALING
═══════════════════════════════════════════════════════

Bipolar signals (I_APF, V_S, I_L1):
  physical = ((float)raw - 2048.0f) / 2047.5f × FULL_SCALE

Unipolar signals (V_dc):
  physical = ((float)raw / 4095.0f) × FULL_SCALE

Full-scale constants (in apf_hystctrl.h):
  I_APF_FULL_SCALE = 5.0f    ±5A   ACS712-05B measurement range
  V_DC_FULL_SCALE  = 100.0f  0-100V covers 90V setpoint with margin
  V_S_FULL_SCALE   = 70.0f   ±70V  covers 67.9V grid peak
  I_L1_FULL_SCALE  = 5.0f    ±5A   ACS712-05B measurement range

Note: These assume correctly designed sensor output dividers.
Verify against actual hardware measurements before finalizing.

═══════════════════════════════════════════════════════
SECTION 8 — ALGORITHM: APF_RefGen
═══════════════════════════════════════════════════════

File:    Core/Src/apf_refgen.c
Header:  Core/Inc/apf_refgen.h
Runs:    Every ISR call at 20kHz (Ts = 50µs)

Constants:
  TS      = 0.00005f         50 microseconds
  TAU     = 0.0318f          LPF time constant (fc = 5Hz)
  ALPHA   = TS/(TAU+TS)      0.001570f (computed at compile time)
  VDC_REF = 90.0f            DC link voltage setpoint

Inputs:
  V_S       grid voltage in volts
  I_L1      load current in amps
  V_dc      DC link voltage in volts

Output:
  I_APF_ref reference compensation current in amps

Steps:
  1. p = V_S × I_L1
  2. P_avg = P_avg + ALPHA × (p - P_avg)
  3. V_S_sq_avg = V_S_sq_avg + ALPHA × (V_S² - V_S_sq_avg)
     V_S_rms = sqrt(V_S_sq_avg)
     guard:  if V_S_rms < 1.0f → V_S_rms = 1.0f
  4. I_active = P_avg × sqrt(2) / V_S_rms
  5. e_Vdc = 90.0f - V_dc
     delta_I = PID(e_Vdc)
  6. V_S_peak = sqrt(2) × V_S_rms
     guard: if V_S_peak < 1.0f → V_S_peak = 1.0f
     u_t = V_S / V_S_peak
  7. I_APF_ref = (I_active + delta_I) × u_t - I_L1

PID:
  Kp = 0.02f
  Ki = 0.5f
  Kd = 0.0f
  Integral clamp: ±0.5A
  Output clamp:   ±1.0A

Initialization:
  P_avg      = 0.0f
  V_S_sq_avg = 2449.7f   (70/√2)² = (70/1.41421)² = 49.497² = 2449.7
                         prevents startup transient
  PID state  = 0.0f

═══════════════════════════════════════════════════════
SECTION 9 — ALGORITHM: APF_HystCtrl
═══════════════════════════════════════════════════════

File:    Core/Src/apf_hystctrl.c
Header:  Core/Inc/apf_hystctrl.h
Runs:    Every ISR call at 20kHz

Constant:
  HYST_BAND = 0.5f amperes

Variables:
  gate — file-scope static uint8_t
         Must be file-scope so Init() can reset it
         Supports software restart without hardware reset

Inputs:
  I_APF_ref   reference current from APF_RefGen
  I_APF       actual inductor current from ACS712

Output:
  Written directly to TIM1->CCR1

Logic:
  error = I_APF_ref - I_APF

  if error > +HYST_BAND:
    gate = 1
    TIM1->CCR1 = TIM1->ARR     Q1+Q4 ON, positive injection

  else if error < -HYST_BAND:
    gate = 0
    TIM1->CCR1 = 20             Q2+Q3 ON, negative injection

  else:
    gate unchanged              hysteresis memory, no switching

CCR1 = 20 when gate = 0:
  Generates 119ns OC1REF pulse each cycle
  119ns < 500ns deadtime → H-bridge sees gate as fully OFF
  Kept as conservative safe default

Initialization:
  gate = 0
  TIM1->CCR1 = 20

═══════════════════════════════════════════════════════
SECTION 10 — STARTUP SEQUENCE
═══════════════════════════════════════════════════════

Mandatory order in main.c USER CODE BEGIN 2:

  APF_RefGen_Init()
  APF_HystCtrl_Init()
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 4)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2)
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE)

Why this order:
  Algorithms initialized before hardware starts
  DMA armed before timer fires first trigger
  PWM outputs active before ISR can run
  Interrupt enabled last

Why __HAL_TIM_ENABLE_IT not HAL_TIM_Base_Start_IT:
  HAL_TIM_PWM_Start sets htim->State = BUSY
  HAL_TIM_Base_Start_IT checks State == READY → fails silently
  __HAL_TIM_ENABLE_IT writes directly to DIER register
  Bypasses HAL state machine correctly

═══════════════════════════════════════════════════════
SECTION 11 — DEBUG VARIABLES (main.c)
═══════════════════════════════════════════════════════

volatile uint16_t adc_buf[4]       raw DMA buffer
volatile float debug_I_APF_ref     reference current
volatile float debug_V_S           scaled grid voltage
volatile float debug_I_L1          scaled load current
volatile float debug_V_dc          scaled DC link voltage
volatile float debug_P_avg         average active power

All monitored via STM32CubeMonitor while firmware runs.

═══════════════════════════════════════════════════════
SECTION 12 — WARNINGS
═══════════════════════════════════════════════════════

WARNING 1 — CubeMX regeneration:
  HAL_TIM_MspPostInit in tim.c reverts to PE9/PA7 pins.
  Manually correct to PA8 and PB13 after every regeneration.

WARNING 2 — ADC trigger architecture:
  TIM1_CH2 CCR2=4199 is the ADC trigger. Never modify CCR2.
  TIM1_CH1 CCR1 is gate control only. Never use for ADC.

WARNING 3 — ACS712 output voltage:
  ACS712-05B outputs 0-5V. STM32 ADC maximum input is 3.3V.
  Voltage divider required on each ACS712 output pin.
  Direct connection without divider will damage STM32.

WARNING 4 — DC link capacitor:
  Value and rating are TBD. Ask user before specifying.
  Must be rated above 90V setpoint with safety margin.

WARNING 5 — Star ground:
  Power ground and signal ground must meet at one point only.
  Never share ground traces between H-bridge and STM32.

WARNING 6 — Code clarity:
  All code must be written clearly so any engineer can read it.
  Every function, variable, and constant must have a comment.
  No magic numbers without explanation.

═══════════════════════════════════════════════════════
SECTION 13 — LESSONS FROM PREVIOUS ITERATION
═══════════════════════════════════════════════════════

Do not repeat these bugs.

Bug 1: CCR1=0 kills ADC trigger
  OC1REF permanently LOW, no rising edge, ADC never fires
  Fix: ADC trigger moved to TIM1_CH2 permanently

Bug 2: CCR1=1 pulse too narrow for ADC detection
  5.95ns < 95.2ns minimum, ADC misses trigger
  Fix: TIM1_CH2 eliminates this entirely

Bug 3: HAL_TIM_Base_Start_IT fails silently
  HAL state BUSY after PWM start, interrupt never enabled
  Fix: use __HAL_TIM_ENABLE_IT

Bug 4: Gate variable in function scope
  Init() cannot reset it for warm restart
  Fix: gate must be file-scope static

Bug 5: V_S_sq_avg initialized to zero
  V_S_rms clamps to 1.0V, u_t reaches ±35, dangerous spikes
  Fix: initialize to (V_S_FULL_SCALE/√2)²

Bug 6: Wrong TIM1 GPIO pins
  PE9/PA7 — PA7 conflicts with Discovery MEMS SPI
  Fix: PA8 and PB13

Bug 7: Wrong I_active formula
  P_avg / V_S_rms² is wrong
  P_avg × sqrt(2) / V_S_rms is correct

═══════════════════════════════════════════════════════
SECTION 14 — PROJECT STATUS
═══════════════════════════════════════════════════════

[ ] CubeMX configuration (TIM1 CH1+CH2, ADC, DMA, GPIO)
[x] apf_refgen.c and apf_refgen.h — written, verified
[x] apf_hystctrl.c and apf_hystctrl.h — written, verified, builds cleanly
[x] main.c ISR callback and startup sequence complete
[x] Multi-agent review passed (Codex 14/14,
         Gemini 7 PASS 3 WARN — all warnings addressed)
[ ] Software-in-loop test with ESP32
[ ] Hardware component calculations
[ ] Real hardware connection
[ ] Scaling constant calibration
[ ] THD measurement before APF
[ ] THD measurement after APF
[ ] GitHub publication complete