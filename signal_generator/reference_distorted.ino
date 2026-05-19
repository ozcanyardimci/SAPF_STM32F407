/*
 * SAPF Signal Generator — ESP32
 * 
 * Generates two test signals for software-in-loop testing
 * of the Single-Phase Active Power Filter (SAPF).
 *
 * Pin 26 → V_S  : clean 50Hz sine wave (grid voltage simulation)
 * Pin 25 → I_L1 : distorted wave (nonlinear load current simulation)
 *                 fundamental + 0.3 × 3rd harmonic + 0.1 × 5th harmonic
 *
 * Sample rate:  100kHz (10µs per sample)
 * Frequency:    50Hz
 * DAC range:    0-255 (8-bit)
 * Offset:       127.5 (midpoint, represents zero current/voltage)
 * Scale:        98    (amplitude, keeps signal within DAC range)
 *
 * Authors: Özcan YARDIMCI, Yusuf Cafer TOK
 * University: Sakarya University
 * Purpose: SAPF undergraduate thesis
 */

/* ── Pin definitions ─────────────────────────────────────── */
#define PIN_VS    26    /* V_S  : clean sine output          */
#define PIN_IL1   25    /* I_L1 : distorted wave output      */

/* ── Signal parameters ───────────────────────────────────── */
#define SAMPLE_RATE_US  10      /* sample interval in microseconds  */
#define FREQUENCY_HZ    50      /* output frequency in Hz           */
#define SCALE           98      /* amplitude (keeps within 0-255)   */
#define OFFSET          127.5f  /* DAC midpoint (zero crossing)     */

/* ── Harmonic amplitudes ─────────────────────────────────── */
#define H1_AMPLITUDE    1.0f    /* fundamental amplitude            */
#define H3_AMPLITUDE    0.3f    /* 3rd harmonic amplitude           */
#define H5_AMPLITUDE    0.1f    /* 5th harmonic amplitude           */

/* ── Derived constants ───────────────────────────────────── */
#define SAMPLES_PER_CYCLE  (1000000 / SAMPLE_RATE_US / FREQUENCY_HZ)
/* 1000000µs / 10µs / 50Hz = 2000 samples per cycle          */

#define TWO_PI          6.28318530f

/* ── Precomputed lookup tables ───────────────────────────── */
/* Computed once at startup to avoid repeated trigonometry    */
/* in the time-critical ISR loop                              */
uint8_t lut_VS[SAMPLES_PER_CYCLE];     /* clean sine          */
uint8_t lut_IL1[SAMPLES_PER_CYCLE];    /* distorted wave      */

/* ── Sample index ────────────────────────────────────────── */
volatile uint32_t sample_index = 0;

/*
 * build_lookup_tables
 * Precomputes both waveforms and stores them in uint8_t arrays.
 * Called once in setup(). Avoids floating point math in loop().
 */
void build_lookup_tables(void)
{
    for (uint32_t i = 0; i < SAMPLES_PER_CYCLE; i++)
    {
        /* Phase angle for this sample */
        float phase = TWO_PI * (float)i / (float)SAMPLES_PER_CYCLE;

        /* V_S: pure fundamental sine */
        float vs_value = H1_AMPLITUDE * sinf(phase);
        lut_VS[i] = (uint8_t)(OFFSET + SCALE * vs_value);

        /* I_L1: fundamental + 3rd harmonic + 5th harmonic    */
        /* This represents a typical nonlinear load current    */
        /* Any nonlinear load generates odd-order harmonics    */
        float il1_value = H1_AMPLITUDE * sinf(phase)
                        + H3_AMPLITUDE * sinf(3.0f * phase)
                        + H5_AMPLITUDE * sinf(5.0f * phase);

        /* Normalize to keep within DAC range 0-255           */
        /* Maximum possible value: 1.0 + 0.3 + 0.1 = 1.4     */
        /* SCALE × 1.4 + OFFSET = 98×1.4 + 127.5 = 264.7     */
        /* Exceeds 255 — apply normalization factor            */
        float normalization = H1_AMPLITUDE + H3_AMPLITUDE + H5_AMPLITUDE;
        il1_value = il1_value / normalization;
        lut_IL1[i] = (uint8_t)(OFFSET + SCALE * il1_value);
    }
}

/*
 * setup
 * Runs once at power-on. Builds lookup tables and
 * confirms readiness via serial monitor.
 */
void setup(void)
{
    Serial.begin(115200);

    /* Set DAC pins as outputs */
    pinMode(PIN_VS,  OUTPUT);
    pinMode(PIN_IL1, OUTPUT);

    /* Initialize DAC outputs to midpoint (zero signal) */
    dacWrite(PIN_VS,  (uint8_t)OFFSET);
    dacWrite(PIN_IL1, (uint8_t)OFFSET);

    /* Build waveform lookup tables */
    build_lookup_tables();

    Serial.println("SAPF Signal Generator ready.");
    Serial.print("Samples per cycle: ");
    Serial.println(SAMPLES_PER_CYCLE);
    Serial.print("Frequency: ");
    Serial.print(FREQUENCY_HZ);
    Serial.println(" Hz");
    Serial.print("Sample interval: ");
    Serial.print(SAMPLE_RATE_US);
    Serial.println(" us");
}

/*
 * loop
 * Outputs one sample per SAMPLE_RATE_US microseconds.
 * Uses micros() for timing — no delay() which would drift.
 * Wraps sample_index at end of each cycle.
 */
void loop(void)
{
    static uint32_t last_sample_time = 0;

    uint32_t current_time = micros();

    /* Wait until next sample interval */
    if ((current_time - last_sample_time) >= SAMPLE_RATE_US)
    {
        last_sample_time = current_time;

        /* Output current sample from lookup tables */
        dacWrite(PIN_VS,  lut_VS[sample_index]);
        dacWrite(PIN_IL1, lut_IL1[sample_index]);

        /* Advance to next sample, wrap at end of cycle */
        sample_index++;
        if (sample_index >= SAMPLES_PER_CYCLE)
        {
            sample_index = 0;
        }
    }
}