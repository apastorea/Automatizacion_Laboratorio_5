/*
 * Laboratorio 5 - Control digital por sintesis directa.
 *
 * Implementa la adquisicion, actuacion y control requeridos en el Lab5. Incluye
 * cuatro controladores seleccionables, una unica amplitud de referencia y
 * telemetria de magnitudes crudas y fisicas. El estado inicial es MONITOR:
 * compilar o programar en este estado mantiene la salida en cero.
 */
#include "project.h"
#include "lab5_config.h"
#include "lab5_control.h"
#include "lab5_hardware.h"
#include <float.h>
#include <stdio.h>

#if LAB5_MODE == LAB5_MODE_MONITOR
    #define LAB5_MODE_NAME "MONITOR"
#elif LAB5_MODE == LAB5_MODE_CONTROL
    #define LAB5_MODE_NAME "CONTROL"
#elif LAB5_MODE == LAB5_MODE_CALIBRATION
    #define LAB5_MODE_NAME "CALIBRATION"
#else
    #define LAB5_MODE_NAME "STATIC_DIAGNOSTIC"
#endif

#define LAB5_FLAG_ADC          (1u)
#define LAB5_FLAG_SATURATION   (2u)
#define LAB5_FLAG_NUMERIC      (4u)
#define LAB5_FLAG_CPU          (8u)
#define LAB5_FLAG_INTERVAL     (16u)

#if ADC_DEFAULT_RESOLUTION != 12u || \
    ADC_DEFAULT_CONV_MODE != ADC__HARDWARE_TRIGGER
    #error Lab5 conserva el ADC SAR de 12 bits con disparo por hardware.
#endif
#if ADC_DEFAULT_RANGE != ADC__VSSA_TO_VDDA
    #error El acondicionador requiere rango VSSA a VDDA.
#endif
#if Timer_TS_Resolution != 16u || Timer_Medicion_Resolution != 16u
    #error Los temporizadores deben ser de 16 bits.
#endif

typedef struct {
    float adc_v;
    float reference_v;
    float output_v;
    float error_v;
    float requested_v;
    float applied_v;
    int16 adc_counts;
    uint16 cpu_us;
    uint16 interval_us;
    uint8 dac_positive;
    uint8 dac_negative;
    uint8 flags;
} Lab5Sample;

static Lab5Sample samples[LAB5_SAMPLE_COUNT];
static volatile uint16 sample_count = 0u;
static volatile uint8 capture_complete = 0u;
static uint16 previous_tick = 0u;
static float runtime_zero_sum = 0.0f;
static float runtime_zero_offset = 0.0f;
static Lab5ControllerState controller_state;
static const Lab5ControllerParameters controller = {
    LAB5_CONTROLLER_B0, LAB5_CONTROLLER_B1, LAB5_CONTROLLER_B2,
    LAB5_CONTROLLER_A1, LAB5_CONTROLLER_A2
};

CY_ISR_PROTO(Lab5SampleInterrupt);

static uint8 Lab5IsFinite(float value)
{
    return (uint8)(value <= FLT_MAX && value >= -FLT_MAX);
}

static uint16 Lab5ReadMicrosecondTimer(void)
{
    Timer_Medicion_SoftwareCapture();
    return Timer_Medicion_ReadCapture();
}

static int32 Lab5ToMillivolts(float value)
{
    float limited;
    if (Lab5IsFinite(value) == 0u) return 0;
    limited = Lab5Clamp(value, -1000.0f, 1000.0f) * 1000.0f;
    return (int32)(limited + ((limited >= 0.0f) ? 0.5f : -0.5f));
}

static float Lab5Reference(uint16 k)
{
#if LAB5_MODE != LAB5_MODE_CONTROL
    (void)k;
    return 0.0f;
#else
    return (k < LAB5_PRETRIGGER_SAMPLES) ?
           0.0f : LAB5_REFERENCE_STEP_VOLTS;
#endif
}

#if LAB5_MODE == LAB5_MODE_CALIBRATION
static float Lab5CalibrationRequest(uint16 k)
{
    if (k < 32u) return 0.0f;
    if (k < 64u) return 0.050f;
    if (k < 96u) return 0.100f;
    if (k < 128u) return 0.0f;
    if (k < 160u) return -0.050f;
    if (k < 192u) return -0.100f;
    return 0.0f;
}
#endif

static void Lab5StopCapture(void)
{
    Timer_TS_Stop();
    Lab5HardwareZeroOutput();
    capture_complete = 1u;
}

CY_ISR(Lab5SampleInterrupt)
{
    const uint16 k = sample_count;
    uint16 start_tick;
    uint16 interval = 0u;
    uint16 cpu_time;
    uint8 flags = 0u;
    uint8 dac_p = 0u;
    uint8 dac_n = 0u;
    uint8 saturated = 0u;
    int16 raw;
    float adc_v;
    float y;
    float y_uncorrected;
    float reference;
    float error;
    float requested = 0.0f;
    float applied;
    Lab5Sample *sample;

    registro_Write(1u);
    if (capture_complete != 0u || k >= LAB5_SAMPLE_COUNT) {
        Lab5StopCapture();
        registro_Write(0u);
        return;
    }

    start_tick = Lab5ReadMicrosecondTimer();
    if (k != 0u) {
        interval = (uint16)(previous_tick - start_tick);
        if (interval < LAB5_TS_US - LAB5_TS_TOLERANCE_US ||
            interval > LAB5_TS_US + LAB5_TS_TOLERANCE_US)
            flags |= LAB5_FLAG_INTERVAL;
    }
    previous_tick = start_tick;

    y_uncorrected = Lab5HardwareReadPlant(&raw, &adc_v);
    if (k < LAB5_PRETRIGGER_SAMPLES) {
        runtime_zero_sum += y_uncorrected;
        if (k + 1u == LAB5_PRETRIGGER_SAMPLES)
            runtime_zero_offset = runtime_zero_sum /
                                  (float)LAB5_PRETRIGGER_SAMPLES;
        y = y_uncorrected;
    } else {
        y = y_uncorrected - runtime_zero_offset;
    }
    reference = Lab5Reference(k);
    error = reference - y;
    if (raw <= 2 || raw >= 4093) flags |= LAB5_FLAG_ADC;

#if LAB5_MODE == LAB5_MODE_CONTROL
    if (k >= LAB5_PRETRIGGER_SAMPLES)
        requested = Lab5ControllerStep(&controller_state, &controller,
                                       reference, y);
    else
        Lab5ControllerReset(&controller_state);
#elif LAB5_MODE == LAB5_MODE_CALIBRATION
    requested = Lab5CalibrationRequest(k);
    Lab5ControllerReset(&controller_state);
#else
    Lab5ControllerReset(&controller_state);
#endif

    if (Lab5IsFinite(requested) == 0u || Lab5IsFinite(y) == 0u) {
        flags |= LAB5_FLAG_NUMERIC;
        requested = 0.0f;
    }
    if ((flags & (LAB5_FLAG_ADC | LAB5_FLAG_NUMERIC |
                  LAB5_FLAG_INTERVAL)) != 0u)
        requested = 0.0f;

    applied = Lab5HardwareWriteOutput(requested, &dac_p, &dac_n, &saturated);
    if (saturated != 0u) flags |= LAB5_FLAG_SATURATION;

    sample = &samples[k];
    sample->adc_v = adc_v;
    sample->reference_v = reference;
    sample->output_v = y;
    sample->error_v = error;
    sample->requested_v = requested;
    sample->applied_v = applied;
    sample->adc_counts = raw;
    sample->dac_positive = dac_p;
    sample->dac_negative = dac_n;
    sample->interval_us = interval;

    cpu_time = (uint16)(start_tick - Lab5ReadMicrosecondTimer());
    sample->cpu_us = cpu_time;
    if (cpu_time >= LAB5_TS_US) flags |= LAB5_FLAG_CPU;
    sample->flags = flags;
    sample_count = k + 1u;

    if ((flags & (LAB5_FLAG_ADC | LAB5_FLAG_SATURATION | LAB5_FLAG_NUMERIC |
                  LAB5_FLAG_CPU | LAB5_FLAG_INTERVAL)) != 0u ||
        sample_count >= LAB5_SAMPLE_COUNT)
        Lab5StopCapture();
    registro_Write(0u);
}

static void Lab5DumpCapture(void)
{
    char line[300];
    uint16 k;
    uint32 elapsed_us = 0u;

    (void)snprintf(line, sizeof line,
        "# LAB5 mode=%s design=%s rev=%u Ts_us=%u samples=%u "
        "pretrigger=%u calibrated=%u calibration=%s\r\n",
        LAB5_MODE_NAME, LAB5_DESIGN_NAME, (unsigned)LAB5_DESIGN_REVISION,
        (unsigned)LAB5_TS_US, (unsigned)sample_count,
        (unsigned)LAB5_PRETRIGGER_SAMPLES, (unsigned)LAB5_CALIBRATED,
        LAB5_CALIBRATION_SOURCE);
    UART_Debug_PutString(line);
    (void)snprintf(line, sizeof line,
        "# reference_step_mV=%ld output_limit_mV=1000 adc=SAR_12bit\r\n",
        (long)(LAB5_REFERENCE_STEP_VOLTS * 1000.0f));
    UART_Debug_PutString(line);
    (void)snprintf(line, sizeof line,
        "# b0_x1e6=%ld b1_x1e6=%ld b2_x1e6=%ld "
        "a1_x1e6=%ld a2_x1e6=%ld\r\n",
        (long)(LAB5_CONTROLLER_B0 * 1.0e6f),
        (long)(LAB5_CONTROLLER_B1 * 1.0e6f),
        (long)(LAB5_CONTROLLER_B2 * 1.0e6f),
        (long)(LAB5_CONTROLLER_A1 * 1.0e6f),
        (long)(LAB5_CONTROLLER_A2 * 1.0e6f));
    UART_Debug_PutString(line);
    (void)snprintf(line, sizeof line,
        "# adc_offset_uV=%ld adc_gain_uV_per_V=%ld\r\n",
        (long)(LAB5_ADC_OFFSET_VOLTS * 1.0e6f),
        (long)(LAB5_ADC_VOLTS_PER_PLANT_VOLT * 1.0e6f));
    UART_Debug_PutString(line);
    (void)snprintf(line, sizeof line,
        "# runtime_zero_offset_uV=%ld pretrigger_samples=%u\r\n",
        (long)(runtime_zero_offset * 1.0e6f),
        (unsigned)LAB5_PRETRIGGER_SAMPLES);
    UART_Debug_PutString(line);
    (void)snprintf(line, sizeof line,
        "# actuator_offset_uV=%ld actuator_slope_x1e6=%ld\r\n",
        (long)(LAB5_ACTUATOR_OFFSET_VOLTS * 1.0e6f),
        (long)(LAB5_ACTUATOR_SLOPE * 1.0e6f));
    UART_Debug_PutString(line);

    UART_Debug_PutString(
        "k,t_us,adc_counts,adc_mV,r_mV,y_mV,e_mV,u_requested_mV,"
        "u_applied_mV,dac_p,dac_n,cpu_us,interval_us,flags\r\n");
    for (k = 0u; k < sample_count; ++k) {
        const Lab5Sample *sample = &samples[k];
        elapsed_us += sample->interval_us;
        (void)snprintf(line, sizeof line,
            "%u,%lu,%d,%ld,%ld,%ld,%ld,%ld,%ld,%u,%u,%u,%u,%u\r\n",
            (unsigned)k, (unsigned long)elapsed_us,
            (int)sample->adc_counts,
            (long)Lab5ToMillivolts(sample->adc_v),
            (long)Lab5ToMillivolts(sample->reference_v),
            (long)Lab5ToMillivolts(sample->output_v),
            (long)Lab5ToMillivolts(sample->error_v),
            (long)Lab5ToMillivolts(sample->requested_v),
            (long)Lab5ToMillivolts(sample->applied_v),
            (unsigned)sample->dac_positive, (unsigned)sample->dac_negative,
            (unsigned)sample->cpu_us, (unsigned)sample->interval_us,
            (unsigned)sample->flags);
        UART_Debug_PutString(line);
    }
    UART_Debug_PutString("# END; output=0; reset board to repeat\r\n");
}

int main(void)
{
#if LAB5_MODE == LAB5_MODE_STATIC_DIAGNOSTIC
    char diagnostic_message[180];
    uint8 dac_p = 0u;
    uint8 dac_n = 0u;
    uint8 saturated = 0u;
    float applied;
#else
    char startup_message[220];
    uint16 last_count = 0u;
    uint8 stalled_checks = 0u;
#endif

    CyGlobalIntDisable;
    Lab5ControllerReset(&controller_state);
    Lab5HardwareInit();
    UART_Debug_Start();
#if LAB5_MODE == LAB5_MODE_STATIC_DIAGNOSTIC
    applied = Lab5HardwareWriteOutput(LAB5_STATIC_OUTPUT_VOLTS,
                                      &dac_p, &dac_n, &saturated);
    (void)snprintf(diagnostic_message, sizeof diagnostic_message,
        "\r\n# LAB5 mode=%s requested_mV=%ld applied_mV=%ld "
        "dac_p=%u dac_n=%u saturated=%u; J8-J5_OPEN\r\n",
        LAB5_MODE_NAME, (long)(LAB5_STATIC_OUTPUT_VOLTS * 1000.0f),
        (long)(applied * 1000.0f), (unsigned)dac_p, (unsigned)dac_n,
        (unsigned)saturated);
    UART_Debug_PutString(diagnostic_message);
    for (;;) {
        CyDelay(1000u);
    }
#else

    (void)snprintf(startup_message, sizeof startup_message,
        "\r\n# LAB5 mode=%s design=%s Ts_us=%u reference_step_mV=%ld "
        "calibrated=%u capture_starts_ms=%u\r\n",
        LAB5_MODE_NAME, LAB5_DESIGN_NAME, (unsigned)LAB5_TS_US,
        (long)(LAB5_REFERENCE_STEP_VOLTS * 1000.0f),
        (unsigned)LAB5_CALIBRATED, (unsigned)LAB5_START_DELAY_MS);
    UART_Debug_PutString(startup_message);

    Clock_Medicion_Start();
    Timer_Medicion_Start();
    Timer_TS_Init();
    Timer_TS_WritePeriod((uint16)(LAB5_TS_COUNTS - 1u));
    isr_StartEx(Lab5SampleInterrupt);
    isr_ClearPending();
    CyDelay(LAB5_START_DELAY_MS);
    CyGlobalIntEnable;
    Clock_1_Start();
    Timer_TS_Enable();

    while (capture_complete == 0u) {
        CyDelay(10u);
        if (sample_count == last_count) ++stalled_checks;
        else stalled_checks = 0u;
        last_count = sample_count;
        if (stalled_checks >= 10u && capture_complete == 0u) {
            const uint8 interrupt_state = CyEnterCriticalSection();
            Lab5StopCapture();
            CyExitCriticalSection(interrupt_state);
            UART_Debug_PutString("# ERROR: no sampling progress for 100 ms\r\n");
        }
    }

    Lab5DumpCapture();
    for (;;) { }
#endif
}
