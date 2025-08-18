#include "main.h"
#include "pH.h"
#include <stdint.h>

uint32_t adc_val;
float calibration_offset = 21.84;
uint32_t adc_buffer[NUM_SAMPLES];
float sensor_voltage = 0.0f;
float ph_value = 0.0f;

int compare_uint32(const void *a, const void *b)
{
    uint32_t arg1 = *(const uint32_t *)a;
    uint32_t arg2 = *(const uint32_t *)b;
    if (arg1 < arg2)
        return -1;
    if (arg1 > arg2)
        return 1;
    return 0;
}

void PH_GetReading(ADC_HandleTypeDef *hadc)
{
    // 1. Take multiple samples to get a stable reading
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        HAL_ADC_Start(hadc);
        if (HAL_ADC_PollForConversion(hadc, 100) == HAL_OK)
        {
            adc_buffer[i] = HAL_ADC_GetValue(hadc);
        }
        HAL_ADC_Stop(hadc);
        HAL_Delay(30);
    }
    // 2. Sort the array to discard the highest and lowest (noisy) readings
    qsort(adc_buffer, NUM_SAMPLES, sizeof(uint32_t), compare_uint32);

    // 3. Average the middle 6 samples for a stable result
    unsigned long int avg_adc_val = 0;
    for (int i = 2; i < (NUM_SAMPLES - 2); i++)
    {
        avg_adc_val += adc_buffer[i];
    }
    float avg_adc_float = (float)avg_adc_val / (NUM_SAMPLES - 4);

    // 4. Convert the average ADC reading back to the ORIGINAL sensor voltage
    // First, calculate the voltage seen at the STM32 pin
    float voltage_at_pin = avg_adc_float * (VREF / 4095.0f);

    // Second, reverse the voltage divider calculation to find the sensor's true output voltage
    sensor_voltage = voltage_at_pin * (R1_OHMS + R2_OHMS) / R2_OHMS;

    // 5. Convert the sensor voltage to a pH value using the formula
    // The formula is: pH = (slope * voltage) + offset
    ph_value = -5.70 * sensor_voltage + calibration_offset;
}

