#ifndef PH_H
#define PH_H

#define NUM_SAMPLES 10   // How many samples to take for an average reading
#define VREF 3.3f        // STM32's reference voltage
#define R1_OHMS 10000.0f // The value of first resistor (R1)
#define R2_OHMS 10000.0f // The value of second resistor (R2)
#define HAL_OK 1 


int compare_uint32(const void *a, const void *b);
void PH_GetReading(ADC_HandleTypeDef *hadc);


#endif /* PH_H */
