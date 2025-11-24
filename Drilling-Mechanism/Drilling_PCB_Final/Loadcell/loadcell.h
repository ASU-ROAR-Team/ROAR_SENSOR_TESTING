#ifndef __LOADCELL_H
#define __LOADCELL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f0xx_hal.h"
#include <stdbool.h>

typedef enum {
    LC_IDLE,
    LC_READING_BITS
} LoadCell_State;

typedef struct {
    GPIO_TypeDef* data_port;
    uint16_t      data_pin;
    GPIO_TypeDef* sck_port;
    uint16_t      sck_pin;
    TIM_HandleTypeDef* timer;
    LoadCell_State state;
    uint32_t       read_buffer;
    uint8_t        bit_counter;
    long          tare_offset;
    float         calibration_factor;
    long          sample_accumulator;
    uint8_t       sample_counter;
    long          current_weight_mg;
    bool          new_weight_available;
} LoadCell;

void LoadCell_Init(LoadCell* lc, GPIO_TypeDef* data_port, uint16_t data_pin, GPIO_TypeDef* sck_port, uint16_t sck_pin, TIM_HandleTypeDef* timer);
void LoadCell_Tare(LoadCell* lc, uint8_t samples);
void LoadCell_Calibrate(LoadCell* lc, long known_weight_mg, uint8_t samples);
void LoadCell_Manager(LoadCell* lc);
long LoadCell_GetWeight(LoadCell* lc);

#ifdef __cplusplus
}
#endif

#endif // __LOADCELL_H
