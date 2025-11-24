#include "loadcell.h"

#define NUM_SAMPLES_DEFAULT 32

static void _micro_delay(LoadCell* lc, uint16_t delay);
static long _read_raw_blocking(LoadCell* lc);
static bool _is_ready(LoadCell* lc);

void LoadCell_Init(LoadCell* lc, GPIO_TypeDef* data_port, uint16_t data_pin, GPIO_TypeDef* sck_port, uint16_t sck_pin, TIM_HandleTypeDef* timer) {
    lc->data_port = data_port;
    lc->data_pin = data_pin;
    lc->sck_port = sck_port;
    lc->sck_pin = sck_pin;
    lc->timer = timer;
    lc->state = LC_IDLE;
    lc->read_buffer = 0;
    lc->bit_counter = 0;
    lc->tare_offset = 0;
    lc->calibration_factor = 1.0f;
    lc->sample_accumulator = 0;
    lc->sample_counter = 0;
    lc->current_weight_mg = 0;
    lc->new_weight_available = false;
    HAL_GPIO_WritePin(lc->sck_port, lc->sck_pin, GPIO_PIN_RESET);
}

void LoadCell_Tare(LoadCell* lc, uint8_t samples) {
    long total = 0;
    for (int i = 0; i < 5; i++) {
        _read_raw_blocking(lc);
        HAL_Delay(10);
    }
    for (int i = 0; i < samples; i++) {
        total += _read_raw_blocking(lc);
        HAL_Delay(10);
    }
    lc->tare_offset = total / samples;
}

void LoadCell_Calibrate(LoadCell* lc, long known_weight_mg, uint8_t samples) {
    long total = 0;
    for (int i = 0; i < samples; i++) {
        total += _read_raw_blocking(lc);
        HAL_Delay(10);
    }
    long average_raw = total / samples;
    if (known_weight_mg != 0) {
        lc->calibration_factor = (float)(average_raw - lc->tare_offset) / (float)known_weight_mg;
    }
}

void LoadCell_Manager(LoadCell* lc) {
    switch (lc->state) {
        case LC_IDLE:
            if (_is_ready(lc)) {
                lc->read_buffer = 0;
                lc->bit_counter = 0;
                lc->state = LC_READING_BITS;
            }
            break;
        case LC_READING_BITS:
            HAL_GPIO_WritePin(lc->sck_port, lc->sck_pin, GPIO_PIN_SET);
            _micro_delay(lc, 1);
            lc->read_buffer = lc->read_buffer << 1;
            if (HAL_GPIO_ReadPin(lc->data_port, lc->data_pin) == GPIO_PIN_SET) {
                lc->read_buffer++;
            }
            HAL_GPIO_WritePin(lc->sck_port, lc->sck_pin, GPIO_PIN_RESET);
            _micro_delay(lc, 1);
            lc->bit_counter++;
            if (lc->bit_counter >= 24) {
                HAL_GPIO_WritePin(lc->sck_port, lc->sck_pin, GPIO_PIN_SET);
                _micro_delay(lc, 1);
                HAL_GPIO_WritePin(lc->sck_port, lc->sck_pin, GPIO_PIN_RESET);
                _micro_delay(lc, 1);
                long final_data = (long)(lc->read_buffer ^ 0x800000);
                lc->sample_accumulator += final_data;
                lc->sample_counter++;
                lc->state = LC_IDLE;
            }
            break;
    }
    if (lc->sample_counter >= NUM_SAMPLES_DEFAULT) {
        long average_raw = lc->sample_accumulator / NUM_SAMPLES_DEFAULT;
        if (lc->calibration_factor != 0.0f) {
           lc->current_weight_mg = (long)(((float)average_raw - (float)lc->tare_offset) / lc->calibration_factor);
        }
        lc->new_weight_available = true;
        lc->sample_accumulator = 0;
        lc->sample_counter = 0;
    }
}

long LoadCell_GetWeight(LoadCell* lc) {
    if (lc->new_weight_available) {
        lc->new_weight_available = false;
    }
    return lc->current_weight_mg;
}

static void _micro_delay(LoadCell* lc, uint16_t delay) {
    __HAL_TIM_SET_COUNTER(lc->timer, 0);
    while (__HAL_TIM_GET_COUNTER(lc->timer) < delay);
}

static bool _is_ready(LoadCell* lc) {
    return HAL_GPIO_ReadPin(lc->data_port, lc->data_pin) == GPIO_PIN_RESET;
}

static long _read_raw_blocking(LoadCell* lc) {
    long data = 0;
    uint32_t start_time = HAL_GetTick();
    while (!_is_ready(lc)) {
        if (HAL_GetTick() - start_time > 200) {
            return 0;
        }
    }
    for (uint8_t i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(lc->sck_port, lc->sck_pin, GPIO_PIN_SET);
        _micro_delay(lc, 1);
        data = data << 1;
        HAL_GPIO_WritePin(lc->sck_port, lc->sck_pin, GPIO_PIN_RESET);
        _micro_delay(lc, 1);
        if (HAL_GPIO_ReadPin(lc->data_port, lc->data_pin) == GPIO_PIN_SET) {
            data++;
        }
    }
    HAL_GPIO_WritePin(lc->sck_port, lc->sck_pin, GPIO_PIN_SET);
    _micro_delay(lc, 1);
    HAL_GPIO_WritePin(lc->sck_port, lc->sck_pin, GPIO_PIN_RESET);
    _micro_delay(lc, 1);
    data = data ^ 0x800000;
    return data;
}
