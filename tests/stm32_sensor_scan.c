/* Host HAL stubs exercise the actual STM32 ADC/I2C callbacks extracted by
 * test_pcb_mapping.py, including GPIO selection and double-buffer publication. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "pcb_sensor_layout.h"

#define ADC1 1
#define ADC2 2
#define I2C2 2
#define GPIOA 0
#define GPIOB 1
#define GPIO_PIN_0 (1U << 0)
#define GPIO_PIN_1 (1U << 1)
#define GPIO_PIN_2 (1U << 2)
#define GPIO_PIN_3 (1U << 3)
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
#define GPIO_PIN_6 (1U << 6)
#define GPIO_PIN_7 (1U << 7)
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define I2C_DIRECTION_TRANSMIT 0
#define I2C_FIRST_AND_LAST_FRAME 0
#define __DMB() ((void)0)
#define __HAL_TIM_GET_COUNTER(timer) 123U

typedef struct { int Instance; } ADC_HandleTypeDef;
typedef struct { int Instance; } I2C_HandleTypeDef;
static ADC_HandleTypeDef hadc1 = {ADC1}, hadc2 = {ADC2};
static I2C_HandleTypeDef hi2c2 = {I2C2};
static uint8_t RxData;
static unsigned gpio[2], generation, sampled[2][16];
static uint8_t transmitted[PCB_SENSOR_COUNT];

static void HAL_GPIO_WritePin(unsigned port, unsigned pin, unsigned state) {
    if (state) gpio[port] |= pin;
    else gpio[port] &= ~pin;
}

static unsigned selected(unsigned mux) {
    return mux == 1 ? (gpio[GPIOA] >> 2) & 15U
        : ((gpio[GPIOA] >> 6) & 3U) | ((gpio[GPIOB] & 3U) << 2);
}

static uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *adc) {
    const unsigned mux = (unsigned)adc->Instance, pin = selected(mux);
    assert(mux == 1 || mux == 2);
    assert(!(mux == 1 && pin == 0) && !(mux == 2 && pin == 14));
    ++sampled[mux - 1][pin];
    const unsigned byte = generation * 50 + mux * 16 + pin;
    return (byte * 4095U + 127U) / 255U;
}

static void HAL_I2C_Slave_Seq_Receive_IT(I2C_HandleTypeDef *i2c, uint8_t *data,
                                       unsigned size, unsigned frame) {
    (void)i2c; (void)data; (void)size; (void)frame;
    assert(false);  /* Sensor reads must not send a command/register byte. */
}

static void HAL_I2C_Slave_Seq_Transmit_IT(I2C_HandleTypeDef *i2c, uint8_t *data,
                                        unsigned size, unsigned frame) {
    (void)i2c; (void)frame;
    assert(size == 30);
    memcpy(transmitted, data, size);
}

#include "stm32_scan_under_test.h"

/* Independent wiring fixture: do not derive expectations from the mapping. */
static const unsigned muxes[30] = {
    2,2,2,2,2,2, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 2,2,2,2,2,2,2,2,2
};
static const unsigned pins[30] = {
    9,10,11,12,13,15, 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15, 0,1,2,3,4,5,6,7,8
};

static void read_scan(int expected_generation) {
    HAL_I2C_AddrCallback(&hi2c2, 1, 0);
    for (unsigned i = 0; i < 30; ++i) {
        const unsigned expected = expected_generation < 0 ? 0
            : (unsigned)expected_generation * 50 + muxes[i] * 16 + pins[i];
        assert(transmitted[i] == expected);
    }
}

int main(void) {
    assert(PCB_SENSOR_COUNT == 30 && PCB_SENSOR_POSITION_COUNT == 32);
    assert(pcb_sensor_index(1, 0) == -1 && pcb_sensor_index(2, 14) == -1);
    assert(pcb_sensor_index(0, 0) == -1 && pcb_sensor_index(2, 16) == -1);
    for (unsigned i = 0; i < 30; ++i) {
        assert(pcb_sensor_index(muxes[i], pins[i]) == (int)i);
        printf("%u %u %u %.2f\n", i, muxes[i], pins[i], pcb_sensor_bearing_deg(i));
    }
    assert(pcb_sensor_bearing_deg(0) == 0);
    assert(pcb_sensor_bearing_deg(4) == 45);
    assert(pcb_sensor_bearing_deg(5) == 78.75);
    assert(pcb_sensor_bearing_deg(6) == 90);
    assert(pcb_sensor_bearing_deg(29) == 348.75);

    select_multiplexer1();
    select_multiplexer2();
    assert(selected(1) == 1 && selected(2) == 0);
    read_scan(-1);
    for (generation = 0; generation < 4; ++generation) {
        memset(sampled, 0, sizeof(sampled));
        for (unsigned round = 0; round < 15; ++round) {
            uint8_t in_flight[30];
            memcpy(in_flight, TxData, sizeof(in_flight));
            multiplexer1_ready = multiplexer2_ready = false;
            const uint32_t published = published_scan_index;
            /* Exercise either ADC finishing first, including at the boundary. */
            HAL_ADC_ConvCpltCallback(generation % 2 ? &hadc2 : &hadc1);
            assert(published_scan_index == published);
            assert(memcmp(TxData, in_flight, sizeof(in_flight)) == 0);
            read_scan((int)generation - 1);
            memcpy(in_flight, TxData, sizeof(in_flight));
            HAL_ADC_ConvCpltCallback(generation % 2 ? &hadc1 : &hadc2);
            assert(memcmp(TxData, in_flight, sizeof(in_flight)) == 0);
            assert((published_scan_index != published) == (round == 14));
            read_scan(round == 14 ? (int)generation : (int)generation - 1);
            assert(multiplexer1_ready && multiplexer2_ready);
            assert(selected(1) == multiplexer1 && selected(2) == multiplexer2);
        }
        for (unsigned mux = 1; mux <= 2; ++mux)
            for (unsigned pin = 0; pin < 16; ++pin)
                assert(sampled[mux-1][pin] == (unsigned)(pcb_sensor_index(mux, pin) >= 0));
        assert(selected(1) == 1 && selected(2) == 0);
    }
}
