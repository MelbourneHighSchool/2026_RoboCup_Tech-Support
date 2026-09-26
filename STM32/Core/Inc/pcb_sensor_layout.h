#ifndef PCB_SENSOR_LAYOUT_H
#define PCB_SENSOR_LAYOUT_H

/* Shared by STM32 acquisition and Pi consumers. Multiplexers are numbered 1/2;
 * pins and transmitted indices are zero-based. See i2c_protocol.md. */
#define PCB_SENSOR_COUNT 30U
#define PCB_SENSOR_POSITION_COUNT 32U
#define PCB_SENSOR_CHANNELS_PER_MUX 15U
#define PCB_SENSOR_FORWARD_MUX 2U
#define PCB_SENSOR_FORWARD_PIN 9U
#define PCB_SENSOR_MUX1_FIRST_PIN 1U
#define PCB_SENSOR_MUX2_FIRST_PIN 0U
#define PCB_SENSOR_GAP_START 5U
#define PCB_SENSOR_GAP_SIZE 2U

/* Return -1 for disconnected/broken channels; these must never be sampled. */
static inline int pcb_sensor_index(unsigned mux, unsigned pin) {
    if (pin > 15U) return -1;
    if (mux == 1U) return pin == 0U ? -1 : (int)(pin + PCB_SENSOR_GAP_START);
    if (mux != PCB_SENSOR_FORWARD_MUX || pin == 14U) return -1;
    if (pin == 15U) return (int)PCB_SENSOR_GAP_START;
    return pin >= PCB_SENSOR_FORWARD_PIN
        ? (int)(pin - PCB_SENSOR_FORWARD_PIN)
        : (int)(pin + PCB_SENSOR_COUNT - PCB_SENSOR_FORWARD_PIN);
}

/* Call only with a working channel on mux 1 or 2. Each cycle has 15 samples. */
static inline unsigned pcb_sensor_next_pin(unsigned mux, unsigned pin) {
    if (pin == 15U)
        return mux == 1U ? PCB_SENSOR_MUX1_FIRST_PIN : PCB_SENSOR_MUX2_FIRST_PIN;
    return mux == 2U && pin == 13U ? 15U : pin + 1U;
}

/* Working indices are packed; physical positions 5 and 6 have no reading. */
static inline unsigned pcb_sensor_position(unsigned index) {
    return index < PCB_SENSOR_GAP_START ? index : index + PCB_SENSOR_GAP_SIZE;
}

static inline float pcb_sensor_bearing_deg(unsigned index) {
    return pcb_sensor_position(index) * (360.0f / PCB_SENSOR_POSITION_COUNT);
}

#endif
