#ifndef FC_MISSION_MESSAGES_H
#define FC_MISSION_MESSAGES_H

#include <stdint.h>

static constexpr char MSGID_STATUS = 's';

struct StatusMsg {
    char id;
    int16_t roll; // in radians * 1e4
    int16_t pitch;
    int16_t yaw;

    int16_t roll_rate; // in radians/sec * 1e4
    int16_t pitch_rate;
    int16_t yaw_rate;

    int16_t roll_bias; // in radians/sec * 1e4
    int16_t pitch_bias;
    int16_t yaw_bias;

    int16_t accel_x_bias; // in m/s^2 * 1e3
    int16_t accel_y_bias; // in m/s^2 * 1e3
    int16_t accel_z_bias; // in m/s^2 * 1e3
} __attribute__((packed));

#endif
