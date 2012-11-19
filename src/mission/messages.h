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

    int16_t roll_p;
    int16_t pitch_p;
    int16_t yaw_p;

    int16_t roll_d;
    int16_t pitch_d;
    int16_t yaw_d;

    int16_t gain_roll_p;
    int16_t gain_pitch_p;
    int16_t gain_yaw_p;

    int16_t gain_roll_d;
    int16_t gain_pitch_d;
    int16_t gain_yaw_d;
} __attribute__((packed));

#endif
