#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <stdio.h>

typedef struct {
    int32_t motor_positions[4];
    int32_t motor_velocities[4];
    int64_t timestamp;
    bool valid_data;
} encoder_data_t;

bool setup_sensor_manager(void);

void start_sensor_manager(void);

bool get_encoder_data(encoder_data_t* out);

#endif // SENSOR_MANAGER_H