#ifndef TC_INTERFACE_H
#define TC_INTERFACE_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include "constants.h"

typedef union {
    struct
    {
        int16_t motor_1_vel;
        int16_t motor_2_vel;
        int16_t motor_3_vel;
        int16_t motor_4_vel;
    };

    int16_t value[MOTOR_COUNT];

} motor_velocities_t; // TODO: Refer to motors by their position, not numbers

bool setup_telecommand_interface(void);

void start_telecommand_interface(TaskHandle_t motor_controller_task);

#endif // TC_INTERFACE_H