#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "constants.h"
#include "telecommand_interface.h"
#include "sensor_manager.h"
#include "utils.h"

extern motor_velocities_t* motor_velocities_read_buf;

void start_motor_controller(void);

bool setup_motor_controller(void);

TaskHandle_t get_motor_controller_handle(void);

#endif // MOTOR_CONTROLLER_H