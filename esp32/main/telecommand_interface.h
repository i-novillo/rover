#ifndef TC_INTERFACE_H
#define TC_INTERFACE_H

typedef struct {
    int16_t motor_1_vel;
    int16_t motor_2_vel;
    int16_t motor_3_vel;
    int16_t motor_4_vel;
} motor_velocities_t; // TODO: Refer to motors by their position, not numbers

motor_velocities_t* motor_velocities_read_buf;

bool setup_telecommand_interface(void);

void start_telecommand_interface(void);

#endif // TC_INTERFACE_H