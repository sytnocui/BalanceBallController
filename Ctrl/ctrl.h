//
// Created by 10798 on 2022/12/14.
//

#ifndef ATK_F405_FW_CTRL_H
#define ATK_F405_FW_CTRL_H


#ifdef __cplusplus
extern "C" {
#endif

#include <mpu6050.h>
#include <ctrl_types.h>
#include <senser_types.h>
#include "stdio.h"
#include "pid.h"


extern ctrl_rc_t ctrl_rc;
extern ctrl_state_t ctrl_state;
extern ctrl_setpoint_t ctrl_setpoint;
extern ctrl_setpoint_t ctrl_setpoint_offboard;
extern ctrl_out_t ctrl_out;

extern pid_calc_t roll_pid;
extern pid_calc_t pitch_pid;
extern pid_calc_t yaw_pid;
extern pid_calc_t rollRate_pid;
extern pid_calc_t pitchRate_pid;
extern pid_calc_t yawRate_pid;

void CtrlPIDInit(void);

void CtrlStateUpdate(const Axis3f* _gyro_f, const attitude_t* _attitude, ctrl_state_t* _state);
void CtrlSetpointUpdate(const ctrl_rc_t* _rc, ctrl_setpoint_t* _setpoint, ctrl_setpoint_t* _setpoint_offboard);
void CtrlUpdate(const ctrl_rc_t* _rc,const ctrl_state_t* _state, ctrl_setpoint_t* _setpoint, ctrl_out_t* _out);


#ifdef __cplusplus
}
#endif

#endif //ATK_F405_FW_CTRL_H
