//
// Created by 10798 on 2022/12/14.
//

#include "ctrl.h"
#include "pid.h"
#include "utils/ctrl_math.h"

#include <math.h>
#include <interface_can.h>
#include "tim.h"
#include "ctrl_types.h"
#include "senser_types.h"
#include "ctrl_sin.h"

//TODO:增加这几个东西的初始化函数
float ctrl_time = 0;
ctrl_rc_t ctrl_rc;
ctrl_state_t ctrl_state;
ctrl_setpoint_t ctrl_setpoint;
ctrl_setpoint_t ctrl_setpoint_offboard;
ctrl_out_t ctrl_out;
motorSpeed_t motorSpeed = {0.0f, 0.0f, 0.0f};

//PID
pid_calc_t roll_pid;
pid_calc_t pitch_pid;
pid_calc_t yaw_pid;
pid_calc_t rollRate_pid;
pid_calc_t pitchRate_pid;
pid_calc_t yawRate_pid;


void CtrlPIDInit(void){
    //初始化PID
    //外环
    PID_init(&roll_pid, PID_POSITION, 100.0f,0,0, 100,0);
    PID_init(&pitch_pid, PID_POSITION, 100.0f,0,0, 100,0);
    PID_init(&yaw_pid, PID_POSITION, 100.0f,0,0, 100,0);
    //内环 不要d，因为角速度本来就很斗
    PID_init(&rollRate_pid, PID_POSITION, 5.0f,0,0, 100,0);
    PID_init(&pitchRate_pid, PID_POSITION, 5.0f,0,0, 100,0);
    PID_init(&yawRate_pid, PID_POSITION, 5.0f,0,0, 100,0);


}

void CtrlStateUpdate(const Axis3f* _gyro_f, const attitude_t* _attitude, ctrl_state_t* _state){
    //通过姿态解算获得的实际值，存在state里，无需进行单位换算
    //TODO:到这里应该是正负都已经弄好的,如果不对记得看看
    _state->attitudeRate.roll = _gyro_f->y;
    _state->attitudeRate.pitch = _gyro_f->x;
    _state->attitudeRate.yaw = _gyro_f->z;

    _state->attitude.roll = _attitude->roll;
    _state->attitude.pitch = _attitude->pitch;
    _state->attitude.yaw = _attitude->yaw;
}


void CtrlSetpointUpdate(const ctrl_rc_t* _rc, ctrl_setpoint_t* _setpoint){
    //通过rc获得期望值，存在setpoint里
//    if(_rc->mode == RC_MODE_STABILIZED) //自稳模式
//    {
//        //期望角速度
        _setpoint->attitudeRate.roll = 0;
        _setpoint->attitudeRate.pitch = 0;
        _setpoint->attitudeRate.yaw = 0;
//    }
//    else if(_rc->mode == RC_MODE_POSITION) //定姿模式
//    {
        //期望角度
//        _setpoint->attitude.roll = 0;//_rc->roll;
//        _setpoint->attitude.pitch = 0;//_rc->pitch;
//        _setpoint->attitude.yaw = 0;//_rc->yaw;
//    }
//    else if(_rc->mode == RC_MODE_FORWARD) //开环前向模式
//    {
//        //pass，到最底下的时候直接让转速写成时间相关的函数
//    }
//    else
//    {
//        //什么mode都没识别到，按自稳的给，全给0
//        _setpoint->attitude.roll = 0;
//        _setpoint->attitude.pitch = 0;
//        _setpoint->attitude.yaw = 0;
//        _setpoint->attitudeRate.roll = 0;
//        _setpoint->attitudeRate.pitch = 0;
//        _setpoint->attitudeRate.yaw = 0;
//    }

}


void CtrlUpdate(const ctrl_rc_t* _rc, const ctrl_state_t* _state, ctrl_setpoint_t* _setpoint, ctrl_out_t* _out){
    //先判断是否解锁
//    if(_rc->armed == RC_ARMED_YES){
//
//        if(_rc->mode == RC_MODE_STABILIZED) //自稳模式
//        {
            //只算内环角速度环
            _out->roll =  PID_calc(&rollRate_pid, _state->attitudeRate.roll, _setpoint->attitudeRate.roll);
            _out->pitch = PID_calc(&pitchRate_pid, _state->attitudeRate.pitch, _setpoint->attitudeRate.pitch);
            _out->yaw = PID_calc(&yawRate_pid, _state->attitudeRate.yaw, _setpoint->attitudeRate.yaw);

//        }
//        else if(_rc->mode == RC_MODE_POSITION) //定姿模式
//        {
//            //先算外环pid
//            _setpoint->attitudeRate.roll = PID_calc(&roll_pid, _state->attitude.roll, _setpoint->attitude.roll);
//            _setpoint->attitudeRate.pitch = PID_calc(&pitch_pid, _state->attitude.pitch, _setpoint->attitude.pitch);
//            _setpoint->attitudeRate.yaw = PID_calc(&yaw_pid, _state->attitude.yaw, _setpoint->attitude.yaw);
//            //再算内环pid
//            _out->roll =  PID_calc(&rollRate_pid, _state->attitudeRate.roll, _setpoint->attitudeRate.roll);
//            _out->pitch = PID_calc(&pitchRate_pid, _state->attitudeRate.pitch, _setpoint->attitudeRate.pitch);
//            _out->yaw = PID_calc(&yawRate_pid, _state->attitudeRate.yaw, _setpoint->attitudeRate.yaw);
//        }
//        else if(_rc->mode == RC_MODE_FORWARD) //开环前向模式
//        {
//            //pass，到最底下的时候直接让转速写成时间相关的函数
//        }
//        else
//        {
//            //什么mode都没识别到，全给0
//            _out->roll = 0;
//            _out->pitch = 0;
//            _out->yaw = 0;
//        }
//
//    }
//    else {
//        //没解锁
//        _out->roll = 0;
//        _out->pitch = 0;
//        _out->yaw = 0;
//    }
}



////---------------------------------------------about-motor-speed-------------------------------------------
void DriverSpeedUpdate(const ctrl_rc_t* _rc, const ctrl_out_t* _out, motorSpeed_t* _motor) {
    //先判断是否解锁
//    if(_rc->armed == RC_ARMED_YES){
//
//        if(_rc->mode == RC_MODE_STABILIZED || _rc->mode == RC_MODE_POSITION) //自稳模式 定点模式
//        {
            //TODO:确认一下正负是不是跟想的一样，怎么感觉是反的
            //转速分配矩阵
            const static float speedMatrix[3][3]={ {0.8164966f, 0.0f, -0.5773503f},
                                                   {-0.4082483f, 0.7071068f, -0.5773503f},
                                                   {-0.4082483f, -0.7071068f, -0.5773503f}};

            _motor->m1 = speedMatrix[0][0] * _out->roll + speedMatrix[0][1] * _out->pitch + speedMatrix[0][2] * _out->yaw;
            _motor->m2 = speedMatrix[1][0] * _out->roll + speedMatrix[1][1] * _out->pitch + speedMatrix[1][2] * _out->yaw;
            _motor->m3 = speedMatrix[2][0] * _out->roll + speedMatrix[2][1] * _out->pitch + speedMatrix[2][2] * _out->yaw;

//        }
//        else if(_rc->mode == RC_MODE_FORWARD) //开环前向模式
//        {
//            //时间的正弦函数
//            _motor->m1 = gain * sin_approx(ctrl_time * M_PIf * 2 / period + theta1) + dcOffset;
//            _motor->m2 = gain * sin_approx(ctrl_time * M_PIf * 2 / period + theta2) + dcOffset;
//            _motor->m3 = gain * sin_approx(ctrl_time * M_PIf * 2 / period + theta3) + dcOffset;
//        }
//        else
//        {
//            _motor->m1 = 0.0f;
//            _motor->m2 = 0.0f;
//            _motor->m3 = 0.0f;
//        }
//
//    }
//    else {
//        //没解锁
//        _motor->m1 = 0.0f;
//        _motor->m2 = 0.0f;
//        _motor->m3 = 0.0f;
//    }

    //限幅
    _motor->m1 = constrainf(_motor->m1,-80.0f,80.0f);
    _motor->m2 = constrainf(_motor->m2,-80.0f,80.0f);
    _motor->m3 = constrainf(_motor->m3,-80.0f,80.0f);
}