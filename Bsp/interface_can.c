//
// Created by 10798 on 2023/7/23.
//

#include "interface_can.h"
#include "ctrl_types.h"


CAN_TxHeaderTypeDef txMessage;


//物理层代码

// 发送数据包
void can_send_message(uint8_t *buf,uint8_t len)
{
    uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;
    // 除去ID地址和功能码后的数据长度
    j = len - 2;

    // 发送数据
    while(i < j)
    {
        // 数据个数
        k = j - i;

        uint32_t tx_mail = CAN_TX_MAILBOX0;
        txMessage.ExtId = ((uint32_t)buf[0] << 8) | (uint32_t)packNum;
        txMessage.IDE = CAN_ID_EXT;
        txMessage.RTR = CAN_RTR_DATA;
        txMessage.TransmitGlobalTime = DISABLE;

        // 只能发送uint_8数组
        uint8_t cmd[8];
        cmd[0] = buf[1];
        // 小于8字节命令
        if(k < 8)
        {
            for(l=0; l < k; l++,i++) {
                cmd[l + 1] = buf[i + 2];
            }
            txMessage.DLC = k + 1;
        }
            // 大于8字节命令，分包发送，每包数据最多发送8个字节
        else
        {
            for(l=0; l < 7; l++,i++) {
                cmd[l + 1] = buf[i + 2];
            }
            txMessage.DLC = 8;
        }

        if (HAL_CAN_AddTxMessage(&hcan, &txMessage, cmd, &tx_mail) != HAL_OK)
        {
            /* Transmission request Error */
            Error_Handler();
        }
//        while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);

        // 记录发送的第几包的数据
        ++packNum;
    }
}

void can_send_cmd(uint16_t id)
{
    uint32_t tx_mail = CAN_TX_MAILBOX0;
//    uint32_t tx_mail;
    txMessage.StdId = id;
    txMessage.DLC = 4;
    txMessage.IDE = CAN_ID_STD;
    txMessage.RTR = CAN_RTR_REMOTE;
    txMessage.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &txMessage, NULL, &tx_mail) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
}

// 各种指令集
/**
  * @brief    将当前位置清零
  * @param    id  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Reset_CurPos_To_Zero(uint8_t id)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                         // 地址
    cmd[1] =  0x0A;                       // 功能码
    cmd[2] =  0x6D;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    id  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Reset_Clog_Pro(uint8_t id)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                         // 地址
    cmd[1] =  0x0E;                       // 功能码
    cmd[2] =  0x52;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    id  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Read_Sys_Params(uint8_t id, SysParams_t s)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                       // 地址

    switch(s)                             // 功能码
    {
        case S_VER   : cmd[1] = 0x1F; break;                  /* 读取固件版本和对应的硬件版本 */
        case S_RL    : cmd[1] = 0x20; break;                  /* 读取读取相电阻和相电感 */
        case S_PID   : cmd[1] = 0x21; break;                  /* 读取PID参数 */
        case S_ORG   : cmd[1] = 0x22; break;                  /* 读取回零参数 */
        case S_VBUS  : cmd[1] = 0x24; break;                  /* 读取总线电压 */
        case S_CBUS  : cmd[1] = 0x26; break;                  /* 读取总线电流 */
        case S_CPHA  : cmd[1] = 0x27; break;                  /* 读取相电流 */
        case S_ENC   : cmd[1] = 0x29; break;                  /* 读取编码器原始值 */
        case S_CPUL  : cmd[1] = 0x30; break;                  /* 读取实时脉冲数（根据实时位置计算得到的脉冲数） */
        case S_ENCL  : cmd[1] = 0x31; break;                  /* 读取经过线性化校准后的编码器值 */
        case S_TPUL  : cmd[1] = 0x32; break;                  /* 读取输入脉冲数 */
        case S_TPOS  : cmd[1] = 0x33; break;                  /* 读取电机目标位置 */
        case S_OPOS  : cmd[1] = 0x34; break;                  /* 读取电机实时设定的目标位置（开环模式的实时位置） */
        case S_VEL   : cmd[1] = 0x35; break;                  /* 读取电机实时转速 */
        case S_CPOS  : cmd[1] = 0x36; break;                  /* 读取电机实时位置（基于角度编码器累加的电机实时位置） */
        case S_PERR  : cmd[1] = 0x37; break;                  /* 读取电机位置误差 */
        case S_TEMP  : cmd[1] = 0x39; break;                  /* 读取电机实时温度 */
        case S_SFLAG : cmd[1] = 0x3A; break;                  /* 读取状态标志位 */
        case S_OFLAG : cmd[1] = 0x3B; break;                  /* 读取回零状态标志位 */
        case S_Conf  : cmd[1] = 0x42; cmd[2] = 0x6C; break;   /* 读取驱动参数 */
        case S_State : cmd[1] = 0x43; cmd[2] = 0x7A; break;   /* 读取系统状态参数 */
        default: break;
    }

    // 发送命令
    if(s >= S_Conf)
    {
        cmd[3] = 0x6B; can_send_message(cmd, 4);
    }
    else
    {
        cmd[2] = 0x6B; can_send_message(cmd, 3);
    }
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    id     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式（对应屏幕上的P_Pul菜单），0是关闭脉冲输入引脚，1是开环模式，2是闭环模式，3是让En端口复用为多圈限位开关输入引脚，Dir端口复用为到位输出高电平功能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Modify_Ctrl_Mode(uint8_t id, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                       // 地址
    cmd[1] =  0x46;                       // 功能码
    cmd[2] =  0x69;                       // 辅助码
    cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
    cmd[4] =  ctrl_mode;                  // 控制模式（对应屏幕上的Ctrl_Mode菜单），0是开环模式，1是FOC矢量闭环模式
    cmd[5] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    id  ：电机地址
  * @param    state ：使能状态     ，true为使能电机，false为关闭电机
  * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_En_Control(uint8_t id, bool state, uint8_t snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                       // 地址
    cmd[1] =  0xF3;                       // 功能码
    cmd[2] =  0xAB;                       // 辅助码
    cmd[3] =  (uint8_t)state;             // 使能状态
    cmd[4] =  snF;                        // 多机同步运动标志
    cmd[5] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 6);
}

/**
  * @brief    力矩模式
  * @param    id  ：电机地址
  * @param    sign  ：符号         ，0为正，其余值为负
  * @param    t_ramp：斜率(Ma/s)   ，范围0 - 65535Ma/s
  * @param    torque：力矩(Ma)     ，范围0 - 4000Ma
  * @param    snF   ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Torque_Control(uint8_t id, uint8_t sign, uint16_t t_ramp, uint16_t torque, uint8_t snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                       // 地址
    cmd[1] =  0xF5;                       // 功能码
    cmd[2] =  sign;                       // 符号（方向）
    cmd[3] =  (uint8_t)(t_ramp >> 8);     // 力矩斜率(Ma/s)高8位字节
    cmd[4] =  (uint8_t)(t_ramp >> 0);     // 力矩斜率(Ma/s)低8位字节
    cmd[5] =  (uint8_t)(torque >> 8);     // 力矩(Ma)高8位字节
    cmd[6] =  (uint8_t)(torque >> 0);     // 力矩(Ma)低8位字节
    cmd[7] =  snF;                        // 多机同步运动标志
    cmd[8] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 9);
}

/**
  * @brief    速度模式
  * @param    id  	：电机地址
  * @param    dir     ：方向         ，0为CW，其余值为CCW
  * @param    v_ramp  ：斜率(RPM/s)  ，范围0 - 65535RPM/s
  * @param    velocity：速度(RPM)    ，范围0.0 - 4000.0RPM
  * @param    snF     ：多机同步标志 ，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Velocity_Control(uint8_t id, uint8_t dir, uint16_t v_ramp, float velocity, uint8_t snF)
{
    uint8_t cmd[16] = {0}; uint16_t vel = 0;

    // 将速度放大10倍发送过去
    vel = (uint16_t)ABS(velocity * 10.0f);

    // 装载命令
    cmd[0] =  id;                       // 地址
    cmd[1] =  0xF6;                       // 功能码
    cmd[2] =  dir;                        // 符号（方向）
    cmd[3] =  (uint8_t)(v_ramp >> 8);     // 速度斜率(RPM/s)高8位字节
    cmd[4] =  (uint8_t)(v_ramp >> 0);     // 速度斜率(RPM/s)低8位字节
    cmd[5] =  (uint8_t)(vel >> 8);        // 速度(RPM)高8位字节
    cmd[6] =  (uint8_t)(vel >> 0);        // 速度(RPM)低8位字节
    cmd[7] =  snF;                        // 多机同步运动标志
    cmd[8] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 9);
}

/**
  * @brief    直通限速位置模式
  * @param    id   	：电机地址
  * @param    dir     ：方向										，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)					，范围0.0 - 4000.0RPM
  * @param    position：位置(°)								，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志	，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志						，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Bypass_Position_LV_Control(uint8_t id, uint8_t dir, float velocity, float position, uint8_t raf, uint8_t snF)
{
    uint8_t cmd[16] = {0}; uint16_t vel = 0; uint32_t pos = 0;

    // 将速度和位置放大10倍发送过去
    vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

    // 装载命令
    cmd[0]  =  id;                      // 地址
    cmd[1]  =  0xFB;                      // 功能码
    cmd[2]  =  dir;                       // 符号（方向）
    cmd[3]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
    cmd[4]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节
    cmd[5]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
    cmd[6]  =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
    cmd[7]  =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
    cmd[8]  =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
    cmd[9]  =  raf;                       // 相位位置/绝对位置标志
    cmd[10] =  snF;                       // 多机同步运动标志
    cmd[11] =  0x6B;                      // 校验字节

    // 发送命令
    can_send_message(cmd, 12);
}

/**
  * @brief    梯形曲线位置模式
  * @param    id  	：电机地址
  * @param    dir     ：方向										，0为CW，其余值为CCW
  * @param    acc     ：加速加速度(RPM/s)			，0为CW，其余值为CCW
  * @param    dec     ：减速加速度(RPM/s)			，0为CW，其余值为CCW
  * @param    velocity：最大速度(RPM)					，范围0.0 - 4000.0RPM
  * @param    position：位置(°)								，范围0.0°- (2^32 - 1)°
  * @param    raf     ：相位位置/绝对位置标志	，0为相对位置，其余值为绝对位置
  * @param    snF     ：多机同步标志						，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Traj_Position_Control(uint8_t id, uint8_t dir, uint16_t acc, uint16_t dec, float velocity, float position, uint8_t raf, uint8_t snF)
{
    uint8_t cmd[32] = {0}; uint16_t vel = 0; uint32_t pos = 0;

    // 将速度和位置放大10倍发送过去
    vel = (uint16_t)ABS(velocity * 10.0f); pos = (uint32_t)ABS(position * 10.0f);

    // 装载命令
    cmd[0]  =  id;                      // 地址
    cmd[1]  =  0xFD;                      // 功能码
    cmd[2]  =  dir;                       // 符号（方向）
    cmd[3]  =  (uint8_t)(acc >> 8);       // 加速加速度(RPM/s)高8位字节
    cmd[4]  =  (uint8_t)(acc >> 0);       // 加速加速度(RPM/s)低8位字节
    cmd[5]  =  (uint8_t)(dec >> 8);       // 减速加速度(RPM/s)高8位字节
    cmd[6]  =  (uint8_t)(dec >> 0);       // 减速加速度(RPM/s)低8位字节
    cmd[7]  =  (uint8_t)(vel >> 8);       // 最大速度(RPM)高8位字节
    cmd[8]  =  (uint8_t)(vel >> 0);       // 最大速度(RPM)低8位字节
    cmd[9]  =  (uint8_t)(pos >> 24);      // 位置(bit24 - bit31)
    cmd[10] =  (uint8_t)(pos >> 16);      // 位置(bit16 - bit23)
    cmd[11] =  (uint8_t)(pos >> 8);       // 位置(bit8  - bit15)
    cmd[12] =  (uint8_t)(pos >> 0);       // 位置(bit0  - bit7 )
    cmd[13] =  raf;                       // 相位位置/绝对位置标志
    cmd[14] =  snF;                       // 多机同步运动标志
    cmd[15] =  0x6B;                      // 校验字节

    // 发送命令
    can_send_message(cmd, 16);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    id  ：电机地址
  * @param    snF   ：多机同步标志，0为不启用，其余值启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Stop_Now(uint8_t id, uint8_t snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                       // 地址
    cmd[1] =  0xFE;                       // 功能码
    cmd[2] =  0x98;                       // 辅助码
    cmd[3] =  snF;                        // 多机同步运动标志
    cmd[4] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    id  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Synchronous_motion(uint8_t id)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                       // 地址
    cmd[1] =  0xFF;                       // 功能码
    cmd[2] =  0x66;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    id  ：电机地址
  * @param    svF   ：是否存储标志，false为不存储，true为存储
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Set_O(uint8_t id, bool svF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                       // 地址
    cmd[1] =  0x93;                       // 功能码
    cmd[2] =  0x88;                       // 辅助码
    cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
    cmd[4] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    id   ：电机地址
  * @param    svF    ：是否存储标志，false为不存储，true为存储
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    o_dir  ：回零方向，0为CW，其余值为CCW
  * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
  * @param    o_tm   ：回零超时时间，单位：毫秒
  * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
  * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
  * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
  * @param    potF   ：上电自动触发回零，false为不使能，true为使能
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Modify_Params(uint8_t id, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    uint8_t cmd[32] = {0};

    // 装载命令
    cmd[0] =  id;                       // 地址
    cmd[1] =  0x4C;                       // 功能码
    cmd[2] =  0xAE;                       // 辅助码
    cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
    cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[5] =  o_dir;                      // 回零方向
    cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
    cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节
    cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
    cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
    cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
    cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
    cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
    cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节
    cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
    cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节
    cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
    cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
    cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
    cmd[19] =  0x6B;                      // 校验字节

    // 发送命令
    can_send_message(cmd, 20);
}

/**
  * @brief    触发回零
  * @param    id   ：电机地址
  * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
  * @param    snF    ：多机同步标志，false为不启用，true为启用
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Trigger_Return(uint8_t id, uint8_t o_mode, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                         // 地址
    cmd[1] =  0x9A;                       // 功能码
    cmd[2] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[3] =  snF;                        // 多机同步运动标志，false为不启用，true为启用
    cmd[4] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    id  ：电机地址
  * @retval   地址 + 功能码 + 命令状态 + 校验字节
  */
void ZDT_X42_V2_Origin_Interrupt(uint8_t id)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] =  id;                         // 地址
    cmd[1] =  0x9C;                       // 功能码
    cmd[2] =  0x48;                       // 辅助码
    cmd[3] =  0x6B;                       // 校验字节

    // 发送命令
    can_send_message(cmd, 4);
}

//驱动层代码(发送指令和数据)

void Driver_Velocity_Control(uint32_t CAN_ID, float target)
{
    ZDT_X42_V2_Velocity_Control((uint8_t)CAN_ID, ((target) > 0 ? (0) : 1),
                                65535, ABS(target), 0);
}

void Driver_Torque_Control(uint32_t CAN_ID, float target)
{
    ZDT_X42_V2_Torque_Control((uint8_t)CAN_ID, ((target) > 0 ? (0) : 1),
                              65535, ABS(target), 0);
}

