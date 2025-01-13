//
// Created by 10798 on 2022/10/5.
//
#include <stdbool.h>
#include <tim.h>
#include <interface_can.h>
#include <attitude.h>
#include <utils/ctrl_math.h>
#include <icm42688.h>
#include <interface_uart.h>
#include <retarget.h>
#include "common_inc.h"
#include "can.h"
#include "RoboRoly.h"
#include "utils/attitude_utils.h"
#include "lqr.h"
#include "nouse/ctrl.h"

// 定义用哪个陀螺仪
enum IMU_CONFIG{
    IMU_USE_NONE,
    IMU_USE_ICM20602,
    IMU_USE_ICM42688P,
} imu_config;

uint16_t safe_time = 0;
/* Thread Definitions -----------------------------------------------------*/

/* Timer Callbacks -------------------------------------------------------*/

float target_vel = 0.0f;
float target_torque = 0.0f;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    //tim3定时器中断
    if (htim->Instance == TIM3) {

        //更新时间
        robot_time += 0.01f; //单位s
        sin_time += 0.01f; //单位s
        if(sin_time > t_d){
            sin_time = 0;
        }

        //读取角速度和加速度
        if(imu_config == IMU_USE_ICM42688P){
            icm42688AccAndGyroRead(&acc_raw, &gyro_raw);
            ImuTransformer(&acc_raw, &acc, &gyro_raw, &gyro);

            CoordinateRotation(&acc, &gyro, &acc_body, &gyro_body);

            icm42688AccTransformUnit(&acc_body,&acc_f,&acc_drift);//转换单位
            icm42688GyroTransformUnit(&gyro_body,&gyro_f,&gyro_drift);//转换单位
        }

        //姿态解算
        //这里传入的参数坐标系有一个转换关系
        MahonyAHRSupdateIMU(q, gyro_f.x, gyro_f.y, gyro_f.z, acc_f.x, acc_f.y, acc_f.z);
        // 四元数反解欧拉角
        QuaternionToEuler_ZXY(q, &state_eul);

        //解算出欧拉角的变化率，供LQR使用
        w2deul_zxy(&state_eul, &gyro_f, &state_deul);
//        // 转换单位(可视化用的)
//        AttitudeRadianToAngle(&state_eul,&state_attitude_angle);

        //更新主控制器
        RoboRolyWalkUpdate();

////////////////////////////////////////////////////////////////////
//        //更新当前姿态和目标姿态
//        CtrlStateUpdate(&gyro_f,&state_eul,&ctrl_state); ////注意！！！在这里用的是弧度制
//        // 更新目标值
//        CtrlSetpointUpdate(&ctrl_rc, &ctrl_setpoint);
//        //更新姿态控制pid
//        CtrlUpdate(&ctrl_rc, &ctrl_state, &ctrl_setpoint, &ctrl_out);
//        //更新飞轮转速
//        DriverSpeedUpdate(&ctrl_rc, &ctrl_out,&ctrl_out_sum, &motorCmd);
//        //发送消息到驱动板
//        DriverCmdSend(&ctrl_rc, &motorCmd);
/////////////////////////////////////////////////////////////////////////////






    }
}

/* UART Callbacks -------------------------------------------------------*/
//空闲中断在这里进行处理
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance==USART2)
    {
        //串口dma+空闲中断，接受wifi的指令
//        WIFIRead(wifi_rx_buffer, &ctrl_rc);

        //如果正常收到了串口指令，则安全计时会一直清零
        safe_time = 0;


//        if (robot_time >= 0.5){
//            HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_RESET);
//        } else{
//            HAL_GPIO_WritePin(LED0_GPIO_Port,LED0_Pin,GPIO_PIN_SET);
//        }

        //重新打开DMA接收 idle中断
        HAL_UARTEx_ReceiveToIdle_DMA(&WIFI_UART, wifi_rx_buffer, sizeof(wifi_rx_buffer));
    }
}

// CAN接收中断回调函数
uint8_t rx_data[8];
uint8_t rxCount = 1;
CAN_RxHeaderTypeDef rxMessage;
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_StatusTypeDef status;

    //接收数据(只能获取8字节以内的单包数据)
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rxMessage, rx_data);

    // 获取返回数据长度
    rxCount = rxMessage.DLC;

    //回调函数中编写程序读取数据

    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/* Default Entry -------------------------------------------------------*/
void Main(void) {

    //！！！
    //NOTE:注意！！！printf已经重定向到WIFI，gh 3p 的那个串口目前没用
    //！！！
    RetargetInit(&WIFI_UART);

    //陀螺仪自检
    HAL_Delay(1000);
    if (!icm42688_self_check()){
        imu_config = IMU_USE_ICM42688P;
    }

    //陀螺仪初始化
    if(imu_config == IMU_USE_ICM42688P){
        icm42688Init();
        HAL_Delay(100);
        ICM42688P_Gyro_And_Acc_Calibrate(&gyro_drift, &acc_drift);
    }

    HAL_Delay(100);

    //改成步进电机后不需要初始化电机


    //LQR 获取期望状态的状态转移矩阵
    get_A_d(0, h, &xa_pitch);
    get_A_d(w_d, h, &xa_roll);
    get_A_d(w_d, h, &xa_yaw);

    //PID
//    CtrlPIDInit();

    //wifi 串口DMA空闲中断 enable
    HAL_UARTEx_ReceiveToIdle_DMA(&WIFI_UART, wifi_rx_buffer, sizeof(wifi_rx_buffer));

    // 启动CAN接收中断
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

    //启动定时器
    HAL_TIM_Base_Start_IT(&htim3);

    while (1){

        // Check for UART errors and restart recieve DMA transfer if required
        if (WIFI_UART.ErrorCode != HAL_UART_ERROR_NONE)
        {
            HAL_UART_AbortReceive(&WIFI_UART);
            HAL_UARTEx_ReceiveToIdle_DMA(&WIFI_UART, wifi_rx_buffer, sizeof(wifi_rx_buffer));
        }

        // can的接收故障重启
        if (hcan.ErrorCode != HAL_CAN_ERROR_NONE)
        {
            HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
        }

        //----------------------------------------wifi通信发送-----------------------------------
        printf("%.3f,%.3f,%.3f\r\n", xa_roll.x_d, xa_pitch.x_d, xa_yaw.x_d);

//        printf("Hello World!\r\n");

        //Blink
        HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
        HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
        //-----------------------------Delay
        HAL_Delay(100);
    }
}
