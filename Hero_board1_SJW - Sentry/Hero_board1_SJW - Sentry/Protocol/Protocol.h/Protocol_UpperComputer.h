/**
 * @file Protocol_UpperComputer.c
 * @author Why
 * @brief 跟上位机通信的协议
 * @version 0.1
 * @date 2023-10-02
 *
 */

#ifndef PROTOCOL_UPPERCOMPUTER_H
#define PROTOCOL_UPPERCOMPUTER_H

#include "CRC.h"
#include "usart.h"
#include "PID.h"
#include "BSP_BoardCommunication.h"
#include "FrictionWheel.h"
#include "Cloud_Control.h"
#include "usbd_cdc_if.h"
#include <math.h>  

void UpperCom_Receive_From_Up(uint8_t Rec[]);
void UpperCom_Send_To_Up(uint8_t COM);
// void coordinate_transform(float little_yaw_enemy_position[2] , float big_yaw_enemy_position[2] , uint16_t little_yaw_code_value);

#define UpperCom_MAX_BUF 30
#define Test_Pitch_SEN 22.75556
#define Test_Yaw_SEN 22.75556 // 8192/360 = 22.75556
//chassis_mode
#define CHASSIS_FOLLOW 1
#define SPINNING 2
#define LACK_BLOOD 3
//cloud_mode
#define ENEMY_LOCKED 0
#define ENEMY_SEARCH 1
//lack_blood_son_mode
#define WAY_OPEN 1
#define WAY_BLOCK 2

#define LITTLE_YAW_COORDINATE_OFFSET 0
#define LITTLE_YAW_MACHINE_ANGLE_OFFSET 0 
#define M_PI 3.14159265358979323846

extern uint16_t global_enemy_position[2];//敌方装甲板在小YAW坐标系下的X,Y坐标，单位mm
//extern uint16_t big_yaw_enemy_position[2];//敌方装甲板在大YAW坐标系下的X,Y坐标，单位mm
extern uint8_t chassis_mode;//底盘跟随/小陀螺/缺血回城模式
extern uint8_t cloud_mode;//自瞄锁敌/扫描锁敌模式
extern bool lack_blood_son_mode;//缺血回城模式下的子模式
extern float Big_Yaw_Angle;//大YAW当前角度

extern positionpid_t Auto_Aim_PID;
extern float Auto_Aim_Yaw;
extern float Auto_Aim_Pitch;
#endif
