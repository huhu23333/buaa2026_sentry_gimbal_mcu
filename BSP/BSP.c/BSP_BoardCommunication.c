/**
 * @file BSP_BoardCommunication.c
 * @author lxr(784457420@qq.com)，ZS
 * @brief
 * @version 2.5
 * @date 2025-1-15
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "BSP_BoardCommunication.h"

ControlMessge ControlMes;
ext_robot_keycommand_t ext_robot_keycommand;
uint8_t data[8] = {0};
uint8_t data2_Fun[8] = {0};

void Board1_To_2(void);
void Board1_getGimbalInfo(Can_Export_Data_t RxMessage);
void Board1_getKeycommandInfo(Can_Export_Data_t RxMessage);
void Board1_getBigYawInfo(Can_Export_Data_t RxMessage);

Board1_FUN_t Board1_FUN = Board1_FunGroundInit;

// 此函数用来按照报文规则生成数据并发送。
void Board1_To_2(void)
{

  // 打包数据(遥控模式下传输的底盘速度)   //仅适用于调试环节，正式代码需要去除这一部分
  data[0] = ControlMes.x_velocity >> 8;
  data[1] = ControlMes.x_velocity;
  data[2] = ControlMes.y_velocity >> 8;
  data[3] = ControlMes.y_velocity;
  data[4] = ControlMes.z_rotation_velocity >> 8;
  data[5] = ControlMes.z_rotation_velocity;
  if(ControlMes.yaw_choose == 1)//小YAW
  {
    data[6] = Cloud.Target_BigYaw >> 8;
    data[7] = Cloud.Target_BigYaw;
  }
  else if(ControlMes.yaw_choose == 2)//大YAW
  {
    int16_t temp_big_yaw_velocity = -ControlMes.yaw_velocity; 
    data[6] = temp_big_yaw_velocity >> 8;//小YAW模式下，底盘不采用数据。大YAW模式下，为大YAW的目标速度。
    data[7] = temp_big_yaw_velocity;
  }
  // 数据发送
  Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_CHASSIS, data);

  uint8_t temp;
  // 打包数据
  data2_Fun[0] = (uint8_t)(M6020s_Yaw.realAngle >> 8);             //小YAW当前角度,单位编码值
  data2_Fun[1] = (uint8_t)M6020s_Yaw.realAngle;
  data2_Fun[2] = 0;
  temp |= (uint8_t)(ControlMes.fric_Flag & 0x01) << 0;  //摩擦轮开关
  temp |= (uint8_t)(ControlMes.AutoAimFlag & 0x01) << 1;//自瞄开关
  temp |= (uint8_t)(ControlMes.change_Flag & 0x01) << 2;//变速小陀螺开关
  temp |= (uint8_t)(ControlMes.modelFlag & 0x01) << 3;  //比赛/检录模式开关
  temp |= (uint8_t)(ControlMes.yaw_choose & 0x03) << 5; //大小yaw选择，1是小yaw，2是大yaw
  temp |= (uint8_t)(cloud_mode & 0x01) << 6;            //云台模式
  data2_Fun[3] = temp;
  temp = 0;

  data2_Fun[4] = (uint8_t)global_enemy_position[0] >> 8 ;              //地面坐标系下敌方装甲板X坐标
  data2_Fun[5] = (uint8_t)global_enemy_position[0] ;                  //地面坐标系下敌方装甲板X坐标
  data2_Fun[6] = (uint8_t)global_enemy_position[1] >> 8;              //地面坐标系下敌方装甲板Y坐标
  data2_Fun[7] = (uint8_t)global_enemy_position[1];                   //地面坐标系下敌方装甲板Y坐标
  // 数据发送
  Can_Fun.CAN_SendData(CAN_SendHandle, &hcan2, CAN_ID_STD, CAN_ID_GIMBAL, data2_Fun);
}

void Board1_getGimbalInfo(Can_Export_Data_t RxMessage)
{
  ControlMes.yaw_realAngle = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);//改成大YAW
  ControlMes.Blood_Volume = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);//无用，改成chassis_mode
  ControlMes.Speed_Bullet = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
  ControlMes.Speed_Bullet /= 1000;
  ControlMes.tnndcolor = (uint8_t)(RxMessage.CANx_Export_RxMessage[6] >> 0) & 0x01;
  ControlMes.game_start = (uint8_t)(RxMessage.CANx_Export_RxMessage[6] >> 1) & 0x01;
  chassis_mode = (uint8_t)(RxMessage.CANx_Export_RxMessage[6] >> 4) & 0x03;
  lack_blood_son_mode = (uint8_t)(RxMessage.CANx_Export_RxMessage[6] >> 5) & 0x01;
  
}

void Board1_getBigYawInfo(Can_Export_Data_t RxMessage)//世界坐标系下的大YAW坐标值
{
  uint32_t negative_Big_Yaw_Angle;
  memcpy(&negative_Big_Yaw_Angle, RxMessage.CANx_Export_RxMessage, sizeof(uint32_t));
  Big_Yaw_Angle = -negative_Big_Yaw_Angle;
}
