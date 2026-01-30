/**
 * @file Cloud_control.c
 * @author Cyx,SJW
 * @brief
 * @version 0.1
 * @date 2023-08-15
 *
 * @copyright
 *
 */
#ifndef __CLOUD_CONTROL_H
#define __CLOUD_CONTROL_H
#include "PID.h"
#include "kalman_filter.h"
#include "M6020_Motor.h"
#include "shoot.h"
#include "DT7.h"
#include "typedef.h"
#include "J4310_Motor.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "FuzzyPID.h"
#include "FeedForward.h"
#include "M3508_Motor.h"
#include "BSP_Can.h"
#include "Extern_Handles.h"
#include "Saber_C3.h"
#include "FuzzyPID.h"
#include "BSP_BoardCommunication.h"
#include "Auto_Aim.h"

#define Cloud_Pitch_level 1.046223f

#define Cloud_Yaw_Center 4060
#define Cloud_Yaw_ReverseCenter 7809
#define Electric_Limit_Midpoint 0



typedef struct
{
  float Yaw_Raw;       //
  float Pitch_Raw;     //
  float Target_Yaw;    //
  int16_t Target_BigYaw;
  float Target_Pitch;  //
  float AutoAim_Pitch; //
  float Little_Yaw_Target;
} Cloud_t;

typedef struct
{
  void (*Cloud_Init)(void);            
  void (*Cloud_Sport_Out)(void);       
  void (*Cloud_Sport_Out_Little_Yaw)(void);
  void (*Cloud_Pitch_Angle_Set)(void); 
  void (*Remote_Change)(void);
  void (*Cloud_Little_Yaw_Angle_Set)(void);
} Cloud_FUN_t;

void Cloud_Init(void);

extern Cloud_t Cloud;
extern Cloud_FUN_t Cloud_FUN;

#define Cloud_FUNGroundInit     \
{                               \
  &Cloud_Init,                  \
  &Cloud_Sport_Out,             \
  &Cloud_Sport_Out_Little_Yaw,  \
  &Cloud_Pitch_Angle_Set,       \
  &Remote_Change,               \
  &Cloud_Little_Yaw_Angle_Set,  \
}

#endif /* __CLOUD_CONTROL_H */
