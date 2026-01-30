
#ifndef AUTO_AIM_H
#define AUTO_AIM_H

typedef struct
{
  bool inited;
  bool reset;
  uint16_t pitch_coder_data; // 与电机编码器值对应(0~65535)
  uint16_t yaw_coder_data; // 与电机编码器值对应(0~8191)
  bool fire_flag;
} Auto_Aim_Control_t;

extern Auto_Aim_Control_t Auto_Aim_Control_Msg;

#endif
