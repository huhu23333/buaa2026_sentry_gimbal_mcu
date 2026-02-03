/**
 * @file DT7.h
 * @author lxr(784457420@qq.com),cyx(1686143358@qq.com)
 * @brief
 * @version 2.0
 * @date 2024-3-9
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __DT7_H
#define __DT7_H

#include "main.h"
#include "usart.h"
// #include "Mecanum_Chassis.h"
#include "BSP_BoardCommunication.h"
#include "Cloud_Control.h"
#include "Protocol_UpperComputer.h"
#include "Dial.h"
#include "Shoot.h"
#include "Cloud_Control.h"
#include "SpeedRamp.h"

#define RemoteController "TDF"
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01 << 3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01 << 4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01 << 5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01 << 6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01 << 7)

#define RC_FRAME_LENGTH 18u

#define KEYMOUSE_AMOUNT 18    // ��������ܺͣ�18������
#define IT_KEYMOUSE_AMOUNT 18 // ͼ����������ܺͣ�18������

#define TIME_KeyMouse_Press 3 // ������ʱ����Ϊ ���¡�
// ������֮����Ϊ ����
#define TIME_KeyMouse_LongPress 60 // ������ʱ����Ϊ ����

#define DR16_ExportDataGroundInit \
    {                             \
        {0, 0},                   \
        {0, 0, 0, {0}},           \
        {0, 0, 0, 0, 0, 0},       \
        0,                        \
        0,                        \
    }

#define Image_Transmission_ExportDataGroundInit \
    {                                           \
        {0, 0},                                 \
        {0, 0, 0, {0}},                         \
        {0, 0, 0, 0, 0, 0},                     \
        0,                                      \
        0,                                      \
    }

#define ForwardBackGroundInit \
    {                         \
        0,                    \
        0,                    \
        -700,                 \
        700,                  \
    }

#define LeftRightGroundInit \
    {                       \
        0,                  \
        0,                  \
        -700,               \
        700,                \
    }

#define RotateGroundInit \
    {                    \
        0,               \
        0,               \
        -400,            \
        400,             \
    }

/* ----------------------- Data Struct ------------------------------------- */
/**
 * @brief  remote control information
 */
typedef struct
{
    struct
    {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        uint8_t s1;
        uint8_t s2;
    } rc;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    /* keyboard key information */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W : 1;
            uint16_t S : 1;
            uint16_t A : 1;
            uint16_t D : 1;
            uint16_t SHIFT : 1;
            uint16_t CTRL : 1;
            uint16_t Q : 1;
            uint16_t E : 1;
            uint16_t R : 1;
            uint16_t F : 1;
            uint16_t G : 1;
            uint16_t Z : 1;
            uint16_t X : 1;
            uint16_t C : 1;
            uint16_t V : 1;
            uint16_t B : 1;
        } bit;
    } key;

    int16_t wheel;

} RC_Ctl_t;

typedef enum
{
    // ��DR16_Export_data.KeyMouse ��flagλһһ��Ӧ
    KEY_W = 0, // ǰ
    KEY_S = 1, // ��
    KEY_A,     // ��
    KEY_D,     // ��
    KEY_SHIFT,
    KEY_CTRL, // ����С����
    KEY_Q,    // С������������
    KEY_E,    // С������������
    KEY_R,    // С��������
    KEY_F,
    KEY_G,
    KEY_Z,      // ����-2��/s
    KEY_X,      // ����+2��/s
    KEY_C,      // �˶�1
    KEY_V,      // �˶�2
    KEY_B,      // �˶�3
    MOUSE_Left, // ����
    MOUSE_Right // ����
} KeyList_e;

typedef enum
{
    KeyAction_CLICK,
    KeyAction_PRESS,
    KeyAction_LONG_PRESS
} KeyAction_e; // �����̣������¼�����

typedef struct
{
    struct
    {
        float x;
        float y;
    } mouse;

    struct
    {

        uint32_t Press_Flag;                // �����±�־
        uint32_t Click_Press_Flag;          // ���󵥻���־
        uint32_t Long_Press_Flag;           // ���󳤰���־
        uint8_t PressTime[KEYMOUSE_AMOUNT]; // �����³���ʱ��
    } KeyMouse;                             // ���Ķ��������

    struct
    {
        float Forward_Back_Value; // Vx
        float Omega_Value;        // ����ֵ��
        float Left_Right_Value;   // Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel;     // ����
    } Robot_TargetValue;      // ң�ؼ����������˶��ٶ�
    uint16_t infoUpdateFrame; // ֡��
    uint8_t OffLineFlag;      // �豸���߱�־
} DR16_Export_Data_t;         // �������ļ�ʹ�õ�������ݡ�

// ͼ��������·
typedef struct
{
    struct
    {
        float x;
        float y;
    } mouse;

    struct
    {

        uint32_t Press_Flag;                // �����±�־
        uint32_t Click_Press_Flag;          // ���󵥻���־
        uint32_t Long_Press_Flag;           // ���󳤰���־
        uint8_t PressTime[KEYMOUSE_AMOUNT]; // �����³���ʱ��
    } KeyMouse;                             // ���Ķ��������

    struct
    {
        float Forward_Back_Value; // Vx
        float Omega_Value;        // ����ֵ��
        float Left_Right_Value;   // Vy
        float Pitch_Value;
        float Yaw_Value;
        float Dial_Wheel;           // ����
    } Robot_TargetValue;            // ң�ؼ����������˶��ٶ�
    uint16_t infoUpdateFrame;       // ֡��
    uint8_t OffLineFlag;            // �豸���߱�־
} Image_Transmission_Export_Data_t; // �������ļ�ʹ�õ�������ݡ�

extern RC_Ctl_t RC_CtrlData;
extern uint8_t DT7_RX_Finish;
extern uint8_t DT7_Rx_Data[RC_FRAME_LENGTH];

void DT7_Init(void);
void DT7_Handle(void);
#endif
