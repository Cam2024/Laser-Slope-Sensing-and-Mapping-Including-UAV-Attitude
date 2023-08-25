/*! ----------------------------------------------------------------------------
 * @file    api.c
 * 
 * @version V1.00 
 *
 * @brief  �ڴ��ļ���ִ��Ӧ�ú���
 *        
 * @Modify
 *
 *      �汾��     ����      ����      ˵��
 *      V1.00   2022/08/02   DSS       ����
 *
 * @author DSS
 */
#include "task.h"
#include "bsp_print.h"
#include "api.h"
#include "main.h"
#include "bsp_led.h"
#include "tim.h"
#include <stdint.h>
#include "bsp_can.h"
#include "commut.h"
#include "laserCompute.h"
#include "bsp_mpu_sensor.h"
#include "inv_mpu.h"
#include "sensor.h"
#include "header.h"
#include "string.h"
#include "crc.h"

/* ���������� */
TaskHandle_t laserTaskHandle;       /* �������ݶ�ȡ������ */
TaskHandle_t imuTaskHandle;         /* IMU���ݶ�ȡ������ */
TaskHandle_t fusionTaskHandle;      /* �����ں������� */
TaskHandle_t ledTaskHandle;         /* LED��˸������ */ 



/* �������� */
void laserTask(void);
void imuTask(void);
void ledTask(void);
void fusionTask(void);



/**
  * @brief  �û�Ӧ�ú�����ʼ��
  * @param  None
  * @retval None
  */
void API_Init(void)
{
	CreatTask(&laserTaskHandle, laserTask, "laserTask", 20);
    CreatTask(&imuTaskHandle, imuTask, "imuTask", 5);
    CreatTask(&fusionTaskHandle, fusionTask, "fusionTask", 5);
	CreatTask(&ledTaskHandle, ledTask, "ledTask", 500);
    
	ShowAllTaskMsg();
	HAL_TIM_Base_Start_IT(&htim1);
}

/**
  * @brief  ���ڶ�ȡ���⴫�������ݣ�ִ��Ƶ��16.67HZ
  * @param  None
  * @retval None
  */
void laserTask(void)
{
    static uint8_t laserCount = 0;
    if(laserCount++ == 4)
    {   
        laserCount = 0;
    }
    
    /* ѭ����ȡ���⴫�������ݣ����10ms */
    if(laserCount == 0)
    {
        SendLaserCMD(LASER1_CAN_ID,LASER_START_MEA);
    }
    else if(laserCount == 1)
    {
        SendLaserCMD(LASER2_CAN_ID,LASER_START_MEA);
    }
    else if(laserCount == 2)
    {
        SendLaserCMD(LASER3_CAN_ID,LASER_START_MEA);
    }
    else if(laserCount == 3)
    {
        SendLaserCMD(LASER4_CAN_ID,LASER_START_MEA);
    }
}

/**
  * @brief  ���ڶ�ȡIMU���ݣ�ִ��Ƶ��200HZ
  * @param  None
  * @retval None
  */
void imuTask(void)
{
    if(MPU_SensorCheckUpdata())
    {
        MPU_MPL_ReadData(0,&IMU_Sensor);
    }
}

input_t inputData = { 0 };
input_t inputFilter = { 0 };
float dis[sensors][filterNUM] = { 0 };
output_t outputData = { 0 };
angle_t angle = { 0 };
float avgDistance = 0;
kalman_t bevel_k = { 0 };
kalman_t direct_k = { 0 };
float yaw[2] = { 0 };
float dir[2] = { 0 };
float dirAngle = 0;
int flag = 0;

/* �����������־λ�����ݱ�׼�����ж� */
uint8_t ErrorDisFlag = 0;
uint8_t LaserSendBuffer[256];
uint16_t crc;
uint16_t count = 0;
/**
  * @brief  ���������ںϴ������������ִ��Ƶ��200HZ
  * @param  None
  * @retval None
  */
void fusionTask(void)
{
    /* ��ȡ���룬�ӵ�λm����λcm */
    inputData.dis[0] = (Sensor_GetLaser().dis1) * 100.0f;
    inputData.dis[1] = (Sensor_GetLaser().dis2) * 100.0f;
    inputData.dis[2] = (Sensor_GetLaser().dis3) * 100.0f;
    inputData.dis[3] = (Sensor_GetLaser().dis4) * 100.0f;
    
    inputFilter.yaw = Sensor_GetIMU().yaw;
    inputFilter.pitch = Sensor_GetIMU().roll;
    inputFilter.roll = Sensor_GetIMU().pitch;
    
    //printf("d0:%.2f d1:%.2f d2:%.2f d3:%.2f",inputData.dis[0],inputData.dis[1],inputData.dis[2],inputData.dis[3]);

//    printf(" pitch:%.2f roll:%.2f yaw:%.2f",inputFilter.pitch,inputFilter.roll,inputFilter.yaw);
    
    /* ���⴫������ƽ���˲� */
    for (int i = 0; i < sensors; i++)
    {
        inputFilter.dis[i] = filter(dis[i], inputData.dis[i]);
    }
    //printf(" fd0:%.2f fd1:%.2f fd2:%.2f fd3:%.2f\r\n",inputFilter.dis[0],inputFilter.dis[1],inputFilter.dis[2],inputFilter.dis[3]);
    
    /* �������ƽ��ֵ */
    avgDistance = (inputFilter.dis[0] + inputFilter.dis[1] + inputFilter.dis[2] + inputFilter.dis[3]) / 4.0f;
    
    /* �����ĸ�����������б��Ǻ�ת��� */
    angle = LaserCalculation(inputFilter);
    //printf("\t%.2f\t",angle.directionAngle[1]);
    
    
    /* ���ݽǶȼ��� */
    ErrorDisFlag = avg(angle, &outputData);
    //printf("\t%.2f\t",outputData.directionAngle);
    
    /* �������ƫ����� */
    if(ErrorDisFlag == 1)
    {
//        printf(" ===MAX_DIS===\r\n");
    }
    else
    {
        flag+=1;
        if(flag==100)
        {
            flag=0;
            bevel_k.X=0;
            direct_k.X=0;
        }
        /* ���㿨�����˲���б��� */
        
        bevel_k.measurement = outputData.bevelAngle;
        bevel_k = kalman(bevel_k);
        
        /* �ں�IMU��YAW�Ǻ�ת��� */
        //fusion(yaw, dir, inputFilter.yaw, outputData.directionAngle, &dirAngle);
        
        /* ���㿨�����˲���ת��� */
        //direct_k.measurement = dirAngle;
        direct_k.measurement = outputData.directionAngle;
        direct_k = kalman(direct_k);
        
        if(count++>10)
        {
            LaserSendBuffer[0] = 0x8a;                            /* HEADER */
            LaserSendBuffer[1] = 0x01;                            /* TAG */
            LaserSendBuffer[2] = 36;                              /* LENTH */
            memcpy(LaserSendBuffer+3,&inputFilter.dis[0],4);      /* dis1 */
            memcpy(LaserSendBuffer+7,&inputFilter.dis[1],4);      /* dis2 */
            memcpy(LaserSendBuffer+11,&inputFilter.dis[2],4);     /* dis3 */
            memcpy(LaserSendBuffer+15,&inputFilter.dis[3],4);     /* dis4 */
            memcpy(LaserSendBuffer+19,&bevel_k.X,4);              /* ƽ��н� */
            memcpy(LaserSendBuffer+23,&direct_k.X,4);             /* ת��н� */
            memcpy(LaserSendBuffer+27,&avgDistance,4);            /* ����ƽ��ֵ */
            memcpy(LaserSendBuffer+31,&inputFilter.pitch,4);      /* ROLL */
            memcpy(LaserSendBuffer+35,&inputFilter.roll,4);       /* PITCH */
            
            crc = RadioComputeCRC(LaserSendBuffer, 39, CRC_TYPE_IBM); 
            
            memcpy(LaserSendBuffer+39,&crc,2);      /* CRCУ�� */
            
            HAL_UART_Transmit(&UART_DEBUG_Handle, (uint8_t *)&LaserSendBuffer, 41,HAL_MAX_DELAY);
            count = 0;
        }
        
        
        
//        printf(" avdis:%.2f  bevelAngle:%.2f  dirAngle:%.2f\r\n",avgDistance,bevel_k.X,direct_k.X);
    }
}



/**
  * @brief  LEDָʾ����˸����ִ��Ƶ��2HZ
  * @param  None
  * @retval None
  */
void ledTask(void)
{
	LED0_TOGGLE;
}

/**
  * @brief  HAL�ⶨʱ���жϻص�����
  * @param  *htim ��ʱ�����
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* ��ʱʱ��1ms */
    if(htim->Instance == TIM1) 
    {
        TaskScan_ms();
    } 
}




