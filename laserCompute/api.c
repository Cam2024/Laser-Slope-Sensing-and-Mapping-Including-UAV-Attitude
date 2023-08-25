/*! ----------------------------------------------------------------------------
 * @file    api.c
 * 
 * @version V1.00 
 *
 * @brief  在此文件中执行应用函数
 *        
 * @Modify
 *
 *      版本号     日期      作者      说明
 *      V1.00   2022/08/02   DSS       发布
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

/* 定义任务句柄 */
TaskHandle_t laserTaskHandle;       /* 激光数据读取任务句柄 */
TaskHandle_t imuTaskHandle;         /* IMU数据读取任务句柄 */
TaskHandle_t fusionTaskHandle;      /* 数据融合任务句柄 */
TaskHandle_t ledTaskHandle;         /* LED闪烁任务句柄 */ 



/* 定义任务 */
void laserTask(void);
void imuTask(void);
void ledTask(void);
void fusionTask(void);



/**
  * @brief  用户应用函数初始化
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
  * @brief  用于读取激光传感器数据，执行频率16.67HZ
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
    
    /* 循环读取激光传感器数据，间隔10ms */
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
  * @brief  用于读取IMU数据，执行频率200HZ
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

/* 定义错误距离标志位，根据标准差来判断 */
uint8_t ErrorDisFlag = 0;
uint8_t LaserSendBuffer[256];
uint16_t crc;
uint16_t count = 0;
/**
  * @brief  用于数据融合处理的任务函数，执行频率200HZ
  * @param  None
  * @retval None
  */
void fusionTask(void)
{
    /* 获取距离，从单位m到单位cm */
    inputData.dis[0] = (Sensor_GetLaser().dis1) * 100.0f;
    inputData.dis[1] = (Sensor_GetLaser().dis2) * 100.0f;
    inputData.dis[2] = (Sensor_GetLaser().dis3) * 100.0f;
    inputData.dis[3] = (Sensor_GetLaser().dis4) * 100.0f;
    
    inputFilter.yaw = Sensor_GetIMU().yaw;
    inputFilter.pitch = Sensor_GetIMU().roll;
    inputFilter.roll = Sensor_GetIMU().pitch;
    
    //printf("d0:%.2f d1:%.2f d2:%.2f d3:%.2f",inputData.dis[0],inputData.dis[1],inputData.dis[2],inputData.dis[3]);

//    printf(" pitch:%.2f roll:%.2f yaw:%.2f",inputFilter.pitch,inputFilter.roll,inputFilter.yaw);
    
    /* 激光传感器的平滑滤波 */
    for (int i = 0; i < sensors; i++)
    {
        inputFilter.dis[i] = filter(dis[i], inputData.dis[i]);
    }
    //printf(" fd0:%.2f fd1:%.2f fd2:%.2f fd3:%.2f\r\n",inputFilter.dis[0],inputFilter.dis[1],inputFilter.dis[2],inputFilter.dis[3]);
    
    /* 计算距离平均值 */
    avgDistance = (inputFilter.dis[0] + inputFilter.dis[1] + inputFilter.dis[2] + inputFilter.dis[3]) / 4.0f;
    
    /* 根据四个输入距离计算斜面角和转向角 */
    angle = LaserCalculation(inputFilter);
    //printf("\t%.2f\t",angle.directionAngle[1]);
    
    
    /* 根据角度计算 */
    ErrorDisFlag = avg(angle, &outputData);
    //printf("\t%.2f\t",outputData.directionAngle);
    
    /* 如果距离偏差过大 */
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
        /* 计算卡尔曼滤波后斜面角 */
        
        bevel_k.measurement = outputData.bevelAngle;
        bevel_k = kalman(bevel_k);
        
        /* 融合IMU的YAW角和转向角 */
        //fusion(yaw, dir, inputFilter.yaw, outputData.directionAngle, &dirAngle);
        
        /* 计算卡尔曼滤波后转向角 */
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
            memcpy(LaserSendBuffer+19,&bevel_k.X,4);              /* 平面夹角 */
            memcpy(LaserSendBuffer+23,&direct_k.X,4);             /* 转向夹角 */
            memcpy(LaserSendBuffer+27,&avgDistance,4);            /* 距离平均值 */
            memcpy(LaserSendBuffer+31,&inputFilter.pitch,4);      /* ROLL */
            memcpy(LaserSendBuffer+35,&inputFilter.roll,4);       /* PITCH */
            
            crc = RadioComputeCRC(LaserSendBuffer, 39, CRC_TYPE_IBM); 
            
            memcpy(LaserSendBuffer+39,&crc,2);      /* CRC校验 */
            
            HAL_UART_Transmit(&UART_DEBUG_Handle, (uint8_t *)&LaserSendBuffer, 41,HAL_MAX_DELAY);
            count = 0;
        }
        
        
        
//        printf(" avdis:%.2f  bevelAngle:%.2f  dirAngle:%.2f\r\n",avgDistance,bevel_k.X,direct_k.X);
    }
}



/**
  * @brief  LED指示灯闪烁任务，执行频率2HZ
  * @param  None
  * @retval None
  */
void ledTask(void)
{
	LED0_TOGGLE;
}

/**
  * @brief  HAL库定时器中断回调函数
  * @param  *htim 定时器句柄
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* 定时时间1ms */
    if(htim->Instance == TIM1) 
    {
        TaskScan_ms();
    } 
}




