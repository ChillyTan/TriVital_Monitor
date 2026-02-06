/*********************************************************************************************************
* 模块名称：ADC.c
* 摘    要：ADC模块
* 当前版本：1.0.0
* 作    者：SZLY(COPYRIGHT 2018 - 2020 SZLY. All rights reserved.)
* 完成日期：2020年01月01日
* 内    容：
* 注    意：                                                                  
**********************************************************************************************************
* 取代版本：
* 作    者：
* 完成日期：
* 修改内容：
* 修改文件：
*********************************************************************************************************/
/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include "ADC.h"
#include "stm32f10x_conf.h"
#include "U16Queue.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
static unsigned short   s_arrADCData[3];          //DMA存储地址

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static void ConfigADC1(void);     //配置ADC1
static void ConfigDMA1Ch1(void);  //配置DMA通道1
static void ConfigTimer3(u16 arr, u16 psc); //配置TIM3

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigADC1
* 函数功能：配置ADC1
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：ADC123_IN1-PA1 PA2
**********************************************************************************************************/
static void ConfigADC1(void)
{                          
  GPIO_InitTypeDef  GPIO_InitStructure; //GPIO_InitStructure用于存放GPIO的参数
  ADC_InitTypeDef   ADC_InitStructure;  //ADC_InitStructure用于存放ADC的参数

  //使能RCC相关时钟
  RCC_ADCCLKConfig(RCC_PCLK2_Div6); //设置ADC时钟分频，ADCCLK=PCLK2/6=12MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1  , ENABLE);  //使能ADC1的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);  //使能GPIOA的时钟
 
  //配置ADC1的GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;    //设置引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN; //设置输入类型
  GPIO_Init(GPIOA, &GPIO_InitStructure);  //根据参数初始化GPIO

  //配置ADC1
  ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;  //设置为独立模式
  ADC_InitStructure.ADC_ScanConvMode       = ENABLE;                //使能扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;               //禁止连续转换模式
  ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO;  //使用TIM3触发
  ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;   //设置为右对齐
  ADC_InitStructure.ADC_NbrOfChannel       = 3; //设置ADC的通道数目
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5); //设置采样时间为239.5个周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_239Cycles5); //设置采样时间为239.5个周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_239Cycles5); //设置采样时间为239.5个周期

  ADC_DMACmd(ADC1, ENABLE);                   //使能ADC1的DMA
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);      //使用外部事件启动ADC转换
  ADC_Cmd(ADC1, ENABLE);                      //使能ADC1
  ADC_ResetCalibration(ADC1);                 //启动ADC复位校准，即将RSTCAL赋值为1
  while(ADC_GetResetCalibrationStatus(ADC1)); //读取并判断RSTCAL，RSTCAL为0跳出while语句
  ADC_StartCalibration(ADC1);                 //启动ADC校准，即将CAL赋值为1
  while(ADC_GetCalibrationStatus(ADC1));      //读取并判断CAL，CAL为0跳出while语句
}

/*********************************************************************************************************
* 函数名称：ConfigDMA1Ch1
* 函数功能：配置DMA通道1
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
static void ConfigDMA1Ch1(void)
{
  DMA_InitTypeDef DMA_InitStructure;  //DMA_InitStructure用于存放DMA的参数
  
  //使能RCC相关时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //使能DMA1的时钟
  
  //配置DMA1_Channel1
  DMA_DeInit(DMA1_Channel1);  //将DMA1_CH1寄存器设置为默认值
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);           //设置外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)s_arrADCData;        //设置存储器地址
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;           //设置为外设到存储器模式
  DMA_InitStructure.DMA_BufferSize         = 3;                               //设置要传输的数据项数目
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //设置外设为非递增模式
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //设置存储器为递增模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //设置外设数据长度为半字
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;     //设置存储器数据长度为半字
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;               //设置为循环模式
  DMA_InitStructure. DMA_Priority           = DMA_Priority_Medium;            //设置为中等优先级
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;                 //禁止存储器到存储器访问
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //根据参数初始化DMA1_Channel1
  
  DMA_Cmd(DMA1_Channel1, ENABLE); //使能DMA1_Channel1
}

/*********************************************************************************************************
* 函数名称：ConfigTimer3
* 函数功能：配置定时器TIM3的参数
* 输入参数：arr-自动重装值，psc-时钟预分频数
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意： 
**********************************************************************************************************/
static void ConfigTimer3(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //TIM_TimeBaseStructure用于存放TIM3的参数
    
  //使能RCC相关时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //使能TIM3的时钟

  //配置TIM3
  TIM_TimeBaseStructure.TIM_Period        = arr;  //设置自动重装载值
  TIM_TimeBaseStructure.TIM_Prescaler     = psc;  //设置预分频器值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;       //设置时钟分割：tDTS = tCK_INT
  TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up; //设置向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);               //根据参数初始化定时器  
  
  TIM_SelectOutputTrigger(TIM3,TIM_TRGOSource_Update);          //选择更新事件为触发输入
  
  TIM_Cmd(TIM3, ENABLE);  //使能定时器
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitADC
* 函数功能：初始化ADC模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
void InitADC(void)
{
  ConfigTimer3(999, 71);   //72MHz/(71+1)=1MHz,0-1999,1ms
  ConfigADC1();             //配置ADC1
  ConfigDMA1Ch1();          //配置DMA1的通道1
}

/*********************************************************************************************************
* 函数名称：ReadCUFFADC
* 函数功能：获取ADC转换值
* 输入参数：void
* 输出参数：void
* 返 回 值：ADC转换值
* 创建日期：2023/05/25
* 注  意：
**********************************************************************************************************/
u16 ReadECGADC(void)
{
	return s_arrADCData[0];
}

/*********************************************************************************************************
* 函数名称：ReadPULSEADC
* 函数功能：获取ADC转换值
* 输入参数：void
* 输出参数：void
* 返 回 值：ADC转换值
* 创建日期：2023/05/25
* 注  意：
**********************************************************************************************************/
u16 ReadRESPADC(void)
{
	return s_arrADCData[1];
}

/*********************************************************************************************************
* 函数名称：ReadSPO2ADC
* 函数功能：获取ADC转换值
* 输入参数：void
* 输出参数：void
* 返 回 值：ADC转换值
* 创建日期：2023/05/25
* 注  意：
**********************************************************************************************************/
u16 ReadSPO2ADC(void)
{
	return s_arrADCData[2];
}
