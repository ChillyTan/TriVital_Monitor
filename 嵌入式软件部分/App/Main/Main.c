/*********************************************************************************************************
* 模块名称：Main.c
* 文件说明：系统主入口文件
*           负责系统软硬件初始化，以及周期性任务调度
* 版本信息：V1.0.0
* 作    者：Chill
* 完成日期：2026-02-06
* 注意事项：需在 Keil 中勾选 Use MicroLIB，否则 printf 无法正常输出
*********************************************************************************************************/

/*********************************************************************************************************
*                                           头文件包含
*********************************************************************************************************/
#include "Main.h"
#include "stm32f10x_conf.h"
#include "DataType.h"
#include "NVIC.h"
#include "SysTick.h"
#include "RCC.h"
#include "Timer.h"
#include "UART1.h"
#include "LED.h"
#include "DAC.h"
#include "ADC.h"
#include "OLED.h"
#include "ECG.h"
#include "RESP.h"
#include "SPO2.h"
#include "PackUnpack.h"
#include "SendDataToHost.h"
#include "ProcHostCmd.h"

/*********************************************************************************************************
*                                           全局变量
*********************************************************************************************************/
/* 当前OLED显示的波形模式（心电 / 呼吸 / 血氧） */
WaveMode_t g_displayMode = WAVE_ECG;

/*********************************************************************************************************
*                                       内部函数声明
*********************************************************************************************************/
static void InitSoftware(void);   // 软件模块初始化
static void InitHardware(void);   // 硬件外设初始化
static void Proc2msTask(void);    // 2ms 周期任务
static void Proc1SecTask(void);   // 1s 周期任务

/*********************************************************************************************************
* 函数名称：InitSoftware
* 功    能：初始化所有纯软件逻辑相关模块
*********************************************************************************************************/
static void InitSoftware(void)
{
	InitPackUnpack();				// 初始化数据包打包解包模块
	InitSendDataToHost();		// 初始化发送数据到主机模块
	InitProcHostCmd();			// 初始化处理主机指令模块
}

/*********************************************************************************************************
* 函数名称：InitHardware
* 功    能：初始化所有硬件外设
*********************************************************************************************************/
static void InitHardware(void)
{
	SystemInit();           // STM32 系统时钟初始化
	InitRCC();              // RCC 时钟配置
	InitNVIC();             // 中断优先级配置
	InitUART1(115200);      // 串口1初始化，用于调试和指令接收
	InitTimer();            // 定时器初始化
	InitLED();              // LED 指示灯初始化
	InitSysTick();          // SysTick 定时器（2ms / 1s 任务基准）
	InitDAC();              // DAC 初始化
	InitADC();              // ADC 初始化
	InitOLED();             // OLED 显示初始化
	InitECG();              // ECG 采集模块初始化
	InitRESP();             // 呼吸信号采集初始化
	InitSPO2();             // 血氧模块初始化
}

/*********************************************************************************************************
* 函数名称：Proc2msTask
* 功    能：2ms 周期执行的实时任务
* 说    明：主要用于数据采样、协议处理等实时性较高的任务
*********************************************************************************************************/
static void Proc2msTask(void)
{
	static u8 s_iCnt2 = 0;  // 2ms 计数器，用于构造 4ms 周期
	// 发送给主机的数据包，包含 ECG、RESP、SPO2 波形数据（每个波形数据占 2 字节）
	static u8 s_waveDataPack[6] = {0, 0, 0, 0, 0, 0};

	int ecgWaveData;        // 心电 ADC 数据
	int respWaveData;       // 呼吸 ADC 数据
	int spo2WaveData;       // 血氧 ADC 数据

	if (Get2msFlag())
	{
		/* 每 4ms 执行一次信号处理任务 */
		if (s_iCnt2 >= 1)
		{
			// 获取波形数据
			ecgWaveData = ECGTask(ReadECGADC());
			respWaveData = RESPTask(ReadRESPADC());
			spo2WaveData = SPO2Task();
			// printf("%d\r\n", spo2WaveData);
			// 组装波形数据包
			s_waveDataPack[0] = ecgWaveData >> 8;
			s_waveDataPack[1] = ecgWaveData & 0xFF;
			s_waveDataPack[2] = respWaveData >> 8;
			s_waveDataPack[3] = respWaveData & 0xFF;
			s_waveDataPack[4] = spo2WaveData >> 8;
			s_waveDataPack[5] = spo2WaveData & 0xFF;
			// 发送波形数据包到主机
			SendWavePackHost(s_waveDataPack);

			s_iCnt2 = 0;
		}
		else
		{
			s_iCnt2++;
		}

		LEDFlicker(250);    // LED 心跳指示
		Clr2msFlag();       // 清除 2ms 标志
	}
}

/*********************************************************************************************************
* 函数名称：Proc1SecTask
* 功    能：1s 周期执行的低频任务
* 说    明：主要用于 OLED 显示刷新
*********************************************************************************************************/
static void Proc1SecTask(void)
{
	static u8 s_paramDataPack[6] = {0, 0, 0, 0, 0, 0};	// 参数数据包
	static u8 s_statusDataPack[6] = {0, 0, 0, 0, 0, 0};	// 状态数据包
	u16 heartRate;
	u16 respRate;
	u16 spo2Value;
	u8 ecgLeadStatus;
	u8 ecgErrStatus;
	u8 respLeadStatus;
	u8 respErrStatus;
	u8 spo2LeadStatus;
	u8 spo2ErrStatus;
	
	if (Get1SecFlag())
	{
		OLEDClear();        // 清屏
		OLED_ECG();         // 显示 ECG 信息
		OLED_RESP();        // 显示 RESP 信息
		OLED_SPO2();        // 显示 SPO2 信息
		OLEDRefreshGRAM();	// 刷新 OLED 显存

		// printf("TriVital-Monitor is ready!\r\n");

		// 获取参数数据
		heartRate = ECGGetHeartRate();
		respRate = RESPGetRespRate();
		spo2Value = SPO2GetSPO2Value();
	
		// 组装参数数据包
		s_paramDataPack[0] = heartRate >> 8;		//心率高位
		s_paramDataPack[1] = heartRate & 0xFF;	//心率低位
		s_paramDataPack[2] = respRate >> 8;
		s_paramDataPack[3] = respRate & 0xFF;
		s_paramDataPack[4] = spo2Value >> 8;
		s_paramDataPack[5] = spo2Value & 0xFF;
		// 发送参数数据包到主机
		SendParamPackHost(s_paramDataPack);

		// 获取状态数据
		ecgLeadStatus = ECGGetLeadStatus();
		ecgErrStatus = 0;
		respLeadStatus = RESPGetLeadStatus();
		respErrStatus = 0;
		spo2LeadStatus = SPO2GetLeadStatus();
		spo2ErrStatus = 0;
		// 组装状态数据包
		s_statusDataPack[0] = ecgLeadStatus;
		s_statusDataPack[1] = ecgErrStatus;
		s_statusDataPack[2] = respLeadStatus;
		s_statusDataPack[3] = respErrStatus;
		s_statusDataPack[4] = spo2LeadStatus;
		s_statusDataPack[5] = spo2ErrStatus;
		// 发送状态数据包到主机
		SendStatusPackHost(s_statusDataPack);

		Clr1SecFlag();
	}
}

/*********************************************************************************************************
* 函数名称：main
* 功    能：程序入口
*********************************************************************************************************/
int main(void)
{
	InitHardware();         // 硬件初始化
	InitSoftware();         // 软件初始化

	DelayNms(300);          // 等待系统稳定
	// printf("Init System has been finished.\r\n");

	while (1)
	{
		Proc2msTask();      // 实时任务
		Proc1SecTask();     // 显示任务
	}
}
