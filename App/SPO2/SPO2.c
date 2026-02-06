/*********************************************************************************************************
 * 模块名称：SPO2.c
 * 摘    要：血氧饱和度（SpO2）与脉率检测模块
 *            基于红光/红外光PPG信号，完成采集、滤波、脉率与SpO2计算
 * 当前版本：1.0.0
 * 作    者：SZLY (COPYRIGHT 2018 - 2020 SZLY. All rights reserved.)
 * 完成日期：2020年01月01日
 *
 * 功能说明：
 *  (1) 控制红光/红外LED时序点亮
 *  (2) 采集ADC信号并进行数字滤波
 *  (3) 计算脉率（Heart Rate）
 *  (4) 计算血氧饱和度（SpO2）
 *  (5) 支持自动调光与OLED显示
 *
 * 注    意：
 *  - SPO2_LED_Task() 需 1ms 周期调用
 *  - SPO2Task()      需 8ms 周期调用
 *********************************************************************************************************/

/*********************************************************************************************************
 *                                              包含头文件
 *********************************************************************************************************/
#include "stm32f10x_conf.h"
#include "string.h"
#include "SPO2.h"
#include "math.h"
#include "UART1.h"
#include "OLED.h"
#include "ADC.h"
#include "Timer.h"
#include "DAC.h"
#include "SysTick.h"

/*********************************************************************************************************
 *                                              宏定义
 *********************************************************************************************************/
/* 红光 / 红外LED控制 */
#define RED_ON GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET)		// 红光LED开启
#define RED_OFF GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_RESET) // 红光LED关闭
#define IR_ON GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_SET)			// 红外LED开启
#define IR_OFF GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET)	// 红外LED关闭

/* 滤波参数 */
#define N 2							// IIR滤波器阶数
#define SMOOTH_LEN 5		// 滑动均值滤波窗口长度
#define SP_WAVE_LEN 300 // SPO2波形分析窗口长度（≈5s）
#define HR_AVG_N 5			// 脉率平均窗口
#define R_BUFSIZE 5			// R值中值滤波缓冲长度

/* 自动调光参数 */
#define RED_INTENSITY_MIN 100 // 红光最小亮度
#define RED_INTENSITY_MAX 500 // 红光最大亮度
#define RED_INTENSITY_STEP 40 // 调光步进
#define ADJUST_STABLE_DELAY 1 // 调光后稳定等待周期

/*********************************************************************************************************
 *                                              枚举结构体定义
 *********************************************************************************************************/

/*********************************************************************************************************
 *                                              内部变量
 *********************************************************************************************************/
static int SPO2_Wave_data_RED = 0; // RED信号
static int SPO2_Wave_data_IR = 0;	 // IR信号

// y[n] = b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] - a1 * y[n-1] - a2 * y[n-2]
// b为分子系数 a为分母系数

// 二阶IIR 低通滤波器 3Hz
static double IIRLowpass_win_RED[N + 1] = {0};
static double IIRLowpass_win_IR[N + 1] = {0};
static double IIRLowpass_b[N + 1] = {0.00512926836610717, 0.0102585367322143, 0.00512926836610717};
static double IIRLowpass_a[N + 1] = {1.0, -1.78743251795648, 0.807949591420913};

// 二阶IIR 高通滤波器 0.3Hz
static double IIRHighpass_win_RED[N + 1] = {0};
static double IIRHighpass_win_IR[N + 1] = {0};
static double IIRHighpass_b[N + 1] = {0.989393726763531, -1.97878745352706, 0.989393726763531};
static double IIRHighpass_a[N + 1] = {1.0, -1.97867495733125, 0.978899949722877};

// 脉率计算
static double arr_SPO2_Wave_Rate[SP_WAVE_LEN] = {0}; // 动态阈值更新窗口
static int SPO2_Wave_index = 0;
static double peakThreshold = 0;
static int lastPeak_index = 0;
static int currentPeak_index = 0;
static int heartRate = 0;
static int hr_buf[HR_AVG_N] = {0};
static int hr_cnt = 0;

// 血氧饱和度计算
static double arr_SPO2_Wave_RED[SP_WAVE_LEN] = {0};
static double arr_SPO2_Wave_IR[SP_WAVE_LEN] = {0};
static double peak2peak_RED = 0;
static double peak2peak_IR = 0;
static double value_R = 0;
static double value_SPO2 = 0;
static int rValue_buf[R_BUFSIZE] = {0};

// 血氧调光 导联检测
static u16 s_DACdata = 0;
static int adjust_wait_cnt = 0; // 调光后等待计数

/*********************************************************************************************************
 *                                              内部函数声明
 *********************************************************************************************************/
static void ConfigCSGPIO(void);

/*input单点输入信号	arrtemp滤波缓冲数组*/
static double IIRLowpass(double input, double *arrtemp);
static double IIRHighpass(double input, double *arrtemp);
static double SmoothingFilter_RED(double NewData);
static double SmoothingFilter_IR(double NewData);

// 计算脉率 血氧饱和度
static void Analyze_SPO2Wave(double *wave1, double *wave2, double *wave3, int waveSize, double *ppRed_output, double *ppIR_output); //
static void calRate(double ppdistance, int *rate_output);
static void bubbleSort(int *arr, int size);
static void calSpO2(double redPeak, double irPeak, double *rValue, double *spo2);

/*********************************************************************************************************
 *                                              内部函数实现
 *********************************************************************************************************/
static void ConfigCSGPIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; // GPIO_InitStructure用于存放GPIO的参数

	// 使能RCC相关时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能GPIOC的时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;					// 设置引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 设置I/O输出速度
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 设置模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);						// 根据参数初始化LED1的GPIO

	GPIO_WriteBit(GPIOA, GPIO_Pin_5, Bit_SET); // 将LED1默认状态设置为点亮

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;					// 设置引脚
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 设置I/O输出速度
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	// 设置模式
	GPIO_Init(GPIOA, &GPIO_InitStructure);						// 根据参数初始化LED2的GPIO

	GPIO_WriteBit(GPIOA, GPIO_Pin_6, Bit_RESET); // 将LED2默认状态设置为熄灭
}

/*********************************************************************************************************
 * 函数名称：各种滤波器实现
 * 函数功能：
 * 输入参数：void
 * 输出参数：void
 * 返 回 值：void
 * 创建日期：2018年01月01日
 * 注    意：
 *********************************************************************************************************/
static double IIRLowpass(double input, double *arrtemp)
{
	double output = 0;
	int i = 0;

	arrtemp[0] = input - IIRLowpass_a[1] * arrtemp[1] - IIRLowpass_a[2] * arrtemp[2]; // 反馈
	output = IIRLowpass_b[0] * arrtemp[0] + IIRLowpass_b[1] * arrtemp[1] + IIRLowpass_b[2] * arrtemp[2];

	// 移动滤波缓冲数组
	for (i = N; i > 0; i--)
	{
		arrtemp[i] = arrtemp[i - 1];
	}

	return output;
}

static double IIRHighpass(double input, double *arrtemp)
{
	double output = 0;
	int i = 0;

	arrtemp[0] = input - IIRHighpass_a[1] * arrtemp[1] - IIRHighpass_a[2] * arrtemp[2]; // 反馈
	output = IIRHighpass_b[0] * arrtemp[0] + IIRHighpass_b[1] * arrtemp[1] + IIRHighpass_b[2] * arrtemp[2];

	// 移动滤波缓冲数组
	for (i = N; i > 0; i--)
	{
		arrtemp[i] = arrtemp[i - 1];
	}

	return output;
}

static double SmoothingFilter_RED(double NewData)
{
	int n = 0;
	int num = 0;
	static int i = 0;
	static double buf[SMOOTH_LEN] = {0}; // 滤波器缓存
	// 将新数据放入缓存
	if (i < SMOOTH_LEN)
	{
		buf[i] = NewData;
		i++;
		return NewData;
	}
	else
	{
		for (n = 0; n < SMOOTH_LEN - 1; n++) // 数组左移
		{
			buf[n] = buf[n + 1];
		}
		buf[SMOOTH_LEN - 1] = NewData;

		// 计算滤波结果
		for (n = 0; n < SMOOTH_LEN; n++)
		{
			num = num + buf[n];
		}
		return (num * 1.0 / SMOOTH_LEN);
	}
}

static double SmoothingFilter_IR(double NewData)
{
	int n = 0;
	int num = 0;
	static int i = 0;
	static double buf[SMOOTH_LEN] = {0}; // 滤波器缓存
	// 将新数据放入缓存
	if (i < SMOOTH_LEN)
	{
		buf[i] = NewData;
		i++;
		return NewData;
	}
	else
	{
		for (n = 0; n < SMOOTH_LEN - 1; n++) // 数组左移
		{
			buf[n] = buf[n + 1];
		}
		buf[SMOOTH_LEN - 1] = NewData;

		// 计算滤波结果
		for (n = 0; n < SMOOTH_LEN; n++)
		{
			num = num + buf[n];
		}
		return (num * 1.0 / SMOOTH_LEN);
	}
}

/*********************************************************************************************************
 * 函数名称：Analyze_SPO2Wave
 * 函数功能：
 *   对红光、红外光及脉率波形进行分析，计算：
 *     (1) 红光峰峰值
 *     (2) 红外光峰峰值
 *     (3) 动态峰值检测阈值
 *
 * 输入参数：
 *   wave1     - 红光波形数组
 *   wave2     - 红外光波形数组
 *   wave3     - 红外光波形数组(用于分析脉率)
 *   waveSize  - 波形长度
 *
 * 输出参数：
 *   ppRed_output - 红光峰峰值
 *   ppIR_output  - 红外光峰峰值
 *********************************************************************************************************/
static void Analyze_SPO2Wave(double *wave1, double *wave2, double *wave3, int waveSize, double *ppRed_output, double *ppIR_output)
{
	int i; // 循环变量
	double wave1_max = 0.0;
	double wave1_min = 4095.0;
	double wave2_max = 0.0;
	double wave2_min = 4095.0;
	double wave3_max = 0.0;
	double wave3_min = 4095.0;

	for (i = 0; i < waveSize; i++)
	{
		// 计算波形1的峰峰值
		if (wave1[i] < wave1_min)
			wave1_min = wave1[i];
		if (wave1[i] > wave1_max)
			wave1_max = wave1[i];

		// 计算波形2的峰峰值
		if (wave2[i] < wave2_min)
			wave2_min = wave2[i];
		if (wave2[i] > wave2_max)
			wave2_max = wave2[i];
		// 计算波形3的峰峰值
		if (wave3[i] < wave3_min)
			wave3_min = wave3[i];
		if (wave3[i] > wave3_max)
			wave3_max = wave3[i];
	}
	*ppRed_output = wave1_max - wave1_min;
	*ppIR_output = wave2_max - wave2_min;
	peakThreshold = wave3_max - (wave3_max - wave3_min) / 3;
}

/*********************************************************************************************************
 * 函数名称：calRate
 * 函数功能：
 * 输入参数：void
 * 输出参数：void
 * 返 回 值：void
 * 创建日期：2018年01月01日
 * 注    意：
 *********************************************************************************************************/
static void calRate(double ppdistance, int *rate_output)
{
	*rate_output = (int)((double)((60000.0 / (ppdistance))));
}

/*********************************************************************************************************
 * 函数名称：冒泡排序
 * 函数功能：
 * 输入参数：void
 * 输出参数：void
 * 返 回 值：void
 * 创建日期：2018年01月01日
 * 注    意：
 *********************************************************************************************************/
static void bubbleSort(int *arr, int size)
{
	int i, j;
	for (i = 0; i < size - 1; i++)
	{
		for (j = 0; j < size - i - 1; j++)
		{
			if (arr[j] > arr[j + 1])
			{
				uint16_t temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
		}
	}
}

/*********************************************************************************************************
 * 函数名称：计算R值和血氧饱和度
 * 函数功能：
 * 输入参数：void
 * 输出参数：void
 * 返 回 值：void
 * 创建日期：2018年01月01日
 * 注    意：
 *********************************************************************************************************/
static void calSpO2(double redPeak, double irPeak, double *rValue, double *spo2)
{
	int rInt = 0;
	int i = 0;
	int tempBuf[R_BUFSIZE] = {0};

	// 计算R值 (AC/DC比率)
	*rValue = redPeak / irPeak;
	*rValue = *rValue * 1000;

	for (i = 0; i < R_BUFSIZE - 1; i++)
	{
		rValue_buf[i] = rValue_buf[i + 1];
	}
	rValue_buf[R_BUFSIZE - 1] = *rValue;

	// 拷贝到临时数组排序
	for (i = 0; i < R_BUFSIZE; i++)
	{
		tempBuf[i] = rValue_buf[i];
	}
	bubbleSort(tempBuf, R_BUFSIZE);

	// 取中间两个数的平均值
	if (R_BUFSIZE % 2 == 0) // 偶数个元素
	{
		rInt = (tempBuf[R_BUFSIZE / 2] + tempBuf[R_BUFSIZE / 2 + 1]) / 2;
	}
	else // 奇数个元素
	{
		rInt = tempBuf[R_BUFSIZE / 2];
	}

	// 使用经验公式计算血氧饱和度(R值表)
	if (rInt <= 450)
	{
		*spo2 = 99.0;
	}
	else if (rInt >= 451 && rInt <= 470)
	{
		*spo2 = 98.0;
	}
	else if (rInt >= 471 && rInt <= 510)
	{
		*spo2 = 97.0;
	}
	else if (rInt >= 511 && rInt <= 540)
	{
		*spo2 = 96.0;
	}
	else if (rInt >= 541 && rInt <= 580)
	{
		*spo2 = 95.0;
	}
	else if (rInt >= 581 && rInt <= 600)
	{
		*spo2 = 94.0;
	}
	else if (rInt >= 601 && rInt <= 620)
	{
		*spo2 = 93.0;
	}
	else if (rInt >= 621 && rInt <= 640)
	{
		*spo2 = 92.0;
	}
	else if (rInt >= 641 && rInt <= 680)
	{
		*spo2 = 91.0;
	}
	else if (rInt >= 681 && rInt <= 720)
	{
		*spo2 = 90.0;
	}
	else if (rInt >= 721 && rInt <= 800)
	{
		*spo2 = 89.0;
	}
	else if (rInt >= 801 && rInt <= 940)
	{
		*spo2 = 88.0;
	}
	else if (rInt >= 941 && rInt <= 980)
	{
		*spo2 = 87.0;
	}
	else if (rInt >= 981 && rInt <= 1020)
	{
		*spo2 = 86.0;
	}
	else if (rInt >= 1021 && rInt <= 1060)
	{
		*spo2 = 85.0;
	}
	else if (rInt >= 1061 && rInt <= 1080)
	{
		*spo2 = 84.0;
	}
	else if (rInt >= 1081 && rInt <= 1100)
	{
		*spo2 = 83.0;
	}
	else if (rInt >= 1101 && rInt <= 1120)
	{
		*spo2 = 82.0;
	}
	else if (rInt >= 1121 && rInt <= 1150)
	{
		*spo2 = 81.0;
	}
	else if (rInt >= 1151 && rInt <= 1170)
	{
		*spo2 = 80.0;
	}
	else if (rInt >= 1171 && rInt <= 1200)
	{
		*spo2 = 79.0;
	}
	else
	{ // 如果超出表格范围，使用默认值 *spo2 = 85.0;
	}

	//	if (rInt <= 1100) {
	//			*spo2 = 99.0;
	//	} else if (rInt <= 1200) {
	//			*spo2 = 98.0;
	//	} else if (rInt <= 1300) {
	//			*spo2 = 97.0;
	//	} else if (rInt <= 1400) {
	//			*spo2 = 96.0;
	//	} else if (rInt <= 1550) {
	//			*spo2 = 95.0;
	//	} else if (rInt <= 1700) {
	//			*spo2 = 94.0;
	//	} else if (rInt <= 1900) {
	//			*spo2 = 93.0;
	//	} else {
	//			*spo2 = 92.0;   // 超出可信范围，保守处理
	//	}

	//	printf("pp = %lf %lf\r\n", redPeak, irPeak);
	//	printf("R = %d\r\n", rInt);
	//	printf("spo2 = %lf\r\n", *spo2);
	//	printf("[[2,%d]]\r\n", rInt);

	// 确保血氧饱和度在合理范围内 (70%-100%)
	if (*spo2 > 100.0f)
	{
		*spo2 = 100.0f;
	}
	else if (*spo2 < 70.0f)
	{
		*spo2 = 70.0f;
	}
}

/*********************************************************************************************************
 *                                              API函数实现
 *********************************************************************************************************/
/*********************************************************************************************************
 * 函数名称：InitSPO2
 * 函数功能：初始化SPO2模块
 * 输入参数：void
 * 输出参数：void
 * 返 回 值：void
 * 创建日期：2018年01月01日
 * 注    意：
 *********************************************************************************************************/
void InitSPO2(void)
{
	ConfigCSGPIO();
	memset(IIRLowpass_win_RED, 0, sizeof(IIRLowpass_win_RED));
	memset(IIRLowpass_win_IR, 0, sizeof(IIRLowpass_win_IR));
	s_DACdata = 240;
}

/*********************************************************************************************************
 * 函数名称：SPO2Task
 * 函数功能：血氧模块顶层任务
 * 输入参数：void
 * 输出参数：void
 * 返 回 值：void
 * 创建日期：2018年01月01日
 * 注    意：
 *********************************************************************************************************/
void SPO2_LED_Task(void) // 每1ms执行一次
{
	// 按下按键开始血氧测量
	// 1ms后红灯发光
	// 1ms后ADC采样一个点
	// 1ms后红灯熄灭
	// 2ms后红外灯发光
	// 1ms后ADC采样一个点
	// 1ms后红外灯熄灭
	static u8 state = 0; // 状态机变量
	state = (state + 1) % 8;

	switch (state)
	{
	case 0: // 初始状态两灯灭
		RED_ON;
		IR_OFF;
		break;

	case 2:																// 1ms后ADC采样红灯数据
		SPO2_Wave_data_RED = ReadSPO2ADC(); // 启动ADC采样红灯通道
		RED_OFF;														// 红灯关闭
		IR_OFF;
		break;

	case 4:		 // 2ms后红外灯亮
		RED_OFF; // 红灯关闭
		IR_ON;	 // 红外灯打开
		break;

	case 6:															 // 1ms后ADC采样红外灯数据
		SPO2_Wave_data_IR = ReadSPO2ADC(); // 启动ADC采样红外灯通道
		RED_OFF;													 // 红灯关闭
		IR_OFF;
		break;

	default:
		break;
	}
}
/*********************************************************************************************************
 * 函数名称：SPO2Task
 * 函数功能：血氧模块顶层任务
 * 输入参数：void
 * 输出参数：void
 * 返 回 值：void
 * 创建日期：2018年01月01日
 * 注    意：
 *********************************************************************************************************/
void SPO2Task() // 每8ms执行一次
{
	double output0[2] = {0};
	double output1[2] = {0};
	double output2[2] = {0};
	int cur_hr = 0;
	int hr_sum = 0;
	int i;

	// 用于计算血氧饱和度的波形
	output0[0] = IIRHighpass(SPO2_Wave_data_RED, IIRHighpass_win_RED);
	output0[1] = IIRHighpass(SPO2_Wave_data_IR, IIRHighpass_win_IR);
	output1[0] = IIRLowpass(output0[0], IIRLowpass_win_RED);
	output1[1] = IIRLowpass(output0[1], IIRLowpass_win_IR);
	output2[0] = SmoothingFilter_RED(output1[0]);
	output2[1] = SmoothingFilter_IR(output1[1]);
	// 用于计算脉率的波形

	arr_SPO2_Wave_RED[SPO2_Wave_index] = output2[0];
	arr_SPO2_Wave_IR[SPO2_Wave_index] = output2[1];
	arr_SPO2_Wave_Rate[SPO2_Wave_index] = output2[1];
	SPO2_Wave_index++;

	// 如果采集到5s波形数据
	if (SPO2_Wave_index >= SP_WAVE_LEN)
	{
		SPO2_Wave_index = 0;

		// 调光等待期间不分析和调光
		if (adjust_wait_cnt > 0)
		{
			adjust_wait_cnt--;
			return;
		}

		// 分析
		Analyze_SPO2Wave(arr_SPO2_Wave_RED, arr_SPO2_Wave_IR, arr_SPO2_Wave_Rate, SP_WAVE_LEN, &peak2peak_RED, &peak2peak_IR);
		calSpO2(peak2peak_RED, peak2peak_IR, &value_R, &value_SPO2);
		// 自动调光
		if (peak2peak_RED < 20 && s_DACdata < RED_INTENSITY_MAX)
		{
			s_DACdata += RED_INTENSITY_STEP;
			if (s_DACdata > RED_INTENSITY_MAX)
			{
				s_DACdata = RED_INTENSITY_MAX;
			}
			AdjustDAC(s_DACdata);
			adjust_wait_cnt = ADJUST_STABLE_DELAY; // 调光后等待
		}
		else if (peak2peak_RED > 80 && s_DACdata > RED_INTENSITY_MIN)
		{
			s_DACdata -= RED_INTENSITY_STEP;
			if (s_DACdata < RED_INTENSITY_MIN)
			{
				s_DACdata = RED_INTENSITY_MIN;
			}
			AdjustDAC(s_DACdata);
			adjust_wait_cnt = ADJUST_STABLE_DELAY; // 调光后等待
		}
	}

	// 实时检测R波
	if ((SPO2_Wave_index > 1) && (SPO2_Wave_index < SP_WAVE_LEN - 1))
	{
		if ((arr_SPO2_Wave_Rate[SPO2_Wave_index - 2] - peakThreshold >= 0) && ((arr_SPO2_Wave_Rate[SPO2_Wave_index - 1] - peakThreshold) <= 0))
		{
			// printf("%d ", 0);
			currentPeak_index = GetTimeCounter();
			calRate(currentPeak_index - lastPeak_index, &cur_hr);
			lastPeak_index = currentPeak_index;
			if (hr_cnt <= 4)
			{
				hr_buf[hr_cnt] = cur_hr;
				hr_cnt++;
			}
			else
			{
				for (i = 0; i < 5; i++)
				{
					hr_sum += hr_buf[i];
				}
				heartRate = hr_sum / 5;
				hr_cnt = 0;
			}
		}
	}

	if (g_displayMode == WAVE_SPO2)
	{
		printf("%d, %d\r\n", (int)output0[0], (int)output0[1]);
	}
}

/*********************************************************************************************************
 * 函数名称：OLED_SPO2
 * 函数功能：OLED显示血氧信息
 * 输入参数：void
 * 输出参数：void
 * 返 回 值：void
 * 创建日期：2018年01月01日
 * 注    意：
 *********************************************************************************************************/
void OLED_SPO2(void)
{
	printf("[[3,%d]]\r\n", (int)heartRate);
	printf("[[4,%d]]\r\n", (int)value_SPO2);
}
