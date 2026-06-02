/*********************************************************************************************************
* 模块名称：ECG.c
* 文件说明：心电（ECG）信号处理模块
*           实现 ECG 信号滤波、R 波检测、心率计算、导联状态判断及显示
* 当前版本：1.0.0
* 作    者：Chill
* 完成日期：2026-04-16
*********************************************************************************************************/

/*********************************************************************************************************
*                                           头文件包含
*********************************************************************************************************/
#include "stm32f10x_conf.h"
#include "string.h"
#include "ECG.h"
#include "math.h"
#include "UART1.h"
#include "OLED.h"
#include "Timer.h"

/*********************************************************************************************************
*                                           宏定义
*********************************************************************************************************/
#define N 2                 // IIR 滤波器阶数（二阶）
#define SmoothWindowsLen 8  // 平滑滤波窗口长度
#define MedianWindowsLen 5  // 中值滤波窗口长度
#define HR_WAVE_LEN 600     // 心率阈值计算窗口长度

/*********************************************************************************************************
*                                           内部变量
*********************************************************************************************************/
/*
 * IIR 滤波器差分方程：
 * 一阶: y[n] = alpha * x + (1 - alpha) * y[n-1];
 * 二阶: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 */

// 50Hz 工频陷波器（抑制电源干扰）
static double IIRNotch_win[N+1] = {0};
static double IIRNotch_b[N+1] = {0.755202, -0.466740, 0.755202};
static double IIRNotch_a[N+1] = {1.000000, -0.466740, 0.510404};

// 高通滤波器（1Hz，去除基线漂移）
static double IIRHighpass_win[N+1] = {0};
static double IIRHighpass_b[N+1] = {0.982385, -1.964771, 0.982385};
static double IIRHighpass_a[N+1] = {1.000000, -1.964461, 0.965081};

// 心率计算相关变量
static double arr_ECG_Wave[HR_WAVE_LEN] = {0}; // ECG 波形缓存
static int ECG_Wave_index = 0;                 // 波形索引
static double peakThreshold = 0;               // R 波检测阈值
static u32 lastPeak_index = 0;                 // 上一次 R 波时间
static u32 currentPeak_index = 0;              // 当前 R 波时间
static int heartRate = 0;                      // 心率（BPM）

/*********************************************************************************************************
*                                           内部函数声明
*********************************************************************************************************/
/*
 * LEAD_OFF  -- PB0  (输入，引导脱落检测)
 * ECG_ZERO  -- PB1  (输出，硬件基线控制)
 */
static void ConfigECGGPIO(void);

static double IIRNotch(double input, double *arrtemp);		// 50Hz工频陷波
static double IIRHighpass(double input, double *arrtemp);	// 2阶IIR高通滤波
static double SmoothingFilter(double newData);						// 平滑滤波
static double MedianFIlter(double newData);						    // 中值滤波

static void Update_Threshold(double *data_window, int windowSize, double *threshold_output);  // 更新心率阈值
static void calRate(double ppdistance, int *rate_output);  // 计算心率

/*********************************************************************************************************
*                                           内部函数实现
*********************************************************************************************************/
/* ECG 导联相关 GPIO 初始化 */
static void ConfigECGGPIO(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  // LEAD_OFF：导联脱落检测（上拉输入）
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // ECG_ZERO：基线控制输出
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
}

/*********************************************************************************************************
* 函数名称：50Hz工频陷波滤波器
* 函数功能：清除50Hz工频干扰
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：
*********************************************************************************************************/
static double IIRNotch(double input, double *arrtemp)
{
  double output = 0;
  int i = 0;

  arrtemp[0] = input
             - IIRNotch_a[1] * arrtemp[1]
             - IIRNotch_a[2] * arrtemp[2];

  output = IIRNotch_b[0] * arrtemp[0]
         + IIRNotch_b[1] * arrtemp[1]
         + IIRNotch_b[2] * arrtemp[2];

  for(i = N; i > 0; i--)
  {
    arrtemp[i] = arrtemp[i - 1];
  }

  return output;
}

/*********************************************************************************************************
* 函数名称：2阶IIR高通滤波器
* 函数功能：对输入信号进行2阶高通滤波
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：
*********************************************************************************************************/
static double IIRHighpass(double input, double *arrtemp)
{
  double output = 0;
  int i = 0;

  arrtemp[0] = input
             - IIRHighpass_a[1] * arrtemp[1]
             - IIRHighpass_a[2] * arrtemp[2];

  output = IIRHighpass_b[0] * arrtemp[0]
         + IIRHighpass_b[1] * arrtemp[1]
         + IIRHighpass_b[2] * arrtemp[2];

  for(i = N; i > 0; i--)
  {
    arrtemp[i] = arrtemp[i - 1];
  }

  return output;
}

/*********************************************************************************************************
* 函数名称：滑动平均平滑滤波
* 函数功能：对输入数据进行滑动平均平滑滤波
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：
*********************************************************************************************************/
static double SmoothingFilter(double newData)
{
  static double buf[SmoothWindowsLen] = {0};
  static int idx = 0;
  static int count = 0;
  static double sum = 0;

  sum -= buf[idx];
  buf[idx] = newData;
  sum += newData;

  idx++;
  if(idx >= SmoothWindowsLen) idx = 0;

  if(count < SmoothWindowsLen) count++;

  return sum / count;
}

/*********************************************************************************************************
* 函数名称：中值滤波
* 函数功能：对输入数据进行中值滤波
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：
*********************************************************************************************************/
static double MedianFIlter(double newData)
{
  static double buf[MedianWindowsLen] = {0};
  static int idx = 0;
  int i = 0;
  int j = 0;
  double temp[MedianWindowsLen];
  
  // 写入环形缓冲
  buf[idx++] = newData;
  if(idx >= MedianWindowsLen) idx = 0;

  // 拷贝一份用于排序
  for(i = 0; i < MedianWindowsLen; i++)
    temp[i] = buf[i];

  // 简单冒泡排序
  for(i = 0; i < MedianWindowsLen - 1; i++)
  {
    for(j = 0; j < MedianWindowsLen - 1 - i; j++)
    {
      if(temp[j] > temp[j + 1])
      {
        double t = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = t;
      }
    }
  }

  // 返回中值
  return temp[MedianWindowsLen / 2];
}

/*********************************************************************************************************
* 函数名称：更新心率阈值
* 函数功能：根据输入数据更新心率阈值
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：
*********************************************************************************************************/
static void Update_Threshold(double *data_window, int windowSize, double *threshold_output)
{
  int i;
  double peakMax = 0.0;
  double peakMin = 4095.0;

  for(i = 1; i < windowSize; i++)
  {
    if(data_window[i] > peakMax) peakMax = data_window[i];
    if(data_window[i] < peakMin) peakMin = data_window[i];
  }

  *threshold_output = peakMax - (peakMax - peakMin) / 4;
}

/*********************************************************************************************************
* 函数名称：计算心率
* 函数功能：根据输入数据计算心率
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：
*********************************************************************************************************/
static void calRate(double ppdistance, int *rate_output)
{
  static int arrRates[5] = {0};
  static int idx = 0;
  static int count = 0;
  int temp[5];
  int i, j;
  int currentRate;

  currentRate = (int)(60000.0 / ppdistance);
  arrRates[idx] = currentRate;
  idx = (idx + 1) % 5;
  
  if(count < 5) count++;

  for(i = 0; i < count; i++)
  {
    temp[i] = arrRates[i];
  }

  for(i = 0; i < count - 1; i++)
  {
    for(j = 0; j < count - 1 - i; j++)
    {
      if(temp[j] > temp[j + 1])
      {
        int t = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = t;
      }
    }
  }

  // 取中位数
  *rate_output = temp[2];
}

/*********************************************************************************************************
*                                           API函数
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：初始化ECG模块
* 函数功能：初始化ECG模块的GPIO引脚和内存
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：
*********************************************************************************************************/
void InitECG(void)
{
  ConfigECGGPIO();

  memset(IIRNotch_win, 0, sizeof(IIRNotch_win));
  memset(IIRHighpass_win, 0, sizeof(IIRHighpass_win));
  memset(arr_ECG_Wave, 0, sizeof(arr_ECG_Wave));

  ECG_Wave_index = 0;
  peakThreshold = 0;
  lastPeak_index = 0;
  currentPeak_index = 0;
  heartRate = 0;
}

/*********************************************************************************************************
* 函数名称：ECG实时处理任务
* 函数功能：对输入的ECG信号进行实时处理，包括滤波、平滑、心率计算等
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：
*********************************************************************************************************/
/*
  函数名称：ECG实时处理任务
  函数功能：对输入的ECG信号进行实时处理，包括滤波、平滑、心率计算等
*/
int ECGTask(u16 inp)
{
  double output1;
  double output2;
  double output3;
  double output4;
  
  output1 = IIRNotch(inp, IIRNotch_win);
  output2 = IIRHighpass(output1, IIRHighpass_win);
  output3 = MedianFIlter(output2);
  output4 = SmoothingFilter(output3);
  
  arr_ECG_Wave[ECG_Wave_index++] = output4;

  if(ECG_Wave_index >= HR_WAVE_LEN)
  {
    ECG_Wave_index = 0;
    Update_Threshold(arr_ECG_Wave, HR_WAVE_LEN, &peakThreshold);
  }

  // R 波上升沿检测
  if((ECG_Wave_index > 1) && (ECG_Wave_index < HR_WAVE_LEN - 1))
  {
    if((arr_ECG_Wave[ECG_Wave_index - 2] <= peakThreshold) &&
       (arr_ECG_Wave[ECG_Wave_index - 1] >= peakThreshold))
    {
      currentPeak_index = GetTimeCounter();
      calRate(currentPeak_index - lastPeak_index, &heartRate);
      lastPeak_index = currentPeak_index;
    }
  }

  return (int)output4;
}

/*********************************************************************************************************
* 函数名称：获取心率
* 函数功能：获取当前心率
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：
*********************************************************************************************************/
u16 ECGGetHeartRate(void)
{
  return heartRate;
}

/*********************************************************************************************************
* 函数名称：获取导联状态
* 函数功能：获取当前导联状态
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：0-导联脱落，1-导联正常
*********************************************************************************************************/
u8 ECGGetLeadStatus(void)
{
  u8 leadFlag;
  leadFlag = (u8)(1 - (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)));
  return leadFlag;
}

/*********************************************************************************************************
* 函数名称：显示LED ECG模块的信息
* 函数功能：在LED显示上显示当前心率、导联状态等信息
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 创建日期：2026年04月16日
* 注    意：
*********************************************************************************************************/
void OLED_ECG(void)
{
  OLEDShowString(0, 0, (u8*)"HR:");
  OLEDShowString(64, 0, (u8*)"BPM");
  OLEDShowString(0, 16, (u8*)"ECG_LEAD:");

  // 导联脱落
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 1)
  {
    GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
    OLEDShowString(88, 16, (u8*)"Noob");
    OLEDShowString(32, 0, (u8*)"Err");
    // printf("[[1,Err]]\r\n");
  }
  else
  {
    GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
    OLEDShowString(88, 16, (u8*)"Good");

    if(heartRate >= 20 && heartRate <= 250)
    {
      OLEDShowNum(32, 0, heartRate, 3, 16);
      // printf("[[1,%d]]\r\n", heartRate);
    }
    else
    {
      OLEDShowString(32, 0, (u8*)"Err");
      // printf("[[1,Err]]\r\n");
    }
  }
}
