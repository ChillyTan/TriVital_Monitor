/*********************************************************************************************************
* 模块名称：ECG.c
* 文件说明：心电（ECG）信号处理模块
*           实现 ECG 信号滤波、R 波检测、心率计算、导联状态判断及显示
* 当前版本：1.0.0
* 作    者：Chill
* 完成日期：2026-02-06
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
#define FIR_ORDER 61        // FIR 滤波器阶数（当前未使用）
#define WindowsLen 3        // 平滑滤波窗口长度
#define HR_WAVE_LEN 300     // 心率阈值计算窗口长度
#define HR_MODE_WINDOWS_LEN 5

/*********************************************************************************************************
*                                           内部变量
*********************************************************************************************************/
/*
 * IIR 滤波器差分方程：
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 */

// 50Hz 工频陷波器（抑制电源干扰）
static double IIRNotch_win[N+1] = {0};
static double IIRNotch_b[N+1] = {0.245237275252786, 0.396802246667420, 0.245237275252786};
static double IIRNotch_a[N+1] = {1.0, 0.396802246667420, -0.509525449494429};

// 低通滤波器（30Hz，保留 ECG 主频段）
static double IIRLowpass_win[N+1] = {0};
static double IIRLowpass_b[N+1] = {0.274726851035635, 0.549453702071270, 0.274726851035635};
static double IIRLowpass_a[N+1] = {1.0, -0.0736238463849785, 0.172531250527518};

// 高通滤波器（1.5Hz，去除基线漂移）
static double IIRHighpass_win[N+1] = {0};
static double IIRHighpass_b[N+1] = {0.948080785129270, -1.89616157025854, 0.948080785129270};
static double IIRHighpass_a[N+1] = {1.0, -1.89346414636183, 0.898858994155252};

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

static double IIRNotch(double input, double *arrtemp);		// 工频陷波
static double IIRLowpass(double input, double *arrtemp);	// 低通滤波
static double IIRHighpass(double input, double *arrtemp);	// 高通滤波
static double SmoothingFilter(double NewData);						// 平滑滤波

static void Update_Threshold(double *data_window, int windowSize, double *threshold_output);
static void calRate(double ppdistance, int *rate_output);

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

/* 50Hz 陷波滤波器 */
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

/* 低通滤波器 */
static double IIRLowpass(double input, double *arrtemp)
{
  double output = 0;
  int i = 0;

  arrtemp[0] = input
             - IIRLowpass_a[1] * arrtemp[1]
             - IIRLowpass_a[2] * arrtemp[2];

  output = IIRLowpass_b[0] * arrtemp[0]
         + IIRLowpass_b[1] * arrtemp[1]
         + IIRLowpass_b[2] * arrtemp[2];

  for(i = N; i > 0; i--)
  {
    arrtemp[i] = arrtemp[i - 1];
  }

  return output;
}

/* 高通滤波器 */
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

/* 滑动平均平滑滤波 */
static double SmoothingFilter(double NewData)
{
  int n = 0;
  int num = 0;
  static int i = 0;
  static double buf[WindowsLen] = {0};

  if(i < WindowsLen)
  {
    buf[i++] = NewData;
    return NewData;
  }
  else
  {
    for(n = 0; n < WindowsLen - 1; n++)
    {
      buf[n] = buf[n + 1];
    }
    buf[WindowsLen - 1] = NewData;

    for(n = 0; n < WindowsLen; n++)
    {
      num += buf[n];
    }
    return (num * 1.0 / WindowsLen);
  }
}

/* 更新 R 波检测阈值 */
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

/* 根据 R-R 间期计算心率 */
static void calRate(double ppdistance, int *rate_output)
{
  *rate_output = (int)(60000.0 / ppdistance); // ppdistance：ms
}

/*********************************************************************************************************
*                                           API函数
*********************************************************************************************************/

/* ECG 模块初始化 */
void InitECG(void)
{
  ConfigECGGPIO();

  memset(IIRNotch_win, 0, sizeof(IIRNotch_win));
  memset(IIRLowpass_win, 0, sizeof(IIRLowpass_win));
  memset(IIRHighpass_win, 0, sizeof(IIRHighpass_win));

  memset(arr_ECG_Wave, 0, sizeof(arr_ECG_Wave));
  ECG_Wave_index = 0;
  peakThreshold = 0;
  lastPeak_index = 0;
  currentPeak_index = 0;
  heartRate = 0;
}

/* ECG 实时处理任务 */
void ECGTask(u32 inp)
{
  double output1 = 0;
  double output2 = 0;
  double output3 = 0;
  double output4 = 0;

  // ECG 信号处理链
  output1 = IIRNotch(inp, IIRNotch_win);
  output2 = IIRHighpass(output1, IIRHighpass_win);
  output3 = IIRLowpass(output2, IIRLowpass_win);
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

  if(g_displayMode == WAVE_ECG)
  {
    printf("%d ", (int)output3);
  }
}

/* OLED ECG 信息显示 */
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
    printf("[[1,Err]]\r\n");
  }
  else
  {
    GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
    OLEDShowString(88, 16, (u8*)"Good");

    if(heartRate >= 20 && heartRate <= 250)
    {
      OLEDShowNum(32, 0, heartRate, 3, 16);
      printf("[[1,%d]]\r\n", heartRate);
    }
    else
    {
      OLEDShowString(32, 0, (u8*)"Err");
      printf("[[1,Err]]\r\n");
    }
  }
}
