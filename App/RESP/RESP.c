/*********************************************************************************************************
* 模块名称：RESP.c
* 文件说明：呼吸（RESP）信号处理模块
*           完成呼吸信号滤波、呼吸率计算、导联状态判断及显示输出
* 当前版本：1.0.0
* 作    者：Chill
* 完成日期：2026-02-06
*********************************************************************************************************/

/*********************************************************************************************************
*                                           头文件包含
*********************************************************************************************************/
#include "stm32f10x_conf.h"
#include "string.h"
#include "RESP.h"
#include "math.h"
#include "UART1.h"
#include "OLED.h"
#include "Timer.h"

/*********************************************************************************************************
*                                           宏定义
*********************************************************************************************************/
#define N 2               // IIR 滤波器阶数（二阶）
#define WindowsLen 100    // 平滑滤波窗口长度
#define BR_WAVE_LEN 1000  // 呼吸波形阈值计算窗口长度

/*********************************************************************************************************
*                                           内部变量
*********************************************************************************************************/
/*
 * IIR 滤波器差分方程：
 * y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
 */

// 50Hz 工频陷波器
static double IIRNotch_win[N+1] = {0};   // 状态缓存
static double IIRNotch_b[N+1] = {0.245237275252786, 0.396802246667420, 0.245237275252786};
static double IIRNotch_a[N+1] = {1.0, 0.396802246667420, -0.509525449494429};

// 二阶 IIR 低通滤波器（10Hz）
static double IIRLowpass_win[N+1] = {0};
static double IIRLowpass_b[N+1] = {0.0461318020933129, 0.0922636041866259, 0.0461318020933129};
static double IIRLowpass_a[N+1] = {1.0, -1.30728502884932, 0.491812237222575};

// 呼吸率计算相关变量
static double arr_BR_Wave[BR_WAVE_LEN] = {0}; // 呼吸波形缓存
static int BR_Wave_index = 0;                 // 波形索引
static double peakThreshold = 0;              // 峰值检测阈值
static double lastPeak_index = 0;              // 上一次峰值时间
static double currentPeak_index = 0;           // 当前峰值时间
static int breathRate = 0;                     // 呼吸率（BPM）

// 导联状态判断（峰峰值）
static double s_peak2peak = 0;

/*********************************************************************************************************
*                                           内部函数声明
*********************************************************************************************************/
static double IIRNotch(double input, double *arrtemp);      // 陷波滤波器
static double IIRLowpass(double input, double *arrtemp);   // 低通滤波器
static double SmoothingFilter(double NewData);              // 平滑滤波

static void Update_Threshold(double *data_window, int windowSize, double *threshold_output); // 阈值更新
static void calRate(double ppdistance, int *rate_output);   // 呼吸率计算

/*********************************************************************************************************
*                                           内部函数实现
*********************************************************************************************************/

/* 50Hz 陷波滤波器 */
static double IIRNotch(double input, double *arrtemp)
{
  double output = 0;
  int i = 0;

  // 反馈计算
  arrtemp[0] = input
             - IIRNotch_a[1] * arrtemp[1]
             - IIRNotch_a[2] * arrtemp[2];

  // 前向计算
  output = IIRNotch_b[0] * arrtemp[0]
         + IIRNotch_b[1] * arrtemp[1]
         + IIRNotch_b[2] * arrtemp[2];

  // 状态左移
  for(i = N; i > 0; i--)
  {
    arrtemp[i] = arrtemp[i - 1];
  }

  return output;
}

/* 低通滤波器（抑制高频噪声） */
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

/* 平滑滤波（滑动平均） */
static double SmoothingFilter(double NewData)
{
  int n = 0;
  int num = 0;
  static int i = 0;
  static double buf[WindowsLen] = {0};

  // 缓冲区未满，直接填充
  if(i < WindowsLen)
  {
    buf[i++] = NewData;
    return NewData;
  }
  else
  {
    // 左移窗口
    for(n = 0; n < WindowsLen - 1; n++)
    {
      buf[n] = buf[n + 1];
    }
    buf[WindowsLen - 1] = NewData;

    // 计算平均值
    for(n = 0; n < WindowsLen; n++)
    {
      num += buf[n];
    }
    return (num * 1.0 / WindowsLen);
  }
}

/* 更新峰值检测阈值 */
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

  s_peak2peak = peakMax - peakMin;  // 更新峰峰值
  *threshold_output = peakMax - (peakMax - peakMin) / 4;
}

/* 根据峰间距计算呼吸率 */
static void calRate(double ppdistance, int *rate_output)
{
  *rate_output = (int)(60000.0 / ppdistance); // ppdistance：ms
}

/*********************************************************************************************************
*                                           API函数
*********************************************************************************************************/

/* RESP 模块初始化 */
void InitRESP(void)
{
  memset(IIRNotch_win, 0, sizeof(IIRNotch_win));
  memset(IIRLowpass_win, 0, sizeof(IIRLowpass_win));

  memset(arr_BR_Wave, 0, sizeof(arr_BR_Wave));
  BR_Wave_index = 0;
  peakThreshold = 0;
  lastPeak_index = 0;
  currentPeak_index = 0;
  breathRate = 0;

  s_peak2peak = 0;
}

/* RESP 实时处理任务（周期调用） */
void RESPTask(u32 inp)
{
  double output1 = 0;
  double output2 = 0;
  double output3 = 0;

  // 信号处理链：陷波 → 低通 → 平滑
  output1 = IIRNotch(inp, IIRNotch_win);
  output2 = IIRLowpass(output1, IIRLowpass_win);
  output3 = SmoothingFilter(output2);

  // 保存波形数据
  arr_BR_Wave[BR_Wave_index++] = output3;

  // 波形缓存满，更新阈值
  if(BR_Wave_index >= BR_WAVE_LEN)
  {
    BR_Wave_index = 0;
    Update_Threshold(arr_BR_Wave, BR_WAVE_LEN, &peakThreshold);
  }

  // 峰值检测（上升沿过阈）
  if((BR_Wave_index > 1) && (BR_Wave_index < BR_WAVE_LEN - 1))
  {
    if((arr_BR_Wave[BR_Wave_index - 2] <= peakThreshold) &&
       (arr_BR_Wave[BR_Wave_index - 1] >= peakThreshold))
    {
      currentPeak_index = GetTimeCounter();
      calRate(currentPeak_index - lastPeak_index, &breathRate);
      lastPeak_index = currentPeak_index;
    }
  }

  // 调试输出
  if(g_displayMode == WAVE_RESP)
  {
    printf("%d ", (u16)output3);
  }
}

/* OLED 呼吸信息显示 */
void OLED_RESP(void)
{
  OLEDShowString(0, 32, (u8*)"BR:");
  OLEDShowString(64, 32, (u8*)"BPM");
  OLEDShowString(0, 48, (u8*)"RESP_LEAD:");

  // 导联异常
  if(s_peak2peak < 600 || s_peak2peak > 3000)
  {
    OLEDShowString(88, 48, (u8*)"Noob");
    OLEDShowString(32, 32, (u8*)"Err");
    printf("[[2,Err]]\r\n");
  }
  else
  {
    OLEDShowString(88, 48, (u8*)"Good");

    // 呼吸率合理
    if(breathRate >= 5 && breathRate <= 100)
    {
      OLEDShowNum(32, 32, breathRate, 3, 16);
      printf("[[2,%d]]\r\n", breathRate);
    }
    else
    {
      OLEDShowString(32, 32, (u8*)"Err");
      printf("[[2,Err]]\r\n");
    }
  }
}
