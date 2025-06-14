#ifndef USER_LIB_H
#define USER_LIB_H
#include "struct_typedef.h"

//typedef  struct
//{
//    fp32 input;        //输入数据
//    fp32 out;          //输出数据
//    fp32 min_value;    //限幅最小值
//    fp32 max_value;    //限幅最大值
//    fp32 frame_period; //时间间隔
//}__packed ramp_function_source_t;

typedef  struct
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num[1];       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
}__packed first_order_filter_type_t;

//快速开方
extern float invSqrt(fp32 num);
extern float my_sqrt(float number);
extern float my_pow(float number);
//斜波函数初始化
//void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//斜波函数计算
//void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period,  fp32 num);
//一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//绝对限制
//判断符号位
extern fp32 sign(fp32 value);
//浮点死区
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//角度 °限幅 180 ~ -180
extern fp32 theta_format(fp32 Ang);
//绝对值限幅
extern void fabs_limit(fp32 *x, fp32 limit);
//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#define VAL_LIMIT(val, min, max) \
  if ((val) <= (min)) {          \
    (val) = (min);               \
  } else if ((val) >= (max)) {   \
    (val) = (max);               \
  }

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


#define RADIAN_COEF  57.3f
#ifndef PI
#define PI  3.14159265358979f
#endif

#ifndef ABS
#define ABS(x) ( (x)>0?(x):-(x) )
#endif

#ifndef abs
#define abs(x) ( (x)>0?(x):-(x) )
#endif
#endif
