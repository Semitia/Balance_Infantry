/**
 ******************************************************************************
 * @file    controller.c
 * @author  Wang Hongxi
 * @author  Zhang Hongyu (fuzzy pid)
 * @version V1.1.3
 * @date    2021/7/3
 * @brief   DWT定时器用于计算控制周期 OLS用于提取信号微分
 ******************************************************************************
 * @attention
 * copied from https://github.com/WangHongxi2001/RoboMaster-C-Board-INS-Example
 ******************************************************************************
 */
#include "pid.h"

/******************************** FUZZY PID **********************************/
static float FuzzyRuleKpRAW[7][7] = {
    PB, PB, PM, PM, PS, ZE, ZE,
    PB, PB, PM, PS, PS, ZE, PS,
    PM, PM, PM, PS, ZE, PS, PS,
    PM, PM, PS, ZE, PS, PM, PM,
    PS, PS, ZE, PS, PS, PM, PM,
    PS, ZE, PS, PM, PM, PM, PB,
    ZE, ZE, PM, PM, PM, PB, PB};

static float FuzzyRuleKiRAW[7][7] = {
    PB, PB, PM, PM, PS, ZE, ZE,
    PB, PB, PM, PS, PS, ZE, ZE,
    PB, PM, PM, PS, ZE, PS, PS,
    PM, PM, PS, ZE, PS, PM, PM,
    PS, PS, ZE, PS, PS, PM, PB,
    ZE, ZE, PS, PS, PM, PB, PB,
    ZE, ZE, PS, PM, PM, PB, PB};

static float FuzzyRuleKdRAW[7][7] = {
    PS, PS, PB, PB, PB, PM, PS,
    PS, PS, PB, PM, PM, PS, ZE,
    ZE, PS, PM, PM, PS, PS, ZE,
    ZE, PS, PS, PS, PS, PS, ZE,
    ZE, ZE, ZE, ZE, ZE, ZE, ZE,
    PB, PS, PS, PS, PS, PS, PB,
    PB, PM, PM, PM, PS, PS, PB};

void Fuzzy_Rule_Init(FuzzyRule_t *fuzzyRule, float (*fuzzyRuleKp)[7], float (*fuzzyRuleKi)[7], float (*fuzzyRuleKd)[7],
                     float kpRatio, float kiRatio, float kdRatio,
                     float eStep, float ecStep)
{
    //初始化模糊PID
    if (fuzzyRuleKp == NULL)
        fuzzyRule->FuzzyRuleKp = FuzzyRuleKpRAW;
    else
        fuzzyRule->FuzzyRuleKp = fuzzyRuleKp;
    if (fuzzyRuleKi == NULL)
        fuzzyRule->FuzzyRuleKi = FuzzyRuleKiRAW;
    else
        fuzzyRule->FuzzyRuleKi = fuzzyRuleKi;
    if (fuzzyRuleKd == NULL)
        fuzzyRule->FuzzyRuleKd = FuzzyRuleKdRAW;
    else
        fuzzyRule->FuzzyRuleKd = fuzzyRuleKd;

    fuzzyRule->KpRatio = kpRatio;
    fuzzyRule->KiRatio = kiRatio;
    fuzzyRule->KdRatio = kdRatio;

    if (eStep < 0.00001f)
        eStep = 1;
    if (ecStep < 0.00001f)
        ecStep = 1;
    fuzzyRule->eStep = eStep;
    fuzzyRule->ecStep = ecStep;
}
void Fuzzy_Rule_Implementation(FuzzyRule_t *fuzzyRule, float measure, float ref)
{
    float eLeftTemp, ecLeftTemp;
    float eRightTemp, ecRightTemp;
    int eLeftIndex, ecLeftIndex;
    int eRightIndex, ecRightIndex;

    fuzzyRule->dt = GetDeltaT((void *)fuzzyRule->DWT_CNT);

    fuzzyRule->e = ref - measure;
    fuzzyRule->ec = (fuzzyRule->e - fuzzyRule->eLast) / fuzzyRule->dt;
    fuzzyRule->eLast = fuzzyRule->e;

    //隶属区间
    eLeftIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 2)));
    eRightIndex = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 6 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e >= 0 ? ((int)(fuzzyRule->e / fuzzyRule->eStep) + 4) : ((int)(fuzzyRule->e / fuzzyRule->eStep) + 3)));
    ecLeftIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 2)));
    ecRightIndex = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 6 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec >= 0 ? ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 4) : ((int)(fuzzyRule->ec / fuzzyRule->ecStep) + 3)));

    //隶属度
    eLeftTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 1 : (eRightIndex - fuzzyRule->e / fuzzyRule->eStep - 3));
    eRightTemp = fuzzyRule->e >= 3 * fuzzyRule->eStep ? 1 : (fuzzyRule->e <= -3 * fuzzyRule->eStep ? 0 : (fuzzyRule->e / fuzzyRule->eStep - eLeftIndex + 3));
    ecLeftTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 1 : (ecRightIndex - fuzzyRule->ec / fuzzyRule->ecStep - 3));
    ecRightTemp = fuzzyRule->ec >= 3 * fuzzyRule->ecStep ? 1 : (fuzzyRule->ec <= -3 * fuzzyRule->ecStep ? 0 : (fuzzyRule->ec / fuzzyRule->ecStep - ecLeftIndex + 3));

    fuzzyRule->KpFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecLeftIndex] +
                         eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecLeftIndex] +
                         eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKp[eLeftIndex][ecRightIndex] +
                         eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKp[eRightIndex][ecRightIndex];

    fuzzyRule->KiFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecLeftIndex] +
                         eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecLeftIndex] +
                         eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKi[eLeftIndex][ecRightIndex] +
                         eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKi[eRightIndex][ecRightIndex];

    fuzzyRule->KdFuzzy = eLeftTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecLeftIndex] +
                         eLeftTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecLeftIndex] +
                         eRightTemp * ecLeftTemp * fuzzyRule->FuzzyRuleKd[eLeftIndex][ecRightIndex] +
                         eRightTemp * ecRightTemp * fuzzyRule->FuzzyRuleKd[eRightIndex][ecRightIndex];
}

/******************************* PID CONTROL *********************************/
// PID优化环节函数声明
static void f_Trapezoid_Intergral(PID_t *pid);
static void f_Integral_Limit(PID_t *pid);
static void f_Derivative_On_Measurement(PID_t *pid);
static void f_Changing_Integration_Rate(PID_t *pid);
static void f_Output_Filter(PID_t *pid);
static void f_Derivative_Filter(PID_t *pid);
static void f_Output_Limit(PID_t *pid);
static void f_Proportion_Limit(PID_t *pid);
static void f_PID_ErrorHandle(PID_t *pid);

/**
 * @brief          PID初始化   PID initialize
 * @param[in]      PID结构体   PID structure
 * @param[in]      略
 * @retval         返回空      null
 */
void PID_Init(
    PID_t *pid,
    float max_out,
    float intergral_limit,
    float deadband,

    float kp,
    float Ki,
    float Kd,

    float A,
    float B,

    float output_lpf_rc,
    float derivative_lpf_rc,

    uint16_t ols_order,

    uint8_t improve)
{
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->Ref = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->ITerm = 0;

    // 变速积分参数
    // coefficient of changing integration rate
    pid->CoefA = A;
    pid->CoefB = B;

    pid->Output_LPF_RC = output_lpf_rc;

    pid->Derivative_LPF_RC = derivative_lpf_rc;

    // 最小二乘提取信号微分初始化
    // differential signal is distilled by OLS
    pid->OLS_Order = ols_order;
    OLS_Init(&pid->OLS, ols_order);

    // DWT定时器计数变量清零
    // reset DWT Timer count counter
    GetDeltaT(&pid->DWT_CNT);

    // 设置PID优化环节
    pid->Improve = improve;

    // 设置PID异常处理 目前仅包含电机堵转保护
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float PID_Calculate(PID_t *pid, float measure, float ref)
{
    if (pid->Improve & ErrorHandle)
        f_PID_ErrorHandle(pid);

    pid->dt = GetDeltaT((void *)&pid->DWT_CNT);

    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    if (pid->User_Func1_f != NULL)
        pid->User_Func1_f(pid);

    if (abs(pid->Err) > pid->DeadBand)
    {
        if (pid->FuzzyRule == NULL)
        {
            pid->Pout = pid->Kp * pid->Err;
            pid->ITerm = pid->Ki * pid->Err * pid->dt;
            if (pid->OLS_Order > 2)
                pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, pid->Err);
            else
                pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;
        }
        else
        {
            pid->Pout = (pid->Kp + pid->FuzzyRule->KpFuzzy) * pid->Err;
            pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * pid->Err * pid->dt;
            if (pid->OLS_Order > 2)
                pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * OLS_Derivative(&pid->OLS, pid->dt, pid->Err);
            else
                pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Err - pid->Last_Err) / pid->dt;
        }

        if (pid->User_Func2_f != NULL)
            pid->User_Func2_f(pid);

        // 梯形积分
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;

        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        // 输出滤波
        if (pid->Improve & OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);

        // 无关紧要
        f_Proportion_Limit(pid);
    }

    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

/**
 * @brief          清除PID缓存量
 * @param[in]      PID结构体
 * @retval         返回空
 */
void PID_Clear(PID_t *pid)
{
    pid->Pout = pid->Output = pid->Measure = pid->Dout = pid->Err = pid->Iout = pid->ITerm = 0;
    pid->Last_Dout = pid->Last_Err = pid->Last_ITerm = pid->Last_Measure = pid->Last_Output = 0;
    pid->Ref = 0;
    GetDeltaT(&pid->DWT_CNT);
}

/**
 * @brief          带微分的需要在清空的同时，保留last_measure来使微分不突变
 * @param[in]      PID结构体
 * @param[in]      本次测量值
 * @retval         返回空
 */
void PID_Clear_withMeasure(PID_t *pid, float measure, float ref)
{
    pid->Last_Measure = measure;
    pid->Last_Err = ref - measure;
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Pout = pid->Output = pid->Dout = pid->Err = pid->Iout = pid->ITerm = 0;
    pid->Last_Dout = pid->Last_ITerm = pid->Last_Output = 0;
    GetDeltaT(&pid->DWT_CNT);
}

static void f_Trapezoid_Intergral(PID_t *pid)
{
    if (pid->FuzzyRule == NULL)
        pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
    else
        pid->ITerm = (pid->Ki + pid->FuzzyRule->KiFuzzy) * ((pid->Err + pid->Last_Err) / 2) * pid->dt;
}

static void f_Changing_Integration_Rate(PID_t *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        // Integral still increasing
        if (abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(PID_t *pid)
{
    static float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            // 积分呈累积趋势
            // Integral still increasing
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_t *pid)
{
    if (pid->FuzzyRule == NULL)
    {
        if (pid->OLS_Order > 2)
            pid->Dout = pid->Kd * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
        else
            pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
    else
    {
        if (pid->OLS_Order > 2)
            pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * OLS_Derivative(&pid->OLS, pid->dt, -pid->Measure);
        else
            pid->Dout = (pid->Kd + pid->FuzzyRule->KdFuzzy) * (pid->Last_Measure - pid->Measure) / pid->dt;
    }
}

static void f_Derivative_Filter(PID_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

static void f_Output_Filter(PID_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

static void f_Output_Limit(PID_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

static void f_Proportion_Limit(PID_t *pid)
{
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

// PID ERRORHandle Function
static void f_PID_ErrorHandle(PID_t *pid)
{
    /*Motor Blocked Handle*/
    if (pid->Output < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        // Motor blocked over 1000times
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

/*************************** FEEDFORWARD CONTROL *****************************/
/**
 * @brief          前馈控制初始化
 * @param[in]      前馈控制结构体
 * @param[in]      略
 * @retval         返回空
 */
void Feedforward_Init(
    Feedforward_t *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order)
{
    ffc->MaxOut = max_out;

    // 设置前馈控制器参数 详见前馈控制结构体定义
    // set parameters of feed-forward controller (see struct definition)
    if (c != NULL && ffc != NULL)
    {
        ffc->c[0] = c[0];
        ffc->c[1] = c[1];
        ffc->c[2] = c[2];
    }
    else
    {
        ffc->c[0] = 0;
        ffc->c[1] = 0;
        ffc->c[2] = 0;
        ffc->MaxOut = 0;
    }

    ffc->LPF_RC = lpf_rc;

    // 最小二乘提取信号微分初始化
    // differential signal is distilled by OLS
    ffc->Ref_dot_OLS_Order = ref_dot_ols_order;
    ffc->Ref_ddot_OLS_Order = ref_ddot_ols_order;
    if (ref_dot_ols_order > 2)
        OLS_Init(&ffc->Ref_dot_OLS, ref_dot_ols_order);
    if (ref_ddot_ols_order > 2)
        OLS_Init(&ffc->Ref_ddot_OLS, ref_ddot_ols_order);

    ffc->DWT_CNT = 0;

    ffc->Output = 0;
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float Feedforward_Calculate(Feedforward_t *ffc, float ref)
{
    ffc->dt = GetDeltaT((void *)&ffc->DWT_CNT);

    ffc->Ref = ref * ffc->dt / (ffc->LPF_RC + ffc->dt) +
               ffc->Ref * ffc->LPF_RC / (ffc->LPF_RC + ffc->dt);

    // 计算一阶导数
    // calculate first derivative
    if (ffc->Ref_dot_OLS_Order > 2)
        ffc->Ref_dot = OLS_Derivative(&ffc->Ref_dot_OLS, ffc->dt, ffc->Ref);
    else
        ffc->Ref_dot = (ffc->Ref - ffc->Last_Ref) / ffc->dt;

    // 计算二阶导数
    // calculate second derivative
    if (ffc->Ref_ddot_OLS_Order > 2)
        ffc->Ref_ddot = OLS_Derivative(&ffc->Ref_ddot_OLS, ffc->dt, ffc->Ref_dot);
    else
        ffc->Ref_ddot = (ffc->Ref_dot - ffc->Last_Ref_dot) / ffc->dt;

    // 计算前馈控制输出
    // calculate feed-forward controller output
    ffc->Output = ffc->c[0] * ffc->Ref + ffc->c[1] * ffc->Ref_dot + ffc->c[2] * ffc->Ref_ddot;

    LIMIT_MAX_MIN(ffc->Output, ffc->MaxOut, -ffc->MaxOut);

    ffc->Last_Ref = ffc->Ref;
    ffc->Last_Ref_dot = ffc->Ref_dot;

    return ffc->Output;
}

void Feedforward_Clear(Feedforward_t *ffc)
{
    ffc->Ref = ffc->Ref_dot = ffc->Ref_ddot = 0;
    ffc->Output = ffc->Last_Ref = ffc->Last_Ref_dot = 0;
    ffc->DWT_CNT = 0;
}
