/**
 ******************************************************************************
 * @file    SuperPower.c
 * @brief   超级电容以及功率控制
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "SuperPower.h"

SuperPower super_power;
BufferEnergy buffer_energy;

/*  缓冲能量估算  */
void BufferEnergyCalc()
{
    if (referee_data_updater.is_max_power_data_update)
    {
        referee_data_updater.is_max_power_data_update = FALSE;
        buffer_energy.max_Power = referee_data.Game_Robot_State.chassis_power_limit;
    }
#if REGEREE_TEST
    float delt_P = (referee_data.Power_Heat_Data.chassis_power - TEST_MAX_POWER);
#else
    float delt_P = (INA260_1.Power / 1000.0f - buffer_energy.max_Power);
#endif
    buffer_energy.Kr = delt_P / buffer_energy.max_Power;

    if (referee_data_updater.is_power_data_update)
    {
        referee_data_updater.is_power_data_update = FALSE;
        buffer_energy.buffering_energy = referee_data.Power_Heat_Data.chassis_power_buffer;
    }
    else
    {
        buffer_energy.buffering_energy = buffer_energy.buffering_energy - delt_P * 0.001f; // 计算缓冲能量
    }

    if (buffer_energy.buffering_energy <= 0) // 计算超功率扣血
    {
        buffer_energy.buffering_energy = 0;
        if (buffer_energy.Kr <= 0.1f)
        {
            buffer_energy.lose_ratio = 10;
        }
        else if (buffer_energy.Kr <= 0.2f)
        {
            buffer_energy.lose_ratio = 20;
        }
        else
        {
            buffer_energy.lose_ratio = 40;
        }
        buffer_energy.HP -= TEST_HP * 0.001f * buffer_energy.lose_ratio / 100.0f;
    }
    else
    {
        if (buffer_energy.buffering_energy > MAX_BUFFERING_ENERGY)
        {
            buffer_energy.buffering_energy = MAX_BUFFERING_ENERGY;
        }
    }
}

void BufferEnergyInit()
{
    buffer_energy.HP = TEST_HP;
    buffer_energy.buffering_energy = MAX_BUFFERING_ENERGY;
    buffer_energy.max_Power = TEST_MAX_POWER;
}

/**
 * @brief 初始化超级电容以及裁判系统功率控制
 */
void SuperPowerInit()
{
    PID_Init(&super_power.superpower_charge_pid, MAX_CHARGE_CURRENT * 1000.0f, 1000.0f, 4000.0f, 0.05f, 0.09f, 0, 0, 0, 0.02, 0, 1, Integral_Limit | OutputFilter);

    BufferEnergyInit();

    super_power.current_reduce = 1.0f;
    super_power.power_limit_state = POWER_LIMIT_BAT;
}

void PowerControl(MF9025 *mf9025_motors, float *send_torque)
{
    INA_READ();

    // 根据INA260计算功率超限制，调试使用
    BufferEnergyCalc();

    // 裁判系统功率限制
    PowerLimit(mf9025_motors, send_torque);

    ADC_Filter();

    ChargeControl();
}

/**
 * @brief 超级电容电压采集以及ADC温度采集
 */
void ADC_Filter()
{
    //    float temp_temperature;

    super_power.SUM_UCAP = 0;
    super_power.SUM_UDCDC = 0;
    super_power.SUM_TEMP = 0;
    for (int i = 0; i < ADC_SAMPLE_NUM; i++)
    {
        super_power.SUM_UCAP += ADC_ConvertedValue[i * 3];
        // super_power.SUM_UDCDC += ADC_ConvertedValue[i * 3 + 1];
        // super_power.SUM_TEMP += ADC_ConvertedValue[i * 3 + 2];
    }

    // /1000.0f表示从mV转到V   // *825 >> 10 / 30 等效于 *3300 / 2^12 等效于
    super_power.UCAP = (super_power.SUM_UCAP * 0.0268554688f) / 1000.0f;
    // super_power.UDCDC = (super_power.SUM_UDCDC * 0.0268554688f) / 1000.0f;
    // temp_temperature = (super_power.SUM_TEMP * 0.0268554688f) / 1000.0f;
    // super_power.temperate = (temp_temperature - ADC_V25) / ADC_AVG_SLOPE + ADC_TEMP_BASE;
    super_power.actual_vol = super_power.UCAP * SUPERPOWER_SAMPLE_RATIO;
}

/* 根据剩余功率计算充电电流 */
void ChargeCal()
{
    if (super_power.power_control_state == POWER_TO_SuperPower)
    {
        super_power.charge_power = buffer_energy.max_Power * 0.6f;
    }
    else
    {
        super_power.charge_power = LIMIT_MAX_MIN(buffer_energy.max_Power - power_limiter.predict_send_power, buffer_energy.max_Power, 0);
    }

    // // 衰减功率充电
    // if (super_power.actual_vol > POWER_CHARGE_VOL_MIN_RANGE && super_power.actual_vol <= POWER_CHARGE_VOL_MAX_RANGE)
    // {
    //     super_power.charge_power = super_power.charge_power * (POWER_CHARGE_VOL_MAX_RANGE - super_power.actual_vol) / (POWER_CHARGE_VOL_MAX_RANGE - POWER_CHARGE_VOL_MIN_RANGE);
    // }
    // // 高电压,恒电压充电
    // else if (super_power.actual_vol >= POWER_CHARGE_VOL_MAX_RANGE)
    // {
    //     super_power.charge_power = 0;
    // }

    // super_power.actual_power = (INA260_1.Power - INA260_2.Power);
    // // 恒功率充电
    // super_power.i_set = PID_Calculate(&super_power.superpower_charge_pid, super_power.actual_power, super_power.charge_power) / 1000.0f;
    if (super_power.actual_vol < 21.0f)
    {
        super_power.current_reduce = 1.0f;
    }
    else if (super_power.actual_vol > 21.5f)
    {
        super_power.current_reduce = 0.0f;
    }

    // 以限幅作为恒电流充电依据
    if (super_power.charge_power < 15.0f)
    {
        super_power.i_set = 0;
    }
    else
    {
        float vol = LIMIT_MAX_MIN(super_power.actual_vol, 24.0f, 6.0f);
        super_power.i_set = LIMIT_MAX_MIN((super_power.charge_power - SUPER_POWER_BIAS) / vol / SUPER_POWER_RATIO, MAX_CHARGE_CURRENT, 0.0f) * super_power.current_reduce;
    }
}

void Bat_use()
{
    Bat_on;
    CAP_off;
    super_power.power_control_state = POWER_TO_BATTERY;
}
void CAP_use()
{
    CAP_on;
    Bat_off;
    super_power.power_control_state = POWER_TO_SuperPower;
}

/* 根据当前缓冲功率选择切换电容还是电池  */
void PowerStateSelect()
{
    // if (super_power.actual_vol > MIN_CAP_VOL_H && buffer_energy.buffering_energy < 5.0f && super_power.power_control_state == POWER_TO_BATTERY)
    // {
    //     CAP_use(); // 缓冲能量过低,自动切换电容
    // }
    // else if (buffer_energy.buffering_energy > 50.0f && super_power.power_control_state == POWER_TO_SuperPower)
    // {
    //     Bat_use();
    // }
    // else if (super_power.actual_vol < MIN_CAP_VOL_L && super_power.power_control_state == POWER_TO_SuperPower)
    // {
    //     // 强制切换为电池供电
    //     Bat_use();
    // }

    // 主动电容
    // if (remote_controller.super_power_state == POWER_TO_SuperPower)
    // {
    //     CAP_use();
    // }
    // else
    // {
    //     Bat_use();
    // }

    // 被动切换电容
    switch (super_power.power_limit_state)
    {
    case POWER_LIMIT_BAT:
        if (buffer_energy.buffering_energy < 5.0f)
        {
            if (super_power.actual_vol > MIN_CAP_VOL_H)
            {
                super_power.power_limit_state = POWER_LIMIT_CAP;
            }
            else
            {
                super_power.power_limit_state = POWER_LIMIT_BAT_ERROR;
            }
        }
        break;
    case POWER_LIMIT_CAP:
        if (buffer_energy.buffering_energy > 55.0f)
        {
            super_power.power_limit_state = POWER_LIMIT_BAT;
        }
        else if (super_power.actual_vol < MIN_CAP_VOL_L)
        {
            super_power.power_limit_state = POWER_LIMIT_BAT_ERROR;
        }
        break;
    case POWER_LIMIT_BAT_ERROR:
        break;
    default:
        super_power.power_limit_state = POWER_LIMIT_BAT;
        break;
    }

    if (super_power.power_limit_state == POWER_LIMIT_CAP)
    {
        CAP_use();
    }
    else
    {
        Bat_use();
    }
}

void Charge_Set()
{
    // 电容充放电控制
    if (buffer_energy.buffering_energy < 55.0f) // 缓冲能量过低时停止充电,等缓冲能量回
    {
        Charge_Off;
        super_power.power_charge_state = POWER_CHARGE_OFF;
    }
    else
    {
        Charge_On;
        super_power.power_charge_state = POWER_CHARGE_ON;
    }
    //	根据充电IC特性，根据电流感应电阻为10m欧姆，可得到如下计算公式，此为最大充电电流,计算公式可查看芯片手册
    //  超级电容电压冲到Vcc时，充电IC变回进入休眠模式，减少电容电压流失
    super_power.charge_dac_set = super_power.i_set * 0.2f / 3.3f * BIT_12_NUM;
    super_power.charge_dac_set = LIMIT_MAX_MIN(super_power.charge_dac_set, BIT_12_NUM, 0);
    DAC_SetChannel2Data(DAC_Align_12b_R, super_power.charge_dac_set);
}

/**
 * @brief 根据当前模式限制充电电流
 */
void limitISet()
{
    /* 若当前为移动状态，使充电电流为0 */
    super_power.i_set = LIMIT_MAX_MIN(super_power.i_set, MAX_CHARGE_CURRENT, 0.0f);
}

/**
 * @brief 电容充放电控制
 */
void ChargeControl(void)
{
    // 电容电池自动切换逻辑
    PowerStateSelect();

    /*  充电  */
    ChargeCal();
    limitISet();
    Charge_Set(); // 设定充电电流值
}
