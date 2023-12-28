#include "PowerLimit.h"

PowerLimiter power_limiter;

/*  无预测功率的功率限制 */
// void PowerLimit(MF9025 *mf9025)
// {
//     float current_predict = (float)(abs(mf9025->frame.wheel_current[0]) + abs(mf9025->frame.wheel_current[1]));
//     float current_feedback = (float)(abs(mf9025->recv_msgs[0].current) + abs(mf9025->recv_msgs[1].current));
//     float total_current_limit;
//     float power_scale;

//     if (super_power.power_control_state == POWER_TO_BATTERY) // 接电池
//     {
//         // 裁判系统功率限制
//         if (Game_Robot_State.chassis_power_limit == 0 || Game_Robot_State.chassis_power_limit == 65535)
//         {
//             total_current_limit = CURRENT_NOT_LIMIT;
//         }
//         else
//         {
//             // 进行功率限制
//             // 缓冲能量较低
//             if (Power_Heat_Data.chassis_power_buffer < WARNING_BUFFER)
//             {
//                 if (Power_Heat_Data.chassis_power_buffer > 5.0f)
//                 {
//                     power_scale = Power_Heat_Data.chassis_power_buffer / WARNING_BUFFER;
//                 }
//                 else
//                 {
//                     power_scale = 5.0f / WARNING_BUFFER;
//                 }
//                 total_current_limit = BASE_CURRENT_LIMIT * power_scale;
//             }
//             else
//             {
//                 if (Power_Heat_Data.chassis_power > Game_Robot_State.chassis_power_limit * 0.75f)
//                 {
//                     if (Power_Heat_Data.chassis_power < Game_Robot_State.chassis_power_limit)
//                     {
//                         power_scale = (Game_Robot_State.chassis_power_limit - Power_Heat_Data.chassis_power) /
//                                       (Game_Robot_State.chassis_power_limit - Game_Robot_State.chassis_power_limit * 0.75f);
//                     }
//                     else
//                     {
//                         power_scale = 0.0f;
//                     }

//                     total_current_limit = BASE_CURRENT_LIMIT + ADD_CURRENT_LIMIT * power_scale;
//                 }
//                 else
//                 {
//                     total_current_limit = BASE_CURRENT_LIMIT + ADD_CURRENT_LIMIT;
//                 }
//             }
//         }
//         // 综合考虑电机发送电流值以及反馈电流值来进行功率预测
//         if (total_current_limit < current_feedback || total_current_limit < current_predict)
//         {
//             //超限制
//         }
//         else
//         {
//             // 计算可充电功率，并给予充电(考虑缓冲能量)
//         }
//     }
//     else // 接电容
//     {
//         // 超级电容功率限制(全功率充电)

//         // 综合考虑电机发送电流值以及反馈电流值来进行功率预测，限制功率输出不要太大
//     }
// }

/* 带功率预测的功率限制 */
void PowerLimit(MF9025 *mf9025, float *send_torque)
{
    float i_2, w_i, send_current_1, send_current_2, a, b;

    // 实时功率预测
    i_2 = mf9025->decoded_msgs[0].current * mf9025->decoded_msgs[0].current + mf9025->decoded_msgs[1].current * mf9025->decoded_msgs[1].current;
    w_i = mf9025->decoded_msgs[0].current * mf9025->decoded_msgs[0].rotate_speed + mf9025->decoded_msgs[1].current * mf9025->decoded_msgs[1].rotate_speed;

    power_limiter.actual_ina260_power = INA260_1.Power / 1000.0f;
    power_limiter.actual_referee_power = referee_data.Power_Heat_Data.chassis_power;
    power_limiter.predict_power = MF9025_K * w_i + MF9025_R * i_2 + MF9025_P0;

    // 发送期望电流之后产生的功率预测
    send_current_1 = send_torque[0] / MF9025_TORQUE_CURRENT_RATIO;
    send_current_2 = send_torque[1] / MF9025_TORQUE_CURRENT_RATIO;
    i_2 = send_current_1 * send_current_1 + send_current_2 * send_current_2;
    w_i = send_current_1 * mf9025->decoded_msgs[0].rotate_speed + send_current_2 * mf9025->decoded_msgs[1].rotate_speed;
    b = MF9025_K * w_i;
    a = MF9025_R * i_2;
    power_limiter.predict_send_power = a + b + MF9025_P0;

    //    float power_scale;

    // if (super_power.power_control_state == POWER_TO_BATTERY) // 接电池
    // {
    //     // 裁判系统功率限制
    //     if (referee_data.Game_Robot_State.chassis_power_limit == 0 || referee_data.Game_Robot_State.chassis_power_limit == 65535)
    //     {
    //         power_limiter.set_power = NO_LIMIT_POWER;
    //         power_limiter.referee_max_power = NO_LIMIT_POWER;
    //     }
    //     else
    //     {
    //         power_limiter.referee_max_power = referee_data.Game_Robot_State.chassis_power_limit;
    //         if (referee_data.Power_Heat_Data.chassis_power_buffer < WARNING_BUFFER && heat_controller.heat_count != 0)
    //         {
    //             power_limiter.set_power = power_limiter.referee_max_power + 1.5f * (referee_data.Power_Heat_Data.chassis_power_buffer - WARNING_BUFFER);
    //         }
    //         else
    //         {
    //             power_limiter.set_power = NO_LIMIT_POWER;
    //         }

    //         // if (referee_data.Power_Heat_Data.chassis_power_buffer > WARNING_BUFFER)
    //         // {
    //         //     power_limiter.set_power = referee_data.Game_Robot_State.chassis_power_limit + 1.0f * (referee_data.Power_Heat_Data.chassis_power_buffer - WARNING_BUFFER);
    //         // }
    //         // else
    //         // {
    //         //     power_limiter.set_power = referee_data.Game_Robot_State.chassis_power_limit * referee_data.Power_Heat_Data.chassis_power_buffer / WARNING_BUFFER;
    //         // }
    //     }

    //     // 功率处理
    //     if (power_limiter.predict_send_power > power_limiter.set_power)
    //     {
    //         // 模型衰减计算
    //         power_limiter.send_torque_lower_scale = (-b + Sqrt(b * b - 4 * (MF9025_P0 - power_limiter.set_power) * a)) / 2 / a;

    //         // 轮子力矩衰减
    //         send_torque[0] *= power_limiter.send_torque_lower_scale;
    //         send_torque[1] *= power_limiter.send_torque_lower_scale;
    //     }
    // }
    // else // 接电容
    // {
    //     // 超级电容功率限制(全功率充电)

    //     // 综合考虑电机发送电流值以及反馈电流值来进行功率预测，限制功率输出不要太大
    // }
}
