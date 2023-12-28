#include "UniTreeA1.h"
#include "motor_rs485.h"
#include "my_filter.h"

UniTreeA1 unitree_a1_motors;
int8_t A1MotorIDs[4] = {LEFT_LEFT_MOTOR_ID, LEFT_RIGHT_MOTOR_ID, RIGHT_LEFT_MOTOR_ID, RIGHT_RIGHT_MOTOR_ID};

inline void modify_data(MOTOR_send *motor_s)
{
    motor_s->motor_send_data.head.start[0] = 0xFE;
    motor_s->motor_send_data.head.start[1] = 0xEE;
    motor_s->motor_send_data.head.motorID = motor_s->id;
    motor_s->motor_send_data.head.reserved = 0x0;
    motor_s->motor_send_data.Mdata.mode = motor_s->mode;
    motor_s->motor_send_data.Mdata.ModifyBit = 0xFF;
    motor_s->motor_send_data.Mdata.ReadBit = 0x0;
    motor_s->motor_send_data.Mdata.reserved = 0x0;
    motor_s->motor_send_data.Mdata.Modify.L = 0;
    motor_s->motor_send_data.Mdata.T = motor_s->T * 256;
    motor_s->motor_send_data.Mdata.W = motor_s->W * 128;
    motor_s->motor_send_data.Mdata.Pos = (int)((motor_s->Pos / (2 * PI)) * 16384.0f);
    motor_s->motor_send_data.Mdata.K_P = motor_s->K_P * 2048.0f;

    motor_s->motor_send_data.Mdata.K_W = motor_s->K_W * 1024.0f;

    motor_s->motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
    motor_s->motor_send_data.Mdata.LowHzMotorCmdByte = 0;
    motor_s->motor_send_data.Mdata.Res[0] = motor_s->Res;
    motor_s->motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)(&(motor_s->motor_send_data)), 7);
}

void RS485Receive(uint8_t rs485_id)
{
    switch (rs485_id)
    {
    case LEFT_KNEE_RS485:
        switch (A1Motorbuffer[rs485_id - 1][2]) // ID
        {
        case LEFT_LEFT_MOTOR_ID:
            memcpy((uint8_t *)&unitree_a1_motors.a1_motor_recv[LEFT_KNEE_LEFT].motor_recv_data, A1Motorbuffer[rs485_id - 1], A1MOTOR_RECVBUF_SIZE);
            extract_data(&unitree_a1_motors.a1_motor_recv[LEFT_KNEE_LEFT]);

            global_debugger.a1_motor_debugger[0].recv_msgs_num++;

            break;
        case LEFT_RIGHT_MOTOR_ID:
            memcpy((uint8_t *)&unitree_a1_motors.a1_motor_recv[LEFT_KNEE_RIGHT].motor_recv_data, A1Motorbuffer[rs485_id - 1], A1MOTOR_RECVBUF_SIZE);
            extract_data(&unitree_a1_motors.a1_motor_recv[LEFT_KNEE_RIGHT]);

            global_debugger.a1_motor_debugger[1].recv_msgs_num++;
            break;
        default:
            break;
        }

        break;
    case RIGHT_KNEE_RS485:
        switch (A1Motorbuffer[rs485_id - 1][2]) // ID
        {
        case RIGHT_LEFT_MOTOR_ID:
            memcpy((uint8_t *)&unitree_a1_motors.a1_motor_recv[RIGHT_KNEE_LEFT].motor_recv_data, A1Motorbuffer[rs485_id - 1], A1MOTOR_RECVBUF_SIZE);
            extract_data(&unitree_a1_motors.a1_motor_recv[RIGHT_KNEE_LEFT]);

            global_debugger.a1_motor_debugger[2].recv_msgs_num++;
            break;
        case RIGHT_RIGHT_MOTOR_ID:
            memcpy((uint8_t *)&unitree_a1_motors.a1_motor_recv[RIGHT_KNEE_RIGHT].motor_recv_data, A1Motorbuffer[rs485_id - 1], A1MOTOR_RECVBUF_SIZE);
            extract_data(&unitree_a1_motors.a1_motor_recv[RIGHT_KNEE_RIGHT]);

            global_debugger.a1_motor_debugger[3].recv_msgs_num++;
            break;
        default:
            break;
        }

        break;
    default:
        break;
    }
}

inline uint8_t extract_data(MOTOR_recv *motor_r)
{
    if (motor_r->motor_recv_data.CRCdata.u32 !=
        crc32_core((uint32_t *)(&(motor_r->motor_recv_data)), 18))
    {
        motor_r->correct = FALSE;
        return motor_r->correct;
    }
    else
    {
        // 解码，详情见参考手册,不用得数据注释掉节省计算时间
        motor_r->motor_id = motor_r->motor_recv_data.head.motorID;
        motor_r->mode = motor_r->motor_recv_data.Mdata.mode;
        motor_r->Temp = motor_r->motor_recv_data.Mdata.Temp;
        motor_r->MError = motor_r->motor_recv_data.Mdata.MError;
        motor_r->T = ((float)motor_r->motor_recv_data.Mdata.T) / 256.0f;
        motor_r->W = ((float)motor_r->motor_recv_data.Mdata.W) / 128.0f;
        // motor_r->LW = motor_r->motor_recv_data.Mdata.LW;

        // motor_r->Acc = (int)motor_r->motor_recv_data.Mdata.Acc;
        motor_r->Pos = 2 * PI * ((float)motor_r->motor_recv_data.Mdata.Pos) / 16384.0f;

        // motor_r->gyro[0] = ((float)motor_r->motor_recv_data.Mdata.gyro[0]) * 0.00107993176f;
        // motor_r->gyro[1] = ((float)motor_r->motor_recv_data.Mdata.gyro[1]) * 0.00107993176f;
        // motor_r->gyro[2] = ((float)motor_r->motor_recv_data.Mdata.gyro[2]) * 0.00107993176f;

        // motor_r->acc[0] = ((float)motor_r->motor_recv_data.Mdata.acc[0]) * 0.0023911132f;
        // motor_r->acc[1] = ((float)motor_r->motor_recv_data.Mdata.acc[1]) * 0.0023911132f;
        // motor_r->acc[2] = ((float)motor_r->motor_recv_data.Mdata.acc[2]) * 0.0023911132f;

        motor_r->correct = TRUE;
        return motor_r->correct;
    }
}

void motorDataPack(MOTOR_send *motor_send, unsigned short id, float T, float W, float Pos, float K_P, float K_W)
{
    // 暂时只使用力矩控制
    //  set the id of motor
    motor_send->id = id;

    motor_send->mode = 10;
    motor_send->T = T;
    motor_send->W = W;
    motor_send->Pos = Pos;
    motor_send->K_P = K_P;
    motor_send->K_W = K_W;

    modify_data(motor_send);
}
void motorOffPack(MOTOR_send *motor_send, unsigned short id)
{
    motor_send->id = id;
    motor_send->mode = 0;
    modify_data(motor_send);
}

void A1MotorInit(UniTreeA1 *knee_motor)
{
    int8_t init_finish = FALSE;
    MOTOR_send knee_send[4];
    float knee_torque[4] = {-1.8, 1.8, -1.8, 1.8};
    float knee_torque_filte[4] = {0};
    float last_knee_pos[4] = {0};
    int is_motor_still[4] = {0}; // 是否所有电机都已经静止(位置传感器反馈与上一次相比没变)

    for (uint8_t i = 0; i < 4; i++)
    {
        motorOffPack(&knee_send[i], A1MotorIDs[i]);
    }

    while (!init_finish)
    {
        init_finish = TRUE;

        memcpy(&SendToA1Motor_Buff[LEFT_KNEE_RS485 - 1], &knee_send[LEFT_KNEE_LEFT].motor_send_data, sizeof(MasterComdDataV3));
        LEFT_KNEE_RS485_SEND();

        memcpy(&SendToA1Motor_Buff[RIGHT_KNEE_RS485 - 1], &knee_send[RIGHT_KNEE_LEFT].motor_send_data, sizeof(MasterComdDataV3));
        RIGHT_KNEE_RS485_SEND();

        delay_us(300);

        memcpy(&SendToA1Motor_Buff[LEFT_KNEE_RS485 - 1], &knee_send[LEFT_KNEE_RIGHT].motor_send_data, sizeof(MasterComdDataV3));
        LEFT_KNEE_RS485_SEND();

        memcpy(&SendToA1Motor_Buff[RIGHT_KNEE_RS485 - 1], &knee_send[RIGHT_KNEE_RIGHT].motor_send_data, sizeof(MasterComdDataV3));
        RIGHT_KNEE_RS485_SEND();

        delay_ms(1);

        for (int8_t i = 0; i < 4; i++)
        {
            if (global_debugger.a1_motor_debugger[i].recv_msgs_num < 40)
            {
                init_finish = FALSE;
            }
        }
    }

#if RESET_AND_CALIB_A1
    // 通过了初步初始化，进行零点校准
    init_finish = FALSE;
    while (!init_finish)
    {
        init_finish = TRUE;
        for (uint8_t i = 0; i < 4; i++)
        {
            rc_filter(&knee_torque_filte[i], knee_torque[i], 0.001, 0.02);
            motorDataPack(&knee_send[i], A1MotorIDs[i], knee_torque_filte[i] / REDUCTION_RATIO, 0, 0, 0, 0);
        }

        memcpy(&SendToA1Motor_Buff[LEFT_KNEE_RS485 - 1], &knee_send[LEFT_KNEE_LEFT].motor_send_data, sizeof(MasterComdDataV3));
        LEFT_KNEE_RS485_SEND();

        memcpy(&SendToA1Motor_Buff[RIGHT_KNEE_RS485 - 1], &knee_send[RIGHT_KNEE_LEFT].motor_send_data, sizeof(MasterComdDataV3));
        RIGHT_KNEE_RS485_SEND();

        delay_us(300);

        memcpy(&SendToA1Motor_Buff[LEFT_KNEE_RS485 - 1], &knee_send[LEFT_KNEE_RIGHT].motor_send_data, sizeof(MasterComdDataV3));
        LEFT_KNEE_RS485_SEND();

        memcpy(&SendToA1Motor_Buff[RIGHT_KNEE_RS485 - 1], &knee_send[RIGHT_KNEE_RIGHT].motor_send_data, sizeof(MasterComdDataV3));
        RIGHT_KNEE_RS485_SEND();

        delay_ms(1);

        // 检测位置传感器反馈是否有变化 在0-0.03之间说明传感器位置没动
        for (int i = 0; i < 4; i++)
        {
            if (fabsf(unitree_a1_motors.a1_motor_recv[i].Pos - last_knee_pos[i]) < 0.003f)
            {
                is_motor_still[i]++;
            }
            else
            {
                is_motor_still[i] = 0;
            }

            if (is_motor_still[i] < 700)
            {
                init_finish = FALSE;
            }

            // 更新上一次角度
            last_knee_pos[i] = unitree_a1_motors.a1_motor_recv[i].Pos;
        }
    }

    // 最后发0完成初始化角度
    for (int i = 0; i < 50; i++)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            motorOffPack(&knee_send[i], A1MotorIDs[i]);
        }

        memcpy(&SendToA1Motor_Buff[LEFT_KNEE_RS485 - 1], &knee_send[LEFT_KNEE_LEFT].motor_send_data, sizeof(MasterComdDataV3));
        LEFT_KNEE_RS485_SEND();

        memcpy(&SendToA1Motor_Buff[RIGHT_KNEE_RS485 - 1], &knee_send[RIGHT_KNEE_LEFT].motor_send_data, sizeof(MasterComdDataV3));
        RIGHT_KNEE_RS485_SEND();

        delay_us(300);

        memcpy(&SendToA1Motor_Buff[LEFT_KNEE_RS485 - 1], &knee_send[LEFT_KNEE_RIGHT].motor_send_data, sizeof(MasterComdDataV3));
        LEFT_KNEE_RS485_SEND();

        memcpy(&SendToA1Motor_Buff[RIGHT_KNEE_RS485 - 1], &knee_send[RIGHT_KNEE_RIGHT].motor_send_data, sizeof(MasterComdDataV3));
        RIGHT_KNEE_RS485_SEND();

        delay_ms(1);
    }
#endif

    // 正式初始化
    for (int8_t i = 0; i < 4; i++)
    {
        knee_motor->init_pos[i] = knee_motor->a1_motor_recv[i].Pos;
    }
}

void limitKneeTorque(float *knee)
{
    for (int8_t i = 0; i < 4; i++)
    {
        knee[i] = LIMIT_MAX_MIN(knee[i], MAX_KNEE_TORQUE, MIN_KNEE_TORQUE);
    }
}
