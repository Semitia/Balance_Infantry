#ifndef __CECILIA_H__
#define __CECILIA_H__

#include "LQR.h"
#include "wb_peripherals.h"

#define WHEEL_RADIUS 0.03

#define LEFT  0
#define RIGHT 1
//各个电机名字
typedef enum __MotorName_e {
    KNEE_RBM = 0,
    KNEE_RFM,
    KNEE_LBM,       
    KNEE_LFM,       
    WHEEL_R,        //右轮
    WHEEL_L,
    MOTOR_NUM
} MotorName_e;
//robot mode
typedef enum __Mode_e {
    JUMP_PRE = 0,       //预备跳跃
    JUNP_UP,            //跳跃
    JUMP_DOWN,          //落地
    BREAKDOWN,          //倒地
    SQART,              //蹲下
    NORMAL,             //正常
    JUMP_FLOPE,         //跳跃后倒地
    MODE_NUM
} Mode_e;

/**
 * @brief Dormily_t
 * @note 机器人结构体，目前只实现运动控制功能
*/
typedef struct __Cecilia_t {
    /* 尺寸参数 */
    double wheel_radius;                    // 机器人轮子半径 m
    double center_height;                   // 机器人中心高度 m
    double body_mass;                       // 机器人质量 kg
    double wheel_mass;                      // 机器人轮子质量 kg

    double DofJ_12;             
    double DofJ_23;
    double DofJ_34;
    double DofJ_25;           // 1~5,5个轴之间的距离

    /* 运动参数 */
    double velocity;                        // m/s
    double w;                               // rad/s
    double w_tgt;                           // rad/s
    double displacement;                    //机器人质心的位移
    double displacement_last;               //上一个时间戳, 机器人质心的位移
    double velocity_target;                 //机器人的目标速度
    bool camera_state;
    int mode;

    /* 外设对象 */
    PosSensor_t PosSensor[MOTOR_NUM];       // 机器人编码器
    Motor_t Motor[MOTOR_NUM];               // 机器人电机
    Display_t Display;                      // 显示器
    Camera_t Camera;                        // 相机
    Gyro_t Gyro;                            // 陀螺仪
    Acce_t Acce;                            // 加速度计
    IMU_t IMU;                              // IMU


    LQR_t LQR;                              // LQR控制器
    float (*k_f)[4][10];                    // LQR增益矩阵拟合系数

    /* 五连杆 */
    float l_len[6];                         // 左腿长度：0-虚拟连杆长度，1-
    float r_len[6];                         // 右腿长度
    float l_phi[6];                         // 左腿角度
    float r_phi[6];                         // 右腿角度
    float J_l[2][2];                        // 左侧VMC雅可比矩阵
    float J_r[2][2];                        // 右侧VMC雅可比矩阵
    float J_inv_l[2][2];                    // 左腿雅可比矩阵逆矩阵
    float J_inv_r[2][2];                    // 右腿雅可比矩阵逆矩阵
    float force_l;                          // 左侧支持力
    float force_r;                          // 右侧支持力
    float L0;                               // 平均腿长
    float L_dot[2];                         // 左右腿速度
    float L_ddot[2];                        // 左右腿加速度
    float last_L[2];                        // 上一时刻腿长: [0]-左腿 [1]-右腿
    float last_dotL[2];                     // 上一时刻腿速度
    float last_d_dotL[2];                   // 上一时刻腿加速度
    float theta[2];                         // 虚拟腿倾角
    float theta_w[2];                       // 虚拟腿倾角角速度
    float last_theta[2];                    // 上一时刻虚拟腿倾角
    float last_dot_theta[2];                // 上一时刻虚拟腿倾角角速度
    float last_d_dot_theta[2];              // 上一时刻虚拟腿倾角角加速度

} Cecilia_t;

/**
 * @brief 初始化机器人
 *        所有参数直接在这里修改
*/
void ceciliaInit(Cecilia_t *Ce) {
    int i;
    Ce->center_height = 0;
    Ce->body_mass = 30;
    Ce->wheel_mass = 0.5;
    Ce->wheel_radius = 0.065;
    Ce->camera_state = false;

    const char *MOTOR_NAMES[] = {"RBM", "RFM", "LBM", "LFM", "wheel1", "wheel2"};
    const char *POS_SENSOR_NAMES[] = {"RBS", "RFS", "LBS", "LFS", "Swheel1", "Swheel2"};
    for(i=0; i < MOTOR_NUM; i++) 
        motorInit(&Ce->Motor[i], MOTOR_NAMES[i], 0); 
    for(i=0; i < MOTOR_NUM; i++) 
        posSensorInit(&Ce->PosSensor[i], POS_SENSOR_NAMES[i]);
    
    imuInit(&Ce->IMU, "imu");
    acceInit(&Ce->Acce, "Acce");
    gyroInit(&Ce->Gyro, "gyro");
    cameraInit(&Ce->Camera, "camera");
    displayInit(&Ce->Display);

    // LQR
    double  A[10][INV] = {0},
            B[10][INV] = {0};
    Ce->LQR.x = matrix_init2(10, 1);//s-位移, s_dot-速度, yaw, yaw_dot, theta_ll-左腿倾角, theta_ll_dot, theta_lr-右腿倾角, theta_lr_dot, theta_b-机体倾角, theta_b_dot
    Ce->LQR.xd = matrix_init2(10, 1);
    Ce->LQR.u = matrix_init2(4, 1); //torque_lw_l-左驱动轮转矩, torque_lw_r, torque_bl_l-左关节转矩, torque_bl_r
    Ce->LQR.A = matrix_init(10, 10, A);
    Ce->LQR.B = matrix_init(10, 4, B);
    Ce->LQR.K = matrix_init2(4, 10);

    static float k_fix[6][4][10] = { {
            {-0.672626,	-2.364401,	-7.401996,	-7.423282,	-8.568568,	-4.880340,	-4.323176,	-1.250817,	-62.175610,	-2.223826},
            {-0.672626,	-2.364401,	7.401996,	7.423282,	-4.323176,	-1.250817,	-8.568568,	-4.880340,	-62.175610,	-2.223826},
            {1.542560,	5.120150,	-5.159871,	-5.189996,	23.684626,	19.171859,	6.609028,	-1.880429,	-88.095716,	-2.868259},
            {1.542560,	5.120150,	5.159871,	5.189996,	6.609028,	-1.880429,	23.684626,	19.171859,	-88.095716,	-2.868259}
        },{
            {0.985643,	4.617739,	-8.249536,	-8.383061,	-40.882341,	2.692072,	26.470177,	2.099027,	84.863670,	3.295617},
            {-2.282033,	-4.937314,	-10.502327,	-10.618469,	-89.498412,	-30.011795,	14.731006,	6.745494,	58.052296,	2.554051},
            {4.452203,	9.253418,	36.675286,	36.931849,	101.980891,	-26.635324,	-22.850035,	-4.086723,	-374.767887,	-10.457984},
            {-7.732475,	-24.131960,	-16.329802,	-16.329316,	-71.991542,	-10.610319,	-34.742189,	4.662664,	143.759072,	5.265869}
        },{
            {-2.282033,	-4.937314,	10.502327,	10.618469,	14.731006,	6.745494,	-89.498412,	-30.011795,	58.052296,	2.554051},
            {0.985643,	4.617739,	8.249536,	8.383061,	26.470177,	2.099027,	-40.882341,	2.692072,	84.863670,	3.295617},
            {-7.732475,	-24.131960,	16.329802,	16.329316,	-34.742189,	4.662664,	-71.991542,	-10.610319,	143.759072,	5.265869},
            {4.452203,	9.253418,	-36.675286,	-36.931849,	-22.850035,	-4.086723,	101.980891,	-26.635324,	-374.767887,	-10.457984}
        },{
            {-0.905311,	-4.063666,	10.158483,	10.303044,	36.256235,	-6.738766,	-31.964619,	-3.975404,	-44.750792,	-1.924747},
            {2.497094,	6.865854,	12.635798,	12.699161,	79.271171,	36.939708,	-14.796886,	-10.741639,	19.324378,	-0.355709},
            {-10.650664,	-27.658702,	-59.730938,	-60.012064,	-157.242215,	17.251548,	23.789170,	12.327299,	369.581000,	9.120055},
            {4.712356,	16.948533,	20.509740,	20.406733,	6.984650,	-2.010435,	21.700162,	-9.538834,	-73.476719,	-3.592381}
        },{
            {-0.036231,	-2.911963,	2.675974,	2.683230,	25.846367,	2.547987,	8.327831,	1.582449,	-95.358566,	-3.790698},
            {-0.036231,	-2.911963,	-2.675974,	-2.683230,	8.327831,	1.582449,	25.846367,	2.547987,	-95.358566,	-3.790698},
            {8.386044,	26.930172,	10.162149,	10.491238,	151.324714,	9.223216,	-60.408371,	1.751704,	-7.281461,	1.008662},
            {8.386044,	26.930172,	-10.162149,	-10.491238,	-60.408371,	1.751704,	151.324714,	9.223216,	-7.281461,	1.008662}
        },{
            {2.497094,	6.865854,	-12.635798,	-12.699161,	-14.796886,	-10.741639,	79.271171,	36.939708,	19.324378,	-0.355709},
            {-0.905311,	-4.063666,	-10.158483,	-10.303044,	-31.964619,	-3.975404,	36.256235,	-6.738766,	-44.750792,	-1.924747},
            {4.712356,	16.948533,	-20.509740,	-20.406733,	21.700162,	-9.538834,	6.984650,	-2.010435,	-73.476719,	-3.592381},
            {-10.650664,	-27.658702,	59.730938,	60.012064,	23.789170,	12.327299,	-157.242215,	17.251548,	369.581000,	9.120055}
        }
    };
    Ce->k_f = k_fix;
}

/**
 * @brief 五连杆解算
 * @param L   五连杆长度
 * @param phi 五连杆角度
 * @param theta 虚拟腿倾角
 * @param theta_w 倾角角速度
 * @param theta_b 机体倾角
*/
void motionSolve(float *L,float *phi,float *theta,float *theta_w,float theta_b)
{
    float x_B = L[1] * cos(phi[1]);
    float y_B = L[1] * sin(phi[1]);
    float x_D = L[4] * cos(phi[4]) + L[5];
    float y_D = L[4] * sin(phi[4]);
    float B_D_distance_2 = pow( (x_D - x_B) ,2) + pow( (y_D - y_B) ,2);   //B、D两点之间的距离
    float A_0 = 2 * L[2] *(x_D - x_B);
    float B_0 = 2 * L[2] *(y_D - y_B);
    float C_0 = pow(L[2], 2) + B_D_distance_2 - pow(L[3], 2);
    //计算phi_2
    float temp = (B_0 + sqrt(pow(A_0, 2) + pow(B_0, 2) - pow(C_0, 2)));
    phi[2] = 2 * atan2(temp, A_0 + C_0);
    //获得当前C点坐标
    float x_C = x_B + L[2] * cos(phi[2]);
    float y_C = y_B + L[2] * sin(phi[2]);
    phi[3] = atan2(y_C - y_D, x_C - x_D);
    float top_half_x = L[5] / 2;
    x_C -= top_half_x;
    //A点到C点的直线距离
    L[0] = sqrt(x_C * x_C + y_C * y_C);
    phi[0] = atan2(y_C, x_C);
    //根据几何关系得到theta角的大小
    float temp_ = -(M_PI / 2 - phi[0] - theta_b);
    //转速经过低通滤波
    *theta_w = *theta_w * 0.95 + (temp_ - (*theta)) * 1000 / TIME_STEP * 0.05;
    //更新当前时刻的theta角
    *theta = temp_;
}

/**
 * @brief update state
*/
void updateState(Cecilia_t *Ce) {
    int i;
    for(i=0; i < MOTOR_NUM; i++) 
        updateMotor(&Ce->PosSensor[i], &Ce->Motor[i]);
    updateIMU(&Ce->IMU);
    updateGyro(&Ce->Gyro);
    updateAcce(&Ce->Acce);
    /* 状态更新 */
    Ce->displacement_last = Ce->displacement;
    Ce->displacement = (Ce->PosSensor[WHEEL_L].position + Ce->PosSensor[WHEEL_R].position) * Ce->wheel_radius / 2;
    Ce->velocity = (Ce->displacement - Ce->displacement_last) / T;
    Ce->w = Ce->w * 0.95 + Ce->Gyro.gyro_value[yaw] * 0.05;
    Ce->l_phi[1] = PI - Ce->PosSensor[KNEE_LBM].position;
    Ce->l_phi[4] = -Ce->PosSensor[KNEE_LFM].position;
    Ce->r_phi[1] = PI - Ce->PosSensor[KNEE_RBM].position;
    Ce->r_phi[4] = -Ce->PosSensor[KNEE_RFM].position;

    Ce->L0 = (Ce->l_len[0] + Ce->r_len[0]) / 2;
    Ce->last_L[0] = Ce->l_len[0];
    Ce->last_dotL[0] = Ce->L_dot[0];
    Ce->last_L[1] = Ce->r_len[0];
    Ce->last_dotL[1] = Ce->L_dot[1];
    Ce->last_theta[0] = Ce->l_phi[0];
    Ce->last_dot_theta[0] = Ce->LQR.x.matrix[5][0];
    Ce->last_theta[1] = Ce->r_phi[0];
    Ce->last_dot_theta[1] = Ce->LQR.x.matrix[7][0];
    motionSolve(Ce->l_len, Ce->l_phi, &Ce->theta[0], &Ce->theta_w[0], Ce->IMU.angle_value[pitch]);
    motionSolve(Ce->r_len, Ce->r_phi, &Ce->theta[1], &Ce->theta_w[1], Ce->IMU.angle_value[pitch]);
    /* 更新状态向量 */
    Ce->LQR.x.matrix[0][0] = Ce->displacement;
    Ce->LQR.x.matrix[1][0] = Ce->velocity;
    Ce->LQR.x.matrix[2][0] = Ce->IMU.angle_value[yaw];
    Ce->LQR.x.matrix[3][0] = Ce->w;
    Ce->LQR.x.matrix[4][0] = Ce->theta[0];
    Ce->LQR.x.matrix[5][0] = Ce->theta_w[0];
    Ce->LQR.x.matrix[6][0] = Ce->theta[1];
    Ce->LQR.x.matrix[7][0] = Ce->theta_w[1];
    Ce->LQR.x.matrix[8][0] = Ce->IMU.angle_value[pitch];
    Ce->LQR.x.matrix[9][0] = Ce->Gyro.gyro_value[pitch];
    return;
}

/**
 * @brief 根据当前腿长多项式拟合LQR的增益矩阵
*/
void updateLqrK(Cecilia_t *Ce) {
    int i, j, k;
    //1, l, r, l^2, r^2, l*r
    float x[6] = {1, Ce->l_len[0], Ce->r_len[0], Ce->l_len[0]*Ce->l_len[0], Ce->r_len[0]*Ce->r_len[0], Ce->l_len[0]*Ce->r_len[0]};
    for(i=0; i<4; i++) {
        for(j=0; j<10; j++) {
            Ce->LQR.K.matrix[i][j] = 0;
            for(k=0; k<6; k++) {
                Ce->LQR.K.matrix[i][j] += Ce->k_f[k][i][j] * x[k];
            }
        }
    }
    return;
}

/**
 * @brief 五连杆VMC解算
 * @param l 五连杆长度
 * @param phi 五连杆角度
 * @param virtual_torque 虚拟力矩 : [0]-驱动轮输出转矩 [1]-关节转矩
 * @param output_torque  输出扭矩 : [0]-驱动轮输出力矩 [1]-左侧关节电机输出转矩 [2]-右侧关节电机输出转矩
 * @param force 支持力
 * 
*/
void solveVMC(float *l, float *phi, float *virtual_torque, float *output_torque, float force, float J[][2])
{
    float sigma_1 = sin(phi[2] - phi[3]);
    float sigma_2 = sin(phi[3] - phi[4]);
    float sigma_3 = sin(phi[1] - phi[2]);

    J[0][0] = -l[1] * cos(phi[0] - phi[3]) * sigma_3 / l[0] / sigma_1;
    J[0][1] = -l[1] * sin(phi[0] - phi[3]) * sigma_3 / sigma_1;
    J[1][0] = -l[4] * cos(phi[0] - phi[2]) * sigma_2 / l[0] / sigma_1;
    J[1][1] = -l[4] * sin(phi[0] - phi[2]) * sigma_2 / sigma_1;
    //轮毂电机输出扭矩
    output_torque[0] = -virtual_torque[0];
    //腿扭矩？还是支持力
    virtual_torque[1] = -virtual_torque[1];
    // force 为额外力，由滚转角和期望腿长计算而得，以上为正方向
    virtual_torque[2] = -(force) * cos(phi[0] - M_PI / 2); //单个杆承受的力
    for (int i = 1; i < 3; i++)
    {
        output_torque[i] = 0;
    }
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
        output_torque[i + 1] += J[i][j] * virtual_torque[j + 1];
        }
    }

    /* DEBUG */
    printf("sigma_1: %f, sigma_2: %f, sigma_3: %f\r\n", sigma_1, sigma_2, sigma_3);
    printf("J: %f, %f, %f, %f\r\n", J[0][0], J[0][1], J[1][0], J[1][1]);
    printf("virtual_torque: %f, %f, %f\r\n", virtual_torque[0], virtual_torque[1], virtual_torque[2]);
    printf("output_torque: %f, %f, %f\r\n", output_torque[0], output_torque[1], output_torque[2]);
    return;
}

/**
 * @brief 计算各个电机输出
*/
void calcOutput(Cecilia_t *Ce) {
    float virtual[3], output_tor[3];
    calcLQR(&Ce->LQR);

    /* left */
    virtual[0] = Ce->LQR.u.matrix[0][0];            //驱动轮输出转矩
    virtual[1] = Ce->LQR.u.matrix[2][0];            //关节转矩
    solveVMC(Ce->l_len, Ce->l_phi, virtual, output_tor, Ce->force_l, Ce->J_l);
    Ce->Motor[WHEEL_L].torque_tgt = output_tor[0];   
    Ce->Motor[KNEE_LBM].torque_tgt = output_tor[1];
    Ce->Motor[KNEE_LFM].torque_tgt = output_tor[2];     

    /* right */
    virtual[0] = Ce->LQR.u.matrix[1][0];            //驱动轮输出转矩
    virtual[1] = Ce->LQR.u.matrix[3][0];            //关节转矩
    solveVMC(Ce->r_len, Ce->r_phi, virtual, output_tor, Ce->force_r, Ce->J_r);
    Ce->Motor[WHEEL_R].torque_tgt = output_tor[0];
    Ce->Motor[KNEE_RBM].torque_tgt = output_tor[1];
    Ce->Motor[KNEE_RFM].torque_tgt = output_tor[2];

    /* DEBUG */
    for(int i=0; i<MOTOR_NUM; i++) {
        printf("Motor %d: %f\r\n", i, Ce->Motor[i].torque_tgt);
    }
    return;
}

/**
 * @brief 输出所有电机转矩
*/
void setAllTorque(Cecilia_t *Ce) {
    int i;
    for(i=0; i < MOTOR_NUM; i++) {
        setTorque(&Ce->Motor[i]);
    }
    return;
}

/**
 * @brief 检查硬件功能
*/
void peripherals_check(Cecilia_t *Ce) {
    int i;
    Ce->Motor[0].torque_tgt = 20;
    Ce->Motor[1].torque_tgt = -20;
    Ce->Motor[2].torque_tgt = 20;
    Ce->Motor[3].torque_tgt = -20;
    updateState(Ce);
    for(i=0; i < MOTOR_NUM; i++) {
        setTorque(&Ce->Motor[i]);
        //printf("Motor %d: %f, %f\r\n", i, Ce->PosSensor[i].position, Ce->PosSensor[i].w);
    }
    //printf("IMU: %f, %f, %f\r\n", Ce->IMU.angle_value[roll], Ce->IMU.angle_value[pitch], Ce->IMU.angle_value[yaw]);
    //printf("Gyro: %f, %f, %f\r\n", Ce->Gyro.gyro_value[roll], Ce->Gyro.gyro_value[pitch], Ce->Gyro.gyro_value[yaw]);
    //printf("Acce: %f, %f, %f\r\n", Ce->Acce.accelerometer_value[roll], Ce->Acce.accelerometer_value[pitch], Ce->Acce.accelerometer_value[yaw]);
    return;
}

/**
 * @brief 绘制机器人数据波形
*/
void drawData(Cecilia_t *Ce) {
    Ce->Display.data[0] = Ce->IMU.angle_value[pitch];

    channelEnable(&Ce->Display, 0, 0x00ff00, PI/5);        //绿色，pitch
    // channelEnable(&Ce->Display, 1, 0xffff00, 10);        //黄色, pitch_vel
    // channelEnable(&Ce->Display, 2, 0xffffff, PI/12);           //白色, alpha
    // channelEnable(&Ce->Display, 3, 0xff0000, 1);         //红色. vx
    // channelEnable(&Ce->Display, 4, 0x0000ff, 3);         //蓝色, motor_torque_out
    addDisData(&Ce->Display);
    updateDis(&Ce->Display);
    printf("pitch: %f\r\n", Ce->IMU.angle_value[pitch]);
    return;
}

#endif // __DORMILY_H__

