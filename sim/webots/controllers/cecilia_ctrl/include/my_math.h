#ifndef __MY_MATH_H__
#define __MY_MATH_H__

#ifndef PI
#define PI 3.14159265358979323846
#endif

/**
 * @brief 角度归一化（弧度制）
 * @param theta 角度
 * @return 归一化后的角度(0~2PI)
*/
float normalize(float theta) {
    while (theta < 0) {
        theta += 2 * PI;
    }
    while (theta > 2 * PI) {
        theta -= 2 * PI;
    }
    return theta;
}

#endif // __MY_MATH_H__