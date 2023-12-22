#ifndef __TOOLS_H__
#define __TOOLS_H__

#define limit(x, min, max) (x < min ? min : (x > max ? max : x))

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

#endif // __TOOLS_H__