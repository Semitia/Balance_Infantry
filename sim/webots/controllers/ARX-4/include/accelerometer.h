/*
 * @Author: your name
 * @Date: 2020-12-27 19:03:44
 * @LastEditTime: 2021-01-19 11:42:48
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \webots\accelerometer.h
 */
/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**********************************************************************************/
/* Description:  Webots C programming interface for the Accelerometer node   
    “加速计”节点可测量1、2或3个轴上的加速度和重力感应反作用力。例如，它可用于检测跌倒，上下方向等。
    问题:如果只有主动获取加速度值的函数的话,没有定时器中断采样周期有啥意义吗
    另外,如何更改分辨率? */
/**********************************************************************************/

#ifndef WB_ACCELEROMETER_H
#define WB_ACCELEROMETER_H

#define WB_USING_C_API
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @description: 启动加速度计
 * @param {WbDeviceTag} tag 调用的时候直接输入ID即可
 * @param {int} sampling_period,//定义//传感器的采样周期, 以毫秒为单位来表示
 * @return {*}
 */
void wb_accelerometer_enable(WbDeviceTag tag, int sampling_period);

/**
 * @description: 关闭加速度计，从而节省了计算时间
 * @param {WbDeviceTag} tag
 * @return {*}
 */
void wb_accelerometer_disable(WbDeviceTag tag);

/**
 * @description: 周期由wb_accelerometer_enable参数给定. 如果禁用了该设备，则返回0。
 * @param {WbDeviceTag} tag
 * @return {*}返回一个周期参数而已
 */
int wb_accelerometer_get_sampling_period(WbDeviceTag tag);

/**
 * @description: 
 * @param {WbDeviceTag} tag
 * @return {*}
 */
int wb_accelerometer_get_lookup_table_size(WbDeviceTag tag);


/**
 * @description: 
 * @param {*}
 * @return {*}
 */
const double *wb_accelerometer_get_lookup_table(WbDeviceTag tag);

// return a pointer to an array of 3 double for X, Y and Z accelerations

/**
 * @description: 
 * @param {*}
 * @return {*}返回的向量是指向由Accelerometer节点管理的内部值的指针，因此释放该指针是非法的。此外，请注意，
 * 指向的值仅在下一次调用wb_robot_step或Robot::step函数之前才有效。如果长时间需要这些值，则必须将其复制。
 */
const double *wb_accelerometer_get_values(WbDeviceTag tag);

#ifdef __cplusplus
}
#endif

#endif /* WB_ACCELEROMETER_H */
