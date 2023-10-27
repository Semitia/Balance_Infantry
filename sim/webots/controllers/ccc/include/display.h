#ifndef __DISPLY_H__
#define __DISPLY_H__

#include "sys.h"

#define MAX_WIDTH 128
#define MAX_HEIGHT 128
#define RING_SIZE MAX_WIDTH
#define CHANNEL_NUM 4

/**
 * @brief 点结构体
*/
typedef struct __Piont_t {
    int x,y;
}Point_t;

/**
 * @brief 环形存储器
 * @note 用于一个通道各个点的存储
*/
typedef struct __RingBuf_t {
    Point_t Points[RING_SIZE];
    short head, tail;
}RingBuf_t;

/**
 * @brief 显示器的一个数据通道
*/
typedef struct __Channel_t {
    bool enable;            // 通道使能, 0:不使能 1:使能
    uint32_t color;         // 颜色
    RingBuf_t R;            // 环形缓存器
    double range;           // 数据上下限 -range ~ +range
}Channel_t;

/**
 * @brief 显示器结构体
*/
typedef struct __Display_t {
    int width, height;                  // 显示器宽高
    WbDeviceTag id;                     // 显示器ID
    Channel_t Channels[CHANNEL_NUM];    // 通道
    double data[CHANNEL_NUM];           // 各个通道最新数据
}Display_t;

void pushPoint(RingBuf_t *R, Point_t P);
uint8_t ringHasMsg(RingBuf_t *R);
Point_t popPoint(RingBuf_t *R);
void channelEnable(Display_t *D, int channel_id, uint32_t color, double range);
void displayInit(Display_t *D);
int linearMap(double data, double raw_up, double raw_down, int up, int down);
void addDisData(Display_t *D);
void updateDis(Display_t *D);


/**
 * @brief 添加数据
 * @note 如果满了，就覆盖最早的数据
 * @param R 环形存储器
 * @param P 点
*/
void pushPoint(RingBuf_t *R, Point_t P) {
    if((R->head+1 % RING_SIZE) == R->tail) {
        R->tail = (R->tail+1) % RING_SIZE;
    }
    R->Points[R->head] = P;
    R->head = (R->head+1) % RING_SIZE;
    return;
}

/**
 * @brief 判断环形缓存器中是否有消息
 * @param R 环形缓存器指针
 * @retval 0:没有消息 1:有消息
 */
uint8_t ringHasMsg(RingBuf_t *R) {
    if(R->head == R->tail) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * @brief 弹出一个点
 * @param R 环形缓存器指针
 * @retval Point_t 弹出的点
*/
Point_t popPoint(RingBuf_t *R) {
    Point_t p = R->Points[R->tail];
    R->tail = (R->tail + 1) % RING_SIZE;
    return p;
}

/**
 * @brief 使能显示器的某个通道
*/
void channelEnable(Display_t *D, int channel_id, uint32_t color, double range) {
    D->Channels[channel_id].enable = true;
    D->Channels[channel_id].color = color;
    D->Channels[channel_id].range = range;
    return;
}

/**
 * @brief 初始化显示器
 * @param D 显示器指针
 * @param name 显示器名字
*/
void displayInit(Display_t *D) {
    D->id = wb_robot_get_device("display");
    D->height = wb_display_get_height(D->id);
    D->width = wb_display_get_width(D->id);
    if (D->id == 0) {
        printf("Error: Can't find the display");
        return;
    }
    else { printf("Display found!\n"); }
    channelEnable(D, 0, 0x0000ff, 1);
    channelEnable(D, 1, 0xff0000, 1);
    return;
}

/**
 * @brief 将浮点数据映射到显示器的像素坐标
 * @param data 数据
 * @param raw_up 数据上限
 * @param raw_down 数据下限
 * @param up 映射域上限
 * @param down 映射域下限
*/
int linearMap(double data, double raw_up, double raw_down, int up, int down) {
    int ans = (int)((data - raw_down) / (raw_up - raw_down) * (up - down) + down);
    return ans;
}

/**
 * @brief 向各个通道添加数据
 * @param D 显示器指针
 * @param data 数据
*/
void addDisData(Display_t *D) {
    Point_t p;
    for (int i = 0; i < CHANNEL_NUM; i++) {
        if (!D->Channels[i].enable) continue;
        //先将原先所有数据向左移动一个像素
        for (int j = D->Channels[i].R.tail; j != D->Channels[i].R.head; j = (j + 1) % RING_SIZE) {
            D->Channels[i].R.Points[j].x--;
        }
        //再在最右侧添加新的数据
        p.x = D->width - 1;
        p.y = linearMap(D->data[i], D->Channels[i].range, -D->Channels[i].range, D->height - 1, 0);
        pushPoint(&D->Channels[i].R, p);
    }
    return;
}

/**
 * @brief 屏幕刷新一次
 * @note 屏幕滚动式示波，最右侧的数据代表最新的数据，
 *       每次刷新，所有的数据都向左移动一个像素
 * @param D 显示器指针
*/
void updateDis(Display_t *D) {
    // 清空显示器
    wb_display_fill_rectangle(D->id, 0, 0, D->width, D->height);

    // 遍历每个通道
    for (int i = 0; i < CHANNEL_NUM; i++) {
        Channel_t *channel = &D->Channels[i];
        // 如果通道被使能，则绘制通道的数据
        if(!channel->enable) continue; 
        wb_display_set_color(D->id, channel->color);

        // 遍历环形缓存器中的每个点
        int index = channel->R.tail;
        while (index != channel->R.head) {
            Point_t point = channel->R.Points[index];
            wb_display_draw_pixel(D->id, point.x, point.y);           // 绘制点
            // 移动到下一个点
            index = (index + 1) % RING_SIZE;
        }
        
    }
    return;
}


#endif