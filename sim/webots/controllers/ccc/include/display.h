#ifndef __DISPLY_H__
#define __DISPLY_H__

#include "sys.h"

#define MAX_WIDTH 512
#define MAX_HEIGHT 256
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
    WbDeviceTag tag;                     // 显示器tag
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
    R->head = (R->head+1) % RING_SIZE;      
    //printf("DEBUG: head:%d, tail:%d, x:%d, y:%d\n", R->head, R->tail, P.x, P.y);
    if(R->head == R->tail) R->tail = (R->tail+1) % RING_SIZE;//如果满了，就覆盖最早的数据
    R->Points[R->head] = P;                     //保证head对应最新数据
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
        return ((R->head + RING_SIZE - R->tail) % RING_SIZE);
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
    D->tag = wb_robot_get_device("display");
    D->height = wb_display_get_height(D->tag);
    D->width = wb_display_get_width(D->tag);
    if (D->tag == 0) {
        printf("Error: Can't find the display");
        return;
    }
    else { printf("Display found! %d x %d\n", D->width, D->height); }

    for(int i=0; i<CHANNEL_NUM; i++) {
        D->Channels[i].enable = false;
        D->Channels[i].R.head = 0;
        D->Channels[i].R.tail = 0;
    }

    // channelEnable(D, 0, 0xff0000, 1);
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
        p.y = D->height - p.y;  // 原点在左上角, 纵坐标反一下
        pushPoint(&D->Channels[i].R, p);
    }
    return;
}

/**
 * @brief 屏幕刷新一次
 * @note 屏幕滚动式示波，最右侧的数据代表最新的数据，
 *       每次刷新，所有的数据都向左移动一个像素
 *       原点在左上角
 * @param D 显示器指针
*/
void updateDis(Display_t *D) {
    // 清空显示器
    wb_display_set_color(D->tag, 0x000000);         //记得修改画笔颜色
    wb_display_fill_rectangle(D->tag, 0, 0, D->width, D->height);

    // 遍历每个通道
    for (int i = 0; i < CHANNEL_NUM; i++) {
        Channel_t *channel = &D->Channels[i];
        // 如果通道被使能，则绘制通道的数据
        if(!channel->enable) continue; 
        wb_display_set_color(D->tag, channel->color);

        //从tail+1遍历到head, +1是为了避免在绘制线的时候最开始的一个点带来的波形突变
        for(int index = channel->R.tail+1; index != channel->R.head; index = (index+1) % RING_SIZE) {
            int next_idx = (index+1) % RING_SIZE;
            Point_t point = channel->R.Points[index];
            Point_t next_point = channel->R.Points[next_idx];      
            wb_display_draw_line(D->tag, point.x, point.y, next_point.x, next_point.y); // 绘制线
            //wb_display_draw_pixel(D->tag, point.x, point.y);                          // 绘制像素点
        }
        //绘制坐标系
        wb_display_set_color(D->tag, 0x646464);
        wb_display_draw_line(D->tag, 0, D->height/2, D->width, D->height/2);//x轴
        wb_display_draw_line(D->tag, D->width/2, 0, D->width/2, D->height);//y轴
    }
    return;
}


#endif