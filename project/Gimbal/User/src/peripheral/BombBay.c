#include "BombBay.h"

enum BOMB_BAY_STATE bomb_bay_state; //记录弹舱盖状态

/**
 * @brief 开关弹舱盖
 * @param[in] void
 */
void BombBay_Set(int position)
{
    if (position == BOMBBAY_SERVOS_ON_POS) //弹舱盖打开状态
    {
        bomb_bay_state = BOMB_BAY_COVER_ON;
    }
    else
    {
        bomb_bay_state = BOMB_BAY_COVER_OFF;
    }

    SERVOS_TIM_SetComparex(SERVOS_TIMx, position);
}
