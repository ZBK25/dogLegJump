#include "maconum_control.h"

/**
 * @brief 麦轮运动解算
 * @param wheelSpeed 四个麦轮的速度，[左前，右前，右后，左后]
 * @param vX 左右移动的速率，以向右为正
 * @param vY 前后移动的速率，以向前为正
 * @param w  旋转的“角速度”
 */
void maconumCal(float wheelSpeed[4], float vX, float vY, float w){
    wheelSpeed[0] = vX + vY -K_W * w;
    wheelSpeed[1] = -vX + vY + K_W * w;
    wheelSpeed[2] = vX + vY +K_W * w;
    wheelSpeed[3] = -vX + vY - K_W * w;
}
