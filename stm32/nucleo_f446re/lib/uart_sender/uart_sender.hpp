#pragma once

#include <cstddef>
#include <cstdint>
#include "tim.h"
#include "usart.h"

class UartSender
{
public:
    UartSender(UART_HandleTypeDef& huart, TIM_HandleTypeDef& htim);
    void init();
    void onTimerTick();
    void sendJointDeg(int32_t joint_1_mrad,
                      int32_t joint_2_mrad,
                      int32_t joint_3_mrad,
                      int32_t joint_4_mrad,
                      int32_t joint_5_mrad,
                      int32_t joint_6_mrad,
                      int32_t gripper_mrad);

private:
    static constexpr size_t TX_BUF_SIZE = 128U;

    UART_HandleTypeDef& huart_;
    TIM_HandleTypeDef& htim_;
    volatile bool tick_pending_{false};
    char tx_buf_[TX_BUF_SIZE]{};
};
