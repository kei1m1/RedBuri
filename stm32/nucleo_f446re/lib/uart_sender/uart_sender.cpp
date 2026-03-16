#include <algorithm>
#include <cstdio>
#include "uart_sender.hpp"

UartSender::UartSender(UART_HandleTypeDef& huart, TIM_HandleTypeDef& htim)
    : huart_(huart), htim_(htim)
{}

void UartSender::init()
{
    HAL_TIM_Base_Start_IT(&htim_);
}

void UartSender::onTimerTick()
{
    tick_pending_ = true;
}

void UartSender::sendJointDeg(int32_t joint_1_mrad,
                              int32_t joint_2_mrad,
                              int32_t joint_3_mrad,
                              int32_t joint_4_mrad,
                              int32_t joint_5_mrad,
                              int32_t joint_6_mrad,
                              int32_t gripper_mrad)
{
    if(!tick_pending_)
    {
        return;
    }

    tick_pending_ = false;

    const int written = std::snprintf(tx_buf_,
                                      TX_BUF_SIZE,
                                      "J,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
                                      static_cast<long>(joint_1_mrad),
                                      static_cast<long>(joint_2_mrad),
                                      static_cast<long>(joint_3_mrad),
                                      static_cast<long>(joint_4_mrad),
                                      static_cast<long>(joint_5_mrad),
                                      static_cast<long>(joint_6_mrad),
                                      static_cast<long>(gripper_mrad));
    if(written <= 0)
    {
        return;
    }

    const size_t tx_len = std::min(static_cast<size_t>(written), TX_BUF_SIZE - 1U);
    HAL_UART_Transmit(&huart_, reinterpret_cast<uint8_t*>(tx_buf_), tx_len, HAL_MAX_DELAY);
}
