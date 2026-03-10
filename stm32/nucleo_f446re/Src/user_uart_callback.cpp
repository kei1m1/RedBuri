#include "main.h"
#include "usart.h"
#include "motor_command_receiver.hpp"
#include "sts3215.hpp"



MotorCommandReceiver motor_command_receiver;
volatile uint8_t g_uart1_rx_pulse = 0;

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        g_uart1_rx_pulse = 1;
        STS3215::onUartRxCplt(huart);
        return;
    }
    if (huart == &huart2) {
        motor_command_receiver.onUart2RxComplete();
        return;
    }

    STS3215::onUartRxCplt(huart);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    STS3215::onUartTxCplt(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    STS3215::onUartError(huart);
}
