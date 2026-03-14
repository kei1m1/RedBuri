#include "main.h"
#include "usart.h"
#include "sts3215.hpp"
#include "motor_command_receiver.hpp"

volatile uint8_t g_uart1_rx_pulse = 0;
extern MotorCommandReceiver cmd;

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        g_uart1_rx_pulse = 1;
        STS3215::onUartRxCplt(huart);
        return;
    }
    else if(huart == &huart2)
    {
        cmd.callback();
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

