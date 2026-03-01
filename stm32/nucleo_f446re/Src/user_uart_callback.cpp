#include "usart.h"
#include "base_msg_receiver.hpp"

extern BaseMsgReceiver base(huart2);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        base.readMsg();
    }
}
