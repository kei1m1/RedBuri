#include "base_motor_receiver.hpp"
#include "usart.h"
#include <cstdio>
#include <cmath>

void BaseMotorReceiver::init()
{
    HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
}

void BaseMotorReceiver::callback()
{
    const char c = static_cast<char>(rx_byte_);

    if(c == '\r')
    {
        HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
        return;
    }

    if(c == '\n')
    {
        if(len_ > 0)
        {
            buf_[len_] = '\0';

            float rpm = 0.0f;
            float steer = 0.0f;
            if(std::sscanf(buf_, "B,%f,%f", &rpm, &steer) == 2)
            {
                motor_rpm_ = rpm;
                target_steer_deg_ = steer;
            }
        }

        len_ = 0;
        HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
        return;
    }

    if(len_ < BUF_SIZE - 1)
    {
        buf_[len_++] = c;
    }
    else
    {
        len_ = 0;
    }

    HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
}

void BaseMotorReceiver::computeMotorRpm()
{
    if(target_steer_deg_ == 90)
    {
        if(std::fabs(current_steer_deg_ -90) < 2)
        {
            front_rpm_ = motor_rpm_;
            rear_right_rpm_ = front_rpm_ * (TREAD_M / 2) / WHEELBASE_M;
            rear_left_rpm_ = -rear_right_rpm_;
            return;
        }

        front_rpm_ = 0.0f;
        rear_right_rpm_ = 0.0f;
        rear_left_rpm_ = 0.0f;
        return;
    }

    if(std::fabs(current_steer_deg_) > 30.0f)
    {
        front_rpm_ = 0.0f;
        rear_right_rpm_ = 0.0f;
        rear_left_rpm_ = 0.0f;
        return;
    }

    if(std::fabs(current_steer_deg_) < 1.0f)
    {
        front_rpm_ = motor_rpm_;
        rear_right_rpm_ = front_rpm_;
        rear_left_rpm_ = front_rpm_;
        return;
    }

    const float steer_rad = current_steer_deg_ * 3.1415926535f / 180.0f;
    const float tan_steer = std::tan(steer_rad);
    const float R = std::fabs(WHEELBASE_M / tan_steer);
    const float half_tread = TREAD_M * 0.5f;
    const float outer = front_rpm_ * ((R + half_tread) / R);
    const float inner = front_rpm_ * ((R - half_tread) / R);

    if(current_steer_deg_ > 0.0f)
    {
        front_rpm_ = motor_rpm_;
        rear_left_rpm_ = inner;
        rear_right_rpm_ = outer;
    }
    else
    {
        front_rpm_ = motor_rpm_;
        rear_left_rpm_ = outer;
        rear_right_rpm_ = inner;
    }
}

void BaseMotorReceiver::setCurrentSteerDeg(float steer_deg)
{
    current_steer_deg_ = steer_deg;
}

float BaseMotorReceiver::getFrontRpm() const
{
    return front_rpm_;
}

float BaseMotorReceiver::getRearRightRpm() const
{
    return rear_right_rpm_;
}

float BaseMotorReceiver::getRearLeftRpm() const
{
    return rear_left_rpm_;
}