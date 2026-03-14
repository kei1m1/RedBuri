#include "motor_command_receiver.hpp"

#include <cmath>

MotorCommandReceiver::MotorCommandReceiver(UART_HandleTypeDef& huart)
    : huart_(huart)
{}

void MotorCommandReceiver::init()
{
    len_ = 0;
    HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
}

void MotorCommandReceiver::callback()
{
    const char c = static_cast<char>(rx_byte_);

    if(c == '\r')
    {
        HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
        return;
    }

    if(c == '\n')
    {
        buf_[len_] = '\0';

        if(buf_[0] == 'B')
        {
            BaseCommand cmd{};
            if(parseBaseCommand(buf_, cmd))
            {
                base_cmd_ = cmd;
                base_ready_ = true;
                recomputeBaseMotorRpm();
            }
        }
        else if(buf_[0] == 'A')
        {
            ArmCommand cmd{};
            if(parseArmCommand(buf_, cmd))
            {
                arm_cmd_ = cmd;
                arm_ready_ = true;
            }
        }

        len_ = 0;
        HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
        return;
    }

    if(len_ < BUF_SIZE - 1)
    {
        buf_[len_] = c;
        len_++;
    }
    else
    {
        len_ = 0;
    }

    HAL_UART_Receive_IT(&huart_, &rx_byte_, 1);
}

void MotorCommandReceiver::setCurrentSteerDeg(float steer_deg)
{
    current_steer_deg_ = steer_deg;
    recomputeBaseMotorRpm();
}

bool MotorCommandReceiver::fetchBaseCommand(BaseCommand& cmd)
{
    if(!base_ready_)
    {
        return false;
    }

    cmd = base_cmd_;
    base_ready_ = false;
    return true;
}

bool MotorCommandReceiver::fetchArmCommand(ArmCommand& cmd)
{
    if(!arm_ready_)
    {
        return false;
    }

    cmd = arm_cmd_;
    arm_ready_ = false;
    return true;
}

float MotorCommandReceiver::getFrontRpm() const
{
    return front_rpm_;
}

float MotorCommandReceiver::getRearRightRpm() const
{
    return rear_right_rpm_;
}

float MotorCommandReceiver::getRearLeftRpm() const
{
    return rear_left_rpm_;
}

float MotorCommandReceiver::getTargetSteerDeg() const
{
    return static_cast<float>(base_cmd_.steer_deg);
}

bool MotorCommandReceiver::parseBaseCommand(const char* line, BaseCommand& cmd)
{
    const char* p = line;

    if(*p != 'B')
    {
        return false;
    }
    p++;

    if(*p != ',')
    {
        return false;
    }
    p++;

    if(!parseInt(p, cmd.motor_rpm))
    {
        return false;
    }

    if(*p != ',')
    {
        return false;
    }
    p++;
    
    if(!parseInt(p, cmd.steer_deg))
    {
        return false;
    }

    if(*p != '\0')
    {
        return false;
    }

    return true;
}

bool MotorCommandReceiver::parseArmCommand(const char* line, ArmCommand& cmd)
{
    const char* p = line;

    if(*p != 'A')
    {
        return false;
    }
    p++;

    if(*p != ',')
    {
        return false;
    }
    p++;

    for(int i = 0; i < 7; i++)
    {
        if(!parseInt(p, cmd.motor_rpm[i]))
        {
            return false;
        }

        if(i < 6)
        {
            if(*p != ',')
            {
                return false;
            }
            p++;
        }
    }

    if(*p != '\0')
    {
        return false;
    }

    return true;
}

bool MotorCommandReceiver::parseInt(const char*& p, int16_t& value)
{
    bool negative = false;
    int32_t result = 0;

    if(*p == '-')
    {
        negative = true;
        p++;
    }

    if(*p < '0' || *p > '9')
    {
        return false;
    }

    while(*p >= '0' && *p <= '9')
    {
        result = result * 10 + (*p - '0');
        p++;
    }

    if(negative)
    {
        result = -result;
    }

    value = static_cast<int16_t>(result);
    return true;
}

void MotorCommandReceiver::recomputeBaseMotorRpm()
{
    const float motor_rpm = static_cast<float>(base_cmd_.motor_rpm);
    const float target_steer_deg = static_cast<float>(base_cmd_.steer_deg);

    if(target_steer_deg == 90.0f)
    {
        if(std::fabs(current_steer_deg_ - 90.0f) < 2.0f)
        {
            front_rpm_ = motor_rpm;
            rear_right_rpm_ = motor_rpm * (TREAD_M / 2.0f) / WHEELBASE_M;
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
        front_rpm_ = motor_rpm;
        rear_right_rpm_ = front_rpm_;
        rear_left_rpm_ = front_rpm_;
        return;
    }

    const float steer_rad = current_steer_deg_ * 3.1415926535f / 180.0f;
    const float tan_steer = std::tan(steer_rad);
    const float R = std::fabs(WHEELBASE_M / tan_steer);
    const float half_tread = TREAD_M / 2.0f;
    const float outer = motor_rpm * ((R + half_tread) / R);
    const float inner = motor_rpm * ((R - half_tread) / R);

    if(current_steer_deg_ > 0.0f)
    {
        front_rpm_ = motor_rpm;
        rear_left_rpm_ = inner;
        rear_right_rpm_ = outer;
    }
    else
    {
        front_rpm_ = motor_rpm;
        rear_left_rpm_ = outer;
        rear_right_rpm_ = inner;
    }
}
