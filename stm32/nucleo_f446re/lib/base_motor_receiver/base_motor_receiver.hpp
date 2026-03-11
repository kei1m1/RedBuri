#pragma once

#include <cstdint>

class BaseMotorReceiver
{
public:
    void init();
    void callback();
    void computeMotorRpm();
    float getFrontRpm() const;
    float getRearRightRpm() const;
    float getRearLeftRpm() const;
    float getTargetSteerDeg() const;

private:
    static constexpr float WHEELBASE_M = 0.93053f;
    static constexpr float TREAD_M = 0.675026f;
    static constexpr uint8_t BUF_SIZE{64};

    float front_rpm_{};
    float rear_right_rpm_{};
    float rear_left_rpm_{};
    float target_steer_deg_{};
    float current_steer_deg_{};
    uint8_t rx_byte_{};
    char buf_[BUF_SIZE]{};
    uint8_t len_{};
};
