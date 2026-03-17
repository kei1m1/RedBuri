#pragma once
#include "main.h"
#include <stdint.h>
#include <cstddef>
#include "usart.h"
#include "gpio.h"

class STS3215{
public:
    STS3215(UART_HandleTypeDef* huart, uint8_t id,
            GPIO_TypeDef* ledPort = nullptr, uint16_t ledPin = 0);

    // ランニングモード設定（Reg 33）
    HAL_StatusTypeDef setMode(uint8_t mode);

    // 位置指令（Reg 42）: 0-4095、time/speed は必要なら
    HAL_StatusTypeDef setPosition(uint16_t position,
                                  uint16_t time_ms = 0,
                                  uint16_t speed   = 0);
    // ID変更（Reg 5）
    HAL_StatusTypeDef setId(uint8_t new_id);
    // EEPROMロック（Reg 48 / 0x30） true=ロック, false=アンロック
    HAL_StatusTypeDef setEepromLock(bool locked);
    // ID変更をEEPROMに永続化（必要ならロック解除→変更→再ロック）
    HAL_StatusTypeDef setIdPersistent(uint8_t new_id, bool relock = true);

    // 現在位置（Reg 56）: 正常 0-4095 / 失敗 -1
    int16_t getPosition(uint32_t timeout_ms = 40);
    // 現在角度[deg]（0.0～360.0未満）/ 失敗時 -1.0f
    float getAngleDeg(uint32_t timeout_ms = 40);
    // StepAxisと命名を揃えた現在角度getter
    float getCurrentDeg(uint32_t timeout_ms = 40) { return getAngleDeg(timeout_ms); }

    // 現在位置が読めるまでリトライして基準化（失敗時 2048）
    int16_t syncCenter(uint32_t retries = 1000, uint32_t interval_ms = 50);

    // 相対移動（基準+相対ティック）
    void moveRelativeTicks(int16_t offset_ticks);

    // 相対移動（角度deg指定）
    void moveRelativeDeg(float offset_deg);

    // 絶対角度移動（0～360deg、範囲外は0/360に飽和）
    HAL_StatusTypeDef setAngleDeg(float angle_deg,
                                  uint16_t time_ms = 0,
                                  uint16_t speed   = 0);
    // 現在位置基準の相対移動
    HAL_StatusTypeDef moveByTicks(int16_t delta_ticks,
                                  uint32_t timeout_ms = 40,
                                  uint16_t time_ms = 0,
                                  uint16_t speed = 0);
    HAL_StatusTypeDef moveByDeg(float delta_deg,
                                uint32_t timeout_ms = 40,
                                uint16_t time_ms = 0,
                                uint16_t speed = 0);
    // 現在位置をゼロ基準として記録
    HAL_StatusTypeDef captureZero(uint32_t timeout_ms = 40);
    // ゼロ基準からの現在角度[deg]（0.0～360.0未満）/ 失敗時 -1.0f
    float getAngleFromZeroDeg(uint32_t timeout_ms = 40);
    float getCurrentDegFromZero(uint32_t timeout_ms = 40) { return getAngleFromZeroDeg(timeout_ms); }
    // ゼロ基準の角度を -180..+180 に正規化
    float getAngleFromZeroDegSigned(uint32_t timeout_ms = 40);
    // ゼロ基準角度の正方向を反転（true: 目標角増加で逆回転側）
    void setZeroDirectionReversed(bool reversed) {
        zeroDirReversed_ = reversed;
        clampTargetInited_ = false;
    }
    // ゼロ基準からの絶対角度移動（0～360deg、範囲外は0/360に飽和）
    HAL_StatusTypeDef setAngleFromZeroDeg(float angle_deg,
                                          uint16_t time_ms = 0,
                                          uint16_t speed = 0);
    // 0..360に飽和した目標角として設定
    HAL_StatusTypeDef setClampedTargetDeg(float angle_deg,
                                          uint16_t time_ms = 0,
                                          uint16_t speed = 0);
    // 目標角をdelta分だけ更新（0..360に飽和）
    HAL_StatusTypeDef moveClampedByDeg(float delta_deg,
                                       uint16_t time_ms = 0,
                                       uint16_t speed = 0);
    float getClampedTargetDeg() const { return clampTargetDeg_; }

    // --- Non-blocking (UART IT) ---
    bool requestPositionIT();
    bool isUartBusy() const { return uart_state_ != UartState::Idle; }
    void pollUart(uint32_t now_ms);
    void setUartTimeoutMs(uint32_t timeout_ms);
    void setRequestIntervalMs(uint32_t interval_ms);
    void setCommandIntervalMs(uint32_t interval_ms);
    void serviceFromLastZero(uint32_t now_ms,
                             float target_deg,
                             uint16_t time_ms = 0,
                             uint16_t speed = 0);
    // 受信（位置取得）だけを回す版：out_deg に相対角を入れる
    bool serviceReceiveFromLastZero(uint32_t now_ms, float* out_deg);
    bool hasLastPosition() const { return last_pos_valid_; }
    int16_t getLastPosition() const { return last_pos_; }
    float getLastAngleDeg() const { return last_pos_valid_ ? ticksToDeg(static_cast<uint16_t>(last_pos_)) : -1.0f; }
    bool captureZeroFromLast();
    float getAngleFromZeroDegFromLast() const;
    float getAngleFromZeroDegSignedFromLast() const;
    bool updateRelativeDegFromLast(float* out_deg);
    HAL_StatusTypeDef setAngleFromLastZeroDeg(float target_deg,
                                              uint16_t time_ms = 0,
                                              uint16_t speed = 0);
    // -180..+180 指定（ゼロ基準）
    HAL_StatusTypeDef setAngleFromZeroDegSigned(float target_deg,
                                                uint16_t time_ms = 0,
                                                uint16_t speed = 0);
    HAL_StatusTypeDef setAngleFromLastZeroDegSigned(float target_deg,
                                                    uint16_t time_ms = 0,
                                                    uint16_t speed = 0);
    static void onUartTxCplt(UART_HandleTypeDef* huart);
    static void onUartRxCplt(UART_HandleTypeDef* huart);
    static void onUartError(UART_HandleTypeDef* huart);

    // 角度→ティック（4096/360）
    static int16_t degToTicks(float deg);
    static uint16_t degToPos(float deg);   // 0～360deg -> 0～4095（飽和）
    static float ticksToDeg(uint16_t ticks); // 0～4095 -> 0～360deg

private:
    enum class UartState : uint8_t { Idle, TxPending, RxPending };
    static void registerInstance(STS3215* inst);

    UART_HandleTypeDef* huart_;
    uint8_t             id_;
    int16_t             startPos_;      // -1 = 未確定
    uint16_t            zeroPos_;       // captureZeroで記録する基準tick
    bool                zeroCaptured_;
    bool                zeroDirReversed_;
    float               lastZeroDeg_;
    bool                lastZeroDegValid_;
    float               clampTargetDeg_;
    bool                clampTargetInited_;
    volatile UartState  uart_state_;
    uint32_t            uart_deadline_ms_;
    uint32_t            uart_timeout_ms_;
    uint32_t            req_interval_ms_;
    uint32_t            cmd_interval_ms_;
    uint32_t            next_req_ms_;
    uint32_t            next_cmd_ms_;
    uint8_t             tx_msg_[8];
    uint8_t             rx_msg_[8];
    int16_t             last_pos_;
    bool                last_pos_valid_;
    GPIO_TypeDef*       ledPort_;       // 任意
    uint16_t            ledPin_;        // 任意

    // 低レベル補助
    static float clampDeg01(float deg);
    static uint8_t calcChecksum(const uint8_t* msg, size_t len);
    HAL_StatusTypeDef writeReg8(uint8_t addr, uint8_t value);
    inline void toTx() { HAL_HalfDuplex_EnableTransmitter(huart_); }
    inline void toRx() { HAL_HalfDuplex_EnableReceiver(huart_);   }
    inline void ledOn()  { if (ledPort_) HAL_GPIO_WritePin(ledPort_, ledPin_, GPIO_PIN_SET); }
    inline void ledOff() { if (ledPort_) HAL_GPIO_WritePin(ledPort_, ledPin_, GPIO_PIN_RESET); }
};

// Simple scheduler for non-blocking STS3215 control using timer ticks.
// Call STS3215Scheduler::onTimerTickAll() from a periodic timer ISR (e.g., TIM6 1kHz).
class STS3215Scheduler
{
public:
    explicit STS3215Scheduler(STS3215& servo);

    // Configure tick intervals (in timer ticks, e.g. 1kHz -> 1 tick = 1ms).
    void setRequestIntervalTicks(uint32_t ticks);
    void setCommandIntervalTicks(uint32_t ticks);

    // Target angle in degrees (0..360 clamped).
    void setTargetDeg(float target_deg);
    void setTargetDegSigned(float target_deg);
    // Clamp at [0..360], and treat 360 as max tick (avoid wrap to 0)
    void setTargetDegClamped(float target_deg);
    void setTargetDegSignedClamped(float target_deg);

    // Call from main loop.
    void update(uint32_t now_ms);

    bool getCurrentDeg(float* out_deg) const;
    bool getCurrentDegSigned(float* out_deg) const;
    float getCurrentRad() const;
    bool hasCurrent() const { return current_valid_; }

    // Call from timer ISR.
    static void onTimerTickAll();

private:
    static void registerInstance(STS3215Scheduler* inst);

    STS3215& servo_;
    float target_deg_{0.0f};
    bool use_clamped_{false};
    float current_deg_{0.0f};
    bool current_valid_{false};
    uint32_t tick_{0};
    uint32_t req_interval_ticks_{50};
    uint32_t cmd_interval_ticks_{20};
    volatile uint8_t req_flag_{0};
    volatile uint8_t cmd_flag_{0};
};
