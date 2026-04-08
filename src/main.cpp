#include "motor_control_app.h"

namespace {
// 基础硬件参数
constexpr int kPolePairs = 7;                  // 电机极对数
constexpr float kMotorKvRpmPerVolt = 137.75f; // 电机 KV
constexpr float kPowerSupplyVoltage = 12.8f;   // 驱动母线电压 [V]
constexpr float kCurrentLimitAmp = 1.8f;       // 电流限制 [A]

// 从动力矩控制默认参数
constexpr float kPassiveTorqueTargetNm = 0.05f;          // 目标阻尼力矩 [Nm]
constexpr float kPassiveTorqueMaxAngleDeg = 1.0f;        // 最大磁场阻尼角 [deg]
constexpr unsigned int kPassiveTorqueUpdateHz = 1000;    // 判定频率 [Hz]

// 运动控制参数
constexpr unsigned int kMotionDownsample = 0; // 0 表示不降采样

// 传感器与驱动引脚
constexpr int kI2cSdaPin = 23;         // I2C SDA
constexpr int kI2cSclPin = 5;          // I2C SCL
constexpr int kPwmUPin = 26;           // U 相 PWM
constexpr int kPwmVPin = 27;           // V 相 PWM
constexpr int kPwmWPin = 14;           // W 相 PWM
constexpr int kDriverEnablePin = 12;   // 驱动使能
constexpr int kCurrentAPin = 35;       // A 相电流采样
constexpr int kCurrentBPin = 34;       // B 相电流采样

// 当前工程的电机应用配置
const MotorAppConfig kMotorConfig = {
    kPolePairs,               // 电机极对数
    kPwmUPin,                 // U 相 PWM
    kPwmVPin,                 // V 相 PWM
    kPwmWPin,                 // W 相 PWM
    kDriverEnablePin,         // 驱动使能
    kCurrentAPin,             // A 相电流采样
    kCurrentBPin,             // B 相电流采样
    kI2cSdaPin,               // I2C SDA
    kI2cSclPin,               // I2C SCL
    32,                       // 需要上拉的 GPIO32
    33,                       // 需要上拉的 GPIO33
    25,                       // 需要上拉的 GPIO25
    26,                       // 需要上拉的 GPIO26
    27,                       // 需要上拉的 GPIO27
    14,                       // 需要上拉的 GPIO14
    0x36,                     // AS5600 I2C 地址
    0x0C,                     // AS5600 角度高字节寄存器
    4096,                     // AS5600 每圈计数
    100000UL,                 // I2C 频率
    20,                       // I2C 超时 [ms]
    kPowerSupplyVoltage,      // 驱动母线电压 [V]
    0.01f,                    // 分流电阻 [ohm]
    50.0f,                    // 电流采样放大倍数
    kCurrentLimitAmp,         // 电流限制 [A]
    20.0f,                    // 电压限制 [V]
    20.0f,                    // 速度限制 [rad/s]
    kMotorKvRpmPerVolt,       // 电机 KV
    kPassiveTorqueTargetNm,   // 目标阻尼力矩 [Nm]
    kPassiveTorqueMaxAngleDeg,// 最大磁场阻尼角 [deg]
    kPassiveTorqueUpdateHz,   // 判定频率 [Hz]
    0.021f,                   // 速度环 P
    0.12f,                    // 速度环 I
    0.0f,                     // 速度环 D
    0.01f,                    // 速度低通滤波时间常数
    20.0f,                    // 角度环 P
    kMotionDownsample,        // 运动控制降采样
    static_cast<uint8_t>(_MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q),
    0,                        // 监控降采样
    115200UL,                 // 串口波特率
    'M',                      // Commander 命令前缀
};
}  // namespace

MotorControlApp app(kMotorConfig);

void setup() {
  app.setup();
}

void loop() {
  app.loop();
}
