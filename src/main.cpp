#include "motor_control_app.h"

namespace {
// 基本硬件参数
constexpr int kPolePairs = 7;
constexpr float kMotorKvRpmPerVolt = 137.75f;
constexpr float kPowerSupplyVoltage = 12.8f;
constexpr float kCurrentLimitAmp = 1.8f;

// 从动力矩默认参数
constexpr float kPassiveTorqueTargetNm = 0.05f;
constexpr float kPassiveTorqueVelOnRadPerSec = 0.20f;
constexpr float kPassiveTorqueVelOffRadPerSec = 0.10f;
constexpr unsigned int kMotionDownsample = 0;

// 传感器与驱动引脚
constexpr int kI2cSdaPin = 23;
constexpr int kI2cSclPin = 5;
constexpr int kPwmUPin = 26;
constexpr int kPwmVPin = 27;
constexpr int kPwmWPin = 14;
constexpr int kDriverEnablePin = 12;
constexpr int kCurrentAPin = 35;
constexpr int kCurrentBPin = 34;

// 当前工程的电机应用配置
const MotorAppConfig kMotorConfig = {
    kPolePairs,               // 电机极对数
    kPwmUPin,                 // U 相 PWM 引脚
    kPwmVPin,                 // V 相 PWM 引脚
    kPwmWPin,                 // W 相 PWM 引脚
    kDriverEnablePin,         // 驱动使能引脚
    kCurrentAPin,             // A 相电流采样引脚
    kCurrentBPin,             // B 相电流采样引脚
    kI2cSdaPin,               // I2C SDA 引脚
    kI2cSclPin,               // I2C SCL 引脚
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
    20,                       // I2C 超时，单位 ms
    kPowerSupplyVoltage,      // 驱动母线电压
    0.01f,                    // 采样分流电阻，单位欧姆
    50.0f,                    // 电流采样放大倍数
    kCurrentLimitAmp,         // 电流限制，单位 A
    20.0f,                    // 电压限制，单位 V
    20.0f,                    // 速度限制，单位 rad/s
    kMotorKvRpmPerVolt,       // 电机 KV
    kPassiveTorqueTargetNm,   // 默认从动力矩，单位 Nm
    kPassiveTorqueVelOnRadPerSec,   // 起控速度阈值
    kPassiveTorqueVelOffRadPerSec,  // 释放速度阈值
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
