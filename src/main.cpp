#include "motor_control_app.h"

namespace {
// 基础硬件参数
constexpr int kPolePairs = 7;                         // 电机极对数
constexpr float kMotorKvRpmPerVolt = 137.75f;        // 电机 KV [rpm/V]
constexpr float kPowerSupplyVoltage = 12.8f;         // 驱动母线电压 [V]
constexpr float kCurrentLimitAmp = 1.8f;             // 电流限制 [A]

// 从动力矩控制默认参数
constexpr float kPassiveTorqueTargetNm = 0.02f;      // 目标阻尼力矩 [Nm]
constexpr float kPassiveTorqueSaturationAngleDeg = 2.0f;  // 饱和磁场阻尼角 [deg]
constexpr float kPassiveTorqueFollowDeadzoneDeg = 0.8f;   // 跟随死区 [deg]
constexpr float kPassiveTorqueRunningSpeedThresholdRadS = 2.0f;  // 旋转工况阈值 [rad/s]
constexpr unsigned int kPassiveTorqueCalculationHz = 1000;       // 计算频率 [Hz]
constexpr unsigned int kPassiveTorqueTransitionBufferMs = 500;  // 切回从动力矩前的释放缓冲时间 [ms]
constexpr float kPassiveTorqueFollowPidLowP = 0.0311f;           // 低速/静止跟随 PID P
constexpr float kPassiveTorqueFollowPidLowI = 0.0f;              // 低速/静止跟随 PID I
constexpr float kPassiveTorqueFollowPidLowD = 0.0f;              // 低速/静止跟随 PID D
constexpr float kPassiveTorqueFollowPidRunP = 0.0311f;           // 旋转工况跟随 PID P
constexpr float kPassiveTorqueFollowPidRunI = 0.0f;              // 旋转工况跟随 PID I
constexpr float kPassiveTorqueFollowPidRunD = 0.0f;              // 旋转工况跟随 PID D

// 运动控制参数
constexpr float kVelocityPidP = 0.021f;             // 速度环 P
constexpr float kVelocityPidI = 0.12f;              // 速度环 I
constexpr float kVelocityPidD = 0.0f;               // 速度环 D
constexpr float kVelocityLpfTf = 0.01f;             // 速度低通滤波时间常数
constexpr float kAnglePidP = 20.0f;                 // 角度环 P
constexpr float kCurrentQPidP = 3.0f;               // 电流 Q 环 P
constexpr float kCurrentQPidI = 120.0f;             // 电流 Q 环 I
constexpr float kCurrentQPidD = 0.0f;               // 电流 Q 环 D
constexpr float kCurrentQLpfTf = 0.005f;            // 电流 Q 环低通滤波时间常数
constexpr float kCurrentDPidP = 7.3728f;               // 电流 D 环 P
constexpr float kCurrentDPidI = 279.936f;             // 电流 D 环 I
constexpr float kCurrentDPidD = 0.0f;               // 电流 D 环 D
constexpr float kCurrentDLpfTf = 0.005f;            // 电流 D 环低通滤波时间常数
constexpr unsigned int kMotionDownsample = 0;       // 运动控制降采样

// 传感器与驱动引脚
constexpr int kI2cSdaPin = 23;                      // I2C SDA
constexpr int kI2cSclPin = 5;                       // I2C SCL
constexpr int kPwmUPin = 26;                        // U 相 PWM
constexpr int kPwmVPin = 27;                        // V 相 PWM
constexpr int kPwmWPin = 14;                        // W 相 PWM
constexpr int kDriverEnablePin = 12;                // 驱动使能
constexpr int kCurrentAPin = 35;                    // A 相电流采样
constexpr int kCurrentBPin = 34;                    // B 相电流采样

const MotorAppConfig kMotorConfig = {
    kPolePairs,                       // 电机极对数
    kPwmUPin,                         // U 相 PWM
    kPwmVPin,                         // V 相 PWM
    kPwmWPin,                         // W 相 PWM
    kDriverEnablePin,                 // 驱动使能
    kCurrentAPin,                     // A 相电流采样
    kCurrentBPin,                     // B 相电流采样
    kI2cSdaPin,                       // I2C SDA
    kI2cSclPin,                       // I2C SCL
    32,                               // 需要上拉的 GPIO32
    33,                               // 需要上拉的 GPIO33
    25,                               // 需要上拉的 GPIO25
    26,                               // 需要上拉的 GPIO26
    27,                               // 需要上拉的 GPIO27
    14,                               // 需要上拉的 GPIO14
    0x36,                             // AS5600 I2C 地址
    0x0C,                             // AS5600 角度高字节寄存器
    4096,                             // AS5600 每圈计数
    100000UL,                         // I2C 频率 [Hz]
    20,                               // I2C 超时 [ms]
    kPowerSupplyVoltage,              // 驱动母线电压 [V]
    0.01f,                            // 分流电阻 [ohm]
    50.0f,                            // 电流采样放大倍数
    kCurrentLimitAmp,                 // 电流限制 [A]
    20.0f,                            // 电压限制 [V]
    42.0f,                            // 速度限制 [rad/s]
    kMotorKvRpmPerVolt,               // 电机 KV [rpm/V]
    kPassiveTorqueTargetNm,           // 目标阻尼力矩 [Nm]
    kPassiveTorqueSaturationAngleDeg, // 饱和磁场阻尼角 [deg]
    kPassiveTorqueFollowDeadzoneDeg,  // 跟随死区 [deg]
    kPassiveTorqueRunningSpeedThresholdRadS,  // 旋转工况阈值 [rad/s]
    kPassiveTorqueCalculationHz,      // 计算频率 [Hz]
    kPassiveTorqueTransitionBufferMs, // 切回从动力矩前的释放缓冲时间 [ms]
    kPassiveTorqueFollowPidLowP,      // 低速/静止跟随 PID P
    kPassiveTorqueFollowPidLowI,      // 低速/静止跟随 PID I
    kPassiveTorqueFollowPidLowD,      // 低速/静止跟随 PID D
    kPassiveTorqueFollowPidRunP,      // 旋转工况跟随 PID P
    kPassiveTorqueFollowPidRunI,      // 旋转工况跟随 PID I
    kPassiveTorqueFollowPidRunD,      // 旋转工况跟随 PID D
    kVelocityPidP,                    // 速度环 P
    kVelocityPidI,                    // 速度环 I
    kVelocityPidD,                    // 速度环 D
    kVelocityLpfTf,                   // 速度低通滤波时间常数
    kAnglePidP,                       // 角度环 P
    kCurrentQPidP,                    // 电流 Q 环 P
    kCurrentQPidI,                    // 电流 Q 环 I
    kCurrentQPidD,                    // 电流 Q 环 D
    kCurrentQLpfTf,                   // 电流 Q 环低通滤波时间常数
    kCurrentDPidP,                    // 电流 D 环 P
    kCurrentDPidI,                    // 电流 D 环 I
    kCurrentDPidD,                    // 电流 D 环 D
    kCurrentDLpfTf,                   // 电流 D 环低通滤波时间常数
    kMotionDownsample,                // 运动控制降采样
    static_cast<uint8_t>(_MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q),
    0,                                // 监控降采样
    115200UL,                         // 串口波特率
    'M',                              // Commander 命令前缀
};
}  // namespace

MotorControlApp app(kMotorConfig);

void setup() {
  app.setup();
}

void loop() {
  app.loop();
}
