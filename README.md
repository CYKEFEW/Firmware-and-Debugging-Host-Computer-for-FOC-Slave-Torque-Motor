# Deng FOC M1

基于 `ESP32 + SimpleFOC 2.2.1` 的电机控制项目，包含两部分：

- 固件工程：位于仓库根目录，使用 `PlatformIO`
- 上位机工程：位于 [SimpleFOCStudio-zh-master](/e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master)，基于 `PyQt5`

这个仓库已经围绕当前电机和调试流程做了较多定制，重点包括：

- 从动力矩控制模式
- 树形视图调参与中文界面修正
- PID 自动测量页面
- 图表性能优化与窗口长度设置

## 项目结构

```text
.
├─ platformio.ini
├─ src/
│  ├─ main.cpp
│  ├─ motor_control_app.h
│  ├─ motor_control_app.cpp
│  ├─ recovering_as5600_sensor.h
│  └─ recovering_as5600_sensor.cpp
├─ SimpleFOCStudio-zh-master/
│  ├─ simpleFOCStudio.py
│  └─ src/
└─ README.md
```

## 固件说明

固件运行在 `WEMOS LOLIN32 Lite` 上，工程配置见 [platformio.ini](/e:/Projects/PIO/Deng%20FOC%20M1/platformio.ini)。

当前做法是：

- [main.cpp](/e:/Projects/PIO/Deng%20FOC%20M1/src/main.cpp) 只保留默认参数和入口
- 主要控制逻辑放在 [motor_control_app.h](/e:/Projects/PIO/Deng%20FOC%20M1/src/motor_control_app.h) 和 [motor_control_app.cpp](/e:/Projects/PIO/Deng%20FOC%20M1/src/motor_control_app.cpp)

### 当前关键默认值

- 极对数：`7`
- 电机 KV：`137.75 rpm/V`
- 母线电压：`12.8 V`
- 电流限制：`1.8 A`
- 串口波特率：`115200`
- Commander 命令 ID：`M`
- 传感器：`AS5600`
- I2C：`100 kHz`

### 固件特性

- 标准 `SimpleFOC Commander` 接口
- I2C 编码器读失败自动恢复
- 电流采样与 `FOC` 电流控制
- 自定义“释放”模式
- 自定义“从动力矩控制”模式

### 从动力矩控制默认参数

这些参数集中定义在 [main.cpp](/e:/Projects/PIO/Deng%20FOC%20M1/src/main.cpp) 顶部：

- 目标阻尼力矩
- 饱和磁场阻尼角
- 跟随死区
- 旋转工况阈值，默认 `2.0 rad/s`
- 计算频率
- 低速/静止跟随 PID `P / I / D`
- 旋转工况跟随 PID `P / I / D`

## 上位机说明

上位机入口文件是 [simpleFOCStudio.py](/e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master/simpleFOCStudio.py)。

在原版 `SimpleFOCStudio` 基础上，这个分支已经做了这些增强：

- 默认打开树形视图
- 启动时尝试自动选择串口
- 默认命令 ID 为 `M`
- 左侧点动区支持手动输入和应用
- 从动力矩模式支持中文参数编辑
- 数字状态显示区支持刷新率设置
- 实时图表支持清空和窗口长度设置
- 顶部增加 `PID自动测量` 页面

### 从动力矩模式可调参数

点动区当前可直接设置：

- 目标阻尼力矩 `[Nm]`
- 饱和磁场阻尼角 `[deg]`
- 跟随死区 `[deg]`
- 旋转工况阈值 `[rad/s]`
- 计算频率 `[Hz]`
- 低速/静止跟随 PID `P / I / D`
- 旋转工况跟随 PID `P / I / D`

数字状态显示区会显示当前磁场阻尼角。

## 从动力矩控制逻辑

“从动力矩控制”是本项目自定义模式，不是原版 SimpleFOC 的标准运动模式。

当前控制逻辑是：

- 电机保持 `FOC` 闭环与传感器采样
- 用“磁场参考角跟随机械角”的方式形成阻尼
- 跟随误差先经过跟随死区
- 再按工况选择两组跟随 PID 之一：
  - 低速/静止工况 PID
  - 正常旋转工况 PID
- 最终得到磁场阻尼角
- 磁场阻尼角再通过二阶关系映射到目标阻尼力矩
- 目标阻尼力矩换算为 `Iq` 并受 `current_limit` 限制

工况切换依据：

- 当机械角速度绝对值小于旋转工况阈值时，使用低速/静止跟随 PID
- 当机械角速度绝对值大于等于旋转工况阈值时，使用旋转工况跟随 PID

## PID 自动测量

顶部工具栏包含 `PID自动测量` 页面，文件位于 [pidAutoTuneTool.py](/e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master/src/gui/pidAutoTuneTool.py)。

当前支持这些测量项：

- 电流 Q PID
- 电流 D PID
- 速度 PID
- 角度 PID
- 从动力矩跟随 PID（低速/静止）
- 从动力矩跟随 PID（正常旋转）

### 自动测量页面功能

- 每个 PID 项都可以单独勾选是否启用
- 每个 PID 项都可以分别勾选是否调 `P / I / D`
- 可设置最大迭代次数
- 可设置阻尼力矩终值
- 可设置阻尼力矩扫描挡数，默认 `10` 挡
- 可设置旋转工况阈值 `[rad/s]`

### 从动力矩跟随 PID 测量逻辑

- 低速/静止工况与正常旋转工况分开测
- 低速/静止工况下，程序直接进行阻尼力矩扫描和偏置衰减测量
- 正常旋转工况下，程序会先提示你手动旋转电机
- 检测到电机速度超过“旋转工况阈值”并持续一小段时间后，才开始测量
- 每一挡都会执行一次磁场偏置扰动和衰减响应采样
- 再按迭代逻辑分别优化你勾选的 `P / I / D`

说明：

- 电流 D PID 目前仍使用 Q 轴响应做代理估计
- 自动测量过程中会临时接管控制模式，并在结束后恢复原状态

## 编译与运行

### 1. 编译固件

在仓库根目录运行：

```powershell
pio run
```

### 2. 烧录固件

```powershell
pio run -t upload
```

### 3. 运行上位机

```powershell
python .\SimpleFOCStudio-zh-master\simpleFOCStudio.py
```

如果 Python 没有加入环境变量，也可以直接使用完整路径：

```powershell
D:/python/python.exe "e:/Projects/PIO/Deng FOC M1/SimpleFOCStudio-zh-master/simpleFOCStudio.py"
```

## 连接流程

1. 给控制板上电并连接串口
2. 打开上位机
3. 直接点击“连接”
4. 如果自动选择的串口不正确，再手动点击“设置”调整

默认连接参数：

- 串口波特率：`115200`
- 命令 ID：`M`

## 串口命令对照表

本项目上位机与固件通信时，实际发送到串口的数据格式为：

```text
<命令ID><子命令><参数>
```

当前默认命令 ID 为 `M`，所以常见命令看起来会像：

```text
MC
MVP0.021
MZT0.05
```

说明：

- 当命令不带参数时，表示“查询当前值”
- 当命令带参数时，表示“设置并回显当前值”
- 下表列的是当前项目里上位机实际会用到的命令，不是 SimpleFOC Commander 的完整全集

### 标准 Commander 命令

| 命令 | 含义 | 说明 |
| --- | --- | --- |
| `MC` | 运动控制类型 | 查询/设置 `torque`、`velocity`、`angle`、`velocity_openloop`、`angle_openloop` |
| `MT` | 力矩控制类型 | 查询/设置 `voltage`、`dc current`、`foc current` |
| `MCD` | 运动控制频率降采样 | 查询/设置 `motion_downsample` |
| `MVP` / `MVI` / `MVD` | 速度环 PID `P/I/D` | 对应 `PID_velocity` |
| `MVR` / `MVL` | 速度环输出斜率 / 输出限制 | 对应 `PID_velocity.output_ramp / limit` |
| `MVF` | 速度环低通滤波时间常数 | 对应 `LPF_velocity.Tf` |
| `MAP` / `MAI` / `MAD` | 角度环 PID `P/I/D` | 对应 `P_angle` |
| `MAR` / `MAL` | 角度环输出斜率 / 输出限制 | 对应 `P_angle.output_ramp / limit` |
| `MAF` | 角度环低通滤波时间常数 | 对应 `LPF_angle.Tf` |
| `MQP` / `MQI` / `MQD` | 电流 Q 环 PID `P/I/D` | 对应 `PID_current_q` |
| `MQR` / `MQL` | 电流 Q 环输出斜率 / 输出限制 | 对应 `PID_current_q.output_ramp / limit` |
| `MQF` | 电流 Q 环低通滤波时间常数 | 对应 `LPF_current_q.Tf` |
| `MDP` / `MDI` / `MDD` | 电流 D 环 PID `P/I/D` | 对应 `PID_current_d` |
| `MDR` / `MDL` | 电流 D 环输出斜率 / 输出限制 | 对应 `PID_current_d.output_ramp / limit` |
| `MDF` | 电流 D 环低通滤波时间常数 | 对应 `LPF_current_d.Tf` |
| `MLV` | 速度限制 | 对应 `motor.velocity_limit` |
| `MLU` | 电压限制 | 对应 `motor.voltage_limit` |
| `MLC` | 电流限制 | 对应 `motor.current_limit` |
| `MR` | 相电阻 | 对应 `motor.phase_resistance` |
| `M` | 目标值 | 直接写 `M<value>`，对应 `motor.target` |
| `MSM` | 机械零点 | 对应 `sensor offset` |
| `MSE` | 电角零点 | 对应 `sensor electrical offset` |
| `ME` | 设备使能状态 | `1=enable`，`0=disable` |
| `MWC` | PWM centered | 对应 `motor.modulation_centered` |
| `MWT` | PWM modulation type | 对应 `motor.foc_modulation` |
| `MMD` | 监控降采样 | 对应 `monitor_downsample` |
| `MMC` | 清空监控变量 | 对应 `monitor clear` |
| `MMS` | 设置监控变量位图 | 例如 `1111111` |
| `MMG7` | 查询全部运行状态 | 返回目标、电压、电流、速度、角度等 |

### 项目自定义命令

| 命令 | 含义 | 说明 |
| --- | --- | --- |
| `MX` | 释放模式 | `1=释放`，`0=退出释放` |
| `MZM` | 从动力矩模式 | `1=进入从动力矩`，`0=退出从动力矩` |
| `MZT` | 目标阻尼力矩 `[Nm]` | 查询/设置当前阻尼力矩目标 |
| `MZA` | 饱和磁场阻尼角 `[deg]` | 查询/设置阻尼角饱和尺度 |
| `MZH` | 跟随死区 `[deg]` | 查询/设置磁场方向跟随死区 |
| `MZR` | 旋转工况阈值 `[rad/s]` | 查询/设置低速/旋转工况切换阈值 |
| `MZF` | 计算频率 `[Hz]` | 查询/设置从动力矩控制外层计算频率 |
| `MZP` / `MZI` / `MZD` | 低速/静止跟随 PID `P/I/D` | 查询/设置低速工况跟随 PID |
| `MZJ` / `MZK` / `MZL` | 旋转工况跟随 PID `P/I/D` | 查询/设置正常旋转工况跟随 PID |
| `MZO` | 磁场参考角偏置 `[deg]` | 自动测量时用来注入扰动 |
| `MZG` | 磁场阻尼角 `[deg]` | 查询当前阻尼角 |
| `MZY` | 从动力矩调试日志开关 | `1=开启日志`，`0=关闭日志` |

### 常见查询示例

```text
MC
MT
MLC
MCD
MZT
MZA
MZH
MZR
MZF
MZP
MZJ
MZG
MMG7
```

### 常见设置示例

```text
ME1
MC2
MT2
MLC1.8
MCD0
MZT0.05
MZA2.0
MZH0.8
MZR2.0
MZF1000
MZP0.0864
MZJ0.0400
MZM1
MX1
```

## 常用文件

- 固件入口：[src/main.cpp](/e:/Projects/PIO/Deng%20FOC%20M1/src/main.cpp)
- 固件主逻辑：[src/motor_control_app.cpp](/e:/Projects/PIO/Deng%20FOC%20M1/src/motor_control_app.cpp)
- 固件配置结构：[src/motor_control_app.h](/e:/Projects/PIO/Deng%20FOC%20M1/src/motor_control_app.h)
- 上位机通信层：[simpleFOCConnector.py](/e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master/src/simpleFOCConnector.py)
- 点动控制：[deviceJoggingControl.py](/e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master/src/gui/configtool/deviceJoggingControl.py)
- PID 自动测量页：[pidAutoTuneTool.py](/e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master/src/gui/pidAutoTuneTool.py)

## 备注

- 仓库里部分旧文件仍有历史编码问题，终端中可能看到乱码，但不影响主要功能
- 如果修改了固件自定义命令或参数结构，记得同步更新上位机通信层
