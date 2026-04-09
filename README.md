# Deng FOC M1

基于 `ESP32 + SimpleFOC 2.2.1` 的电机控制项目，包含两部分：

- 固件工程：位于根目录，使用 `PlatformIO`
- 上位机工程：位于 [SimpleFOCStudio-zh-master](e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master)，基于 `PyQt5`

当前项目已经针对本机电机和调试流程做了较多定制，尤其是“从动力矩控制”模式、树形视图调参、PID 自动测量等功能。

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

固件运行在 `WEMOS LOLIN32 Lite` 上，配置见 [platformio.ini](e:/Projects/PIO/Deng%20FOC%20M1/platformio.ini)。

当前硬件和默认参数集中写在 [src/main.cpp](e:/Projects/PIO/Deng%20FOC%20M1/src/main.cpp) 的常量区，`main.cpp` 只保留初始化参数和入口，主要控制逻辑在：

- [src/motor_control_app.h](e:/Projects/PIO/Deng%20FOC%20M1/src/motor_control_app.h)
- [src/motor_control_app.cpp](e:/Projects/PIO/Deng%20FOC%20M1/src/motor_control_app.cpp)

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
- I2C 编码器读取失败自动恢复
- 电流采样与 FOC 电流控制
- 自定义“释放”模式
- 自定义“从动力矩控制”模式
- 从动力矩模式支持以下参数：
  - 目标阻尼力矩
  - 饱和磁场阻尼角
  - 跟随死区
  - 计算频率
  - 跟随 PID `P / I / D`

## 上位机说明

上位机入口文件是 [SimpleFOCStudio-zh-master/simpleFOCStudio.py](e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master/simpleFOCStudio.py)。

在原版 `SimpleFOCStudio` 基础上，这个分支已经做了较多增强：

- 默认打开树形视图
- 启动时尝试自动选择串口
- 默认命令 ID 为 `M`
- 左侧点动控制支持手动输入和应用
- 从动力矩模式支持中文参数编辑
- 数字状态显示区支持刷新率设置
- 实时图表支持清空和窗口长度设置
- 顶部增加 `PID自动测量` 页面
- PID 自动测量支持：
  - 选择要测的 PID 项
  - 分别勾选是否调 `P / I / D`
  - 设置最大迭代次数
  - 从动力矩跟随 PID 按目标阻尼力矩 `20` 挡步进进行测试

## 编译与运行

### 1. 编译固件

在项目根目录运行：

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

如果你的 Python 不在环境变量里，也可以直接使用完整路径：

```powershell
D:/python/python.exe "e:/Projects/PIO/Deng FOC M1/SimpleFOCStudio-zh-master/simpleFOCStudio.py"
```

## 连接流程

1. 给控制板上电并连接串口
2. 打开上位机
3. 直接点击“连接”
4. 如自动串口选择不正确，再手动点“设置”调整

默认连接参数：

- 串口波特率：`115200`
- 命令 ID：`M`

## 从动力矩控制

“从动力矩控制”是本项目自定义模式，不是原版 SimpleFOC 标准模式。

它的核心思路是：

- 电机保持 FOC 闭环和传感器采样
- 通过“磁场参考角跟随 + 阻尼角映射 + 跟随 PID”形成阻尼感
- 同时保留跟随死区，降低低速抖动

上位机可调参数包括：

- 目标阻尼力矩 `[Nm]`
- 饱和磁场阻尼角 `[deg]`
- 跟随死区 `[deg]`
- 计算频率 `[Hz]`
- 跟随 PID `P / I / D`

数字状态显示区可以显示当前磁场阻尼角。

## PID 自动测量

顶部工具栏包含 `PID自动测量` 入口。

当前支持：

- 电流 Q PID
- 电流 D PID
- 速度 PID
- 角度 PID
- 从动力矩跟随 PID

说明：

- 每个 PID 项都可以单独勾选是否启用，以及是否调 `P / I / D`
- 可以设置最大迭代次数
- 从动力矩跟随 PID 测量时，目标阻尼力矩按 `20` 挡步进扫描

## 常用文件

- 固件入口：[src/main.cpp](e:/Projects/PIO/Deng%20FOC%20M1/src/main.cpp)
- 固件主逻辑：[src/motor_control_app.cpp](e:/Projects/PIO/Deng%20FOC%20M1/src/motor_control_app.cpp)
- 固件配置结构：[src/motor_control_app.h](e:/Projects/PIO/Deng%20FOC%20M1/src/motor_control_app.h)
- 上位机通信层：[SimpleFOCStudio-zh-master/src/simpleFOCConnector.py](e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master/src/simpleFOCConnector.py)
- 树形视图页：[SimpleFOCStudio-zh-master/src/gui/configtool/treeViewConfigTool.py](e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master/src/gui/configtool/treeViewConfigTool.py)
- 点动控制：[SimpleFOCStudio-zh-master/src/gui/configtool/deviceJoggingControl.py](e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master/src/gui/configtool/deviceJoggingControl.py)
- PID 自动测量页：[SimpleFOCStudio-zh-master/src/gui/pidAutoTuneTool.py](e:/Projects/PIO/Deng%20FOC%20M1/SimpleFOCStudio-zh-master/src/gui/pidAutoTuneTool.py)

## 备注

- 项目中部分旧文件仍存在历史编码问题，终端里可能看到乱码，但不影响主要功能。
- 如果修改了固件自定义命令或参数结构，记得同时同步上位机通信层。
