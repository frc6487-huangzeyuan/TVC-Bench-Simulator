# TVC Bench Simulator

一个面向 **双轴 TVC（Thrust Vector Control）轴承台架** 的本地仿真与飞控测试项目。

项目目标是把真实机械结构、执行器、IMU 与飞控代码放进同一个可重复测试的仿真环境中，用于姿态稳定算法开发，并逐步支持 SIL（Software-in-the-Loop）和 HIL（Hardware-in-the-Loop）。

> 当前仓库处于早期开发阶段。现阶段包含 ESP32-S3 + MPU6050 单轴飞控原型，以及双轴仿真环境的完整设计说明；3D/动力学仿真器正在按该架构实现。

## 目标

- 模拟一个由双轴轴承约束的 TVC 台架，而不是先做完整六自由度火箭飞行。
- 从真实 CAD 装配生成 URDF / 刚体模型，并保留质量、质心、惯量和关节轴信息。
- 建模 TVC 推力、舵机速度/延迟、轴承阻尼、重力矩和外部扰动。
- 模拟 MPU6050 陀螺仪与加速度计数据，并支持噪声、零偏和采样率配置。
- 通过统一控制器接口运行真实飞控逻辑，而不是只在仿真器里写一套独立 PID。
- 提供现代化实时 UI：3D 台架、姿态、推力矢量、舵机状态、参数调节和时序曲线。
- 后续支持真实 ESP32 串口 HIL。

## 当前内容

### `TVC.ino`

当前硬件控制原型：

- MCU：ESP32-S3-N16R8
- IMU：MPU6050
- 当前控制轴：1 轴（X / roll 原型）
- 控制频率：200 Hz
- 遥测频率：10 Hz
- 姿态估计：陀螺积分 + 加速度计重力倾角互补滤波
- 舵机输出：角度死区 + 比例补偿
- 默认舵机中位：90°
- 默认角度限制：45°～135°

当前代码用于验证传感器读取、姿态估计和舵机补偿链路。它还不是最终双轴飞控。

### `TVC_Simulator_Project_Guide.md`

详细项目设计文档，包含：

- 双轴机械模型与动力学约定
- CAD → URDF 转换要求
- TVC 力矩模型
- 舵机动态模型
- IMU 仿真模型
- 飞控代码 SIL / HIL 接入方案
- 推荐软件架构
- UI 布局与交互要求
- MVP 测试场景与验收标准

详见 [TVC_Simulator_Project_Guide.md](./TVC_Simulator_Project_Guide.md)。

## 仿真架构

计划采用以下分层：

```text
TVC Bench Simulator
├─ ui/                 React + TypeScript
├─ renderer/           Three.js / React Three Fiber
├─ sim/                刚体动力学与约束
├─ controllers/        飞控适配层 / SIL / HIL
├─ models/             URDF、mesh、惯量与材料参数
├─ telemetry/          实时数据、记录与导出
├─ configs/            台架、IMU、执行器配置
└─ tools/              CAD / URDF 转换与校验工具
```

仿真核心统一使用 SI 单位，UI 和飞控兼容层负责角度、角速度与 `g` 等单位转换。

## 飞控代码接入

仿真器会使用统一的数据接口：

```text
ControllerInput
  time_s
  dt_s
  imu.accel
  imu.gyro
  target_attitude

ControllerOutput
  servo_pitch_deg
  servo_yaw_deg
  debug_channels...
```

第一阶段计划将 `TVC.ino` 中的姿态估计和控制律抽成无硬件依赖的 C/C++ 控制核心，使真实 ESP32 固件与仿真 SIL 共用同一套算法。

之后再增加串口 HIL，让真实 ESP32 接收模拟 IMU 数据并返回舵机指令。

## 机械模型

目标机构是一个简单的双轴轴承台架：

```text
world
  └─ stand_base
      └─ outer_gimbal
          └─ inner_gimbal / rocket_body
              └─ TVC assembly
```

箭体主体使用 PETG 打印，台架主体使用铝型材。仿真模型将以实际 CAD 的质量、质心、惯量、转轴位置和 TVC 推力作用点为准。

## 第一阶段测试场景

1. 初始姿态偏角恢复
2. 目标姿态阶跃
3. 短时外部扰动力矩
4. 持续偏置扰动
5. IMU 噪声和零偏
6. 舵机速度与延迟限制
7. 控制参数快速调节

每次运行计划记录配置、控制器版本、完整遥测，以及稳定时间、最大超调和稳态误差等指标。

## 开发状态

- [x] ESP32-S3 + MPU6050 单轴飞控原型
- [x] 双轴台架仿真需求与架构设计
- [ ] CAD / URDF 模型整理
- [ ] 双轴刚体动力学
- [ ] 舵机与 IMU 仿真
- [ ] SIL 控制器接口
- [ ] 现代化 3D UI 与实时遥测
- [ ] 扰动与测试场景
- [ ] ESP32 HIL

## 硬件原型

当前原型的默认连接：

| 部件 | ESP32-S3 |
| --- | --- |
| 舵机信号 | GPIO1 |
| MPU6050 SCL | GPIO11 |
| MPU6050 SDA | GPIO12 |

MPU6050 使用 3.3 V；舵机使用符合型号规格的独立电源，并与 ESP32 共地。

Arduino 侧依赖：

- `esp32 by Espressif Systems`
- `ESP32Servo`
- Arduino `Wire`

## 项目原则

- 真实机械参数优先于视觉效果。
- 编译成功不等于闭环控制已经在实物上验证。
- URDF 能加载不等于关节轴、惯量和质心正确。
- 仿真控制器应尽量复用真实飞控算法。
- 未测得的机械参数应明确标为可配置值，不伪装成真实测量数据。

## Roadmap

近期工作重点是完成 CAD → URDF、双轴物理模型和 SIL 控制器接口。物理链路验证稳定后，再继续完善 UI 和 HIL。
