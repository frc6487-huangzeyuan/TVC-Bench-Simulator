# TVC 双轴台架仿真环境项目说明

> 本文档用于让新的 Agent 直接接手项目。执行时以当前仓库、用户配置的 `CAD_ROOT` 中的 CAD 模型和最新指令为准；不要把本文中的待确认项当成已验证事实。

## 1. 项目目标

为现有 TVC（Thrust Vector Control）双轴轴承台架搭建一个本地仿真与飞控测试环境，重点验证姿态稳定算法，而不是完整火箭自由飞行。

第一阶段目标：

- 模拟一个被双轴轴承约束的箭体，可绕两个互相垂直的转轴转动。
- 箭体主体为 PETG 打印件；台架主体为铝型材。
- 根据已有 SolidWorks 装配建立可用于动力学仿真的 URDF/等价刚体模型。
- 支持双轴 TVC 执行机构、舵机动态、IMU 传感器和外部扰动。
- 能加载/接入用户的飞控代码，并在仿真中观察姿态、目标量、执行器输出和稳定效果。
- UI 要现代、简洁、易读，适合持续调参和观察实时数据，而不是传统工程软件式的大量灰色控件。

本项目当前不以六自由度火箭飞行为主要目标。后续可以在双轴台架模型稳定后增加自由飞行模式。

## 2. 当前真实项目状态

当前仓库根目录：

```text
<repo-root>
```

目前仓库中至少有：

```text
README.md
TVC.ino
```

`TVC.ino` 是现有的 ESP32-S3 + MPU6050 单轴控制原型，当前逻辑为 X 轴姿态补偿：

- ESP32-S3-N16R8。
- MPU6050：SDA GPIO12，SCL GPIO11。
- 单舵机信号：GPIO1。
- 控制/采样周期：200 Hz（`SAMPLE_PERIOD_US = 5000`）。
- 串口遥测：10 Hz，115200 baud。
- 当前姿态估计：X 轴陀螺仪积分 + 重力倾角互补滤波。
- 当前控制：相对零位角度经过死区和比例增益后直接生成舵机角度。
- 默认舵机中位 90°，限制 45°～135°。
- 当前 `servo_cmd_deg` 是命令角，不是舵机真实反馈位置。

因此当前代码是“单轴原型”，不能把它描述成已经完成双轴飞控。仿真环境应先兼容它，再为双轴控制留好接口。

## 3. CAD / 机械模型来源

SolidWorks 模型目录通过 `CAD_ROOT` 配置，例如：

```text
CAD_ROOT=<path-to-solidworks-projects>
```

机械结构是一个简单的双轴轴承台架。建模时应先读取真实 SolidWorks 装配和零件，不要凭想象重新设计结构。

已知 CAD 文件命名存在引用变化：

- 总装中原先引用的 `01-45扭力单轴舵机.SLDPRT`，用户确认实际应替换为 `机械部分\01-单轴舵机.SLDPRT`。
- 另有一个“金属舵盘”存在同类改名/替换情况，需要在 `CAD_ROOT` 中根据装配引用和实际文件确认对应文件名后修复，不要猜文件名。

材料信息：

| 部件 | 材料/处理方式 |
| --- | --- |
| 箭体主体 | PETG 3D 打印 |
| 台架 | 铝型材 |
| 舵机/轴承/金属舵盘 | 按实际 CAD 或实物参数 |

从 SolidWorks 转换到 URDF 时，重点不是外观，而是：质量、质心、惯量张量、两根转轴的位置/方向、关节限位、TVC 推力作用点和方向。

## 4. 建议的物理模型

第一版使用“受约束双轴刚体模型”：

```text
world
  └─ stand_base          固定铝型材台架
      └─ outer_gimbal    外框，第 1 转轴
          └─ inner_gimbal / rocket_body
              └─ TVC / motor assembly，第 2 转轴或推力矢量机构
```

至少需要两个旋转自由度：

- pitch 轴
- yaw 轴

具体哪个 CAD 轴对应 pitch/yaw，必须通过真实装配坐标确认。

动力学至少包含：

\[
I\dot{\omega}=\tau_{tvc}+\tau_g+\tau_f+\tau_d
\]

其中：

- `I`：绕当前台架转轴的等效转动惯量。
- `τ_tvc`：TVC 推力产生的力矩。
- `τ_g`：重力对偏心质心产生的力矩。
- `τ_f`：轴承摩擦、阻尼等力矩。
- `τ_d`：用户施加的扰动力矩。

TVC 力矩应根据推力矢量和推力作用点计算，而不是直接把舵机角度当成姿态力矩：

\[
\tau_{tvc}=r\times F
\]

第一版可采用可调常推力；之后再加入推力曲线。

## 5. 执行器模型

舵机模型至少包括：

- 目标角 `command_deg`
- 实际角 `actual_deg`
- 最大角速度
- 角度限位
- 可选死区
- 可选一阶响应延迟
- 后续可加入回差和饱和

不要让仿真中的舵机瞬间到达目标角，否则会高估控制器性能。

建议第一版使用：

\[
\dot{\delta}=\operatorname{clamp}\left(\frac{\delta_{cmd}-\delta}{T_s},-\omega_{max},\omega_{max}\right)
\]

所有参数放到独立配置文件或 UI 参数面板中，禁止散落在渲染代码里。

## 6. IMU / 传感器模型

为了测试现有飞控，需要模拟与 MPU6050 对应的数据接口：

- 三轴陀螺仪 `gx, gy, gz`，单位 deg/s 或 rad/s，但整个链路必须统一。
- 三轴加速度计 `ax, ay, az`，建议内部 SI 单位，给兼容层时再换成 g。
- 可配置零偏。
- 可配置高斯噪声。
- 可配置采样率。
- 后续增加低频漂移、延迟和丢包。

默认第一版的“理想 IMU”也要保留噪声开关，便于区分控制器问题和传感器问题。

## 7. 飞控代码接入设计

核心要求是“能读取和测试真实飞控代码”，不要只做一个写死 PID 的演示器。

建议把接入分为三层：

### A. Controller API（统一仿真接口）

仿真核心只认识一个稳定接口，例如：

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

这样物理引擎、UI 和具体飞控语言解耦。

### B. SIL：本机运行飞控逻辑

优先支持当前 `TVC.ino` 的控制逻辑。不要试图完整模拟 ESP32 外设和 Arduino 运行时作为第一步。

更可维护的做法是：

1. 将姿态估计和控制律抽成无硬件依赖的 C/C++ 控制核心。
2. `TVC.ino` 继续负责真实 `Wire`、`Servo`、`Serial`。
3. 仿真侧使用同一个控制核心，通过 native DLL/进程接口运行。
4. 用模拟 IMU 输入代替 `readMotion()`，捕获舵机输出代替 `servo.write()`。

若用户直接选择一个 `.ino/.cpp` 文件，UI 应能显示：

- 文件路径
- 编译/加载状态
- 检测到的可调参数
- 控制循环频率
- 编译错误/运行错误
- 当前输出通道

可以后续增加 Arduino 兼容 shim，让部分 `setup()/loop()` 代码无需人工重构即可运行，但不要把“任意 Arduino 工程直接执行”当作第一版承诺。

### C. HIL：真实 ESP32 接入

后续增加串口 HIL：

- 仿真发送 IMU/姿态数据给真实控制器。
- ESP32 返回舵机命令和调试数据。
- 仿真继续完成刚体动力学。

HIL 必须与 SIL 共用 `ControllerInput/ControllerOutput` 数据模型。

## 8. 推荐软件架构

第一版推荐保持本地、模块化、易调试：

```text
TVC Simulator
├─ ui/                 React + TypeScript
├─ renderer/           Three.js / React Three Fiber
├─ sim/                Python 或 C++ 动力学核心
├─ controllers/        飞控适配层
├─ models/             URDF、mesh、惯量和材料配置
├─ telemetry/          时序缓存、记录、导出
├─ configs/            台架、执行器、传感器参数
└─ tools/              CAD/URDF 转换与校验工具
```

桌面壳优先考虑 Tauri；若当前实现已经使用 Web 本地服务，也可以先保留浏览器模式，等核心功能稳定后再封装桌面应用。

物理引擎第一版优先选择对 URDF、关节和外力支持成熟的方案，例如 PyBullet；若后续需要更高质量接触/约束和参数辨识，可评估 MuJoCo。选择后不要同时维护两个物理后端。

## 9. UI 设计要求

界面目标：现代、简洁、实时、适合工程调试。

推荐布局：

```text
┌──────────────────────────────────────────────────────────┐
│ Project / Run / Flight Code / Connection / Status        │
├───────────────────────────────┬──────────────────────────┤
│                               │ Attitude / IMU           │
│       3D 双轴台架视图          │ Servo Pitch / Yaw        │
│                               │ Controller State         │
│                               │ Tunable Parameters       │
├───────────────────────────────┴──────────────────────────┤
│ 实时时序图：pitch / yaw / target / servo / gyro / accel  │
├──────────────────────────────────────────────────────────┤
│ Timeline | disturbance | reset | pause | x0.5 x1 x2      │
└──────────────────────────────────────────────────────────┘
```

视觉原则：

- 深色或自动浅/深主题。
- 大面积 3D 视图，不让控制面板挤压模型。
- 参数使用分组卡片、数字输入、slider 和 unit label。
- 重要状态用小型 status chip，不用大块日志占据主界面。
- 图表支持选择信号、缩放、暂停、游标查看。
- 3D 中显示世界坐标轴、两根台架旋转轴、推力矢量和目标姿态。
- 支持一键 Reset、Pause、Step、Run。
- UI 中必须区分“命令舵机角”和“实际舵机角”。

## 10. 场景与测试功能

MVP 至少包含以下测试：

1. **初始角度恢复**：给定 pitch/yaw 初始偏角，观察能否回到目标角。
2. **阶跃目标**：目标姿态从 0° 切到指定小角度。
3. **脉冲扰动**：运行中施加短时外力矩。
4. **持续偏置扰动**：模拟重心偏差或线缆拉力。
5. **传感器噪声**：逐步增加 gyro/accel 噪声。
6. **舵机速度限制**：检查控制器在真实执行器带宽下是否仍稳定。
7. **参数扫描**：至少支持手动快速修改增益；批量扫描可后续实现。

每次运行应能保存：

- 仿真配置快照
- 飞控代码版本/文件 hash
- 时间序列遥测
- 稳定时间、最大超调、稳态误差等指标

## 11. URDF / 模型转换要求

转换 CAD 时：

1. 先解析总装配层级和 mate/轴关系。
2. 修复已确认的舵机、金属舵盘文件引用。
3. 将视觉 mesh 与碰撞 mesh 分开；碰撞体尽量简化。
4. 坐标系统一，建议：X 前、Y 左、Z 上；若现有模型使用其他坐标，必须在导出层明确转换。
5. 从 CAD/材料信息计算或提取质量、质心和惯量。
6. 对 PETG 打印件，实际填充率和内部结构会影响质量/惯量；CAD 理论值只能作为初始值，应允许用户在配置中覆盖。
7. 两个主转轴使用 revolute joint，并写入真实角度限制。
8. 仿真启动时检查 URDF 是否存在惯量为零、轴方向错误、关节循环或 mesh 路径失效。

## 12. 数据与单位约定

仿真核心统一采用 SI：

| 量 | 内部单位 |
| --- | --- |
| 长度 | m |
| 质量 | kg |
| 时间 | s |
| 角度 | rad |
| 角速度 | rad/s |
| 力 | N |
| 力矩 | N·m |

UI 可显示 °、°/s；飞控兼容层负责与当前 `TVC.ino` 的 deg/s、deg、g 互相转换。

## 13. MVP 完成标准

只有同时满足以下条件，第一版才算完成：

- 可以启动本地仿真并看到真实双轴台架/箭体模型。
- 两个机械转轴运动方向和 CAD 一致。
- 能设置初始 pitch/yaw 并运行物理仿真。
- 可以显示实际姿态、角速度、TVC/舵机状态。
- 能加载一个控制器适配器并闭环控制至少一个轴。
- 当前 `TVC.ino` 的单轴控制逻辑能通过 SIL 兼容方式在仿真中运行，或明确列出尚未完成的具体兼容点。
- 可施加扰动并在图表中看到响应。
- Reset/Pause/Run 正常。
- UI 不依赖手工修改源码才能调整常用仿真参数。
- 仿真参数、控制器文件和遥测可以保存/导出。

## 14. 后续 Agent 的建议执行顺序

接手后按以下顺序推进，不要先花大量时间做 UI 动画：

1. 读取当前仓库的 `README.md`、`TVC.ino` 和 Git 状态。
2. 检查 `CAD_ROOT` 中真实 SolidWorks 总装、零件、引用关系和单位。
3. 确认双轴机械结构、轴方向、限位、箭体质心和 TVC 推力作用点。
4. 修复装配中已确认的舵机引用和金属舵盘引用。
5. 输出最小 URDF，并用简单 viewer 验证关节轴和姿态。
6. 实现双轴刚体动力学、舵机和 IMU 模型。
7. 定义 `ControllerInput/ControllerOutput`。
8. 将 `TVC.ino` 中可复用控制逻辑抽成硬件无关控制核心，并让原固件和 SIL 共用。
9. 实现实时遥测和测试场景。
10. 再完成现代 UI、3D 表现、代码加载面板和参数调节。
11. 最后增加串口 HIL。

每一步都要用实际运行结果验证。URDF 能加载不代表轴方向正确；程序能编译不代表闭环稳定；串口能连接不代表 HIL 数据语义正确。

## 15. 当前明确已知 / 待确认参数

### 已知

```yaml
mechanism: dual-axis bearing test stand
rocket_body_material: PETG
stand_material: aluminum extrusion
cad_root: configured externally
firmware: ./TVC.ino
current_controller_axes: 1
target_simulation_axes: 2
current_imu: MPU6050
current_control_rate_hz: 200
current_telemetry_rate_hz: 10
current_servo_center_deg: 90
current_servo_limit_deg: [45, 135]
```

### 必须从 CAD / 实物 / 用户配置中确认

```yaml
pitch_axis_origin: unknown
pitch_axis_direction: unknown
yaw_axis_origin: unknown
yaw_axis_direction: unknown
joint_limits: unknown
rocket_mass_kg: unknown
rocket_center_of_mass: unknown
rocket_inertia: unknown
tvc_pivot_position: unknown
thrust_application_point: unknown
thrust_newton: unknown
servo_max_speed: unknown
servo_latency: unknown
servo_backlash: unknown
bearing_friction: unknown
imu_mount_transform: unknown
second_servo_pin_and_direction: unknown
```

不要为这些 `unknown` 参数编造默认“真实值”。为了开发可以设置明确标记的临时仿真默认值，但必须集中在配置文件中，并在 UI 中显示为可修改参数。

## 16. 项目边界

- 第一阶段服务于现有双轴台架姿态稳定算法测试。
- 不先做完整空气动力学和六自由度飞行。
- 不把漂亮 3D 动画当作物理模型正确的证明。
- 不重写一套与真实固件完全无关的控制器来冒充“飞控代码测试”。
- 不直接修改真实飞控硬件行为来迁就仿真；应通过适配层实现兼容。
- 若需要修改 `TVC.ino`，应保持实机版本仍可编译，并说明仿真接口与硬件接口如何共用。

---

这份文档应作为项目上下文基线。新的 Agent 在开始实现前需要重新读取实际代码、CAD 和当前目录状态，并在发现与本文不一致时以当前真实环境和用户最新要求为准。
