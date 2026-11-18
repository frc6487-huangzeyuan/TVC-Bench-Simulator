# TVC Studio Agent Guide

- 「真实飞控 → 飞控操作 → 可动重心」通过 `cg_servo_*` 命令控制 GPIO7 位置舵机。按住正转或反转时定期续发，松开保持当前指令角。协议 10 的 `outputs.cg_servo` 为可选字段，旧固件保持可连接并禁用新按钮。实际重心位置没有传感器反馈。

本文件适用于整个 `TVC-Simulation` 仓库。目标是让新的 Agent 在不重新猜测架构、不破坏真实飞控代码和本地运行数据的前提下，直接继续开发和验证。

## 1. 先确认当前事实

- 当前产品是 **TVC Studio 0.1.0**，用于双轴轴承台架上的 TVC 姿态稳定仿真，不是六自由度火箭自由飞行模拟器。
- 当前实现采用 **FastAPI + MuJoCo** 后端，**React + TypeScript + Three.js + Recharts + CodeMirror** 前端，本地地址为 `http://127.0.0.1:18767`。
- `README.md` 和实际源码描述当前实现；`TVC_Simulator_Project_Guide.md` 保留项目目标与早期设计背景，其中的“建议方案”不能覆盖已经实现的架构。
- 修改前先读与任务直接相关的源码、`README.md`、当前 Git 状态和已有报告，不根据旧文档或文件名猜运行逻辑。

## 2. 目录与职责

```text
app/                    FastAPI、工程扫描/编译、MuJoCo 仿真、运行管理
controllers/            sim_api v1、当前 TVC 适配器、C++ 示例控制器
frontend/               React/Vite 前端
mjcf/                   MuJoCo 主模型
urdf/                    可移植 URDF
meshes/                  从 CAD 转换得到的 STL 网格
tools/                   模型构建、验证、Studio 集成验证、启动器工具
reports/                 模型/Studio 验证报告、预览图、日志
runs/                    单次实验的参数、摘要、遥测和 CSV
work/                    本地工程状态、编译快照和运行时数据
backups/                 已记录的 CAD 引用修复备份
run_studio.py            Studio 后端启动入口
preview.py               模型预览入口
mass_parameters.json     材料、质量与称重覆盖参数
```

不要把 `.venv/`、`frontend/node_modules/`、`frontend/dist/`、`work/browser-profile/`、`work/controllers/` 加入 Git。`work/browser-profile/` 可能包含 Cookies、登录状态和其他隐私数据；`work/controllers/` 是外部飞控源码快照和编译产物。

## 3. 物理与坐标约定

- 内部统一使用米、千克、秒、弧度；控制器兼容层按 `controllers/sim_api.h` 使用秒、度/秒、g、微秒。
- 世界坐标 Z 向上。当前 CAD 到世界的映射、刚体质心和惯量以 `reports/model_metadata.json` 与模型文件为准。
- 台架结构是两条互相垂直的被动轴承轴，箭体安装在内框；TVC 机构再包含方位与俯仰两个关节。
- 当前物理步长为 1 ms，即 **1000 Hz**；模拟 IMU 为 **200 Hz**；控制循环为 **100 Hz**。
- TVC 推力必须在 `thrust_point` 沿真实风扇轴方向施加，通过力和力臂自然产生力矩。不要把 PID 输出直接写成理想恢复力矩。
- 舵机必须保留速度、扭矩和一阶响应约束；不要改成瞬时到位。
- 质量、推力曲线、舵机扭矩、响应、阻尼、噪声和安装零位仍包含未实物标定参数。不能把仿真调通描述成实物参数已经验证。

## 4. 飞控代码接入边界

仿真核心只通过 `controllers/sim_api.h` 的 ABI v1 调用控制器：

```cpp
int sim_abi_version();
void sim_reset(const SimConfig* config);
int sim_sample(const SimInput* input);
int sim_step(const SimInput* input, SimOutput* output);
```

- 当前 TVC 工程默认位于 `C:\Projects\代码\TVC_Electronics`，实际路径以当前环境和 UI 选择为准。
- 对已识别的 TVC 工程，`app/compiler.py` 会复制依赖源码到 `work/controllers/<build-id>/source/`，再编译 DLL；**不得修改原飞控工程来迁就仿真器**。
- 当前 TVC 适配器复用原工程的姿态估计、历史预测、Gimbal PID、限幅与限速逻辑。
- 普通自定义 C++ 工程在根目录实现 `sim_controller.cpp`；多编译单元通过 `sim_project.json` 的 `sources` 和 `include_dirs` 声明。
- Arduino/ESP32 的 I2C、UART、Wi-Fi、FreeRTOS 和寄存器时序不在 SIL 第一阶段模拟范围内。不要声称任意 `.ino` 可以直接在桌面执行。
- 控制器在独立进程中运行。编译失败、异常退出、超时、NaN/Inf 或越界输出必须继续以明确错误暴露给 UI，并保存已有故障报告。
- 源码指纹用于保证实验可复现；外部源码变化后必须重新编译，不能继续使用过期快照冒充当前代码。

## 5. 前端与交互原则

- 主界面用于工程导入、编译、仿真实验、实时 3D、姿态/IMU/PID/执行器曲线和历史报告。
- UI 保持现代、简洁、信息密度适中；不要用大块滚动日志替代主要状态。
- 真实姿态、估计姿态、命令 PWM、实际舵机角、推力、IMU、PID 和故障状态必须语义清楚，不能混用。
- 修改可见行为后，除了编译成功，还要实际打开对应页面或通过 API/运行结果确认状态已经变化。
- 前端 API 调用约定以 `frontend/src/App.tsx` 和 `app/server.py` 当前实现为准，不另外维护第二套重复接口。

## 6. CAD / URDF / MJCF 修改规则

- 源 CAD 当前来自 `C:\Projects\机械部分\04-总装配.SLDASM`；路径属于当前机器环境，使用前重新确认。
- 现有模型从 101 个组件实例中提取并合并为 5 个刚体。修改结构前先检查 `reports/cad_extract.json`、`reports/component_manifest.csv` 和 `reports/model_metadata.json`。
- `tools/extract_cad.py` 只在确实需要从当前 SolidWorks 装配重新提取数据时使用。
- `tools/save_repaired_references.py` 是历史引用修复记录脚本，不是常规构建步骤，不要重复运行来“刷新模型”。
- 修改质量、质心、惯量、关节轴、推力点或网格后，应同步更新对应模型/报告并重新做模型验证。
- 不根据视觉外形臆造材料、机械限位、摩擦、舵机扭矩或未提供的电池/电调质量。

## 7. 开发与验证命令

前端依赖以 `frontend/package-lock.json` 为准：

```powershell
cd frontend
npm ci
npm run build
cd ..
```

启动 Studio：

```powershell
.\.venv\Scripts\python.exe run_studio.py --no-browser
```

模型重建与验证，仅在 CAD/模型相关改动后运行：

```powershell
.\.venv\Scripts\python.exe tools\build_model.py
.\.venv\Scripts\python.exe tools\validate_model.py
```

Studio 集成验证位于 `tools/check_studio.py`。它会编译控制器、创建验证运行并调用 `127.0.0.1:18767` 的 API，因此运行前需要启动当前 Studio 后端。验证覆盖初始姿态、真实 C++ 输出对关节的作用、固定随机种子、自定义/多文件控制器、编译错误、源码指纹失效、控制器进程隔离、暂停/继续/扰动/停止和 CSV 导出。

验证应与改动范围对应：

- 纯文档：检查内容、路径和命令与当前仓库一致。
- 前端：至少 `npm run build`，可见交互改动再实际核对页面。
- 后端/API：启动 Studio 后执行相关 API 或集成验证。
- 控制器/编译器：至少编译当前 TVC 或 `controllers/example`，并核对源码指纹/错误路径。
- MJCF/URDF/CAD：运行模型验证，并检查生成的姿态/报告；涉及可见几何时查看渲染结果。

不要因为命令返回 0 就声称实物 TVC、舵机、IMU 或推力已经验证。

## 8. 运行数据与提交卫生

- 不覆盖用户已有运行记录、参数和源码改动，除非任务明确要求。
- `runs/` 和 `reports/` 中可以保留有价值的代表性验证结果；临时测试数据应避免无意义地堆入提交。
- 不提交本机环境、浏览器状态、编译 DLL、第三方依赖目录、凭据、Cookies、访问令牌或其他个人数据。
- 新增外部飞控适配时，优先修改通用接口/适配层，不把某一份用户飞控源码复制成仓库内长期分叉。
- 修改后先检查 `git diff` 和 `git status`，只提交与当前任务相关的文件。
- 推送前确认远端、分支和提交内容；推送后用远端分支 SHA 验证结果，不把本地 commit 当成已经发布。

## 9. 任务完成标准

完成一个开发任务时，最终说明应包含：

1. 实际修改了哪些文件和行为。
2. 做了哪些与本次改动直接相关的验证，以及实际结果。
3. 哪些参数或实物结果仍未验证。
4. 若已发布，给出实际远端仓库、分支和提交 SHA。

如果 README、历史项目指南和当前源码互相冲突，先以实际运行代码定位事实，再更新文档使其重新一致；不要为了保持旧文档不变而把代码改回过时架构。

## 本副本环境（2026-11-13）

- 本目录是 `C:\Projects\工作台\02_仿真平台` 下的独立源码副本，并非 Git 工作区；本机 `.venv` 使用 Python 3.12 和 `requirements-lock.txt` 创建。
- 为避免与 `D:\Code\TVC-Simulation` 的 8765 端口冲突，本副本使用 127.0.0.1:18767。模型经 `app/model_loader.py` 从内存加载 XML 与网格，以支持 Windows 中文路径。

- 真实飞控页使用 `app/device_console/` 内的 Studio 控制资源，不依赖所选工程的 `web/dist`。本副本的飞控工程选择已修正为同一工作台下的 `01_硬件飞控/TVC_Electronics`；旧选择备份于 `backups/studio_project-before-path-repair-20261113.json`。硬件通信与输出仍需实物验证。

- 2026-11-13 用户指定只维护 C:\Projects\工作台 下的新版副本。真实设备页联动油门须提供角速度增益输入，与 01_硬件飞控 的 2.4.7 配套。仿真 PWM 限速默认和当前配置为 25 μs/10 ms，机械舵速上限仍以独立仿真参数为准。

- 工作台中文目录下的 C++ 链接从源码快照目录使用相对文件名；ESP32 CLI 构建输出应放在纯 ASCII 路径，2.4.7 验证产物在 D:/Code/TVC-Build/workbench-2.4.7。新版 Studio 前端、仿真适配 DLL、ESP32 构建和模拟联动启停已通过；实物未烧录、未试转。

- 真实飞控页的机械回中读取设备回传的 control.azimuth_zero_deg。旧固件回传 180° 时跟随旧零位；2.4.8 默认回传 230°。此改动属于实物零位校准，仿真模型使用自己的角度基准。

- 真实飞控的方向标定使用 control.servo_calibration 确认设备保存，手动模式可读取 outputs.servo_pwm_us 填入原始角度中位。旧固件仍可使用原有功能，方向标定保存需 2.4.10。

- 手动方位及俯仰滑块在 input 事件中实时发送，步进为 0.2°。Studio 通信层最多保留一个待确认命令和一个最新目标，拖动期间按约 40 ms 合并，松手发送剩余目标，停止、断连或回中时清除排队的滑块目标。

- 可动重心角度以固件回传的 min_deg/max_deg 换算，旧 2.4.11 保留 0～360° 刻度。−360°～720° 是临时指令刻度。

- 可动重心标定前直接显示 outputs.cg_servo.command_us。2.4.12 的临时角度字段兼容接收，页面不再将其当作实际角度。

- 「真实设备状态」中连接过期表示 2500 ms 内未接收有效整包遥测。Wi-Fi 出现此症状时，先核对 Windows WLAN 当前 SSID、热点切换事件、到 192.168.4.1 的路由及 /api/real/device，再判断 IMU 数据。2026-11-15 排查确认故障时电脑已从 TVC-Bench 切换至其他 Wi-Fi。

- 「姿态与 IMU」显示重力参考并可恢复当前 IMU 默认校准。命令 ACK 超时按提示显示，确认失败不报成功。GPIO7 控制要求固件回传 2000 µs 的上限，避免旧固件越过用户指定的软件限位。

- 真实飞控手动俯仰显示相对角 −20～+20°，协议仍发送以 135° 为逻辑中位的脉宽。实物 155～195° 和重启中位映射需要 2.4.16。

- 2.4.16 默认方向标定方位 93°、俯仰 175°。

- 方向标定的方位中位支持 0.1°，回传验证允许浮点传输误差。

- 2.4.18：俯仰逻辑偏转范围 −35°～+35°，对应舵机物理刻度 140°～210°；硬件机械行程尚待实物验证。Studio 滑块按设备遥测限位显示。

- 推力力矩模式需要编译接入新版工作台 2.5.0 固件。原 C++ 接口保持 ABI v1，通过 `sim_set_torque` / `sim_set_target` 调用同一固件控制逻辑。仿真使用正向电调指令斜率与平方推力模型，MuJoCo 记录电机推力在被动轴上的实际计算作用；实物页只显示请求和模型分配力矩。在线目标通过现有 `/api/simulation/action` 的 `target` 操作改变，目标误差以最新目标为准。
- 2026-11-15 验证记录位于 `D:\Code\TVC-Build\workbench-2.5.0-final-1794741925987`。12 组各 30 s 的估算台架工况分别在第 9 秒和第 21 秒注入 150 ms 的双轴扰动力矩，全部回到目标误差 2° 内。完整生成 sketch 的宿主硬件桩验证覆盖预定位、显式启动、在线目标和失联停推。实物推力、绝对方向和动态响应未验证。

- 2.5.1：推力模式仿真的俯仰初始角范围为 110°～160°，闭环限幅设置 `torque_deflection_limit_deg` 默认 25°。适配器使用同一固件回位逻辑，帧和 CSV 包含 `azimuth_rewind`；真实飞控页使用设备回传的回位标志和俯仰限位。方位回位全程保持基准推力及可达纠偏，实物方位到位和机械速度仍需核对。
- 2.5.1 快速回位验证位于 `D:\Code\TVC-Build\workbench-2.5.1-fast-return-1794743465775`。12 组常规抗扰工况达到 2° 误差标准；禁用预定位的 14 组方位端点起步工况中 13 组达到标准，0° 方位 / Y +0.3° 一组倾倒。确认 32 次完整回位的油门指令均大于零。Studio API 和 CSV 验证覆盖两个端点到初始位的方向。

- 2026-11-15 Studio 倍速和单步由后端播放时钟推进，`/api/simulation/action` 的 `speed` 接受 0.1 / 0.25 / 0.5 / 1 / 2 / 4，0 表示不限速。暂停后 `step` 运行 10 个 1 ms 物理步并维持暂停，预定位和闭环均支持。播放速度不改变飞控 100 Hz、IMU 200 Hz 或物理步长。三维显示在独立的数据副本中使用本拍关节位置计算刚体位姿，执行器曲线两轴分开并显示 PWM 指令。
- 桨叶网格由 `tools/build_model.py --visuals-only` 从现有 CAD 提取缓存导出，`thrust_housing.stl` 和 `propeller.stl` 仅供浏览器。轴和原点取同心配合，MuJoCo/URDF 保留原合并网格、质量和推力点。桨叶相位随仿真 RPM 积分，高速视觉限速。接口测试结果和内置浏览器特写见 `D:\Code\TVC-Build\studio-playback-1794744866058`。

- 重心舵机使用 `balance_carriage` 刚体和 `cg_slide` 关节。舵机、齿轮、舵盘和连接座移动，齿条固定在箭体，正方向为机体 +X。−91～+27 mm 来自 CAD 几何投影。约 70 g 的本体从原箭体质量中拆出，默认另加用户提供的约 300 g 配重（集中在滑架质心），可调 `cg_ballast_g`。无配重且滑架在零位时，原四轴惯性矩阵及箭体重心与拆分前一致。
- `/api/simulation/action` 的 `balance` 操作接收 `position_mm`，暂停时仅更新目标。滑架约束按 10 ms 周期推进，沿用 GPIO7 的 500～2000 µs 范围和 12 µs/拍限速，两侧按 CAD 行程线性映射。该换算、实际安装偏心与带载驱动力尚未标定。特写使用独立齿条和滑架网格，物理合并网格仍保留。构建、验证和演示记录见 `D:\Code\TVC-Build\studio-balance-carriage-1794745963006`。

- 实物页「零位与行程标定」读取飞控 2.5.2 的 control.servo_travel 能力并显示单轴工作流。点按或长按 ±5 / ±20 µs 按钮。记录零位后移动两侧，保存以设备返回的三点与 config_rev 验证。旧固件可继续使用方向和原始零位设置。虚拟飞控接入 Studio 的浏览器验证与模拟重启结果记录在 D:\Code\TVC-Build\servo-travel-1794834567294，实物行程须现场核对。

- 实物轴程标定为方位、俯仰各提供两个点动按钮。飞控 2.5.3 支持显式确认两侧端点，旧版继续兼容自动记录。只有成功保存新三点才替换该轴记录。界面回读飞控的已保存范围与生效范围，飞控断电重启自动加载。
