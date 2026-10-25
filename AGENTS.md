# TVC Studio Agent Guide

本文件适用于整个 `TVC-Simulation` 仓库。目标是让新的 Agent 在不重新猜测架构、不破坏真实飞控代码和本地运行数据的前提下，直接继续开发和验证。

## 1. 先确认当前事实

- 当前产品是 **TVC Studio 0.1.0**，用于双轴轴承台架上的 TVC 姿态稳定仿真，不是六自由度火箭自由飞行模拟器。
- 当前实现采用 **FastAPI + MuJoCo** 后端，**React + TypeScript + Three.js + Recharts + CodeMirror** 前端，本地地址为 `http://127.0.0.1:8765`。
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

Studio 集成验证位于 `tools/check_studio.py`。它会编译控制器、创建验证运行并调用 `127.0.0.1:8765` 的 API，因此运行前需要启动当前 Studio 后端。验证覆盖初始姿态、真实 C++ 输出对关节的作用、固定随机种子、自定义/多文件控制器、编译错误、源码指纹失效、控制器进程隔离、暂停/继续/扰动/停止和 CSV 导出。

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
