# TVC Studio 0.1.0 · 飞控仿真实验室

双击本目录的 **TVC Studio.exe** 启动现代界面的本地仿真应用。也可以使用 `启动TVC Studio.cmd`。运行地址为 <http://127.0.0.1:8765>，仅监听本机。

## Studio 使用流程

1. 打开“飞控工程”，填写工程文件夹并点击“读取工程”。当前默认工程为 `C:\Projects\代码\TVC_Electronics`。
2. 查看实际源码，点击“编译接入”。当前 TVC 适配器直接编译并调用 `Attitude.h`、`HistoryAttitude.h`、`GimbalControl.h` 等原工程代码，原文件不被修改。
3. 回到“仿真实验台”，设置初始机体倾角、PID、基准油门和其他工况。控制参数由界面传给算法，覆盖固件中的默认调参值。
4. 点击“开始实验”。可以暂停／继续、停止、施加持续 150ms 的外力扰动。三维台架、真实／估计姿态、执行器和 PID 曲线均来自运行中的物理与控制计算。
5. 在“测试记录”查看报告，导出 CSV。每次运行保存参数、随机种子、源码指纹和遥测，位于 `runs/<运行编号>/`。

右上角电源按钮用于关闭本地引擎；请先停止当前实验。关闭浏览器应用窗口不会强制结束后台引擎，重复启动 EXE 会复用现有引擎并打开界面。

当前为本机可运行的启动器，需要保留整个项目和 `.venv`，并使用本机的 MSYS2 C++ 编译器；不是一个可拷到任意电脑独立运行的安装包。

## 控制代码接入范围

- **当前 TVC 工程**：运行原始姿态估计、缺帧历史预测、云台 PID 和限幅／限速逻辑；不模拟 ESP32 CPU 指令、FreeRTOS 完整任务调度、I2C/UART 寄存器、网页会话和网络状态机。
- **其他 C++ 工程**：根目录放置 `sim_controller.cpp`，实现 `controllers/sim_api.h` 中的 `sim_abi_version`、`sim_reset`、`sim_sample`、`sim_step`。可从界面“自定义控制器”开始，编辑后点击“保存并编译”。工作副本写入本项目 `controllers/custom/`，不会覆盖原飞控源码。
- 多个编译单元可在工程根目录的 `sim_project.json` 声明，例如 `{"sources":["src/control.cpp"],"include_dirs":["include"]}`。`sim_controller.cpp` 自动加入编译，不需要重复声明。
- 普通 `.ino` 可以读取与查看；需要硬件驱动的整套固件不能直接当作桌面控制器执行，必须通过上述明确接口接入。
- 接口 IMU 数据已经位于机体系：角速度为 °/s，加速度计为比力 g。默认通过原 C++ 姿态估计闭环；“直接使用真实姿态”仅用于隔离估计误差。自定义控制器可自行决定使用哪些输入，提供的 PD 参考例子明确使用真值。
- 控制器在独立进程运行。编译错误、运行返回错误、异常退出、长时间无响应都会在界面显示；异常退出也保留故障报告。源码发生变化后必须重新编译。

仿真物理频率 1000Hz，模拟 IMU 200Hz，控制循环 100Hz。推力在模型实际 `thrust_point` 位置沿风扇轴施加，云台由有速度／扭矩约束的响应模型驱动，作用于真实台架关节。这里没有直接把 PID 输出变成“理想恢复力矩”。

界面中的满油门推力、油门二次推力曲线、电机／舵机响应、扭矩、阻尼、噪声和延迟均是可调实验假设；尚未完成实物标定。仿真方位零位由几何建立为可调外参，实物安装方向仍需独立核对。

## Studio 开发与验证

前端位于 `frontend/`，采用 React、Three.js、Recharts、CodeMirror；后端位于 `app/`，采用 FastAPI、MuJoCo 和原生 C++ 控制器。

```powershell
cd frontend
npm ci
npm run build
cd ..
.\.venv\Scripts\python.exe run_studio.py --no-browser
```

后端依赖版本固定在 `requirements-lock.txt`；前端依赖固定在 `frontend/package-lock.json`。C++ 编译器位置为 `C:\msys64\ucrt64\bin\g++.exe`，可在 `app/compiler.py` 调整。源码快照与 DLL 位于 `work/controllers/`，不会覆盖当前已加载的控制器。

本次集成验证覆盖 14 项：机体初始倾角对应、原 C++ 输出影响实际关节、固定种子重复性、自定义和多文件 C++ 接入、编译错误、源码变更失效、控制器进程异常隔离与报告、启动／暂停／继续／扰动／停止，以及 CSV 导出。记录为 `reports/studio_validation.json`。

界面运行与启动日志位于 `reports/studio-server.log`。参数／记录属于本地工程数据，保存于 `work/studio_settings.json` 和 `runs/`。

---

以下为 CAD 模型来源与转换说明。

源装配：`C:\Projects\机械部分\04-总装配.SLDASM`。

本阶段完成模型转换和运动关系核对。默认箭体竖直，保留原 CAD 保存姿态用于比对。模型没有连接真实硬件，尚未接入飞控 PID，也没有标定推进器与舵机动力学。

## 打开模型

双击本目录的 **启动模型预览.cmd**。已配置本项目独立 Python 环境，无需启动 SolidWorks。

- 鼠标左键拖动：旋转视角；滚轮：缩放。
- 右侧 `Joint` 面板：调整四个关节。
- `1`：回到竖直状态；`2`：显示原 CAD 保存姿态；`3`：显示关节运动示例。
- `R`：开始／暂停重力运动。默认保持姿态，便于检查。这里是无控制器的刚体运动，不是姿态稳定演示。
- 关闭三维窗口退出。

## 文件

| 文件 | 用途 |
|---|---|
| `urdf/tvc_bench.urdf` | 可移植 URDF，网格通过相对路径引用 |
| `mjcf/tvc_bench.xml` | MuJoCo 原生场景，含地面、理想 IMU 和推力作用点 |
| `meshes/base.stl` 等 5 个刚体网格 | 合并后的可视几何，单位为米 |
| `meshes/part_*.stl` | SolidWorks 导出的原始零件网格，单位为米 |
| `mass_parameters.json` | PETG/铝密度、打印体积修正和单零件称重覆盖 |
| `reports/component_manifest.csv` | 每个零件实例对应哪个刚体、质量及依据 |
| `reports/cad_extract.json` | 原始零件变换、配合轴和 CAD 质量属性 |
| `reports/model_metadata.json` | 坐标变换、旋转中心、惯量和原始姿态 |
| `reports/validation.json` | 实际加载与关节验证结果 |
| `reports/upright.png` | 竖直模型预览 |
| `reports/cad_saved_pose.png` | 原 CAD 保存姿态预览 |
| `reports/joint_motion.png` | 关节偏转预览 |
| `reports/reference_repairs.json` | 两处引用修复的保存结果、备份和 SHA256 |
| `backups/20261025-204912/` | 保存修复前的两份原始装配文件 |

## 运动结构

```text
base（固定外框）
└─ stand_outer（第一条被动轴承轴）→ outer_frame（内框）
   └─ stand_inner（第二条被动轴承轴）→ rocket_body（箭体）
      └─ tvc_azimuth（有限行程方位轴）→ azimuth_stage
         └─ tvc_elevation（俯仰轴）→ thrust_unit（电机、桨与支架）
```

从 101 个组件实例中提取了 80 个实体零件实例，合并为 5 个刚体；其余 21 个实例是装配层级，不重复计入网格和质量。两条台架轴取自顶层同心配合，云台轴取自电机组装中的同心配合。原 CAD 的两轴台架分别偏转约 7.90°、6.38°，风扇相对箭体另偏约 3.33°；URDF 零位已将箭体和推力轴回正，不改变源 CAD 保存姿态。

统一采用米、千克、秒、弧度。世界 Z 向上，坐标关系为 `(X,Y,Z)世界=(Y,-Z,-X)CAD`，世界原点位于固定底部型材下表面、台架轴交点的正下方。`body_reference` 的 Z 轴指向箭头，传感器轴和固件安装轴还需要在接算法时单独核对。

台架轴承轴按理想自由旋转处理，未臆造机械限位、摩擦和阻尼；TVC 方位行程为 ±180°，俯仰为 ±50°。54°/s、40.5°/s 来自现有固件 PWM 指令限速，并非实测舵机速度。方位机械零位仍需与实物校准；当前几何零位不能直接证明与 1500µs 中位完全一致。

配重／齿条机构按 CAD 保存位置并入箭体，不增加本次双轴台架任务之外的活动机构。没有启用自碰撞，URDF 未生成会填满框架空隙的包围盒碰撞体。

## 质量参数的依据和限制

用户确认箭体打印材料为 PETG、台架为铝型材。原 CAD 所有零件都未指定材料，原质量按默认 1000kg/m³ 计算；不能直接作为真实质量使用。

- 已识别的打印结构采用 PETG 标称密度 **1270kg/m³**，铝型材采用 **2700kg/m³**，按 CAD 实体体积重新计算质量和惯量。
- 打印结构的孔洞／薄壁以 CAD 几何为准，未模拟切片后的内部填充。`printed_cad_volume_factor=1.0` 表示按完整 CAD 实体估算，不代表实物 100% 填充。
- 电机、舵机、轴承、轴、金属舵盘等硬件仍保留 CAD 默认质量，清单中标为未核实。模型内也没有凭空加入电池、电调等缺少位置和质量的数据。
- 可在 `measured_part_mass_kg` 中按 `model_id` 填入实测单件质量；同型号重复实例会使用同一质量，惯量按 CAD 分布等比例修正。若实物内部质量分布与 CAD 不同，还需进一步修正惯量。
- 未提供真实舵机扭矩，URDF 的 `effort=0` 明确表示未标定，MJCF 尚未添加驱动执行器。预览通过关节坐标检查运动，不能据此宣称舵机带载能力足够。

因此目前适合核对几何、坐标、关节和搭建算法接口；可靠 PID 调参还需要实测整机／分组质量、重心、推力曲线和执行器响应。

## 验证与复现

已实际加载 URDF 和 MJCF，核对四个关节的独立运动、原始 CAD 保存姿态重建、竖直轴、惯量正定及 1 秒无控制重力积分的数值有限性。三种姿态已渲染并检查。

```powershell
.\.venv\Scripts\python.exe tools\build_model.py
.\.venv\Scripts\python.exe tools\validate_model.py
```

`tools/extract_cad.py` 仅用于从 SolidWorks 当前打开的总装配重新提取数据，不保存源文档。`tools/save_repaired_references.py` 是本次引用修复记录脚本，会先备份再保存，不需要重复运行。

后续接入控制时，在 `thrust_point` 施加沿真实风扇轴的推力，从理想 IMU 加入安装外参、噪声和延迟，然后复用原项目的 C++ 姿态估计和 GimbalPid。应先确认机体轴与实物零位，再接推力和舵机响应。

## 采用的参考

- [ROS SolidWorks URDF Exporter](https://github.com/ros/solidworks_urdf_exporter)：参考从 SolidWorks 读取重心／中心惯量并转换坐标的实现；本次使用现有 SolidWorks API，未安装导出插件。
- [MuJoCo 模型与 URDF 导入文档](https://mujoco.readthedocs.io/en/stable/modeling.html)、[Python 接口](https://mujoco.readthedocs.io/en/stable/python.html)：模型导入、关节、传感器及预览。
- [Prusament PETG 技术数据表](https://prusament.com/wp-content/uploads/2022/10/PETG_Prusament_TDS_2021_10_EN.pdf)：密度 1.27g/cm³，作为未指定品牌 PETG 的标称估算值。
- [Nedal Aluminium 6063 数据表](https://nedal.com/images/Files__pdfs/legeringen/nedal_datasheet_6063.pdf)：铝型材密度 2700kg/m³，合金具体牌号仍未核实。

生成项目位于 `D:\Code\TVC-Simulation`；原始模型仍位于 `C:\Projects`。本次只保存了已备份的 `03-重力模块舵机.SLDASM` 和 `04-总装配.SLDASM` 的引用修复，没有修改 CAD 材料、尺寸或飞控固件。
