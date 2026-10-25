"""台架模型检查器：默认保持姿态，可用右侧 Joint 滑块检查关节。"""
import json,time
from pathlib import Path
import mujoco
import mujoco.viewer
import numpy as np

ROOT=Path(__file__).resolve().parent
meta=json.loads((ROOT/'reports/model_metadata.json').read_text(encoding='utf-8'))
model=mujoco.MjModel.from_xml_path(str(ROOT/'mjcf/tvc_bench.xml'))
data=mujoco.MjData(model)
running=False
pending=[]
def key_callback(key):pending.append(key)
print('TVC 双轴台架模型预览')
print('右侧 Joint 滑块：外框轴、箭体轴、TVC 方位、TVC 俯仰。')
print('按 1：竖直；2：CAD 保存姿态；3：关节偏转示例；R：开始/暂停重力运动。')
print('质量是初步估算；尚未连接 PID；没有连接真实硬件。')
mujoco.mj_forward(model,data)
with mujoco.viewer.launch_passive(model,data,key_callback=key_callback) as viewer:
    viewer.cam.lookat[:]=[0,0,.23];viewer.cam.distance=1.05
    viewer.cam.azimuth=135;viewer.cam.elevation=-23
    viewer.opt.geomgroup[:]=1
    while viewer.is_running():
        started=time.perf_counter()
        with viewer.lock():
            while pending:
                key=pending.pop(0)
                if key in [49,50,51]:
                    running=False;mujoco.mj_resetData(model,data)
                    viewer.opt.geomgroup[:]=1
                    if key==50:data.qpos[:]=meta['cad_saved_pose_qpos']
                    elif key==51:data.qpos[:]=np.radians([12,-10,30,15])
                elif key in [82,114]:running=not running
            if running:
                for _ in range(10):mujoco.mj_step(model,data)
            else:mujoco.mj_forward(model,data)
        viewer.sync()
        time.sleep(max(0,.01-(time.perf_counter()-started)))
