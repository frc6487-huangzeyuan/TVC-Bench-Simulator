"""MuJoCo 1000Hz 物理 + 原生 C++ 飞控。每个实验在独立进程中运行。"""
from contextlib import ExitStack
import csv,ctypes as ct,json,math,multiprocessing as mp,queue,time,traceback,uuid,threading,functools
from collections import deque
from pathlib import Path
import mujoco
from app.model_loader import load_model
import numpy as np
from pydantic import BaseModel,Field,ConfigDict,model_validator
from typing import Annotated,Literal

ROOT=Path(__file__).resolve().parents[1]
META=json.loads((ROOT/'reports/model_metadata.json').read_text(encoding='utf-8'))
NAMES=['base','outer_frame','rocket_body','azimuth_stage','thrust_unit','balance_carriage']
BALANCE=META['balance_carriage']
CG_PWM_MIN=500
CG_PWM_HOME=1500
CG_PWM_MAX=2000
CG_PWM_STEP=12 # 新版固件 GPIO7 的每 10 ms 指令变化量。
PLAYBACK_SPEEDS=(0.,.1,.25,.5,1.,2.,4.)
STEP_TICKS=10  # 单步跨过一个 100 Hz 飞控周期；物理仍以 1 ms 积分。

class ThrustPoint(BaseModel):
    model_config=ConfigDict(extra='forbid',allow_inf_nan=False)
    rpm:float=Field(ge=0,le=200000)
    thrust_n:float=Field(ge=0,le=100)

Moment=Annotated[float,Field(gt=0,le=100)]
Offset=Annotated[float,Field(ge=-200,le=200)]
class BodyOverride(BaseModel):
    model_config=ConfigDict(extra='forbid',allow_inf_nan=False)
    mass_kg:float|None=Field(None,ge=.001,le=100)
    # 主轴沿用 CAD 惯性坐标系，不把主惯量误标为机体 X/Y/Z 惯量。
    inertia_kg_m2:tuple[Moment,Moment,Moment]|None=None
    cg_offset_mm:tuple[Offset,Offset,Offset]=(0,0,0)

    @model_validator(mode='after')
    def valid_inertia(self):
        if self.inertia_kg_m2 and 2*max(self.inertia_kg_m2)>=sum(self.inertia_kg_m2):
            raise ValueError('主惯量必须满足任意两项之和大于第三项。')
        return self

class Experiment(BaseModel):
    model_config=ConfigDict(extra='forbid',allow_inf_nan=False)

    @model_validator(mode='before')
    @classmethod
    def ignore_removed_preposition(cls, values):
        if isinstance(values,dict) and 'auto_prepare_servos' in values:
            return {key:value for key,value in values.items() if key!='auto_prepare_servos'}
        return values

    duration_s:float=Field(20,ge=1,le=180)
    initial_x_deg:float=Field(5,ge=-30,le=30)
    initial_y_deg:float=Field(0,ge=-30,le=30)
    initial_azimuth_deg:float=Field(180,ge=0,le=360)
    initial_elevation_deg:float=Field(135,ge=85,le=185)
    control_law:Literal['pid','model','torque']='pid'
    target_x_deg:float=Field(0,ge=-20,le=20)
    target_y_deg:float=Field(0,ge=-20,le=20)
    torque_base_pct:float=Field(65,gt=0,le=100)
    torque_max_pct:float=Field(75,gt=0,le=100)
    torque_deflection_limit_deg:float=Field(25,ge=1,le=25)
    model_accel_limit_dps2:float=Field(20,ge=3,le=100)
    kp_x:float=Field(.6,ge=0,le=10);ki_x:float=Field(0,ge=0,le=2);kd_x:float=Field(.03,ge=0,le=2)
    kp_y:float=Field(.6,ge=0,le=10);ki_y:float=Field(0,ge=0,le=2);kd_y:float=Field(.03,ge=0,le=2)
    correction_limit_deg:float=Field(15,ge=1,le=50)
    slew_us:float=Field(25,ge=1,le=100)
    throttle_pct:float=Field(55,ge=0,le=100)
    max_thrust_n:float=Field(12,ge=0,le=100)
    motor_tau_s:float=Field(.08,ge=.005,le=2)
    servo_tau_s:float=Field(.06,ge=.005,le=1)
    servo_torque_nm:float=Field(1.5,ge=.01,le=10)
    servo_speed_deg_s:float=Field(180,ge=1,le=720)
    bearing_damping:float=Field(.01,ge=0,le=1)
    mass_scale:float=Field(1,ge=.1,le=5)
    cg_offset_mm:float=Field(0,ge=-100,le=100)
    cg_initial_mm:float=Field(0,ge=BALANCE['min_mm'],le=BALANCE['max_mm'])
    cg_ballast_g:float=Field(300,ge=0,le=5000)
    thrust_mode:Literal['quadratic','rpm_curve']='quadratic'
    max_rpm:float=Field(20000,ge=100,le=200000)
    thrust_curve:list[ThrustPoint]=Field(default_factory=lambda:[ThrustPoint(rpm=r,thrust_n=f) for r,f in [(0,0),(5000,.75),(10000,3),(15000,6.75),(20000,12)]],min_length=2,max_length=16)
    body_overrides:dict[Literal['outer_frame','rocket_body','azimuth_stage','thrust_unit','balance_carriage'],BodyOverride]=Field(default_factory=dict)
    gyro_noise_dps:float=Field(.03,ge=0,le=5)
    accel_noise_g:float=Field(.003,ge=0,le=.2)
    latency_ms:float=Field(5,ge=0,le=300)
    drop_pct:float=Field(0,ge=0,le=80)
    imu_height_mm:float=Field(0,ge=-200,le=200)
    azimuth_trim_deg:float=Field(0,ge=-180,le=180)
    use_truth:bool=False
    realtime:bool=True
    playback_speed:Literal[.1,.25,.5,1.,2.,4.]=1.
    seed:int=Field(42,ge=0,le=2**31-1)

    @model_validator(mode='after')
    def valid_curve(self):
        points=self.thrust_curve
        if points[0].rpm!=0 or points[0].thrust_n!=0:
            raise ValueError('推力曲线必须从 0 RPM、0 N 开始。')
        if any(b.rpm<=a.rpm or b.thrust_n<a.thrust_n for a,b in zip(points,points[1:])):
            raise ValueError('转速必须严格递增，推力不能递减。')
        if self.thrust_mode=='rpm_curve' and points[-1].rpm<self.max_rpm:
            raise ValueError('推力曲线必须覆盖最高转速。')
        if self.control_law=='torque':
            if abs(self.initial_elevation_deg-135)>25:
                raise ValueError('推力力矩模式的初始俯仰角需在 110°～160° 内。')
            if self.thrust_mode!='quadratic' or self.max_thrust_n<=0 or self.torque_base_pct>self.torque_max_pct:
                raise ValueError('推力力矩使用平方推力模型，且须满足 0 < 基准油门 ≤ 油门上限。')
            if self.kp_x<=0 or self.kp_y<=0:
                raise ValueError('推力力矩的两路比例增益须大于零。')
        return self

def thrust_at_rpm(cfg,rpm):
    return float(np.interp(rpm,[p.rpm for p in cfg.thrust_curve],[p.thrust_n for p in cfg.thrust_curve]))

class NativeConfig(ct.Structure):
    _fields_=[('kp',ct.c_double*2),('ki',ct.c_double*2),('kd',ct.c_double*2),
      ('integral_limit',ct.c_double),('derivative_filter',ct.c_double),('correction_limit',ct.c_double),
      ('sign',ct.c_double*2),('azimuth_zero',ct.c_double),('slew_us',ct.c_double),
      ('max_gap_us',ct.c_double),('throttle_pct',ct.c_double),('use_truth',ct.c_double)]
class NativeInput(ct.Structure):
    _fields_=[('time_us',ct.c_double),('dt',ct.c_double),('gyro_dps',ct.c_double*3),
              ('accel_g',ct.c_double*3),('truth_rotation',ct.c_double*9),('sample_age_s',ct.c_double)]
class NativeOutput(ct.Structure):
    _fields_=[('pwm_us',ct.c_double*2),('throttle_pct',ct.c_double),('estimated_lean_deg',ct.c_double*2),
              ('pid',ct.c_double*6),('limited',ct.c_double),('status',ct.c_double)]

def initial_pwm(cfg):
    return [math.floor(500+cfg.initial_azimuth_deg*2000/360+.5),
            math.floor(500+cfg.initial_elevation_deg*2000/270+.5)]

def load_native(dll,cfg,start_pwm=None):
    lib=ct.CDLL(dll)
    lib.sim_abi_version.restype=ct.c_int
    if lib.sim_abi_version()!=1:raise RuntimeError('控制器接口版本不匹配，需要 sim_api v1。')
    lib.sim_reset.argtypes=[ct.POINTER(NativeConfig)];lib.sim_reset.restype=None
    lib.sim_sample.argtypes=[ct.POINTER(NativeInput)];lib.sim_sample.restype=ct.c_int
    lib.sim_step.argtypes=[ct.POINTER(NativeInput),ct.POINTER(NativeOutput)];lib.sim_step.restype=ct.c_int
    throttle=cfg.torque_base_pct if cfg.control_law=='torque' else cfg.throttle_pct
    deflection=cfg.torque_deflection_limit_deg if cfg.control_law=='torque' else cfg.correction_limit_deg
    # 模型几何在自身 X/Y 坐标中采用 +1/+1。实物测向结果只用于飞控。
    c=NativeConfig((cfg.kp_x,cfg.kp_y),(cfg.ki_x,cfg.ki_y),(cfg.kd_x,cfg.kd_y),
                   3,.03,deflection,(1,1),180,cfg.slew_us,50000,throttle,int(cfg.use_truth))
    lib.sim_reset(ct.byref(c))
    if cfg.control_law=='model':
        from .model_control import model_compensation
        if not hasattr(lib,'sim_set_model'):
            raise RuntimeError('当前 C++ 控制器不支持模型换算，请编译更新后的 TVC 飞控源码。')
        model=model_compensation(cfg)
        values=(ct.c_double*11)(*model['bias'],*np.array(model['gravity']).ravel(),
                                *np.array(model['inverse']).ravel(),model['acceleration_limit'])
        lib.sim_set_model.argtypes=[ct.POINTER(ct.c_double)];lib.sim_set_model.restype=ct.c_int
        if lib.sim_set_model(values):raise RuntimeError('C++ 控制器拒绝模型换算参数。')
    if cfg.control_law=='torque':
        from .model_control import torque_compensation
        if not hasattr(lib,'sim_set_torque'):
            raise RuntimeError('当前 C++ 控制器不支持推力力矩，请重新编译 2.5.0 或更新的飞控源码。')
        torque=torque_compensation(cfg)
        values=(ct.c_double*10)(torque['full_torque_nm'],cfg.torque_max_pct,cfg.target_x_deg,cfg.target_y_deg,
                                *torque['trim_nm'],*np.array(torque['gravity_nm_per_deg']).ravel())
        lib.sim_set_torque.argtypes=[ct.POINTER(ct.c_double)];lib.sim_set_torque.restype=ct.c_int
        if lib.sim_set_torque(values):raise RuntimeError('C++ 控制器拒绝推力力矩参数。')
        lib.sim_set_target.argtypes=[ct.c_double,ct.c_double];lib.sim_set_target.restype=ct.c_int
    # 可选 v1 扩展：同步原控制器的已发送 PWM，绝不以复位中位冒充真实初始位置。
    pwm=start_pwm if start_pwm is not None else initial_pwm(cfg)
    if hasattr(lib,'sim_set_initial_pwm'):
        lib.sim_set_initial_pwm.argtypes=[ct.c_double,ct.c_double];lib.sim_set_initial_pwm.restype=None
        lib.sim_set_initial_pwm(*pwm)
    elif tuple(pwm)!=(1500,1500):
        raise RuntimeError('此控制器未支持初始舵机位置。请重新编译 TVC 适配器，或使用 180° / 135° 中位。')
    return lib

def make_model(cfg):
    model=load_model();data=mujoco.MjData(model)
    model.opt.timestep=.001;model.dof_damping[:2]=cfg.bearing_damping
    for name in NAMES[2:]:
        i=model.body(name).id;model.body_mass[i]*=cfg.mass_scale;model.body_inertia[i]*=cfg.mass_scale
    for name in ('rocket_body','balance_carriage'):
        model.body_ipos[model.body(name).id]+=np.array(META['body_z_in_link'])*cfg.cg_offset_mm/1000
    site=model.site('body_reference').id
    model.site_pos[site]+=np.array(META['body_z_in_link'])*cfg.imu_height_mm/1000
    mujoco.mj_forward(model,data)
    # 把 CAD 保存的方位角转成“正俯仰时推力朝机体 -X”的仿真安装零位。
    # 这是可调的仿真外参，仍须在上实物前核对机械方向。
    r=data.site_xmat[site].reshape(3,3)
    el_axis=data.xaxis[model.joint('tvc_elevation').id]
    direction=data.xmat[model.body('thrust_unit').id].reshape(3,3)@META['thrust_direction_in_link']
    derivative=r.T@np.cross(el_axis,direction)
    trim=(math.pi-math.atan2(derivative[1],derivative[0])+math.pi)%(2*math.pi)-math.pi
    trim+=math.radians(cfg.azimuth_trim_deg)
    model.jnt_range[2]=[-math.pi+trim,math.pi+trim]
    if cfg.control_law=='torque':model.jnt_range[3]=np.radians([-25,25])
    data.qpos[2]=trim
    # 偏移量统一按零位机体系 XYZ 给定，再转换到各刚体局部坐标。
    mujoco.mj_forward(model,data)
    reference=data.site_xmat[model.site('body_reference').id].reshape(3,3).copy()
    for name,override in cfg.body_overrides.items():
        i=model.body(name).id
        if override.mass_kg is not None:
            ratio=override.mass_kg/model.body_mass[i]
            model.body_mass[i]=override.mass_kg;model.body_inertia[i]*=ratio
        if override.inertia_kg_m2 is not None:model.body_inertia[i]=override.inertia_kg_m2
        rotation=data.xmat[i].reshape(3,3)
        model.body_ipos[i]+=rotation.T@reference@np.array(override.cg_offset_mm)/1000
    # 未知配重形状和安装偏心。先把附加质量视作滑架质心处的集中质量。
    model.body_mass[model.body('balance_carriage').id]+=cfg.cg_ballast_g/1000
    mujoco.mj_setConst(model,data)
    data.qpos[2]=trim+math.radians(cfg.initial_azimuth_deg-180)
    data.qpos[3]=math.radians(cfg.initial_elevation_deg-135)
    data.qpos[model.joint('cg_slide').qposadr[0]]=cfg.cg_initial_mm/1000
    model.eq_data[model.equality('cg_position_hold').id,0]=cfg.cg_initial_mm/1000
    # 以机体 X/Y 倾斜为初始条件，而不是把两条轴承转角直接当成机体倾角。
    # 当前 CAD 零位机体 XY 与世界 XY 相反；两轴同时偏转时还需补偿 cos(Y)。
    data.qpos[0]=math.atan(math.tan(math.radians(cfg.initial_x_deg))*math.cos(math.radians(cfg.initial_y_deg)))
    data.qpos[1]=math.radians(cfg.initial_y_deg)
    mujoco.mj_forward(model,data);return model,data,trim

def balance_pwm(position_mm):
    # CAD 保存位映射至 1500 us。两侧沿各自几何边界线性换算，仅供仿真。
    if position_mm<0:
        return CG_PWM_HOME+(CG_PWM_MIN-CG_PWM_HOME)*position_mm/BALANCE['min_mm']
    return CG_PWM_HOME+(CG_PWM_MAX-CG_PWM_HOME)*position_mm/BALANCE['max_mm']

def balance_position(pwm):
    if pwm<CG_PWM_HOME:
        return BALANCE['min_mm']*(CG_PWM_HOME-pwm)/(CG_PWM_HOME-CG_PWM_MIN)
    return BALANCE['max_mm']*(pwm-CG_PWM_HOME)/(CG_PWM_MAX-CG_PWM_HOME)

def balance_state(model,data,target_mm,pwm):
    actual_mm=float(data.qpos[model.joint('cg_slide').qposadr[0]])*1000
    rocket=model.body('rocket_body').id
    carriage=model.body('balance_carriage').id
    shift_mm=actual_mm*model.body_mass[carriage]/model.body_subtreemass[rocket]
    return {'position_mm':actual_mm,'target_mm':target_mm,'pwm_us':pwm,
            'cg_shift_mm':float(shift_mm),'mass_kg':float(model.body_mass[carriage]),
            'com_world':data.subtree_com[rocket].tolist()}

class Playback:
    def __init__(self,cfg):
        self.speed=cfg.playback_speed if cfg.realtime else 0.
        self.paused=False
        self.pending_ticks=0
        self.rebase(0.)

    def rebase(self,sim_time):
        self.origin_time=sim_time
        self.origin_wall=time.perf_counter()

    def ready(self,sim_time,commands,publish,handle=None):
        """按墙钟限制推进速度。等待命令时保持物理状态，唤醒后沿同一时基推进。"""
        while True:
            wait=0.
            if self.paused and not self.pending_ticks:
                wait=.05
            elif self.speed and not self.paused:
                wait=max(0.,self.origin_wall+(sim_time-self.origin_time)/self.speed-time.perf_counter())
            try:
                cmd=commands.get(timeout=min(wait,.05)) if wait>.0001 else commands.get_nowait()
            except queue.Empty:
                if wait>.0001:continue
                return True
            action=cmd['action']
            if action=='stop':return False
            if action=='speed':
                self.speed=cmd['speed']
                self.rebase(sim_time)
                publish()
            elif action=='pause' and not self.paused:
                self.paused=True
                publish()
            elif action=='resume' and self.paused:
                self.paused=False
                self.pending_ticks=0
                self.rebase(sim_time)
                publish()
            elif action=='step' and self.paused:
                self.pending_ticks+=STEP_TICKS
            elif handle is not None and handle(cmd,sim_time):
                publish()

    def advanced(self):
        if not self.paused:return False
        self.pending_ticks-=1
        return self.pending_ticks%STEP_TICKS==0

def poses(model,data):
    return [{'name':name,'position':data.xpos[model.body(name).id].tolist(),
             'quaternion':data.xquat[model.body(name).id].tolist()} for name in NAMES]

def device_model():
    """Upright bench geometry for telemetry-driven rendering, without physics steps."""
    model,data,_=make_model(Experiment(initial_x_deg=0))
    bodies=[]
    for name in NAMES:
        i=model.body(name).id
        parent=model.body_parentid[i]
        parent_rotation=data.xmat[parent].reshape(3,3)
        local_rotation=parent_rotation.T@data.xmat[i].reshape(3,3)
        joint=model.body_jntadr[i]
        hinge=model.body_jntnum[i] and model.jnt_type[joint]==mujoco.mjtJoint.mjJNT_HINGE
        quaternion=np.empty(4)
        mujoco.mju_mat2Quat(quaternion,local_rotation.ravel())
        bodies.append({'name':name,'parent':model.body(parent).name if parent else None,
                       'position':(parent_rotation.T@(data.xpos[i]-data.xpos[parent])).tolist(),
                       'quaternion':quaternion.tolist(),
                       'axis':model.jnt_axis[joint].tolist() if hinge else None})
    return bodies

def servo_angles(data,trim):
    return [math.degrees(data.qpos[2]-trim)+180,math.degrees(data.qpos[3])+135]

def parameter_preview(cfg):
    """与实验共用 make_model；预览不改当前运行、设置或源模型。"""
    model,data,_=make_model(cfg);site=model.site('body_reference').id
    rotation=data.site_xmat[site].reshape(3,3);origin=data.site_xpos[site]
    bodies=[]
    for name in NAMES[1:]:
        i=model.body(name).id;quat=np.zeros(4)
        mujoco.mju_mat2Quat(quat,data.ximat[i])
        bodies.append({'name':name,'mass_kg':float(model.body_mass[i]),
          'inertia_kg_m2':model.body_inertia[i].tolist(),'com_world':data.xipos[i].tolist(),
          'com_body_mm':(rotation.T@(data.xipos[i]-origin)*1000).tolist(),
          'inertia_quaternion':quat.tolist()})
    mass=sum(b['mass_kg'] for b in bodies)
    com=sum(b['mass_kg']*np.array(b['com_world']) for b in bodies)/mass
    return {'bodies':bodies,'total_mass_kg':mass,'com_world':com.tolist(),
      'com_body_mm':(rotation.T@(com-origin)*1000).tolist(),
      'frame':{'pose':poses(model,data),'thrust':0},
      'curve':[{'rpm':float(r),'thrust_n':thrust_at_rpm(cfg,r) if cfg.thrust_mode=='rpm_curve' else cfg.max_thrust_n*(r/cfg.max_rpm)**2} for r in np.linspace(0,cfg.max_rpm,81)]}

def lean(r):return np.degrees([math.atan2(-r[2,0],r[2,2]),math.atan2(-r[2,1],r[2,2])])

def idle_packet():
    m,d,_=make_model(Experiment(initial_x_deg=0))
    return {'kind':'frame','status':'ready','time':0,'pose':poses(m,d),'x':0,'y':0,'est_x':0,'est_y':0,
            'pwm':[1500,1500],'servo_deg':[180,135],'thrust':0,'rpm':0,'throttle':0,'propeller_turns':0.,'pid':[0]*6,
            'balance':balance_state(m,d,0.,CG_PWM_HOME),
            'limited':False,'age_ms':0,'rtf':0,'samples':0,'gyro':[0,0,0],'accel':[0,0,1]}

CSV_COLUMNS={
    'pwm':['azimuth_pwm_us','elevation_pwm_us'],
    'servo_deg':['azimuth_deg','elevation_deg'],
    'pid':['p_x','i_x','d_x','p_y','i_y','d_y'],
    'gyro':['gyro_x_dps','gyro_y_dps','gyro_z_dps'],
    'accel':['accel_x_g','accel_y_g','accel_z_g'],
    'motor_joint_torque_nm':['motor_outer_torque_nm','motor_inner_torque_nm'],
    'thrust_origin':['thrust_origin_x_m','thrust_origin_y_m','thrust_origin_z_m'],
    'thrust_direction':['thrust_dir_x','thrust_dir_y','thrust_dir_z'],
}


def csv_row(packet):
    row={}
    for key,value in packet.items():
        if key in CSV_COLUMNS:
            row.update(zip(CSV_COLUMNS[key],value))
        elif key=='balance':
            for name in ('position_mm','target_mm','pwm_us','cg_shift_mm','mass_kg'):
                row['balance_'+name]=value[name]
        elif key!='pose':
            row[key]=value
    return row


def write_csv(folder,history):
    rows=map(csv_row,history)
    first=next(rows, None)
    with (folder/'telemetry.csv').open('w',encoding='utf-8-sig',newline='') as file:
        writer=csv.DictWriter(file,fieldnames=list(first) if first else ['time'])
        writer.writeheader()
        if first:
            writer.writerow(first)
            writer.writerows(rows)


class RunMetrics:
    """逐帧统计，持续运行无需在内存中保留完整轨迹。"""
    def __init__(self, torque):
        self.torque=torque
        self.count=self.limited=0
        self.peak_tilt=self.tilt_sq=self.error_sq=0.
        self.tail={}
        self.reset_settling()

    def reset_settling(self):
        self.settled_at=None
        self.settled_samples=0

    def add(self, packet):
        tilt=packet['tilt']
        error=packet['error_deg'] if self.torque else tilt
        self.count+=1
        self.limited+=packet['limited']
        self.peak_tilt=max(self.peak_tilt,tilt)
        self.tilt_sq+=tilt**2
        self.error_sq+=error**2
        self.tail=packet
        if error<2:
            if self.settled_samples==0:
                self.settled_at=packet['time']
            self.settled_samples+=1
        else:
            self.reset_settling()

    def as_dict(self):
        summary={
            'duration_s':self.tail.get('time',0),'sample_count':self.count,
            'peak_tilt_deg':self.peak_tilt if self.count else None,
            'rms_tilt_deg':math.sqrt(self.tilt_sq/self.count) if self.count else None,
            'final_tilt_deg':self.tail.get('tilt'),
            'settling_time_s':self.settled_at if self.settled_samples>=26 else None,
            'limited_pct':100*self.limited/self.count if self.count else 0,
        }
        if self.torque and self.count:
            summary.update(target_deg=[self.tail['target_x'],self.tail['target_y']],
                           final_error_deg=self.tail['error_deg'],
                           rms_error_deg=math.sqrt(self.error_sq/self.count))
        return summary


class ContinuousTrace:
    """每帧写入 JSONL 与 CSV，沿用现有 CSV 字段。"""
    def __init__(self, json_file, csv_file):
        self.json_file=json_file
        self.csv_file=csv_file
        self.writer=None
        self.last_flush=time.monotonic()

    def add(self, packet):
        row=csv_row(packet)
        if self.writer is None:
            self.writer=csv.DictWriter(self.csv_file,fieldnames=list(row))
            self.writer.writeheader()
        self.json_file.write(json.dumps(packet,ensure_ascii=False,separators=(',',':'))+'\n')
        self.writer.writerow(row)
        now=time.monotonic()
        if now-self.last_flush>=1.:
            self.json_file.flush()
            self.csv_file.flush()
            self.last_flush=now


def continuous_samples(folder):
    with (folder/'telemetry.jsonl').open(encoding='utf-8',errors='replace') as file:
        for line in file:
            # 异常退出时末行可能只写了一部分。
            if line.endswith('\n'):
                yield json.loads(line)

def run_experiment(config,build,run_id,outbox,commands,record=True,scheduled_disturbances=None,capture_pose=False,*,continuous=False):
    folder=ROOT/'runs'/run_id
    if record:folder.mkdir(exist_ok=True)
    cfg=Experiment(**config);started=time.time();history=[];reason='completed';message='实验完成'
    target_x,target_y=cfg.target_x_deg,cfg.target_y_deg
    deadline=math.inf if continuous else cfg.duration_s-1e-9
    metrics=RunMetrics(cfg.control_law=='torque')
    playback=Playback(cfg)
    files=ExitStack()
    trace=None
    try:
        if record and continuous:
            trace=ContinuousTrace(
                files.enter_context((folder/'telemetry.jsonl').open('w',encoding='utf-8',newline='\n')),
                files.enter_context((folder/'telemetry.csv').open('w',encoding='utf-8-sig',newline='')),
            )
        model,data,trim=make_model(cfg);prepared_pwm=initial_pwm(cfg)
        lib=load_native(build['dll'],cfg,prepared_pwm)
        site=model.site('body_reference').id;thrust_body=model.body('thrust_unit').id
        thrust_site=model.site('thrust_point').id;rocket=model.body('rocket_body').id
        rng=np.random.default_rng(cfg.seed);pending=deque();last_sample_t=0.;samples=0
        measured_gyro=np.zeros(3);measured_accel=data.sensordata[3:6].copy()/9.80665
        inp=NativeInput();out=NativeOutput();out.pwm_us[:]=prepared_pwm;out.throttle_pct=cfg.throttle_pct
        def fill_input(dt):
            inp.time_us=1_000_000+data.time*1e6;inp.dt=dt
            inp.gyro_dps[:]=measured_gyro;inp.accel_g[:]=measured_accel
            inp.truth_rotation[:]=data.site_xmat[site];inp.sample_age_s=max(0,data.time-last_sample_t)
        fill_input(.005)
        if lib.sim_sample(ct.byref(inp)):raise RuntimeError('姿态估计初始化失败，请检查控制器输入单位。')
        tick=0;force=0.;rpm=0.;propeller_turns=0.;push_until=-1.;push=np.zeros(3)
        throttle=0. if cfg.control_law=='torque' else cfg.throttle_pct
        motor_torque=np.zeros(2)
        balance_target=cfg.cg_initial_mm
        carriage_pwm=balance_pwm(balance_target)
        balance_constraint=model.equality('cg_position_hold').id
        # 只为显示刷新积分后的世界位姿，和本拍关节角同步；物理、IMU 与控制状态走相同的原循环。
        view_data=mujoco.MjData(model)
        def publish(remember=False):
            view_data.qpos[:]=data.qpos
            mujoco.mj_kinematics(model,view_data)
            mujoco.mj_comPos(model,view_data)
            rotation=view_data.site_xmat[site].reshape(3,3)
            truth=lean(rotation)
            elapsed=time.perf_counter()-playback.origin_wall
            packet={'kind':'frame','run_id':run_id,'status':'paused' if playback.paused else 'running',
              'phase':'simulation','time':round(data.time,6),'playback_speed':playback.speed,'control_law':cfg.control_law,
              'pose':poses(model,view_data),'x':float(truth[0]),'y':float(truth[1]),
              'thrust_origin':view_data.site_xpos[thrust_site].tolist(),
              'thrust_direction':(view_data.xmat[thrust_body].reshape(3,3)@META['thrust_direction_in_link']).tolist(),
              'tilt':math.degrees(math.acos(float(np.clip(rotation[2,2],-1,1)))),
              'est_x':out.estimated_lean_deg[0],'est_y':out.estimated_lean_deg[1],
              'pwm':list(out.pwm_us),'servo_deg':servo_angles(data,trim),
              'balance':balance_state(model,view_data,balance_target,carriage_pwm),
              'thrust':force,'rpm':rpm,'throttle':throttle,'propeller_turns':propeller_turns,
              'pid':list(out.pid),'limited':bool(out.limited),'gyro':measured_gyro.tolist(),'accel':measured_accel.tolist(),
              'samples':samples,'age_ms':(data.time-last_sample_t)*1000,
              'rtf':0. if playback.paused else (data.time-playback.origin_time)/max(elapsed,.001)}
            if cfg.control_law=='torque':
                packet.update(target_x=target_x,target_y=target_y,
                              error_deg=math.hypot(truth[0]-target_x,truth[1]-target_y),
                              azimuth_rewind=out.status==2,motor_joint_torque_nm=motor_torque.tolist())
            if remember:
                sample=packet.copy() if capture_pose else {k:v for k,v in packet.items() if k!='pose'}
                metrics.add(sample)
                if continuous:
                    if trace is not None:trace.add(sample)
                else:
                    history.append(sample)
            outbox.put(packet)

        def handle(cmd,sim_time):
            nonlocal push,push_until,target_x,target_y,balance_target
            if cmd['action']=='disturb':
                push=np.array([cmd.get('x',0),cmd.get('y',.15),0.]);push_until=sim_time+.15
            elif cmd['action']=='target' and cfg.control_law=='torque':
                if lib.sim_set_target(cmd['x'],cmd['y']):raise RuntimeError('C++ 控制器拒绝新目标角。')
                target_x,target_y=cmd['x'],cmd['y']
                metrics.reset_settling()
            elif cmd['action']=='balance':
                balance_target=cmd['position_mm']
                return True
            return False

        playback.rebase(data.time)
        publish()
        while data.time<deadline:
            if not playback.ready(data.time,commands,publish,handle):
                reason='stopped';message='实验已停止';break
            r=data.site_xmat[site].reshape(3,3)
            if tick%5==0:
                if rng.random()>=cfg.drop_pct/100:
                    pending.append((data.time+cfg.latency_ms/1000,data.time,
                      np.degrees(data.sensordata[:3])+rng.normal(0,cfg.gyro_noise_dps,3),
                      data.sensordata[3:6]/9.80665+rng.normal(0,cfg.accel_noise_g,3)))
            while pending and pending[0][0]<=data.time+1e-10:
                _,st,measured_gyro,measured_accel=pending.popleft()
                dt=max(.0001,st-last_sample_t);last_sample_t=st;fill_input(dt)
                if lib.sim_sample(ct.byref(inp)):raise RuntimeError('原生姿态估计拒绝样本（sim_sample）。检查采样间隔或控制器返回值。')
                samples+=1
            if data.time-last_sample_t>2:raise RuntimeError('IMU 连续 2 秒无有效数据，已停止本次实验。')
            if tick%STEP_TICKS==0:
                delta=balance_pwm(balance_target)-carriage_pwm
                carriage_pwm+=max(-CG_PWM_STEP,min(CG_PWM_STEP,delta))
                model.eq_data[balance_constraint,0]=balance_position(carriage_pwm)/1000
                fill_input(.01)
                code=lib.sim_step(ct.byref(inp),ct.byref(out))
                if code==-8 and build.get('adapter')=='tvc':
                    reason='failed';message='估计姿态超出闭环有效范围，已结束实验。';break
                if code:raise RuntimeError(f'原生控制器返回错误 {code}（sim_step）。')
                if not np.isfinite(np.ctypeslib.as_array(ct.cast(ct.pointer(out),ct.POINTER(ct.c_double)),shape=(13,))).all():
                    raise RuntimeError('控制器输出出现 NaN/Inf，已停止实验。')
                if not (500<=out.pwm_us[0]<=2500 and 1130<=out.pwm_us[1]<=1870 and 0<=out.throttle_pct<=100):
                    raise RuntimeError('控制器输出超出舵机／油门范围，已停止实验。')
                # 固件 100 Hz、双向 ESC 正向窗口 1500～1700 μs、每拍 5 μs。
                throttle=(throttle+float(np.clip(out.throttle_pct-throttle,-2.5,2.5))
                          if cfg.control_law=='torque' else out.throttle_pct)
            target=np.array([(out.pwm_us[0]-1500)*math.pi/1000+trim,
                             (out.pwm_us[1]-1500)*math.radians(270)/2000])
            desired_rate=np.clip((target-data.qpos[2:4])/cfg.servo_tau_s,
                                 -math.radians(cfg.servo_speed_deg_s),math.radians(cfg.servo_speed_deg_s))
            data.qfrc_applied[:]=0
            data.qfrc_applied[2:4]=np.clip(.08*(desired_rate-data.qvel[2:4]),-cfg.servo_torque_nm,cfg.servo_torque_nm)
            response=1-math.exp(-.001/cfg.motor_tau_s)
            rpm+=(cfg.max_rpm*throttle/100-rpm)*response
            # 将高速 RPM 缩为可见转速，4× 播放下仍能区分桨叶。
            propeller_turns+=min(3.,rpm/3600)*model.opt.timestep
            if cfg.thrust_mode=='rpm_curve':force=thrust_at_rpm(cfg,rpm)
            else:
                requested=cfg.max_thrust_n*(throttle/100)**2
                force+=(requested-force)*response
            thrust_dir=data.xmat[thrust_body].reshape(3,3)@META['thrust_direction_in_link']
            mujoco.mj_applyFT(model,data,force*thrust_dir,np.zeros(3),data.site_xpos[thrust_site],thrust_body,data.qfrc_applied)
            if cfg.control_law=='torque':motor_torque[:]=data.qfrc_applied[:2]
            if data.time<push_until:
                mujoco.mj_applyFT(model,data,np.zeros(3),r@push,data.xpos[rocket],rocket,data.qfrc_applied)
            # 离线训练和对比可按时间施加机体系力矩；仍由同一 MuJoCo 模型计算。
            for disturbance in scheduled_disturbances or ():
                if disturbance['start_s']<=data.time<disturbance['start_s']+disturbance['duration_s']:
                    torque=np.array([disturbance['x_nm'],disturbance['y_nm'],0.])
                    mujoco.mj_applyFT(model,data,np.zeros(3),r@torque,data.xpos[rocket],rocket,data.qfrc_applied)
            mujoco.mj_step(model,data);tick+=1
            if not np.isfinite(data.qpos).all() or abs(data.qpos[:2]).max()>math.pi:
                reason='failed';message='轴承转角超过 180°，结束本次姿态稳定测试。';break
            stepped=playback.advanced()
            if stepped or (not playback.paused and tick%20==0) or data.time>=deadline:
                publish(remember=True)
    except Exception as exc:
        reason='error';message=str(exc)
        if record:(folder/'error.log').write_text(traceback.format_exc(),encoding='utf-8')
    finally:
        files.close()
    statistics=metrics.as_dict()
    if reason=='completed' and statistics['settling_time_s'] is None:
        message='实验运行结束，姿态未持续保持在目标误差 2° 内。'
    summary={'id':run_id,'status':reason,'message':message,**statistics,
      'created_at':time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(started)),
      'controller':build['fingerprint'],'scope':build['scope'],'mass_calibrated':False,'seed':cfg.seed,
      'control_law':cfg.control_law,
      'model_compensation':None}
    if cfg.control_law=='model' and reason!='error':
        from .model_control import model_compensation
        summary['model_compensation']=model_compensation(cfg)
    if record:
        manifest={'config':config,'build':build}
        if continuous:
            manifest['continuous']=True
            if not metrics.count:
                write_csv(folder,[])
        else:
            (folder/'telemetry.json').write_text(json.dumps(history,ensure_ascii=False),encoding='utf-8')
            write_csv(folder,history)
        (folder/'run.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        (folder/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    outbox.put({'kind':'done','status':reason,'message':message,'summary':summary})
    return summary,history

def synchronized(fn):
    @functools.wraps(fn)
    def wrapper(self,*args,**kwargs):
        with self.lock:return fn(self,*args,**kwargs)
    return wrapper

class SimulationManager:
    def __init__(self):
        self.lock=threading.RLock();self.context=mp.get_context('spawn');self.process=None;self.latest=idle_packet();self.run_id=None
        self.outbox=None;self.commands=None;self.done=True;self.last_update=time.monotonic()
        self.capture=[];self.continuous=False;self.current_config=None;self.current_build=None
    @synchronized
    def start(self,cfg,build,*,continuous=False):
        self.poll()
        if self.process and self.process.is_alive():raise ValueError('请先停止当前实验。')
        self.run_id=time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:5]
        self.outbox=self.context.Queue();self.commands=self.context.Queue();self.done=False
        self.capture=[];self.continuous=continuous;self.current_config=cfg.model_dump();self.current_build=build
        self.latest=idle_packet()|{'status':'starting','run_id':self.run_id,'time':0,'playback_speed':cfg.playback_speed if cfg.realtime else 0.};self.last_update=time.monotonic()
        self.process=self.context.Process(target=run_experiment,
            args=(self.current_config,build,self.run_id,self.outbox,self.commands),
            kwargs={'continuous':continuous},daemon=True)
        self.process.start();return {'run_id':self.run_id,'status':'starting'}
    @synchronized
    def poll(self):
        if self.outbox is not None:
            while True:
                try:p=self.outbox.get_nowait()
                except queue.Empty:break
                self.last_update=time.monotonic()
                if p['kind']=='frame':
                    self.latest=p
                    if not self.continuous and p.get('phase')=='simulation' and p['time']>0 and (not self.capture or self.capture[-1]['time']!=p['time']):
                        self.capture.append({k:v for k,v in p.items() if k!='pose'})
                elif p['kind']=='status':self.latest|={'status':p['status']}
                else:self.latest|=p;self.done=True
        if self.process and self.process.pid is not None and not self.done:
            if not self.process.is_alive():
                self.persist_crash(f'控制器进程异常退出（代码 {self.process.exitcode}）。')
            elif self.latest['status']!='paused' and time.monotonic()-self.last_update>10:
                self.process.terminate();self.process.join(2);self.persist_crash('控制器超过 10 秒无响应，已结束实验进程。')
        return self.latest
    def persist_crash(self,message):
        folder=ROOT/'runs'/self.run_id;folder.mkdir(exist_ok=True)
        manifest={'config':self.current_config,'build':self.current_build}
        torque=self.current_config['control_law']=='torque'
        metrics=RunMetrics(torque)
        if self.continuous:
            manifest['continuous']=True
            path=folder/'telemetry.jsonl'
            path.touch(exist_ok=True)
            for sample in continuous_samples(folder):
                metrics.add(sample)
            if not (folder/'telemetry.csv').is_file():
                write_csv(folder,[])
        else:
            for sample in self.capture:
                metrics.add(sample)
            (folder/'telemetry.json').write_text(json.dumps(self.capture,ensure_ascii=False,indent=2),encoding='utf-8')
            write_csv(folder,self.capture)
        statistics=metrics.as_dict()
        statistics.update(duration_s=self.latest.get('time',0),settling_time_s=None)
        summary={'id':self.run_id,'status':'error','message':message,**statistics,
          'created_at':time.strftime('%Y-%m-%d %H:%M:%S'),'controller':self.current_build['fingerprint'],
          'scope':self.current_build['scope'],'mass_calibrated':False,'seed':self.current_config['seed']}
        for name,value in [('run',manifest),('summary',summary)]:
            (folder/(name+'.json')).write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
        self.latest|={'kind':'done','status':'error','message':message,'summary':summary};self.done=True
    @synchronized
    def command(self,action,**values):
        latest=self.poll()
        if self.done or not self.process or not self.process.is_alive():
            raise ValueError('当前没有运行中的实验。')
        if action=='target' and (self.current_config['control_law']!='torque' or latest.get('phase')!='simulation'):
            raise ValueError('只有正在运行的推力力矩实验可更新目标。')
        if action=='step' and latest['status']!='paused':
            raise ValueError('请先暂停实验，再单步推进。')
        if action=='balance' and latest.get('phase')!='simulation':
            raise ValueError('实验启动后可以修改滑架位置。')
        self.commands.put({'action':action,**values})
    @synchronized
    def close(self):
        if self.process and self.process.is_alive():
            self.commands.put({'action':'stop'})
            deadline=time.monotonic()+3
            while self.process.is_alive() and time.monotonic()<deadline:
                self.process.join(.05)
                self.poll()
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(2)
            self.poll()
