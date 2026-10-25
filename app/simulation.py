"""MuJoCo 1000Hz 物理 + 原生 C++ 飞控。每个实验在独立进程中运行。"""
import csv,ctypes as ct,json,math,multiprocessing as mp,queue,time,traceback,uuid,threading,functools
from collections import deque
from pathlib import Path
import mujoco
import numpy as np
from pydantic import BaseModel,Field,ConfigDict

ROOT=Path(__file__).resolve().parents[1]
META=json.loads((ROOT/'reports/model_metadata.json').read_text(encoding='utf-8'))
NAMES=['base','outer_frame','rocket_body','azimuth_stage','thrust_unit']

class Experiment(BaseModel):
    model_config=ConfigDict(extra='forbid',allow_inf_nan=False)
    duration_s:float=Field(20,ge=1,le=180)
    initial_x_deg:float=Field(5,ge=-30,le=30)
    initial_y_deg:float=Field(0,ge=-30,le=30)
    kp_x:float=Field(.6,ge=0,le=10);ki_x:float=Field(0,ge=0,le=2);kd_x:float=Field(.03,ge=0,le=2)
    kp_y:float=Field(.6,ge=0,le=10);ki_y:float=Field(0,ge=0,le=2);kd_y:float=Field(.03,ge=0,le=2)
    correction_limit_deg:float=Field(15,ge=1,le=50)
    slew_us:float=Field(3,ge=1,le=100)
    throttle_pct:float=Field(55,ge=0,le=100)
    max_thrust_n:float=Field(12,ge=0,le=100)
    motor_tau_s:float=Field(.08,ge=.005,le=2)
    servo_tau_s:float=Field(.06,ge=.005,le=1)
    servo_torque_nm:float=Field(1.5,ge=.01,le=10)
    servo_speed_deg_s:float=Field(180,ge=1,le=720)
    bearing_damping:float=Field(.01,ge=0,le=1)
    mass_scale:float=Field(1,ge=.1,le=5)
    cg_offset_mm:float=Field(0,ge=-100,le=100)
    gyro_noise_dps:float=Field(.03,ge=0,le=5)
    accel_noise_g:float=Field(.003,ge=0,le=.2)
    latency_ms:float=Field(5,ge=0,le=300)
    drop_pct:float=Field(0,ge=0,le=80)
    imu_height_mm:float=Field(0,ge=-200,le=200)
    azimuth_trim_deg:float=Field(0,ge=-180,le=180)
    use_truth:bool=False
    realtime:bool=True
    seed:int=Field(42,ge=0,le=2**31-1)

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

def load_native(dll,cfg):
    lib=ct.CDLL(dll)
    lib.sim_abi_version.restype=ct.c_int
    if lib.sim_abi_version()!=1:raise RuntimeError('控制器接口版本不匹配，需要 sim_api v1。')
    lib.sim_reset.argtypes=[ct.POINTER(NativeConfig)];lib.sim_reset.restype=None
    lib.sim_sample.argtypes=[ct.POINTER(NativeInput)];lib.sim_sample.restype=ct.c_int
    lib.sim_step.argtypes=[ct.POINTER(NativeInput),ct.POINTER(NativeOutput)];lib.sim_step.restype=ct.c_int
    c=NativeConfig((cfg.kp_x,cfg.kp_y),(cfg.ki_x,cfg.ki_y),(cfg.kd_x,cfg.kd_y),
                   3,.03,cfg.correction_limit_deg,(1,1),180,cfg.slew_us,50000,cfg.throttle_pct,int(cfg.use_truth))
    lib.sim_reset(ct.byref(c));return lib

def make_model(cfg):
    model=mujoco.MjModel.from_xml_path(str(ROOT/'mjcf/tvc_bench.xml'));data=mujoco.MjData(model)
    model.opt.timestep=.001;model.dof_damping[:2]=cfg.bearing_damping
    for name in ['rocket_body','azimuth_stage','thrust_unit']:
        i=model.body(name).id;model.body_mass[i]*=cfg.mass_scale;model.body_inertia[i]*=cfg.mass_scale
    model.body_ipos[model.body('rocket_body').id]+=np.array(META['body_z_in_link'])*cfg.cg_offset_mm/1000
    site=model.site('body_reference').id
    model.site_pos[site]+=np.array(META['body_z_in_link'])*cfg.imu_height_mm/1000
    mujoco.mj_setConst(model,data);mujoco.mj_forward(model,data)
    # 把 CAD 保存的方位角转成“正俯仰时推力朝机体 -X”的仿真安装零位。
    # 这是可调的仿真外参，仍须在上实物前核对机械方向。
    r=data.site_xmat[site].reshape(3,3)
    el_axis=data.xaxis[model.joint('tvc_elevation').id]
    direction=data.xmat[model.body('thrust_unit').id].reshape(3,3)@META['thrust_direction_in_link']
    derivative=r.T@np.cross(el_axis,direction)
    trim=(math.pi-math.atan2(derivative[1],derivative[0])+math.pi)%(2*math.pi)-math.pi
    trim+=math.radians(cfg.azimuth_trim_deg)
    model.jnt_range[2]=[-math.pi+trim,math.pi+trim]
    data.qpos[2]=trim
    # 以机体 X/Y 倾斜为初始条件，而不是把两条轴承转角直接当成机体倾角。
    # 当前 CAD 零位机体 XY 与世界 XY 相反；两轴同时偏转时还需补偿 cos(Y)。
    data.qpos[0]=math.atan(math.tan(math.radians(cfg.initial_x_deg))*math.cos(math.radians(cfg.initial_y_deg)))
    data.qpos[1]=math.radians(cfg.initial_y_deg)
    mujoco.mj_forward(model,data);return model,data,trim

def poses(model,data):
    return [{'name':name,'position':data.xpos[model.body(name).id].tolist(),
             'quaternion':data.xquat[model.body(name).id].tolist()} for name in NAMES]

def lean(r):return np.degrees([math.atan2(-r[2,0],r[2,2]),math.atan2(-r[2,1],r[2,2])])

def idle_packet():
    m,d,_=make_model(Experiment(initial_x_deg=0))
    return {'kind':'frame','status':'ready','time':0,'pose':poses(m,d),'x':0,'y':0,'est_x':0,'est_y':0,
            'pwm':[1500,1500],'servo_deg':[180,135],'thrust':0,'throttle':0,'pid':[0]*6,
            'limited':False,'age_ms':0,'rtf':0,'samples':0,'gyro':[0,0,0],'accel':[0,0,1]}

def write_csv(folder,history):
    columns={'pwm':['azimuth_pwm_us','elevation_pwm_us'],
             'servo_deg':['azimuth_deg','elevation_deg'],
             'pid':['p_x','i_x','d_x','p_y','i_y','d_y'],
             'gyro':['gyro_x_dps','gyro_y_dps','gyro_z_dps'],
             'accel':['accel_x_g','accel_y_g','accel_z_g'],
             'thrust_origin':['thrust_origin_x_m','thrust_origin_y_m','thrust_origin_z_m'],
             'thrust_direction':['thrust_dir_x','thrust_dir_y','thrust_dir_z']}
    rows=[]
    for p in history:
        row={}
        for k,v in p.items():
            if k in columns:row.update(zip(columns[k],v))
            elif k!='pose':row[k]=v
        rows.append(row)
    with (folder/'telemetry.csv').open('w',encoding='utf-8-sig',newline='') as f:
        fields=list(rows[0]) if rows else ['time']
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)

def run_experiment(config,build,run_id,outbox,commands):
    folder=ROOT/'runs'/run_id;folder.mkdir(exist_ok=True)
    cfg=Experiment(**config);started=time.time();history=[];reason='completed';message='实验完成'
    try:
        model,data,trim=make_model(cfg);lib=load_native(build['dll'],cfg)
        site=model.site('body_reference').id;thrust_body=model.body('thrust_unit').id
        thrust_site=model.site('thrust_point').id;rocket=model.body('rocket_body').id
        rng=np.random.default_rng(cfg.seed);pending=deque();last_sample_t=0.;samples=0
        measured_gyro=np.zeros(3);measured_accel=data.sensordata[3:6].copy()/9.80665
        inp=NativeInput();out=NativeOutput();out.pwm_us[:]=[1500,1500];out.throttle_pct=cfg.throttle_pct
        def fill_input(dt):
            inp.time_us=1_000_000+data.time*1e6;inp.dt=dt
            inp.gyro_dps[:]=measured_gyro;inp.accel_g[:]=measured_accel
            inp.truth_rotation[:]=data.site_xmat[site];inp.sample_age_s=max(0,data.time-last_sample_t)
        fill_input(.005)
        if lib.sim_sample(ct.byref(inp)):raise RuntimeError('姿态估计初始化失败，请检查控制器输入单位。')
        tick=0;force=0.;paused=False;pause_wall=0.;start_wall=time.perf_counter();push_until=-1.;push=np.zeros(3)
        while data.time<cfg.duration_s-1e-9:
            while True:
                try:cmd=commands.get_nowait()
                except queue.Empty:break
                if cmd['action']=='stop':reason='stopped';message='实验已停止';break
                if cmd['action']=='pause' and not paused:paused=True;pause_wall=time.perf_counter()
                if cmd['action']=='resume' and paused:start_wall+=time.perf_counter()-pause_wall;paused=False
                if cmd['action']=='disturb':
                    push=np.array([cmd.get('x',0),cmd.get('y',.15),0.]);push_until=data.time+.15
            if reason=='stopped':break
            if paused:
                outbox.put({'kind':'status','status':'paused'});time.sleep(.05);continue
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
            if tick%10==0:
                fill_input(.01)
                code=lib.sim_step(ct.byref(inp),ct.byref(out))
                if code:raise RuntimeError(f'原生控制器返回错误 {code}（sim_step）。')
                if not np.isfinite(np.ctypeslib.as_array(ct.cast(ct.pointer(out),ct.POINTER(ct.c_double)),shape=(13,))).all():
                    raise RuntimeError('控制器输出出现 NaN/Inf，已停止实验。')
                if not (500<=out.pwm_us[0]<=2500 and 1130<=out.pwm_us[1]<=1870 and 0<=out.throttle_pct<=100):
                    raise RuntimeError('控制器输出超出舵机／油门范围，已停止实验。')
            target=np.array([(out.pwm_us[0]-1500)*math.pi/1000+trim,
                             (out.pwm_us[1]-1500)*math.radians(270)/2000])
            desired_rate=np.clip((target-data.qpos[2:4])/cfg.servo_tau_s,
                                 -math.radians(cfg.servo_speed_deg_s),math.radians(cfg.servo_speed_deg_s))
            data.qfrc_applied[:]=0
            data.qfrc_applied[2:4]=np.clip(.08*(desired_rate-data.qvel[2:4]),-cfg.servo_torque_nm,cfg.servo_torque_nm)
            requested=cfg.max_thrust_n*(out.throttle_pct/100)**2
            force+=(requested-force)*(1-math.exp(-.001/cfg.motor_tau_s))
            thrust_dir=data.xmat[thrust_body].reshape(3,3)@META['thrust_direction_in_link']
            mujoco.mj_applyFT(model,data,force*thrust_dir,np.zeros(3),data.site_xpos[thrust_site],thrust_body,data.qfrc_applied)
            if data.time<push_until:
                mujoco.mj_applyFT(model,data,np.zeros(3),r@push,data.xpos[rocket],rocket,data.qfrc_applied)
            mujoco.mj_step(model,data);tick+=1
            if not np.isfinite(data.qpos).all() or abs(data.qpos[:2]).max()>math.pi:
                reason='failed';message='轴承转角超过 180°，结束本次姿态稳定测试。';break
            if tick%20==0:
                truth=lean(data.site_xmat[site].reshape(3,3));elapsed=time.perf_counter()-start_wall
                packet={'kind':'frame','run_id':run_id,'status':'running','time':round(data.time,6),
                  'pose':poses(model,data),'x':float(truth[0]),'y':float(truth[1]),
                  'thrust_origin':data.site_xpos[thrust_site].tolist(),
                  'thrust_direction':(data.xmat[thrust_body].reshape(3,3)@META['thrust_direction_in_link']).tolist(),
                  'tilt':math.degrees(math.acos(float(np.clip(data.site_xmat[site].reshape(3,3)[2,2],-1,1)))),
                  'est_x':out.estimated_lean_deg[0],'est_y':out.estimated_lean_deg[1],
                  'pwm':list(out.pwm_us),'servo_deg':[math.degrees(data.qpos[2]-trim)+180,math.degrees(data.qpos[3])+135],
                  'thrust':force,'throttle':out.throttle_pct,'pid':list(out.pid),'limited':bool(out.limited),
                  'gyro':measured_gyro.tolist(),'accel':measured_accel.tolist(),
                  'samples':samples,'age_ms':(data.time-last_sample_t)*1000,
                  'rtf':data.time/max(elapsed,.001)}
                history.append({k:v for k,v in packet.items() if k!='pose'});outbox.put(packet)
            if cfg.realtime:
                wait=start_wall+data.time-time.perf_counter()
                if wait>.0001:time.sleep(min(wait,.01))
    except Exception as exc:
        reason='error';message=str(exc)
        (folder/'error.log').write_text(traceback.format_exc(),encoding='utf-8')
    tail=history[-1] if history else {}
    tilt=np.array([p.get('tilt',math.hypot(p['x'],p['y'])) for p in history])
    settled=None
    for i in range(max(0,len(tilt)-25)):
        if np.max(tilt[i:])<2 and (len(tilt)-i)>=25:settled=history[i]['time'];break
    summary={'id':run_id,'status':reason,'message':message,'duration_s':tail.get('time',0),'sample_count':len(history),
      'peak_tilt_deg':float(tilt.max()) if len(tilt) else None,'rms_tilt_deg':float(np.sqrt(np.mean(tilt**2))) if len(tilt) else None,
      'final_tilt_deg':float(tilt[-1]) if len(tilt) else None,'settling_time_s':settled,
      'limited_pct':100*sum(p['limited'] for p in history)/len(history) if history else 0,
      'created_at':time.strftime('%Y-%m-%d %H:%M:%S',time.localtime(started)),
      'controller':build['fingerprint'],'scope':build['scope'],'mass_calibrated':False,'seed':cfg.seed}
    (folder/'run.json').write_text(json.dumps({'config':config,'build':build},ensure_ascii=False,indent=2),encoding='utf-8')
    (folder/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    (folder/'telemetry.json').write_text(json.dumps(history,ensure_ascii=False),encoding='utf-8')
    write_csv(folder,history)
    outbox.put({'kind':'done','status':reason,'message':message,'summary':summary})

def synchronized(fn):
    @functools.wraps(fn)
    def wrapper(self,*args,**kwargs):
        with self.lock:return fn(self,*args,**kwargs)
    return wrapper

class SimulationManager:
    def __init__(self):
        self.lock=threading.RLock();self.context=mp.get_context('spawn');self.process=None;self.latest=idle_packet();self.run_id=None
        self.outbox=None;self.commands=None;self.done=True;self.last_update=time.monotonic()
        self.capture=[];self.current_config=None;self.current_build=None
    @synchronized
    def start(self,cfg,build):
        self.poll()
        if self.process and self.process.is_alive():raise ValueError('请先停止当前实验。')
        self.run_id=time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:5]
        self.outbox=self.context.Queue();self.commands=self.context.Queue();self.done=False
        self.capture=[];self.current_config=cfg.model_dump();self.current_build=build
        self.latest=idle_packet()|{'status':'starting','run_id':self.run_id,'time':0};self.last_update=time.monotonic()
        self.process=self.context.Process(target=run_experiment,args=(cfg.model_dump(),build,self.run_id,self.outbox,self.commands),daemon=True)
        self.process.start();return {'run_id':self.run_id,'status':'starting'}
    @synchronized
    def poll(self):
        if self.outbox is not None:
            while True:
                try:p=self.outbox.get_nowait()
                except queue.Empty:break
                self.last_update=time.monotonic()
                if p['kind']=='frame':
                    self.latest=p;self.capture.append({k:v for k,v in p.items() if k!='pose'})
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
        tilts=[p.get('tilt',0) for p in self.capture]
        summary={'id':self.run_id,'status':'error','message':message,'duration_s':self.latest.get('time',0),
          'sample_count':len(self.capture),'peak_tilt_deg':max(tilts) if tilts else None,
          'rms_tilt_deg':math.sqrt(sum(v*v for v in tilts)/len(tilts)) if tilts else None,
          'final_tilt_deg':tilts[-1] if tilts else None,'settling_time_s':None,
          'limited_pct':100*sum(p['limited'] for p in self.capture)/len(self.capture) if self.capture else 0,
          'created_at':time.strftime('%Y-%m-%d %H:%M:%S'),'controller':self.current_build['fingerprint'],
          'scope':self.current_build['scope'],'mass_calibrated':False,'seed':self.current_config['seed']}
        for name,value in [('summary',summary),('run',{'config':self.current_config,'build':self.current_build}),('telemetry',self.capture)]:
            (folder/(name+'.json')).write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
        write_csv(folder,self.capture)
        self.latest|={'kind':'done','status':'error','message':message,'summary':summary};self.done=True
    @synchronized
    def command(self,action,**values):
        if self.process and self.process.is_alive():self.commands.put({'action':action,**values})
        else:raise ValueError('当前没有运行中的实验。')
    @synchronized
    def close(self):
        if self.process and self.process.is_alive():
            self.commands.put({'action':'stop'});self.process.join(3)
            if self.process.is_alive():self.process.terminate();self.process.join(2)
