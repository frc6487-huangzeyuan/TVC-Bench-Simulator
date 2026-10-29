"""仅仿真：从当前 MuJoCo 模型求静态配平和小角度控制效能。"""
import json,math
from functools import lru_cache
import mujoco
import numpy as np
from scipy.optimize import least_squares


def model_compensation(cfg):
    # 控制增益、初始状态和噪声不改变该标定。物理参数变化后自动重新求解。
    excluded={'control_law','duration_s','initial_x_deg','initial_y_deg','initial_azimuth_deg',
              'initial_elevation_deg','auto_prepare_servos','realtime','seed','use_truth',
              'kp_x','ki_x','kd_x','kp_y','ki_y','kd_y','gyro_noise_dps','accel_noise_g',
              'latency_ms','drop_pct','imu_height_mm','slew_us'}
    return _calibrate(json.dumps(cfg.model_dump(exclude=excluded),sort_keys=True))


@lru_cache(maxsize=16)
def _calibrate(encoded):
    from .simulation import Experiment,make_model,META,thrust_at_rpm
    cfg=Experiment(**json.loads(encoded),initial_x_deg=0,initial_y_deg=0,
                   initial_azimuth_deg=180,initial_elevation_deg=135)
    model,data,trim=make_model(cfg)
    force=(thrust_at_rpm(cfg,cfg.max_rpm*cfg.throttle_pct/100) if cfg.thrust_mode=='rpm_curve'
           else cfg.max_thrust_n*(cfg.throttle_pct/100)**2)
    if force<1e-6:raise ValueError('模型换算需要非零基准推力。')
    thrust_body=model.body('thrust_unit').id;site=model.site('thrust_point').id
    mass=np.zeros((model.nv,model.nv))

    def acceleration(lean,vector):
        # 从已知的机体倾角重建台架位置；标定不读取在线真实姿态。
        data.qpos[0]=math.atan(math.tan(math.radians(lean[0]))*math.cos(math.radians(lean[1])))
        data.qpos[1]=math.radians(lean[1])
        x,y=vector;magnitude=math.hypot(x,y)
        azimuth=(math.degrees(math.atan2(y,x))+180)%360;elevation=magnitude
        # 固定选择靠近 135° 的机械分支；另一个等价推力分支的质心并非完全相同。
        if abs(azimuth-135)>90:azimuth=(azimuth+180)%360;elevation=-magnitude
        data.qpos[2]=trim+math.radians(azimuth-180);data.qpos[3]=math.radians(elevation)
        data.qvel[:]=0;mujoco.mj_forward(model,data)
        direction=data.xmat[thrust_body].reshape(3,3)@META['thrust_direction_in_link']
        torque=np.zeros(model.nv)
        mujoco.mj_applyFT(model,data,force*direction,np.zeros(3),data.site_xpos[site],thrust_body,torque)
        mujoco.mj_fullM(model,data,mass)
        # 配平时舵机位置固定。只解两条被动台架轴，避免把舵机自由转动当作控制能力。
        return np.degrees(np.linalg.solve(mass[:2,:2],torque[:2]-data.qfrc_bias[:2]))

    limit=cfg.correction_limit_deg
    solved=least_squares(lambda v:acceleration([0,0],v),[max(-limit+.01,-3),0],
                         bounds=(-limit,limit),xtol=1e-11,ftol=1e-11,gtol=1e-11)
    bias=solved.x;residual=acceleration([0,0],bias)
    if not solved.success or np.linalg.norm(bias)>limit or np.linalg.norm(residual)>.01:
        raise ValueError('当前推力和最大修正角无法在所选模型分支配平；请核对模型参数。')
    epsilon=.001;axes=np.eye(2)*epsilon
    gravity=np.column_stack([(acceleration(a,bias)-acceleration(-a,bias))/(2*epsilon) for a in axes])
    effectiveness=np.column_stack([(acceleration([0,0],bias+a)-acceleration([0,0],bias-a))/(2*epsilon) for a in axes])
    if not np.isfinite(effectiveness).all() or np.linalg.cond(effectiveness)>1e5:
        raise ValueError('当前模型控制效能矩阵接近奇异，不能进行模型换算。')
    return {'bias':bias.tolist(),'gravity':gravity.tolist(),'inverse':np.linalg.inv(effectiveness).tolist(),
            'effectiveness':effectiveness.tolist(),'acceleration_limit':cfg.model_accel_limit_dps2,'force_n':force,
            'residual_acceleration_dps2':residual.tolist(),'calibrated_on_hardware':False,
            'branch_reference_deg':135.}
