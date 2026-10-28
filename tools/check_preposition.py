"""检查预定位舵机方向及推力在模型中产生的初始力矩，不接触实物。"""
import math
import queue
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app.simulation import Experiment, META, lean, make_model, prepare_servos


def thrust_lean_acceleration(model, data):
    """单位推力作用在实际推力点时，两向倾角的瞬时加速度。"""
    reference = model.site('body_reference').id
    tilt = np.array(lean(data.site_xmat[reference].reshape(3, 3)))
    jacobian = np.zeros((2, model.nv))
    for axis in range(2):
        old = data.qpos[axis]
        data.qpos[axis] = old + 1e-6
        mujoco.mj_forward(model, data)
        jacobian[:, axis] = (np.array(lean(data.site_xmat[reference].reshape(3, 3))) - tilt) / 1e-6
        data.qpos[axis] = old
    mujoco.mj_forward(model, data)
    body = model.body('thrust_unit').id
    direction = data.xmat[body].reshape(3, 3) @ META['thrust_direction_in_link']
    force = np.zeros(model.nv)
    mujoco.mj_applyFT(model, data, direction, np.zeros(3), data.site_xpos[model.site('thrust_point').id], body, force)
    mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, mass)
    return tilt, jacobian @ np.linalg.solve(mass, force)


def prepare(config, adapter='tvc'):
    model, data, trim = make_model(config)
    pwm, info = prepare_servos(config, model, data, trim, {'adapter': adapter}, queue.Queue(), queue.Queue(), 'check')
    return model, data, pwm, info


def main():
    cases = [(5, 0, 180), (0, 5, 180), (-5, 0, 20), (0, -5, 350),
             (5, 5, 180), (-5, 5, 90), (5, -5, 270), (-5, -5, 360)]
    for x, y, azimuth in cases:
        cfg = Experiment(initial_x_deg=x, initial_y_deg=y, initial_azimuth_deg=azimuth, realtime=False)
        model, data, pwm, info = prepare(cfg)
        tilt, acceleration = thrust_lean_acceleration(model, data)
        assert pwm[1] != 1500 and np.dot(tilt, acceleration) < 0, (x, y, info)
    cfg = Experiment(initial_x_deg=.3, initial_y_deg=0, kp_x=1, realtime=False)
    _, _, pwm, info = prepare(cfg)
    assert pwm[1] != 1500 and abs(info['release_servo_deg'][1] - 135) > .05
    cfg = Experiment(initial_x_deg=0, initial_y_deg=0, initial_azimuth_deg=110, realtime=False)
    _, _, pwm, info = prepare(cfg)
    assert pwm[1] == 1500 and math.isclose(info['release_servo_deg'][0], 110, abs_tol=1)
    _, _, pwm, _ = prepare(Experiment(initial_x_deg=5, initial_y_deg=5, realtime=False), 'custom')
    assert pwm == [1500, 1500]
    print('预定位检查通过：8 个方向、微小修正、竖直姿态与自定义控制器。')


if __name__ == '__main__':
    main()
