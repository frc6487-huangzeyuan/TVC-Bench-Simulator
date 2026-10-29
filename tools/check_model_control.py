"""检查模型配置、C++ 预定位、原生仿真和调参载入的一致性。"""
import copy,ctypes as ct,json,queue,sys,tempfile,threading
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from app import server,tuning
from app.compiler import build_project
from app.model_control import model_compensation
from app.simulation import Experiment,load_native,run_experiment
from fastapi import HTTPException


def main():
    cfg=Experiment(control_law='model',duration_s=2,realtime=False,kp_x=2.56,kp_y=2.56,
                   ki_x=.05,ki_y=.05,kd_x=2,kd_y=2)
    model=model_compensation(cfg)
    assert model['acceleration_limit']==20
    assert model_compensation(cfg.model_copy(update={'model_accel_limit_dps2':15}))['acceleration_limit']==15
    assert max(abs(v) for v in model['residual_acceleration_dps2'])<.01
    assert model is model_compensation(cfg.model_copy(update={'initial_x_deg':-5,'seed':654,'kp_x':1}))
    assert model!=model_compensation(cfg.model_copy(update={'throttle_pct':60}))
    try:model_compensation(cfg.model_copy(update={'throttle_pct':0}))
    except ValueError:pass
    else:raise AssertionError('zero thrust must not produce a usable model')
    build=build_project(ROOT/'firmware/TVC_Electronics',persist=False)['build']
    lib=load_native(build['dll'],cfg);pwm=(ct.c_double*2)()
    lib.sim_preposition.argtypes=[ct.c_double,ct.c_double,ct.c_double,ct.POINTER(ct.c_double)]
    assert lib.sim_preposition(cfg.initial_x_deg,cfg.initial_y_deg,cfg.initial_azimuth_deg,pwm)==0
    summary,history=run_experiment(cfg.model_dump(),build,'model-check',queue.Queue(),queue.Queue(),record=False)
    assert summary['status'] in ('completed','failed'),summary['message']
    assert summary['preparation']['target_pwm']==list(pwm)
    assert summary['model_compensation']==model and history
    example=build_project(ROOT/'controllers/example',persist=False)['build']
    try:load_native(example['dll'],cfg)
    except RuntimeError as error:assert '不支持模型换算' in str(error)
    else:raise AssertionError('unsupported kernel must reject model mode')
    request=tuning.TuningRequest(experiment=cfg,iterations=1,batch_size=4,episode_s=2,
        training_cases=4,selection_cases=4,validation_cases=4,disturbance_start_range=(.5,1))
    with tempfile.TemporaryDirectory(prefix='tvc-model-check-') as temp:
        root=Path(temp);(root/'work').mkdir();tuning.ROOT=root
        tuning.train(request.model_dump(),build,'model',queue.Queue(),threading.Event())
        result=json.loads((root/'tuning/model/result.json').read_text(encoding='utf-8'))
        assert result['status']=='completed',result['message']
        assert result['request']['experiment']['control_law']=='model'
        server.ROOT=root;server.tuner=SimpleNamespace(poll=lambda:result)
        server.current_project=lambda:{'build':build};server.manager=SimpleNamespace(poll=lambda:{'status':'ready'})
        try:server.tuning_apply(cfg.model_copy(update={'control_law':'pid'}))
        except HTTPException as error:assert error.status_code==400
        else:raise AssertionError('different control laws must not share historical gains')
        applied=server.tuning_apply(cfg)
        assert applied['config']['control_law']=='model'
        # 旧记录未显式保存上限，但标定快照记录了 15；只能用于相同上限。
        legacy=copy.deepcopy(result)
        del legacy['request']['experiment']['model_accel_limit_dps2']
        legacy['model_compensation']=model_compensation(cfg.model_copy(update={'model_accel_limit_dps2':15}))
        server.tuner=SimpleNamespace(poll=lambda:legacy)
        try:server.tuning_apply(cfg)
        except HTTPException as error:assert error.status_code==400
        else:raise AssertionError('legacy 15 limit must not be applied to a 20 limit experiment')
        applied=server.tuning_apply(cfg.model_copy(update={'model_accel_limit_dps2':15}))
        assert applied['config']['model_accel_limit_dps2']==15
    print('PASS model calibration, C++ startup, real simulation, optional ABI, tuning and load compatibility')


if __name__=='__main__':main()
