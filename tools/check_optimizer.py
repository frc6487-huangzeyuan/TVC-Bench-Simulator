"""验证差分进化在真实仿真、候选载入及停止流程中的行为。"""
import json
import queue
import sys
import tempfile
import threading
from pathlib import Path
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from app import server,tuning
from app.compiler import build_is_current,current_project
from app.simulation import Experiment


def main():
    def history(tilt,count):
        return [{'time':(i+1)*.02,'tilt':tilt,'pwm':[1500,1500],'limited':False} for i in range(count)]
    steady=tuning.episode_metrics({'status':'completed','settling_time_s':None},history(1,50),1)
    tilted=tuning.episode_metrics({'status':'completed','settling_time_s':None},history(30,50),1)
    early=tuning.episode_metrics({'status':'failed','settling_time_s':None},history(1,10),1)
    late=tuning.episode_metrics({'status':'failed','settling_time_s':None},history(1,30),1)
    assert 0<=early['reward']<late['reward']<=20 and tilted['reward']<steady['reward']<=100
    very_early=tuning.episode_metrics({'status':'failed','settling_time_s':None},history(1,10),5)
    assert 0<=very_early['reward']<early['reward'] and very_early['tail_peak_deg']==1
    build=current_project()['build']
    if not build_is_current(build):raise RuntimeError('请先编译当前飞控工程。')
    cfg=Experiment(duration_s=2,realtime=False)
    request=tuning.TuningRequest(experiment=cfg,algorithm='de',optimize_limit=True,
        iterations=1,batch_size=4,episode_s=2,training_cases=4,selection_cases=4,validation_cases=4,
        disturbance_start_range=(.5,1),disturbance_duration_range=(.1,.3))
    with tempfile.TemporaryDirectory(prefix='tvc-de-check-') as temp:
        root=Path(temp);(root/'work').mkdir();tuning.ROOT=root
        tuning.train(request.model_dump(),build,'completed',queue.Queue(),threading.Event())
        result=json.loads((root/'tuning/completed/result.json').read_text(encoding='utf-8'))
        assert result['status']=='completed',result['message']
        assert result['evaluated']==4 and len(result['history'])==4
        assert set(result['best_gains'])==set(tuning.GAINS)|{'correction_limit_deg'}
        assert len(result['validation']['candidate']['cases'])==4
        assert result['reward_version']==2 and 0<=result['best']['reward']<=100
        assert result['validation']['candidate']['tail_cost']>=result['validation']['candidate']['cost']
        assert result['validation']['candidate']['tail_reward']<=result['validation']['candidate']['reward']
        assert (root/'tuning/completed/comparison.json').is_file()
        # 载入使用隔离目录，验证优化的最大修正角与 PID 同时保存。
        server.ROOT=root;server.tuner=SimpleNamespace(poll=lambda:result)
        server.manager=SimpleNamespace(poll=lambda:{'status':'ready'})
        server.current_project=lambda:{'build':build}
        applied=server.tuning_apply(cfg)
        for name,value in result['best_gains'].items():assert applied['config'][name]==value
        saved=json.loads((root/'work/studio_settings.json').read_text(encoding='utf-8'))
        assert saved['correction_limit_deg']==result['best_gains']['correction_limit_deg']
        # 已请求停止的优化不得伪装成完成或推荐参数。
        stop=threading.Event();stop.set()
        tuning.train(request.model_dump(),build,'stopped',queue.Queue(),stop)
        stopped=json.loads((root/'tuning/stopped/result.json').read_text(encoding='utf-8'))
        assert stopped['status']=='stopped' and not stopped['recommended']
    print('DE: reward behavior, real simulation, robust validation, optional angle apply and cancellation passed.')


if __name__=='__main__':main()
