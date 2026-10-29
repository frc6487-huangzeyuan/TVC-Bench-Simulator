"""用相同仿真预算和工况比较 DE 与现有 REINFORCE；不改实验台设置。"""
import argparse
import json
import tempfile
import threading
import time
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from app import tuning
from app.compiler import build_is_current,current_project
from app.simulation import Experiment


class Progress:
    def __init__(self,label):self.label=label;self.phase=None;self.count=-1
    def put(self,packet):
        if 'heartbeat' in packet:return
        phase=packet.get('phase');count=packet.get('evaluated',0)
        if phase!=self.phase or (count!=self.count and count%8==0):
            print(f'{self.label}: {phase} · {count}/{packet.get("total",0)}',flush=True)
            self.phase,self.count=phase,count


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--iterations',type=int,default=4)
    parser.add_argument('--batch',type=int,default=8)
    parser.add_argument('--cases',type=int,default=8)
    parser.add_argument('--validation-cases',type=int,default=12)
    parser.add_argument('--seconds',type=float,default=5)
    parser.add_argument('--seed',type=int,default=42)
    args=parser.parse_args()
    if args.output.exists():raise SystemExit('输出文件已存在，不会覆盖。')
    state=current_project();build=state.get('build')
    if not build_is_current(build) or build['adapter']!='tvc':raise SystemExit('请先编译当前 TVC 飞控工程。')
    settings=ROOT/'work/studio_settings.json'
    cfg=Experiment(**json.loads(settings.read_text(encoding='utf-8')) if settings.exists() else {})
    cfg=cfg.model_copy(update={'duration_s':args.seconds,'realtime':False})
    common={'experiment':cfg,'iterations':args.iterations,'batch_size':args.batch,'episode_s':args.seconds,
            'training_cases':args.cases,'selection_cases':args.cases,'validation_cases':args.validation_cases,
            'seed':args.seed,'optimize_limit':False}
    results={}
    # 临时训练目录只保存验证过程，不污染用户已有的训练历史或设置。
    with tempfile.TemporaryDirectory(prefix='tvc-optimizer-') as temp:
        tuning.ROOT=Path(temp)
        for algorithm in ('de','rl'):
            request=tuning.TuningRequest(**common,algorithm=algorithm)
            start=time.monotonic()
            tuning.train(request.model_dump(),build,algorithm,Progress(algorithm.upper()),threading.Event())
            result=json.loads((Path(temp)/'tuning'/algorithm/'result.json').read_text(encoding='utf-8'))
            if result['status']!='completed':raise RuntimeError(f"{algorithm}: {result['message']}")
            results[algorithm]={'algorithm':result['algorithm'],'evaluated':result['evaluated'],
                                'seconds':round(time.monotonic()-start,2),'gains':result['best_gains'],
                                'baseline':result['validation']['baseline'],
                                'candidate':result['validation']['candidate'],
                                'recommended':result['recommended']}
    for key in ('baseline','candidate'):
        for algorithm in results:results[algorithm][key].pop('curves',None)
    report={'request':tuning.TuningRequest(**common,algorithm='de').model_dump(mode='json'),
            'controller_fingerprint':build['fingerprint'],'results':results,
            'note':'仅仿真，同一参数预算与独立工况；不代表实物已验证。'}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    for algorithm,result in results.items():
        b=result['baseline'];c=result['candidate']
        print(f"{algorithm.upper()}: stable {b['stable_count']}/{b['case_count']} -> {c['stable_count']}/{c['case_count']}; "
              f"failed {b['failed_count']} -> {c['failed_count']}; worst-score {b['tail_reward']:.2f} -> {c['tail_reward']:.2f}; "
              f"mean-score {b['reward']:.2f} -> {c['reward']:.2f}",flush=True)
    print('Report:',args.output,flush=True)


if __name__=='__main__':main()
