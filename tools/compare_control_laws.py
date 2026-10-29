"""同工况比较角度 PID 与模型换算；不改工作台设置、不覆盖已有报告。"""
import argparse,json,queue,sys,threading
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from app.compiler import build_project
from app.simulation import Experiment
from app.model_control import model_compensation
from app.tuning import TuningRequest,evaluate,GAINS


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--project',type=Path,default=ROOT/'firmware/TVC_Electronics')
    parser.add_argument('--baseline-gains',type=Path,help='包含 gains 字段或六项增益的 JSON')
    parser.add_argument('--seconds',type=float,default=60)
    parser.add_argument('--cases',type=int,default=24)
    parser.add_argument('--seed',type=int,default=41029)
    args=parser.parse_args()
    if args.output.exists():raise SystemExit('报告已存在，不会覆盖。')
    cfg=Experiment(**json.loads((ROOT/'work/studio_settings.json').read_text(encoding='utf-8')))
    base={k:getattr(cfg,k) for k in GAINS}
    if args.baseline_gains:
        supplied=json.loads(args.baseline_gains.read_text(encoding='utf-8'))
        base={k:supplied.get('gains',supplied)[k] for k in GAINS}
    candidate={k:(2.56 if k.startswith('kp') else 2. if k.startswith('kd') else .05) for k in GAINS}
    build=build_project(args.project,persist=False)['build'];results={}
    for law,gains in [('pid',base),('model',candidate)]:
        local=Experiment(**(cfg.model_dump()|gains|{'control_law':law,'duration_s':args.seconds,'realtime':False}))
        request=TuningRequest(experiment=local,episode_s=min(10,args.seconds),seed=args.seed,validation_cases=args.cases,
                              disturbance_start_range=(.2,min(.5,args.seconds-.3)),
                              bounds={k:(0.,10 if k.startswith('kp') else 2.) for k in GAINS})
        # 两个控制律完全相同的初始状态、噪声、扰动力矩和时刻。
        # 使用标准评估器的默认扰动范围，短于 3.3 秒的检查才缩短它。
        if args.seconds>=3.3:request=request.model_copy(update={'disturbance_start_range':(.5,3.)})
        score=evaluate(request,build,gains,threading.Event(),queue.Queue(),validation=True,
                       progress=lambda i,n:print(law,i,'/',n,flush=True) if i%4==0 or i==n else None)
        score.pop('curves',None)
        results[law]={'gains':gains,'metrics':score}
        print(law,'stable',score['stable_count'],'failed',score['failed_count'],'cost',round(score['cost'],2),flush=True)
    paired=list(zip(results['pid']['metrics']['cases'],results['model']['metrics']['cases']))
    report={'request':request.model_dump(mode='json'),'build':build,'results':results,
            'model_compensation':model_compensation(local),
            'rescued':[a['case_id'] for a,b in paired if a['failed'] and not b['failed']],
            'regressed':[a['case_id'] for a,b in paired if not a['failed'] and b['failed']],
            'note':'固定物理参数的有限样本仿真；两个控制律使用各自增益，并非单独归因试验。'}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print('Report:',args.output)


if __name__=='__main__':main()
