"""仅仿真：差分进化或有界 REINFORCE 搜索固定 PID 参数。

每次动作选一组参数，回合内部仍由原 C++ PID 闭环控制。属于单步回合
（连续动作 bandit），不是逐控制周期自适应 PID，也不是 PPO。
"""
import copy,json,math,multiprocessing as mp,queue,threading,time,traceback,uuid
from pathlib import Path
from typing import Literal
import numpy as np
from pydantic import BaseModel,ConfigDict,Field,model_validator
from .simulation import ROOT,Experiment,run_experiment,synchronized

GAINS=('kp_x','ki_x','kd_x','kp_y','ki_y','kd_y')
DEFAULT_BOUNDS={k:(0.,3. if k.startswith('kp') else .5) for k in GAINS}
ACTIVE={'starting','running','stopping'}

class TuningRequest(BaseModel):
    model_config=ConfigDict(extra='forbid',allow_inf_nan=False)
    experiment:Experiment
    algorithm:Literal['de','rl']='de'
    optimize_limit:bool=False
    limit_bounds:tuple[float,float]=(3,25)
    iterations:int=Field(8,ge=1,le=50)
    batch_size:int=Field(6,ge=4,le=32)
    episode_s:float=Field(10,ge=1,le=30)
    learning_rate:float=Field(.08,ge=.001,le=.5)
    exploration:float=Field(.08,ge=.02,le=2)
    disturbance_nm:float=Field(.003,ge=0,le=.5)
    disturbance_start_range:tuple[float,float]=(.5,3)
    disturbance_duration_range:tuple[float,float]=(.1,.3)
    initial_x_range:tuple[float,float]=(-5,5)
    initial_y_range:tuple[float,float]=(-5,5)
    azimuth_range:tuple[float,float]=(0,360)
    elevation_range:tuple[float,float]=(85,185)
    training_cases:int=Field(16,ge=4,le=64)
    selection_cases:int=Field(12,ge=4,le=64)
    validation_cases:int=Field(24,ge=4,le=128)
    seed:int=Field(42,ge=0,le=2**31-1)
    bounds:dict[str,tuple[float,float]]=Field(default_factory=lambda:dict(DEFAULT_BOUNDS))

    @model_validator(mode='after')
    def check_bounds(self):
        if self.optimize_limit:
            if self.algorithm!='de':raise ValueError('最大修正角搜索仅支持差分进化。')
            lo,hi=self.limit_bounds
            if not 3<=lo<hi<=50 or not lo<=self.experiment.correction_limit_deg<=hi:
                raise ValueError('最大修正角搜索范围须在 3～50° 内，并包含当前值。')
        for name,minimum,maximum in [('initial_x_range',-30,30),('initial_y_range',-30,30),
                                     ('azimuth_range',0,360),('elevation_range',85,185),
                                     ('disturbance_start_range',.01,30),('disturbance_duration_range',.02,2)]:
            lo,hi=getattr(self,name)
            if not minimum<=lo<=hi<=maximum:raise ValueError(f'{name} 必须满足 {minimum} ≤ 下限 ≤ 上限 ≤ {maximum}。')
        if self.disturbance_nm>0 and self.disturbance_start_range[1]+self.disturbance_duration_range[1]>self.episode_s:
            raise ValueError('初筛时长必须覆盖最晚扰动结束时间；请调整扰动时间范围或延长初筛。')
        if set(self.bounds)!=set(GAINS):raise ValueError('须提供 X/Y 两轴全部六项增益范围。')
        for k,(lo,hi) in self.bounds.items():
            maximum=10 if k.startswith('kp') else 2
            if not 0<=lo<hi<=maximum:raise ValueError(f'{k} 范围必须满足 0 ≤ 下限 < 上限 ≤ {maximum}。')
            if not lo<=getattr(self.experiment,k)<=hi:raise ValueError(f'当前 {k} 不在探索范围内，请扩大范围。')
        return self


class GaussianPolicy:
    def __init__(self,request):
        self.low=np.array([request.bounds[k][0] for k in GAINS])
        self.high=np.array([request.bounds[k][1] for k in GAINS])
        ratio=np.array([(getattr(request.experiment,k)-lo)/(hi-lo) for k,lo,hi in zip(GAINS,self.low,self.high)])
        # 保留接近零增益的探索能力；精确基线另行评估并保留。
        self.mean=np.arctanh(2*np.clip(ratio,.03,.97)-1)
        self.log_std=np.full(6,math.log(request.exploration))

    def gains(self,latent):
        values=self.low+(self.high-self.low)*(np.tanh(latent)+1)/2
        return {k:float(v) for k,v in zip(GAINS,values)}

    def proposals(self,rng,count,iteration):
        """沿单个随机高斯方向成对采样；各维轮流探索，减少六维同时突变。"""
        noise=[];pairs=(count+1)//2
        for j in range(pairs):
            direction=np.zeros(6);direction[(iteration*pairs+j)%6]=rng.normal()
            noise.append(direction)
            if len(noise)<count:noise.append(-direction)
        noise=np.asarray(noise)
        return noise,self.mean+np.exp(self.log_std)*noise

    def update(self,noise,rewards,rate):
        # 每维仅使用该维采样回合；奖励基线和标准化降低采样方差。
        rewards=np.asarray(rewards,dtype=float)
        advantage=(rewards-rewards.mean())*len(rewards)/(len(rewards)-1)
        advantage/=max(float(rewards.std()),1e-8)
        mean_grad=np.zeros(6);std_grad=np.zeros(6)
        for axis in range(6):
            selected=noise[:,axis]!=0
            if selected.any():
                mean_grad[axis]=np.mean(advantage[selected]*noise[selected,axis]/np.exp(self.log_std[axis]))
                std_grad[axis]=np.mean(advantage[selected]*(noise[selected,axis]**2-1))
        self.mean+=rate*np.clip(mean_grad,-.5,.5)
        self.mean=np.clip(self.mean,-4,4)
        self.log_std=np.clip(self.log_std+rate*.1*np.clip(std_grad,-1,1),math.log(.02),math.log(1.5))

    def snapshot(self):return {'mean':self.mean.tolist(),'log_std':self.log_std.tolist()}

    def widen_if_no_stable(self):
        # 所有采样都不稳定时扩大下一轮探索，避免在失败区域只做极小扰动。
        self.log_std=np.minimum(self.log_std+math.log(1.3),math.log(.8))


class Stopped(Exception):pass

class StopCommands:
    def __init__(self,event):self.event=event
    def get_nowait(self):
        if self.event.is_set():return {'action':'stop'}
        raise queue.Empty

class Heartbeat:
    def __init__(self,outbox):self.outbox=outbox;self.last=0
    def put(self,packet):
        now=time.monotonic()
        if now-self.last>.5:self.outbox.put({'heartbeat':True});self.last=now


def episode_metrics(summary,history,duration):
    """固定时长归一化；提前失稳要罚剩余时长，不能靠早停获得低误差。"""
    if summary['status']=='error':raise RuntimeError(summary['message'])
    if summary['status']=='stopped':raise Stopped()
    if not history:raise RuntimeError('仿真没有生成遥测数据。')
    tilt=np.array([p['tilt'] for p in history]);times=np.array([p['time'] for p in history])
    dt=np.diff(np.r_[0,times]);missing=max(0,duration-times[-1])
    missing_tilt=180 if summary['status']!='completed' else float(tilt[-1])
    iae=float(np.sum(dt*tilt)+missing*missing_tilt)
    rms=float(np.sqrt((np.sum(dt*tilt**2)+missing*missing_tilt**2)/duration))
    pwm=np.array([p['pwm'] for p in history]);variation=float(np.abs(np.diff(pwm,axis=0)).sum()/duration)
    limited=float(np.mean([p['limited'] for p in history]))
    failed=summary['status']!='completed'
    # 成功要求最后一秒持续小于 2°；仅完成时长不等于稳定。
    tail=tilt[times>=max(0,duration-1)]
    stabilized=not failed and len(tail)>=40 and bool(np.all(tail<2))
    tail_peak=float(np.max(tail)) if len(tail) else float(tilt[-1])
    cost=iae/duration+float(tilt[-1])*.3+.002*variation+2*limited+(100 if failed else 0)+(0 if stabilized else 10)
    # 满分 100：正常运行时综合全过程倾角、最后一秒峰值、受限和动作量。
    # 提前倒下仅按运行比例给 0–20 分，避免短暂运行因为误差积分少而占便宜。
    observed=max(float(times[-1]),1e-9);mean_tilt=float(np.sum(dt*tilt)/observed)
    if failed:
        reward=20*min(observed/duration,1)*math.exp(-mean_tilt/90)
    else:
        reward=(40*math.exp(-mean_tilt/10)+30*math.exp(-tail_peak/5)+
                20*(1-limited)+10*math.exp(-variation/120))
    return {'cost':cost,'reward':reward,'mean_tilt_deg':mean_tilt,'survival_pct':100*min(observed/duration,1),
            'tail_peak_deg':tail_peak,'rms_tilt_deg':rms,'iae_deg_s':iae,
            'final_tilt_deg':float(tilt[-1]),'peak_tilt_deg':float(tilt.max()),
            'pwm_variation_us_s':variation,'limited_pct':100*limited,'failed':failed,'stabilized':stabilized,
            'settling_time_s':summary['settling_time_s'],'preparation':summary.get('preparation')}


def scenarios(request,validation=False,selection=False):
    """分层覆盖四个姿态/舵机维度，阶段间种子和组合独立且可复现。"""
    n=request.validation_cases if validation else request.selection_cases if selection else request.training_cases
    offset=100001 if validation else 50001 if selection else 17
    rng=np.random.default_rng((request.seed+offset)%2**32)
    dimensions=[request.initial_x_range,request.initial_y_range,request.azimuth_range,request.elevation_range,
                request.disturbance_start_range,request.disturbance_duration_range]
    columns=[]
    for lo,hi in dimensions:
        ranks=(np.arange(n)+rng.random(n))/n;rng.shuffle(ranks)
        columns.append(lo+(hi-lo)*ranks)
    # 显式包含 X/Y 极值与方位边界；各阶段改变舵机组合和噪声种子。
    fractions=[0,.25,.5,.75,1,.125,.375,.625]
    for i in range(min(n,8)):
        sx,sy=[(0,0),(0,1),(1,0),(1,1)][i%4]
        columns[0][i]=dimensions[0][sx];columns[1][i]=dimensions[1][sy]
        phase_shift=2 if validation else 1 if selection else 0
        fraction=fractions[(i+phase_shift)%len(fractions)]
        columns[2][i]=dimensions[2][0]+(dimensions[2][1]-dimensions[2][0])*fraction
        columns[3][i]=dimensions[3][(i+phase_shift)%2]
    columns[4][0]=dimensions[4][0];columns[4][1]=dimensions[4][1]
    columns[5][2]=dimensions[5][0];columns[5][3]=dimensions[5][1]
    angles=rng.uniform(-math.pi,math.pi,n);magnitudes=request.disturbance_nm*np.sqrt(rng.random(n))
    magnitudes[0]=0;magnitudes[1]=request.disturbance_nm
    cases=[]
    for i in range(n):
        cases.append({'case_id':f'{"validation" if validation else "selection" if selection else "train"}-{i+1:02d}',
          'initial_x_deg':float(columns[0][i]),'initial_y_deg':float(columns[1][i]),
          'initial_azimuth_deg':float(columns[2][i]),'initial_elevation_deg':float(columns[3][i]),
          'seed':(request.seed+offset+i)%2**31,
          'disturbance_start_s':float(columns[4][i]),'disturbance_duration_s':float(columns[5][i]),
          'disturbance_x_nm':float(magnitudes[i]*math.cos(angles[i])),
          'disturbance_y_nm':float(magnitudes[i]*math.sin(angles[i]))})
    if selection:
        # 筛选时额外复现工作台当前的真实起始工况和种子，避免通过采样却在当前实验中倒下。
        e=request.experiment
        cases.append({'case_id':'selection-current-workbench','initial_x_deg':e.initial_x_deg,
          'initial_y_deg':e.initial_y_deg,'initial_azimuth_deg':e.initial_azimuth_deg,
          'initial_elevation_deg':e.initial_elevation_deg,'seed':e.seed,
          'disturbance_start_s':request.disturbance_start_range[0],
          'disturbance_duration_s':request.disturbance_duration_range[0],
          'disturbance_x_nm':0.,'disturbance_y_nm':0.})
    return cases


def evaluate(request,build,gains,event,outbox,validation=False,selection=False,progress=None):
    metrics=[];curves=[]
    duration=max(request.episode_s,request.experiment.duration_s) if validation or selection else request.episode_s
    suite=scenarios(request,validation,selection)
    for index,case in enumerate(suite):
        if event.is_set():raise Stopped()
        cfg=request.experiment.model_dump()|gains|{'realtime':False,'duration_s':duration,
                 'initial_x_deg':case['initial_x_deg'],'initial_y_deg':case['initial_y_deg'],
                 'initial_azimuth_deg':case['initial_azimuth_deg'],'initial_elevation_deg':case['initial_elevation_deg'],'seed':case['seed']}
        disturbance=[{'start_s':case['disturbance_start_s'],'duration_s':case['disturbance_duration_s'],
                      'x_nm':case['disturbance_x_nm'],'y_nm':case['disturbance_y_nm']}]
        summary,history=run_experiment(cfg,build,'training',Heartbeat(outbox),StopCommands(event),record=False,
                                       scheduled_disturbances=disturbance,capture_pose=validation)
        metric=episode_metrics(summary,history,duration)
        metrics.append(metric|case|{'status':summary['status'],'duration_s':summary['duration_s']})
        if validation:
            # 50 Hz 真实遥测降采样到约 10 Hz；保留提前失稳时的最后真实帧。
            sampled=history[::5]
            if history and sampled[-1]['time']!=history[-1]['time']:sampled.append(history[-1])
            curves.append([{k:p[k] for k in ('time','tilt','x','y','pwm','servo_deg','thrust',
                                             'thrust_origin','thrust_direction','pose')} for p in sampled])
        if progress is not None:progress(index+1,len(suite))
    aggregate={k:float(np.mean([m[k] for m in metrics])) for k in
               ['cost','reward','mean_tilt_deg','survival_pct','tail_peak_deg','rms_tilt_deg','iae_deg_s','final_tilt_deg','peak_tilt_deg','pwm_variation_us_s','limited_pct']}
    settled=[m['settling_time_s'] for m in metrics]
    stable_count=sum(m['stabilized'] for m in metrics)
    failed_durations=[m['duration_s'] for m in metrics if m['failed']]
    tail_count=max(1,math.ceil(len(metrics)*.1))
    tail_cost=float(np.mean(sorted((m['cost'] for m in metrics),reverse=True)[:tail_count]))
    tail_reward=float(np.mean(sorted(m['reward'] for m in metrics)[:tail_count]))
    return aggregate|{'failed':any(m['failed'] for m in metrics),'failed_count':sum(m['failed'] for m in metrics),
                       'stable_count':stable_count,'case_count':len(metrics),'stable_pct':100*stable_count/len(metrics),
                       'mean_failure_time_s':float(np.mean(failed_durations)) if failed_durations else None,
                       'worst_cost':max(m['cost'] for m in metrics),'tail_cost':tail_cost,'tail_reward':tail_reward,'cases':metrics,'curves':curves,
                       'settling_time_s':float(np.mean(settled)) if all(v is not None for v in settled) else None}

def ranking(score):
    # 固定同一工况集：先提高达标数量，再减少失稳次数；失稳数量相同时，
    # 优先让失败工况坚持更久，最后才比较整体和最差成本。
    failure_time=score['mean_failure_time_s'] if score['mean_failure_time_s'] is not None else 0.
    return (-score['stable_count'],score['failed_count'],-failure_time,-score['reward'],score['worst_cost'])


def robust_ranking(score):
    """先比较失稳数，其次最差 10% 的平均得分，再比较全体得分。"""
    return (score['failed_count'],-score['tail_reward'],-score['reward'],-score['stable_count'])


def search_keys(request):
    return GAINS+(('correction_limit_deg',) if request.optimize_limit else ())


class DifferentialEvolution:
    """标准 DE/rand/1/bin；归一化参数、固定种子，按多工况排序选择。"""
    def __init__(self,request,rng,baseline):
        self.keys=search_keys(request);self.rng=rng
        bounds=[request.limit_bounds if k=='correction_limit_deg' else request.bounds[k] for k in self.keys]
        self.low=np.array([b[0] for b in bounds]);self.high=np.array([b[1] for b in bounds])
        n=request.batch_size;self.population=np.empty((n,len(self.keys)))
        for axis in range(len(self.keys)):
            self.population[:,axis]=(rng.permutation(n)+rng.random(n))/n
        self.population[0]=np.array([(baseline[k]-lo)/(hi-lo) for k,lo,hi in zip(self.keys,self.low,self.high)])
        # 一半种群在当前参数附近探索，另一半分层覆盖范围；避免可用基线被全局随机点淹没。
        local_count=n//2
        if local_count>1:
            self.population[1:local_count]=np.clip(self.population[0]+rng.normal(0,.12,(local_count-1,len(self.keys))),0,1)
        self.scores=[None]*n

    def gains(self,vector):
        return {k:float(v) for k,v in zip(self.keys,self.low+np.asarray(vector)*(self.high-self.low))}

    def trial(self,index):
        others=np.delete(np.arange(len(self.population)),index)
        a,b,c=self.rng.choice(others,3,replace=False)
        mutant=np.clip(self.population[a]+.8*(self.population[b]-self.population[c]),0,1)
        mask=self.rng.random(len(self.keys))<.9;mask[self.rng.integers(len(self.keys))]=True
        return np.where(mask,mutant,self.population[index])

    def accept(self,index,vector,score):
        if self.scores[index] is None or robust_ranking(score)<robust_ranking(self.scores[index]):
            self.population[index]=vector;self.scores[index]=score


def reference_regressed(baseline,candidate):
    """同一工作台工况不得从完整运行退化，或较原来明显更早倒下。"""
    if not baseline['failed'] and candidate['failed']:return True
    if baseline['stabilized'] and not candidate['stabilized']:return True
    if baseline['failed'] and candidate['failed']:
        return candidate['duration_s']+0.1<baseline['duration_s']
    return False


def policy_utilities(scores):
    """用与筛选一致的顺序给策略更新评分；相同表现取得相同名次。"""
    order=sorted(range(len(scores)),key=lambda i:ranking(scores[i]))
    utilities=[0.]*len(scores);start=0
    while start<len(order):
        end=start+1
        while end<len(order) and ranking(scores[order[end]])==ranking(scores[order[start]]):end+=1
        # 最优名次最高；保持原始成本和奖励不变，只改变策略的学习信号。
        utility=len(order)-1-(start+end-1)/2
        for i in order[start:end]:utilities[i]=utility
        start=end
    return utilities


def save_result(folder,result):
    temporary=folder/'result.tmp'
    temporary.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    temporary.replace(folder/'result.json')


def train(request_data,build,training_id,outbox,event):
    request=TuningRequest(**request_data);folder=ROOT/'tuning'/training_id;folder.mkdir(parents=True)
    result={'id':training_id,'status':'running','phase':'评估原始参数','message':'正在建立仿真基线',
            'algorithm':('差分进化 · 多工况鲁棒搜索' if request.algorithm=='de' else 'REINFORCE · 对称局部探索 / 固定 PID'),'request':request.model_dump(mode='json'),
            'build':build,'history':[],'iteration':0,'evaluated':0,
            'total':request.iterations*request.batch_size,'reward_version':2,'mass_calibrated':False,'recommended':False,
            'created_at':time.strftime('%Y-%m-%d %H:%M:%S')}
    def publish():
        save_result(folder,result);outbox.put(copy.deepcopy(result))
    try:
        publish();baseline={k:getattr(request.experiment,k) for k in search_keys(request)}
        best=evaluate(request,build,baseline,event,outbox);result['baseline']=best
        result['best']=best;result['best_gains']=baseline;publish()
        rng=np.random.default_rng(request.seed);candidates=[]
        score_rank=robust_ranking if request.algorithm=='de' else ranking
        policy=GaussianPolicy(request) if request.algorithm=='rl' else DifferentialEvolution(request,rng,baseline)
        for iteration in range(request.iterations):
            # RL 批量更新策略；DE 第一代分层采样，后续逐个变异、交叉并择优保留。
            if request.algorithm=='rl':noise,latent=policy.proposals(rng,request.batch_size,iteration)
            scores=[];batch_stable=0;result['phase']='差分进化搜索' if request.algorithm=='de' else '训练策略';result['iteration']=iteration+1
            for index in range(request.batch_size):
                action=(policy.population[index].copy() if iteration==0 else policy.trial(index)) if request.algorithm=='de' else latent[index]
                gains=policy.gains(action);score=evaluate(request,build,gains,event,outbox);scores.append(score)
                if request.algorithm=='de':policy.accept(index,action,score)
                batch_stable=max(batch_stable,score['stable_count'])
                result['evaluated']+=1
                candidates.append((score_rank(score),gains))
                if score_rank(score)<score_rank(result['best']):
                    result['best']=score;result['best_gains']=gains
                result['history'].append({'episode':result['evaluated'],'iteration':iteration+1,
                    'reward':score['reward'],'best_reward':result['best']['reward'],'gains':gains,'failed':score['failed'],
                    'stable_count':score['stable_count'],'failed_count':score['failed_count'],'case_count':score['case_count'],
                    'mean_failure_time_s':score['mean_failure_time_s'],'tail_cost':score['tail_cost'],'tail_reward':score['tail_reward']})
                result['message']=f"已评估 {result['evaluated']} / {result['total']} 组 PID（每组 {request.training_cases} 个工况）"
                publish()
            if request.algorithm=='rl':
                policy.update(noise,policy_utilities(scores),request.learning_rate)
                if batch_stable==0:policy.widen_if_no_stable()
                result['policy']=policy.snapshot()
            publish()
        # 短回合筛选不等于长时间可用：在独立于最终验证的种子/扰动下
        # 比较基线与多组前列候选，选择全时长表现最好的参数。
        result['phase']='完整时长筛选';result['message']='正在用新种子筛选多组候选';publish()
        screen_base=evaluate(request,build,baseline,event,outbox,selection=True)
        reference_base=next(c for c in screen_base['cases'] if c['case_id']=='selection-current-workbench')
        chosen_gains=baseline;chosen=screen_base;screen=[];seen=set()
        shortlist=[]
        for _,gains in sorted(candidates,key=lambda row:row[0]):
            key=tuple(round(gains[k],10) for k in search_keys(request))
            if key not in seen:seen.add(key);shortlist.append(gains)
            if len(shortlist)>=8:break
        for index,gains in enumerate(shortlist):
            score=evaluate(request,build,gains,event,outbox,selection=True)
            reference=next(c for c in score['cases'] if c['case_id']=='selection-current-workbench')
            regressed=reference_regressed(reference_base,reference)
            screen.append({'gains':gains,'cost':score['cost'],'failed':score['failed'],
                           'stable_count':score['stable_count'],'failed_count':score['failed_count'],'case_count':score['case_count'],
                           'mean_failure_time_s':score['mean_failure_time_s'],'tail_cost':score['tail_cost'],'tail_reward':score['tail_reward'],
                           'reference_regressed':regressed})
            if not regressed and score_rank(score)<score_rank(chosen):
                chosen_gains=gains;chosen=score
            result['message']=f'完整时长筛选 {index+1} / {len(shortlist)} 组候选';publish()
        result['selection']={'baseline':screen_base,'candidate':chosen,'candidates':screen}
        result['best_gains']=chosen_gains
        result['phase']='独立验证';result['message']='在未参与训练的偏角与随机种子上比较';publish()
        def validation_progress(label):
            def report(index,total):
                if index%4==0 or index==total:
                    result['message']=f'独立验证{label}：{index} / {total} 个工况';publish()
            return report
        base_validation=evaluate(request,build,baseline,event,outbox,validation=True,progress=validation_progress('原参数'))
        best_validation=evaluate(request,build,result['best_gains'],event,outbox,validation=True,progress=validation_progress('候选参数'))
        improvement=100*(base_validation['cost']-best_validation['cost'])/max(base_validation['cost'],1e-9)
        # 大尺寸三维姿态记录单独保存；状态轮询只返回轻量曲线。
        comparison={'id':training_id,'duration_s':max(request.episode_s,request.experiment.duration_s),
                    'cases':base_validation['cases'],'baseline':base_validation['curves'],
                    'candidate':best_validation['curves']}
        temporary=folder/'comparison.tmp'
        temporary.write_text(json.dumps(comparison,ensure_ascii=False,allow_nan=False),encoding='utf-8')
        temporary.replace(folder/'comparison.json')
        for data in (base_validation,best_validation):data['curves']=[]
        result['validation']={'baseline':base_validation,'candidate':best_validation,'improvement_pct':improvement}
        result['comparison_available']=True
        result['recommended']=(best_validation['stable_count']==best_validation['case_count'] and
                               score_rank(best_validation)<score_rank(base_validation))
        result['status']='completed';result['phase']='训练完成'
        if result['recommended']:
            result['message']=f"独立验证 {best_validation['stable_count']} / {best_validation['case_count']} 个工况稳定，并优于原参数"
        elif best_validation['stable_count']<best_validation['case_count']:
            result['message']=(f"独立验证 {best_validation['stable_count']} / {best_validation['case_count']} 个工况稳定；"
                               '仍有未达标工况，请查看分布和回放')
        else:
            result['message']=(f"独立验证 {best_validation['stable_count']} / {best_validation['case_count']} 个工况稳定；"
                               '未发现优于原参数的候选')
    except Stopped:
        result['status']='stopped';result['phase']='已停止';result['message']='已保存训练进度；已评估的候选可载入仿真继续试验'
    except Exception as exc:
        result['status']='error';result['phase']='训练错误';result['message']=str(exc)
        (folder/'error.log').write_text(traceback.format_exc(),encoding='utf-8')
    result['recommended']=result['status']=='completed' and result['recommended']
    publish()


class TuningManager:
    def __init__(self):
        self.lock=threading.RLock();self.context=mp.get_context('spawn');self.process=None;self.outbox=None;self.event=None
        self.latest={'status':'ready','history':[]};self.last_update=time.monotonic()
        saved=sorted((ROOT/'tuning').glob('*/result.json'),reverse=True)
        if saved:
            self.latest=json.loads(saved[0].read_text(encoding='utf-8'))
            if self.latest['status'] in ACTIVE:
                self.latest|={'status':'error','message':'上次训练被中断，请重新开始。','recommended':False}
    @synchronized
    def start(self,request,build):
        if self.poll()['status'] in ACTIVE:raise ValueError('请先停止当前训练。')
        self.dispose()
        training_id=time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:5]
        self.outbox=self.context.Queue();self.event=self.context.Event();self.last_update=time.monotonic()
        self.latest={'id':training_id,'status':'starting','history':[],'message':'启动仿真训练进程'}
        self.process=self.context.Process(target=train,args=(request.model_dump(),build,training_id,self.outbox,self.event),daemon=True)
        self.process.start();return self.latest
    @synchronized
    def poll(self):
        if self.outbox is not None:
            while True:
                try:packet=self.outbox.get_nowait()
                except queue.Empty:break
                self.last_update=time.monotonic()
                if 'heartbeat' not in packet:self.latest=packet
        if self.process and self.latest['status'] in ACTIVE:
            message=None
            if not self.process.is_alive():message=f'训练进程异常退出（代码 {self.process.exitcode}）。'
            elif time.monotonic()-self.last_update>30:
                self.process.terminate();self.process.join(2);message='训练控制器超过 30 秒无响应。'
            if message:
                self.latest|={'status':'error','phase':'训练错误','message':message,'recommended':False}
                folder=ROOT/'tuning'/self.latest['id'];folder.mkdir(parents=True,exist_ok=True)
                save_result(folder,self.latest);(folder/'error.log').write_text(message,encoding='utf-8')
        return self.latest
    @synchronized
    def stop(self):
        self.poll()
        if self.latest['status'] not in ACTIVE:raise ValueError('当前没有运行中的训练。')
        self.event.set();self.latest['status']='stopping';return {'ok':True}
    @synchronized
    def select(self,training_id):
        if self.poll()['status'] in ACTIVE:raise ValueError('训练进行中，不能切换记录。')
        if Path(training_id).name!=training_id or training_id in ('.','..'):
            raise ValueError('无效的训练记录编号。')
        path=ROOT/'tuning'/training_id/'result.json'
        if not path.is_file():raise ValueError('未找到训练记录。')
        record=json.loads(path.read_text(encoding='utf-8'))
        if record.get('id')!=training_id or record.get('status') in ACTIVE:
            raise ValueError('该记录尚未完成或已中断，不能作为训练结果载入。')
        self.dispose();self.process=None;self.outbox=None;self.event=None
        self.latest=record
        return self.latest
    def dispose(self):
        if self.process:
            if self.process.is_alive():
                self.event.set();self.process.join(3)
                if self.process.is_alive():self.process.terminate();self.process.join(2)
            else:self.process.join()
        if self.outbox is not None:self.outbox.close();self.outbox.cancel_join_thread()
    @synchronized
    def close(self):self.dispose()
