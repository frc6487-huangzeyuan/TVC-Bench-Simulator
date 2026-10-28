import {useEffect,useMemo,useState} from 'react';
import {Pause,Play,RotateCcw} from 'lucide-react';
import {CartesianGrid,Line,LineChart,ReferenceLine,ResponsiveContainer,Tooltip,XAxis,YAxis} from 'recharts';
import Scene from './Scene';
import type {Frame,Pose} from './types';
import './comparison.css';

type ReplayFrame={time:number;tilt:number;x:number;y:number;pwm:number[];servo_deg:number[];thrust:number;thrust_origin:number[];thrust_direction:number[];pose:Pose[]};
type Case={initial_x_deg:number;initial_y_deg:number;initial_azimuth_deg?:number;initial_elevation_deg?:number;seed:number;disturbance_x_nm:number;disturbance_y_nm:number;disturbance_start_s:number;disturbance_duration_s?:number;status:string;duration_s:number};
type CaseResult={status:string;duration_s:number;peak_tilt_deg:number;final_tilt_deg:number;settling_time_s:number|null;stabilized?:boolean;preparation?:{duration_s:number;release_servo_deg:number[]}|null};
type ComparisonData={id:string;duration_s:number;cases:Case[];baseline:ReplayFrame[][];candidate:ReplayFrame[][]};
type ComparisonProps={id:string;baseCases:CaseResult[];candidateCases:CaseResult[]};
type Metric='tilt'|'x'|'y'|'azimuth'|'elevation';
const f=(n:number|undefined|null,d=2)=>typeof n==='number'&&Number.isFinite(n)?n.toFixed(d):'—';
const caseStatus=(r:CaseResult)=>r.stabilized===true?'稳定':r.status!=='completed'?'倒下':r.stabilized===false?'未收敛':'运行完成';
const graphValues:Record<Metric,{label:string;value:(p:ReplayFrame)=>number}>= {
 tilt:{label:'整体倾角',value:p=>p.tilt},x:{label:'机体 X 倾角',value:p=>p.x},y:{label:'机体 Y 倾角',value:p=>p.y},
 azimuth:{label:'实际方位角',value:p=>p.servo_deg[0]},elevation:{label:'实际俯仰角',value:p=>p.servo_deg[1]},
};
function sampleAt(frames:ReplayFrame[],time:number){
 let lo=0,hi=frames.length-1;
 while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(frames[mid].time<=time+1e-7)lo=mid;else hi=mid-1;}
 return frames[lo];
}
function ReplayScene({name,frames,time,result,color}:{name:string;frames:ReplayFrame[];time:number;result:CaseResult;color:string}){
 const p=sampleAt(frames,time);
 const frame=p?{kind:'frame',status:'replay',time:p.time,pose:p.pose,x:p.x,y:p.y,tilt:p.tilt,
  pwm:p.pwm,servo_deg:p.servo_deg,thrust:p.thrust,thrust_origin:p.thrust_origin,thrust_direction:p.thrust_direction} as Frame:null;
 const ended=!!p&&result.status!=='completed'&&time>=(frames.at(-1)?.time??0);
 return <div className="compare-pane"><div className="compare-pane-heading"><span><i style={{background:color}}/>{name}</span><b className={result.stabilized===false||result.status!=='completed'?'compare-failed':'compare-ok'}>{caseStatus(result)}</b></div>
  <Scene frame={frame}/><div className="compare-pane-values"><span>倾角 <b>{f(p?.tilt)}°</b></span><span>实际方位 <b>{f(p?.servo_deg[0])}°</b></span><span>实际俯仰 <b>{f(p?.servo_deg[1])}°</b></span></div>
  {ended&&<p className="compare-ended">该次在 {f(result.duration_s)} s 提前结束，模型停留在最后一帧。</p>}
 </div>;
}

export default function Comparison({id,baseCases,candidateCases}:ComparisonProps){
 const [data,setData]=useState<ComparisonData|null>(null),[error,setError]=useState(''),[which,setWhich]=useState(0);
 const [time,setTime]=useState(0),[playing,setPlaying]=useState(false),[metric,setMetric]=useState<Metric>('tilt');
 useEffect(()=>{const controller=new AbortController();setData(null);setError('');setWhich(0);setTime(0);setPlaying(false);
  fetch(`/api/tuning/comparison/${encodeURIComponent(id)}`,{signal:controller.signal}).then(async r=>{if(!r.ok)throw new Error('读取对比记录失败');return r.json();}).then(setData).catch(e=>{if(e.name!=='AbortError')setError(e.message);});
  return()=>controller.abort();},[id]);
 const left=data?.baseline[which]||[],right=data?.candidate[which]||[],testCase=data?.cases[which];
 const end=Math.max(left.at(-1)?.time||0,right.at(-1)?.time||0);
 useEffect(()=>{if(!playing||!end)return;let last=performance.now();const timer=setInterval(()=>{const now=performance.now(),delta=(now-last)/1000;last=now;setTime(old=>{if(old+delta>=end){setPlaying(false);return end;}return old+delta;});},80);return()=>clearInterval(timer);},[playing,end]);
 const points=useMemo(()=>{const rows=new Map<number,{time:number;baseline?:number;candidate?:number}>();
  for(const [name,frames] of [['baseline',left],['candidate',right]] as const)for(const p of frames){const row=rows.get(p.time)||{time:p.time};row[name]=graphValues[metric].value(p);rows.set(p.time,row);}
  return [...rows.values()].sort((a,b)=>a.time-b.time);
 },[left,right,metric]);
 const selectCase=(n:number)=>{setPlaying(false);setTime(0);setWhich(n);};
 if(error)return <section className="panel compare-card"><h2>三维同步对比</h2><p className="tuning-inline-error" role="alert">{error}</p></section>;
 if(!data)return <section className="panel compare-card"><h2>三维同步对比</h2><p className="field-note">正在读取真实仿真轨迹…</p></section>;
 if(!testCase||!left.length||!right.length||!baseCases[which]||!candidateCases[which])return <section className="panel compare-card"><h2>三维同步对比</h2><p className="tuning-inline-error">此工况没有完整的对比轨迹。</p></section>;
 return <section className="panel compare-card"><div className="compare-title"><div><span className="eyebrow">BEFORE / AFTER</span><h2>三维同步对比</h2><p>同一工况、扰动计划和随机种子；两侧均为 MuJoCo 记录的真实姿态。</p></div><label><span>对比工况</span><select aria-label="三维对比工况" value={which} onChange={e=>selectCase(Number(e.target.value))}>{data.cases.map((c,i)=><option key={i} value={i}>工况 {i+1} · X {f(c.initial_x_deg,1)}° / Y {f(c.initial_y_deg,1)}° · {caseStatus(candidateCases[i])}</option>)}</select></label></div>
 {candidateCases.some(c=>c.stabilized!==undefined)&&<><div className="case-grid-label">独立验证工况 · 按编号切换 <span>绿色：稳定　橙色：未收敛　红色：倒下</span></div><div className="case-grid">{data.cases.map((c,i)=><button key={i} className={`${caseStatus(candidateCases[i])==='稳定'?'stable':caseStatus(candidateCases[i])==='倒下'?'fallen':'unsettled'} ${which===i?'selected':''}`} aria-label={`查看工况 ${i+1}，${caseStatus(candidateCases[i])}`} title={`X ${f(c.initial_x_deg,1)}° / Y ${f(c.initial_y_deg,1)}° · ${caseStatus(candidateCases[i])}`} onClick={()=>selectCase(i)}>{i+1}</button>)}</div></>}
 <div className="compare-case-note">初始 X / Y {f(testCase.initial_x_deg,1)}° / {f(testCase.initial_y_deg,1)}°{testCase.initial_azimuth_deg!==undefined&&<> · 初始方位 / 俯仰 {f(testCase.initial_azimuth_deg,1)}° / {f(testCase.initial_elevation_deg,1)}°</>} · 种子 {testCase.seed} · 计划于 {f(testCase.disturbance_start_s,2)} s 扰动 ({f(testCase.disturbance_x_nm,4)}, {f(testCase.disturbance_y_nm,4)}) N·m，持续 {f((testCase.disturbance_duration_s??.15)*1000,0)} ms；提前结束时可能尚未施加。</div>
 {baseCases[which].preparation&&<p className="field-note">上述初始舵机位置为预定位前；固定台架、零推力准备 {f(baseCases[which].preparation?.duration_s)} s 后，释放时实际方位 / 俯仰为 {f(baseCases[which].preparation?.release_servo_deg[0])}° / {f(baseCases[which].preparation?.release_servo_deg[1])}°。回放从释放开始。</p>}
 <div className="compare-scenes"><ReplayScene name="训练前" frames={left} time={time} result={baseCases[which]} color="#ddaa78"/><ReplayScene name="训练后候选" frames={right} time={time} result={candidateCases[which]} color="#78c4ca"/></div>
 <div className="compare-controls"><button className="button secondary compact" onClick={()=>{if(time>=end)setTime(0);setPlaying(v=>!v);}} disabled={end<=0}>{playing?<Pause size={14}/>:<Play size={14}/>} {playing?'暂停':'播放'}</button><button className="icon-button" aria-label="回到开头" onClick={()=>{setPlaying(false);setTime(0);}}><RotateCcw size={14}/></button><input aria-label="回放时间" type="range" min={0} max={Math.max(end,.01)} step={.01} value={Math.min(time,end)} onChange={e=>{setPlaying(false);setTime(Number(e.target.value));}}/><span>{f(time,1)} / {f(end,1)} s</span></div>
 <div className="compare-graph-heading"><div><h3>响应曲线</h3><p><i className="compare-dot before"/>训练前 <i className="compare-dot after"/>训练后候选 · 虚线为回放位置</p></div><div className="compare-graph-tabs">{(Object.keys(graphValues) as Metric[]).map(k=><button key={k} className={metric===k?'active':''} onClick={()=>setMetric(k)}>{graphValues[k].label}</button>)}</div></div>
 <div className="compare-graph"><ResponsiveContainer width="100%" height="100%"><LineChart data={points} margin={{top:8,right:20,left:0,bottom:0}}><CartesianGrid stroke="#34434d" strokeDasharray="3 6"/><XAxis dataKey="time" type="number" domain={[0,end]} tick={{fill:'#8396a4',fontSize:10}} unit="s"/><YAxis tick={{fill:'#8396a4',fontSize:10}} unit="°" width={55}/><Tooltip contentStyle={{background:'#222b33',border:'1px solid #3c4853',borderRadius:8,fontSize:11}}/><ReferenceLine x={time} stroke="#e6edf2" strokeDasharray="4 4"/><Line name="训练前" dataKey="baseline" stroke="#ddaa78" dot={false} connectNulls isAnimationActive={false}/><Line name="训练后候选" dataKey="candidate" stroke="#78c4ca" dot={false} connectNulls isAnimationActive={false}/></LineChart></ResponsiveContainer></div>
 <div className="compare-metrics"><div><span>峰值倾角</span><b>{f(baseCases[which].peak_tilt_deg)}° → {f(candidateCases[which].peak_tilt_deg)}°</b></div><div><span>最后记录倾角</span><b>{f(baseCases[which].final_tilt_deg)}° → {f(candidateCases[which].final_tilt_deg)}°</b></div><div><span>稳定时间（持续在 2° 内）</span><b>{f(baseCases[which].settling_time_s)} s → {f(candidateCases[which].settling_time_s)} s</b></div></div>
 </section>;
}
