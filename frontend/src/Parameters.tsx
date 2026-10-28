import {useEffect,useRef,useState} from 'react';
import {ArrowLeft,Check,ChevronRight,LoaderCircle,Plus,RotateCcw,Save,Scale,SlidersHorizontal,Trash2,Wind} from 'lucide-react';
import {CartesianGrid,Line,LineChart,ReferenceLine,ResponsiveContainer,Tooltip,XAxis,YAxis} from 'recharts';
import Scene from './Scene';
import type {BodyName,BodyOverride,Config,ParameterPreview,ThrustPoint,Triple} from './types';

const names:Record<BodyName,string>={outer_frame:'轴承内框',rocket_body:'箭体',azimuth_stage:'方位机构',thrust_unit:'推力组件'};
const f=(n:number,d=3)=>n.toLocaleString('en-US',{maximumFractionDigits:d});
const tooltip={background:'#222b33',border:'1px solid #3c4853',borderRadius:8,fontSize:11};

function Adjust({label,value,unit,min,max,step,onChange}:{label:string;value:number;unit:string;min:number;max:number;step:number;onChange:(n:number)=>void}){
 const [text,setText]=useState(String(value));
 useEffect(()=>setText(String(value)),[value]);
 return <label className="adjust-control"><span>{label}<small>{unit}</small></span><div><input aria-label={label} type="number" min={min} max={max} step={step} value={text} onChange={e=>{setText(e.target.value);const n=Number(e.target.value);if(e.target.value!==''&&Number.isFinite(n)&&n>=min&&n<=max)onChange(n);}} onBlur={()=>setText(String(value))}/><input aria-label={`${label}滑块`} type="range" min={min} max={max} step={step} value={Math.min(max,Math.max(min,value))} onChange={e=>onChange(Number(e.target.value))}/></div></label>;
}

function Curve({config,onChange}:{config:Config;onChange:(c:Config)=>void}){
 const [selected,setSelected]=useState(2),drag=useRef<number|null>(null);
 const svg=useRef<SVGSVGElement>(null),points=config.thrust_curve;
 const active=Math.min(selected,points.length-1),p=points[active];
 const maxY=Math.max(1,...points.map(p=>p.thrust_n))*1.2;
 const x=(rpm:number)=>52+rpm/config.max_rpm*500,y=(force:number)=>220-force/maxY*184;
 const setPoint=(i:number,point:ThrustPoint)=>onChange({...config,thrust_curve:points.map((p,j)=>i===j?point:p)});
 const preset=(linear=false)=>onChange({...config,thrust_mode:'rpm_curve',thrust_curve:[0,.25,.5,.75,1].map(t=>({rpm:Math.round(t*config.max_rpm),thrust_n:Number((config.max_thrust_n*(linear?t:t*t)).toFixed(3))}))});
 const curve=config.thrust_mode==='rpm_curve'?points:Array.from({length:41},(_,i)=>({rpm:config.max_rpm*i/40,thrust_n:config.max_thrust_n*(i/40)**2}));
 const top=config.thrust_mode==='rpm_curve'?maxY:Math.max(1,config.max_thrust_n)*1.2;
 const cy=(force:number)=>220-force/top*184;
 return <section className="parameter-card panel"><div className="parameter-title"><div><Wind size={17}/><h2>推力与转速</h2></div><span>PROPULSION</span></div>
  <div className="parameter-mode"><button className={config.thrust_mode==='quadratic'?'active':''} onClick={()=>onChange({...config,thrust_mode:'quadratic'})}>油门平方</button><button className={config.thrust_mode==='rpm_curve'?'active':''} onClick={()=>onChange({...config,thrust_mode:'rpm_curve'})}>转速曲线</button></div>
  <p className="parameter-help">{config.thrust_mode==='rpm_curve'?'拖动曲线上的节点，或输入实测转速和推力。节点之间采用线性插值。':'兼容已有实验：推力 = 满油门推力 × 油门²。切换到转速曲线可填写实测数据。'}</p>
  <svg ref={svg} className="thrust-editor" viewBox="0 0 590 260" role="img" aria-label="转速与推力曲线，可拖动节点调整" onPointerMove={e=>{
   const i=drag.current;if(i===null||!svg.current||i===0)return;
   const bounds=svg.current.getBoundingClientRect(),px=(e.clientX-bounds.left)/bounds.width*590,py=(e.clientY-bounds.top)/bounds.height*260;
   const rpm=i===points.length-1?config.max_rpm:Math.round(Math.min(points[i+1].rpm-1,Math.max(points[i-1].rpm+1,(px-52)/500*config.max_rpm)));
   const thrust_n=Number(Math.min(i<points.length-1?points[i+1].thrust_n:100,Math.max(points[i-1].thrust_n,(220-py)/184*maxY)).toFixed(3));
   setPoint(i,{rpm,thrust_n});
  }} onPointerUp={()=>drag.current=null} onPointerCancel={()=>drag.current=null}>
   {[0,.25,.5,.75,1].map(t=><g key={t}><line x1="52" x2="552" y1={220-t*184} y2={220-t*184} stroke="#34404a" strokeDasharray="3 5"/><text x="43" y={224-t*184} textAnchor="end">{f(t*top,1)}</text><text x={52+t*500} y="239" textAnchor="middle">{f(t*config.max_rpm,0)}</text></g>)}
   <text x="11" y="20" className="chart-axis-title">推力 N</text><text x="552" y="256" textAnchor="end" className="chart-axis-title">转速 RPM</text>
   <path d={`M${curve.map(p=>`${x(p.rpm)},${cy(p.thrust_n)}`).join(' L')} L552,220 L52,220 Z`} fill="#e1ac7610"/>
   <polyline points={curve.map(p=>`${x(p.rpm)},${cy(p.thrust_n)}`).join(' ')} fill="none" stroke="#e1ac76" strokeWidth="2.5"/>
   {config.thrust_mode==='rpm_curve'&&points.map((p,i)=><circle key={i} cx={x(p.rpm)} cy={y(p.thrust_n)} r={active===i?7:5} fill={active===i?'#e1ac76':'#20272e'} stroke="#e1ac76" strokeWidth="2" role="button" tabIndex={0} aria-label={`曲线节点 ${i+1}`} onKeyDown={e=>{if(e.key==='Enter'||e.key===' ')setSelected(i);}} onPointerDown={e=>{setSelected(i);drag.current=i;e.currentTarget.setPointerCapture(e.pointerId);}}/>) }
  </svg>
  <div className="parameter-grid"><Adjust label="最高转速" value={config.max_rpm} unit="RPM" min={100} max={100000} step={100} onChange={v=>onChange({...config,max_rpm:v,thrust_curve:points.map(p=>({...p,rpm:p.rpm/config.max_rpm*v}))})}/><Adjust label="电机响应时间" value={config.motor_tau_s} unit="s" min={.005} max={2} step={.005} onChange={v=>onChange({...config,motor_tau_s:v})}/></div>
  {config.thrust_mode==='quadratic'?<Adjust label="满油门推力" value={config.max_thrust_n} unit="N" min={0} max={100} step={.1} onChange={v=>onChange({...config,max_thrust_n:v})}/>:<>
   <div className="curve-toolbar"><div>{points.map((_,i)=><button key={i} className={i===active?'active':''} onClick={()=>setSelected(i)}>{i+1}</button>)}</div><button title="插入曲线节点" disabled={points.length>=16} onClick={()=>{const i=Math.min(active,points.length-2),a=points[i],b=points[i+1];onChange({...config,thrust_curve:[...points.slice(0,i+1),{rpm:(a.rpm+b.rpm)/2,thrust_n:(a.thrust_n+b.thrust_n)/2},...points.slice(i+1)]});setSelected(i+1);}}><Plus size={14}/></button><button title="删除当前节点" disabled={active===0||active===points.length-1||points.length<=2} onClick={()=>{onChange({...config,thrust_curve:points.filter((_,i)=>i!==active)});setSelected(active-1);}}><Trash2 size={14}/></button></div>
   <div className="parameter-grid"><Adjust label={`节点 ${active+1} 转速`} value={p.rpm} unit="RPM" min={active===0?0:points[active-1].rpm+1} max={active===points.length-1?p.rpm:points[active+1].rpm-1} step={1} onChange={v=>active>0&&active<points.length-1&&setPoint(active,{...p,rpm:v})}/><Adjust label={`节点 ${active+1} 推力`} value={p.thrust_n} unit="N" min={active===0?0:points[active-1].thrust_n} max={active===0?0:active===points.length-1?100:points[active+1].thrust_n} step={.01} onChange={v=>active>0&&setPoint(active,{...p,thrust_n:v})}/></div>
   <div className="preset-row"><span>生成参考曲线</span><button onClick={()=>preset()}>平方曲线</button><button onClick={()=>preset(true)}>线性曲线</button></div>
  </>}
  <p className="parameter-footnote">转速曲线模式：油门线性映射目标 RPM，电机响应后查表得到推力。最高转速和曲线均需实测标定。</p>
 </section>;
}

export default function Parameters({config,onChange,disabled,onBack,onSaved}:{config:Config;onChange:(c:Config)=>void;disabled:boolean;onBack:()=>void;onSaved:()=>void}){
 const [preview,setPreview]=useState<ParameterPreview|null>(null),[baseline,setBaseline]=useState<ParameterPreview|null>(null),[selected,setSelected]=useState<BodyName>('rocket_body'),[error,setError]=useState(''),[saving,setSaving]=useState(false),[saved,setSaved]=useState(false),[pending,setPending]=useState(true);
 const [showInertia,setShowInertia]=useState(true);
 const modelConfig={...config,initial_x_deg:0,initial_y_deg:0};
 useEffect(()=>{let active=true;fetch('/api/parameters/preview',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({...config,initial_x_deg:0,initial_y_deg:0,mass_scale:1,cg_offset_mm:0,body_overrides:{}})}).then(async r=>{if(!r.ok)return;const data=await r.json();if(active)setBaseline(data);}).catch(()=>{});return()=>{active=false;};},[]);
 useEffect(()=>{
  const controller=new AbortController();setPending(true);setSaved(false);
  const timer=setTimeout(async()=>{try{const r=await fetch('/api/parameters/preview',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(modelConfig),signal:controller.signal});const data=await r.json();if(!r.ok)throw new Error(typeof data.detail==='string'?data.detail:(data.detail?.[0]?.msg||'参数无效'));setPreview(data);setError('');}catch(e){if(!controller.signal.aborted)setError(e instanceof Error?e.message:String(e));}finally{if(!controller.signal.aborted)setPending(false);}},180);
  return()=>{clearTimeout(timer);controller.abort();};
 },[config]);
 const change=(c:Config)=>{setSaved(false);onChange(c);};
 const body=preview?.bodies.find(b=>b.name===selected),base=baseline?.bodies.find(b=>b.name===selected),override=config.body_overrides[selected]||{};
 const setBody=(value:Partial<BodyOverride>)=>change({...config,body_overrides:{...config.body_overrides,[selected]:{...override,...value}}});
 const inertia=override.inertia_kg_m2||body?.inertia_kg_m2;
 const servo=Array.from({length:151},(_,i)=>{let a=0;for(let n=0;n<i;n++)a+=Math.max(-config.servo_speed_deg_s,Math.min(config.servo_speed_deg_s,(60-a)/config.servo_tau_s))*.01;return {time:i*.01,angle:a,target:60};});
 const save=async()=>{setSaving(true);try{const r=await fetch('/api/parameters/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(config)});const data=await r.json();if(!r.ok)throw new Error(typeof data.detail==='string'?data.detail:'参数保存失败');setSaved(true);onSaved();}catch(e){setError(e instanceof Error?e.message:String(e));}finally{setSaving(false);}};
 return <><div className="page-heading"><div><div className="eyebrow">MODEL PARAMETERS</div><h1>模型参数<span className="version-tag">可视化标定</span></h1><p>调整动力学模型，在下一次实验中验证你的假设。</p></div><div className="heading-actions"><button className="button secondary" onClick={onBack}><ArrowLeft size={15}/>实验台</button><button className="button primary" disabled={disabled||saving||pending||!!error} onClick={save}>{saving?<LoaderCircle className="spin" size={15}/>:saved?<Check size={15}/>:<Save size={15}/>} {saved?'已保存':'保存参数'}</button></div></div>
  <div className="parameter-notice"><SlidersHorizontal size={16}/><span>{disabled?'实验进行中，参数暂时锁定。停止实验后可调整。':'拖动即可预览；保存后用于下一次实验。当前参数仍为仿真假设，尚未完成实物标定。'}</span></div>
  <div className="parameter-layout"><fieldset disabled={disabled} className="parameter-editors"><Curve config={config} onChange={change}/>
   <section className="parameter-card panel"><div className="parameter-title"><div><SlidersHorizontal size={17}/><h2>舵机动态</h2></div><span>ACTUATORS</span></div><div className="parameter-grid"><Adjust label="舵机旋转速度" value={config.servo_speed_deg_s} unit="°/s" min={1} max={720} step={1} onChange={v=>change({...config,servo_speed_deg_s:v})}/><Adjust label="舵机响应时间" value={config.servo_tau_s} unit="s" min={.005} max={1} step={.005} onChange={v=>change({...config,servo_tau_s:v})}/><Adjust label="舵机扭矩上限" value={config.servo_torque_nm} unit="N·m" min={.01} max={10} step={.01} onChange={v=>change({...config,servo_torque_nm:v})}/><Adjust label="控制指令限速" value={config.slew_us} unit="µs / 10 ms" min={1} max={100} step={1} onChange={v=>change({...config,slew_us:v})}/></div>
    <div className="parameter-chart-caption">60° 阶跃参考响应 <span>空载 · 一阶限速模型</span></div><div className="servo-chart"><ResponsiveContainer width="100%" height={165}><LineChart data={servo} margin={{left:0,right:15,top:10,bottom:0}}><CartesianGrid stroke="#303b45" strokeDasharray="3 5"/><XAxis dataKey="time" tick={{fontSize:10,fill:'#81909d'}} tickFormatter={v=>`${v}s`} minTickGap={45}/><YAxis domain={[0,70]} tick={{fontSize:10,fill:'#81909d'}} tickFormatter={v=>`${v}°`} width={40}/><Tooltip contentStyle={tooltip}/><ReferenceLine y={60} stroke="#687987" strokeDasharray="4 4"/><Line dataKey="angle" name="参考角度" dot={false} stroke="#79c6c5" strokeWidth={2} isAnimationActive={false}/></LineChart></ResponsiveContainer></div>
    <p className="parameter-footnote">控制器 PWM 限速对应方位 ≤ {f(config.slew_us*18,1)}°/s、俯仰 ≤ {f(config.slew_us*13.5,1)}°/s。图示不含 PWM 限速与负载；实际运动还受惯量和扭矩影响。</p>
   </section>
   <section className="parameter-card panel"><div className="parameter-title"><div><Scale size={17}/><h2>质量、惯量与重心</h2></div><span>MASS PROPERTIES</span></div>
    <div className="body-selector">{(Object.keys(names) as BodyName[]).map(name=><button key={name} className={selected===name?'active':''} onClick={()=>setSelected(name)}>{names[name]}</button>)}</div>
    {body&&<><div className="body-summary"><span>{names[selected]}<small>{selected}</small></span><button className="text-button" onClick={()=>{const next={...config.body_overrides};delete next[selected];change({...config,body_overrides:next});}}><RotateCcw size={13}/>恢复该刚体</button></div>
     <Adjust label={`${names[selected]}质量`} value={override.mass_kg??body.mass_kg} unit="kg" min={.001} max={Math.max(10,(base?.mass_kg||1)*5)} step={.001} onChange={v=>setBody({mass_kg:v})}/>
     <div className="parameter-subtitle">质心处主惯量 <small>kg·m² · CAD 惯性主轴</small></div><p className="parameter-help">未覆盖时随质量同比缩放；输入后独立生效。三项须满足三角不等式。</p>
     <div className="inertia-inputs">{inertia?.map((v,i)=><Adjust key={i} label={`主惯量 I${i+1}`} value={v} unit="kg·m²" min={.00000001} max={Math.max(.02,(base?.inertia_kg_m2[i]||.01)*5)} step={.000001} onChange={value=>{const next=[...inertia] as Triple;next[i]=value;setBody({inertia_kg_m2:next});}}/>)}</div><button className="text-button" onClick={()=>{const next={...override};delete next.inertia_kg_m2;change({...config,body_overrides:{...config.body_overrides,[selected]:next}});}}>恢复按质量缩放的惯量<ChevronRight size={13}/></button>
     <div className="parameter-subtitle">重心偏移 <small>mm · 零位机体系 X / Y / Z</small></div><div className="cg-controls">{(['X','Y','Z'] as const).map((axis,i)=><Adjust key={axis} label={`重心 ${axis} 偏移`} value={override.cg_offset_mm?.[i]??0} unit="mm" min={-200} max={200} step={1} onChange={v=>{const next=[...(override.cg_offset_mm||[0,0,0])] as Triple;next[i]=v;setBody({cg_offset_mm:next});}}/>)}</div>
     <p className="parameter-footnote">偏移相对基准 CAD 质心，以箭头方向为 +Z。仅移动质量分布，不移动网格。惯量定义在质心处；转轴惯量由物理引擎计入平行轴效应。</p>
    </>}
    <details><summary>整体倍率与其他参数</summary><div className="parameter-grid"><Adjust label="原有质量倍率" value={config.mass_scale} unit="×" min={.1} max={5} step={.05} onChange={v=>change({...config,mass_scale:v})}/><Adjust label="原有箭体轴向偏移" value={config.cg_offset_mm} unit="mm" min={-100} max={100} step={1} onChange={v=>change({...config,cg_offset_mm:v})}/><Adjust label="轴承阻尼" value={config.bearing_damping} unit="N·m·s" min={0} max={1} step={.001} onChange={v=>change({...config,bearing_damping:v})}/></div><p className="parameter-help">质量倍率作用于箭体和 TVC 两个刚体。单体质量覆盖优先；原有轴向偏移与三轴偏移叠加。</p></details>
   </section>
  </fieldset><aside className="parameter-preview"><section className="panel"><div className="preview-heading"><span>质量分布预览</span><span>{pending?<LoaderCircle className="spin" size={12}/>:<Check size={12}/>} {pending?'更新中':'零位模型'}</span></div><Scene frame={preview?.frame||null} overlay={preview?{body:preview.bodies.find(b=>b.name===selected)!,totalCom:preview.com_world,showInertia}:undefined}/><div className="preview-legend"><span><i className="cg-marker"/>所选刚体质心</span><span><i className="total-marker"/>运动部分总重心</span><label><input type="checkbox" checked={showInertia} onChange={e=>setShowInertia(e.target.checked)}/>惯量椭球</label></div></section>
   {error&&<div className="parameter-error" role="alert">{error}<small>请修正参数。预览保留上次有效模型，保存暂不可用。</small></div>}
   {preview&&<section className="parameter-card panel"><div className="parameter-title"><div><Scale size={15}/><h2>当前模型</h2></div></div><div className="parameter-total"><strong>{f(preview.total_mass_kg)}<small>kg</small></strong><span>运动部分总质量 · 不含固定底座</span></div><div className="mass-bars">{preview.bodies.map(b=><button key={b.name} onClick={()=>setSelected(b.name)} className={selected===b.name?'active':''}><span>{names[b.name]}</span><div><i style={{width:`${100*b.mass_kg/preview.total_mass_kg}%`}}/></div><b>{f(b.mass_kg)} kg</b></button>)}</div><div className="parameter-subtitle">总重心坐标 <small>相对 body_reference · mm</small></div><div className="com-readout">{preview.com_body_mm.map((v,i)=><span key={i}>{['X','Y','Z'][i]}<b>{f(v,1)}</b></span>)}</div><p className="parameter-footnote">椭球是与主惯量等效的均匀椭球，显示质量分布尺度，非碰撞体或真实外形。</p></section>}
   <div className="parameter-footnote">参数保存至本地工程；每次实验记录曲线节点、刚体覆盖值和源码指纹，便于复现与比较。</div>
  </aside></div>
 </>;
}
