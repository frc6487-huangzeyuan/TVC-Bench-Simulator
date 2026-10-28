import {useEffect,useRef,useState} from 'react';
import {Activity,Download,Square} from 'lucide-react';
import {CartesianGrid,Line,LineChart,ResponsiveContainer,Tooltip,XAxis,YAxis} from 'recharts';
import './real-device.css';

type Engine=Record<string,(...args:any[])=>any>;
type Packet={v:number;type:string;seq:number;t_us:number;boot:string;firmware:string;state:string;fault:string;warning?:string;calibrating:boolean;operating_mode:string;
 control_imu?:any;imus?:any[];imu2?:any;control?:any;outputs?:any;world?:any;config_rev?:number};
type Snapshot={mode:string|null;connecting:boolean;tele:Packet|null;lastRx:number;operation:string;stopping:number;heartbeat:boolean;autoFanWanted:boolean;autoThrottleWanted:boolean};
type Entry={id:string;status:string;transport:string;count:number;started_utc:string;firmware?:string};
type Live={packet:Packet;transport:'usb'|'wifi';received:number};
type Point={time:number;x:number;y:number};
const format=(n:unknown,d=2)=>typeof n==='number'&&Number.isFinite(n)?n.toFixed(d):'—';
const vector=(v:unknown,d=2)=>Array.isArray(v)&&v.length===3?v.map(n=>format(n,d)).join(' / '):'—';
async function request(path:string,body?:unknown){const r=await fetch('/api/real'+path,body===undefined?undefined:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const data=await r.json();if(!r.ok)throw new Error(typeof data.detail==='string'?data.detail:'请求失败');return data;}

const pidFields=[['kp-x','X · Kp'],['ki-x','X · Ki'],['kd-x','X · Kd'],['kp-y','Y · Kp'],['ki-y','Y · Ki'],['kd-y','Y · Kd'],['max-tilt','最大修正 / °'],['azimuth-zero','方位零位 / °']] as const;
const throttleFields=[['throttle-base','基础油门 / %',0],['throttle-gain','倾斜增益 / %每度',1],['throttle-max','油门上限 / %',30],['throttle-deadband','倾斜死区 / °',2],['throttle-ramp','变化速度 / %每秒',10]] as const;

export default function RealDevice({backendConnected,onActivityChange}:{backendConnected:boolean;onActivityChange:(active:boolean)=>void}){
 const engine=useRef<Engine|null>(null),socket=useRef<WebSocket|null>(null),liveRef=useRef<Live|null>(null),recordId=useRef(''),trendKey=useRef('');
 const preview=useRef<((rotation:number[],azimuth:number,elevation:number,options:object)=>string)|null>(null);
 const [snap,setSnap]=useState<Snapshot|null>(null),[live,setLive]=useState<Live|null>(null),[now,setNow]=useState(performance.now()),[points,setPoints]=useState<Point[]>([]);
 const [records,setRecords]=useState<Entry[]>([]),[recording,setRecording]=useState(false),[busy,setBusy]=useState(false),[error,setError]=useState(''),[notice,setNotice]=useState('');
 const [servoX,setServoX]=useState(180),[servoY,setServoY]=useState(135),[motor,setMotor]=useState(25),[autoMotor,setAutoMotor]=useState(0),[propConfirmed,setPropConfirmed]=useState(false);
 const [pidDirty,setPidDirty]=useState(false),[pidValues,setPidValues]=useState<Record<string,number>>({'kp-x':.6,'ki-x':0,'kd-x':.03,'kp-y':.6,'ki-y':0,'kd-y':.03,'max-tilt':15,'azimuth-zero':180,'sign-x':1,'sign-y':1});
 const refresh=()=>request('/records').then(setRecords).catch(e=>setError(e.message));
 useEffect(()=>{let disposed=false;void refresh();const timer=setInterval(()=>setNow(performance.now()),500);
  const onState=(e:Event)=>setSnap((e as CustomEvent<Snapshot>).detail);
  const onNotice=(e:Event)=>{const n=(e as CustomEvent<{message:string;type:string}>).detail;if(n.type==='error')setError(n.message);else setNotice(n.message);};
  const onDisconnected=()=>{liveRef.current=null;setLive(null);setPoints([]);trendKey.current='';socket.current?.close();};
  const onTelemetry=(e:Event)=>{
   const data=(e as CustomEvent<{packet:Packet;transport:'usb'|'wifi'}>).detail;
   if(!data||!['usb','wifi'].includes(data.transport)||data.packet?.v!==10||data.packet?.type!=='telemetry')return;
   const current={packet:data.packet,transport:data.transport,received:performance.now()};liveRef.current=current;setLive(current);
   const p=current.packet,values=p.control?.error_deg;
   if(p.control?.tilt_valid&&Array.isArray(values)&&values.length>=2&&values.slice(0,2).every(Number.isFinite)){
    const key=p.boot+':'+String(p.world?.reference_id??'');const changed=key!==trendKey.current;trendKey.current=key;
    // 固件 error_deg 顺序为 roll / pitch；原页面把 pitch 作为 X。
    setPoints(old=>[...(changed?[]:old),{time:p.t_us/1e6,x:values[1],y:values[0]}].filter(v=>v.time>=p.t_us/1e6-30));
   }
   const ws=socket.current;if(ws?.readyState===WebSocket.OPEN){
    if(ws.bufferedAmount>1024*1024){setError('本地采集写入跟不上实时数据，已结束采集；请检查磁盘。');ws.close();}
    else ws.send(JSON.stringify({type:'telemetry',transport:current.transport,packet:p}));
   }
  };
  window.addEventListener('tvc-native-state',onState);window.addEventListener('tvc-native-notice',onNotice);
  window.addEventListener('tvc-native-disconnected',onDisconnected);window.addEventListener('tvc-native-telemetry',onTelemetry);
  const controlUrl='/device-console/native-control.mjs';import(/* @vite-ignore */ controlUrl).then(m=>{if(!disposed){engine.current=m as Engine;m.activate();}}).catch(e=>setError(`加载原飞控控制代码失败：${e.message}`));
  const previewUrl='/device-console/preview.mjs';import(/* @vite-ignore */ previewUrl).then(m=>{if(!disposed)preview.current=m.sceneSvg;}).catch(e=>setError(`加载姿态预览失败：${e.message}`));
  return()=>{disposed=true;clearInterval(timer);window.removeEventListener('tvc-native-state',onState);window.removeEventListener('tvc-native-notice',onNotice);window.removeEventListener('tvc-native-disconnected',onDisconnected);window.removeEventListener('tvc-native-telemetry',onTelemetry);socket.current?.close();socket.current=null;const stopping=engine.current?.deactivate();engine.current=null;void Promise.resolve(stopping).finally(()=>onActivityChange(false));};
 },[]);
 const fresh=!!live&&now-live.received<2500,tele=fresh?snap?.tele:null,connected=!!snap?.mode,opBusy=!!snap?.operation||!!snap?.stopping;
 useEffect(()=>{onActivityChange(connected||recording||!!snap?.connecting);},[connected,recording,snap?.connecting,onActivityChange]);
 useEffect(()=>{if(!backendConnected&&connected){setError('本地引擎已断开，已请求停止设备输出并断开连接。');void engine.current?.disconnect(true);}},[backendConnected,connected]);
 const outputActive=!!(tele?.outputs?.servo_enabled||tele?.outputs?.esc_armed||tele?.outputs?.esc_arming);
 const ready=!!(fresh&&tele&&!tele.fault&&!tele.calibrating&&tele.control_imu?.valid),manual=tele?.operating_mode==='MANUAL',attitude=tele?.operating_mode==='ATTITUDE';
 const world=!!tele?.world?.valid,auto=!!tele?.control?.auto_enabled,armed=!!tele?.outputs?.esc_armed,arming=!!tele?.outputs?.esc_arming,servoOn=!!tele?.outputs?.servo_enabled&&!auto;
 const servoUs=tele?.outputs?.servo_us,rotation=tele?.world?.r_bw;
 const scene=fresh&&world&&preview.current&&Array.isArray(rotation)&&rotation.length===9&&Array.isArray(servoUs)&&servoUs.length===2
  ?preview.current(rotation,(servoUs[0]-500)*360/2000,(servoUs[1]-500)*270/2000,{zero:tele?.control?.azimuth_zero_deg??180}):'';
 useEffect(()=>{if(!snap?.autoFanWanted)setAutoMotor(0);},[snap?.autoFanWanted]);
 useEffect(()=>{const p=tele,c=p?.control,pid=c?.pid;if(!fresh||pidDirty||!pid?.x||!pid?.y)return;
  setPidValues({'kp-x':pid.x.kp,'ki-x':pid.x.ki,'kd-x':pid.x.kd,'kp-y':pid.y.kp,'ki-y':pid.y.ki,'kd-y':pid.y.kd,'max-tilt':pid.output_limit_deg,'azimuth-zero':c.azimuth_zero_deg,'sign-x':c.sign_x,'sign-y':c.sign_y});
 },[tele?.config_rev,fresh,pidDirty]);
 const invoke=(label:string,fn:string,...args:any[])=>{setError('');setNotice('');const e=engine.current;if(!e){setError('控制代码尚未加载。');return;}void e.run(label,()=>e[fn](...args));};
 const applyPid=async()=>{setError('');try{if(!engine.current)throw new Error('控制代码尚未加载。');await engine.current.applyPid();setPidDirty(false);setNotice('PID 已由设备回传确认。');}catch(e){setError(e instanceof Error?e.message:String(e));}};
 const start=async()=>{if(!liveRef.current||performance.now()-liveRef.current.received>=2500)return;setBusy(true);setError('');let started=false;
  try{const meta=await request('/record/start',{transport:liveRef.current.transport});started=true;recordId.current=meta.id;
   const ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/api/real/record/ws/${encodeURIComponent(meta.id)}`);socket.current=ws;
   await new Promise<void>((resolve,reject)=>{ws.onopen=()=>resolve();ws.onerror=()=>reject(new Error('无法连接本地采集服务'));});
   ws.onmessage=e=>{try{const p=JSON.parse(e.data);if(p.type==='error')setError(p.message);}catch{setError('采集服务返回了无法识别的数据');}};
   ws.onclose=()=>{if(socket.current===ws)socket.current=null;setRecording(false);setBusy(false);void refresh();};
   if(!liveRef.current||performance.now()-liveRef.current.received>=2500||liveRef.current.transport!==meta.transport){ws.send(JSON.stringify({type:'stop'}));throw new Error('设备连接在采集启动时中断，请重新连接。');}
   setRecording(true);
  }catch(e){socket.current?.close();socket.current=null;if(started)await request('/record/stop',{}).catch(()=>{});setError(e instanceof Error?e.message:String(e));}finally{setBusy(false);}
 };
 const stop=()=>{const ws=socket.current;setBusy(true);if(ws?.readyState===WebSocket.OPEN)ws.send(JSON.stringify({type:'stop'}));else{ws?.close();void request('/record/stop',{}).then(refresh).catch(e=>setError(e.message)).finally(()=>setBusy(false));}setRecording(false);};
 const sensor=(s:any)=>s?`${s.status??'—'} · ${format(s.sample_hz,1)} Hz · ${format(s.temperature??s.temp_c,1)} ℃`:'等待数据';
 return <div className="real-page"><div className="page-heading"><div><div className="eyebrow">REAL DEVICE</div><h1>真实飞控与数据采集</h1><p>使用原飞控协议连接、控制与记录设备；实时数据和仿真分开。</p></div><div className="heading-actions"><button className="button secondary" disabled={connected||!!snap?.connecting||!engine.current} onClick={()=>invoke('','connectUsb')}>连接 USB</button><button className="button secondary" disabled={connected||!!snap?.connecting||!engine.current} onClick={()=>invoke('','connectWifi')}>连接 Wi-Fi</button><button className="button secondary" disabled={!connected} onClick={()=>invoke('','disconnect')}>断开</button><button className="button danger" disabled={!connected} onClick={()=>invoke('舵机和电机已全部停止。','stopAll')}>全部停止</button></div></div>
  {error&&<p className="tuning-inline-error" role="alert">{error}</p>}{notice&&<p className="field-note" role="status">{notice}</p>}
  <section className="panel real-overview"><div className="real-heading"><div><h2>真实设备状态</h2><p>{fresh?`已连接 · ${live?.transport==='usb'?'USB':'Wi-Fi'} · 固件 ${live?.packet.firmware}`:connected?'数据过期':'未连接真实设备'}</p></div><div className="real-actions">{recording?<button className="button secondary" onClick={stop}><Square size={14}/>结束采集</button>:<button className="button primary" onClick={start} disabled={!fresh||busy}><Activity size={14}/>{busy?'准备采集…':'开始采集'}</button>}</div></div>
   <div className="real-metrics"><div><span>设备状态 / 故障</span><b>{fresh?`${tele?.state} / ${tele?.fault||tele?.warning||'无'}`:'—'}</b></div><div><span>控制 IMU</span><b>{fresh&&tele?.control?.source_id?'IMU'+tele.control.source_id:'—'}</b></div><div><span>舵机方位 / 俯仰指令</span><b>{fresh?`${format(tele?.outputs?.servo_deg?.[0])}° / ${format(tele?.outputs?.servo_deg?.[1])}°`:'—'}</b></div><div><span>电调 PWM 指令</span><b>{fresh?`${format(tele?.outputs?.esc_us,0)} µs`:'—'}</b></div></div>
   <p className="field-note">{recording?`正在采集 · ${recordId.current}`:'连接真实飞控并收到遥测后可开始采集。'} 舵机角和电调 PWM 是设备输出指令，并非实测位置或转速；设备断开或离开此页面时结束采集。</p>
   <div className="real-metrics"><div><span>姿态估计 Roll / Pitch / Yaw</span><b>{fresh&&world?(tele?.world?.euler_deg||[]).map((n:number)=>format(n,1)+'°').join(' / '):'—'}</b></div><div><span>控制源加速度 · g</span><b>{fresh&&tele?.control_imu?.valid?(tele.control_imu.a||[]).map((n:number)=>format(n,2)).join(' / '):'—'}</b></div><div><span>控制源角速度 · °/s</span><b>{fresh&&tele?.control_imu?.valid?(tele.control_imu.g||[]).map((n:number)=>format(n,1)).join(' / '):'—'}</b></div><div><span>PID 输出受限</span><b>{fresh&&tele?.control?.pid?(tele.control.pid.limited?'是':'否'):'—'}</b></div></div>
   <div className="real-chart">{points.length?<ResponsiveContainer width="100%" height="100%"><LineChart data={points}><CartesianGrid stroke="#34434d" strokeDasharray="3 6"/><XAxis dataKey="time" type="number" domain={['dataMin','dataMax']} tickFormatter={v=>format(v,1)+'s'} tick={{fill:'#8396a4',fontSize:10}}/><YAxis tick={{fill:'#8396a4',fontSize:10}} unit="°"/><Tooltip contentStyle={{background:'#222b33',border:'1px solid #3c4853',borderRadius:8}}/><Line name="X 控制误差" dataKey="x" stroke="#ddaa78" dot={false} isAnimationActive={false}/><Line name="Y 控制误差" dataKey="y" stroke="#78c4ca" dot={false} isAnimationActive={false}/></LineChart></ResponsiveContainer>:<div className="tuning-empty">等待真实飞控遥测；此处不显示仿真数据。</div>}</div>
  </section>
  <section className="panel real-preview"><h2>姿态与云台指向预览</h2><p className="field-note">箭体使用飞控回传的姿态估计，云台使用 PWM 对应指令角；图形不代表舵机实际位置或实测推力。</p>{scene?<svg viewBox="0 0 1000 530" role="img" aria-label="真实设备姿态估计与云台指令预览" dangerouslySetInnerHTML={{__html:scene}}/>:<div className="real-preview-empty">等待有效的设备姿态与舵机指令。</div>}</section>
  <section className="panel real-controls"><div className="real-heading"><div><h2>飞控操作</h2><p>设备的校准、模式切换与控制状态均以固件回传为准。</p></div><div className="real-actions"><button className="button secondary compact" disabled={!fresh||opBusy} onClick={()=>invoke('已切换到姿态模式。','setView','auto')}>姿态回中</button><button className="button secondary compact" disabled={!fresh||opBusy} onClick={()=>invoke('已切换到手动模式。','setView','manual')}>手动与行程</button></div></div>
   <div className="real-control-grid"><div><h3>姿态与 IMU</h3><p className="field-note">当前模式：{fresh?(attitude?'姿态回中':'手动与行程'):'等待设备'} · 安装偏角 {fresh?format(tele?.control?.mount_yaw_deg,1)+'°':'—'}</p>
    <div className="real-actions"><button className="button secondary compact" disabled={!(fresh&&attitude&&!tele?.calibrating&&!outputActive&&!opBusy)} onClick={()=>invoke('正在校准，请保持竖直静止。','recalibrate')}>竖直静止校准</button><button className="button secondary compact" disabled={!(fresh&&!opBusy&&!outputActive&&!tele?.calibrating)} onClick={()=>invoke('已切换控制源，等待校准。','switchImuSource',tele?.control?.source_id===1?2:1)}>切换至 IMU{tele?.control?.source_id===1?'2':'1'}</button></div>
    <p className="field-note">IMU1：{fresh?sensor(tele?.imus?.[0]):'等待数据'}<br/>加速度 / 角速度：{fresh&&tele?.imus?.[0]?.valid?vector(tele.imus[0].a):'—'} g · {fresh&&tele?.imus?.[0]?.valid?vector(tele.imus[0].g,1):'—'} °/s<br/>IMU2：{fresh?sensor(tele?.imu2):'等待数据'}<br/>加速度 / 角速度：{fresh&&tele?.imu2?.valid?vector(tele.imu2.a_g):'—'} g · {fresh&&tele?.imu2?.valid?vector(tele.imu2.g_dps,1):'—'} °/s</p>
    <div className="real-actions"><button className="button primary compact" disabled={!(ready&&attitude&&world&&tele?.control?.mount_ready&&tele?.control?.tilt_valid&&!auto&&(!armed&&!arming||tele?.control?.auto_motor_allowed)&&!opBusy)} onClick={()=>invoke('姿态回中已启动。','startAutomatic')}>{snap?.autoThrottleWanted?'开始回中 + 自动油门':'开始姿态回中'}</button><button className="button secondary compact" disabled={!(auto||snap?.autoFanWanted||snap?.autoThrottleWanted||armed||arming||snap?.operation==='auto'||snap?.operation==='auto-motor')} onClick={()=>invoke('自动控制已停止。','stopAutomatic')}>停止自动控制</button></div>
   </div><div><h3>手动舵机</h3><div className="real-actions"><button className="button secondary compact" disabled={!(ready&&manual&&!servoOn&&!opBusy)} onClick={()=>{setServoX(180);setServoY(135);invoke('舵机已启用。','enableServos');}}>启用舵机</button><button className="button secondary compact" disabled={!(servoOn||snap?.operation==='servo')} onClick={()=>invoke('舵机已停止。','disableServos')}>停止舵机</button><button className="button secondary compact" disabled={!(manual&&ready&&servoOn)} onClick={()=>{setServoX(180);setServoY(135);invoke('已发送舵机回中指令。','centerServo');}}>机械回中</button></div>
    <label>方位 {servoX}°<input id="servo-x" type="range" min="0" max="360" value={servoX} disabled={!(manual&&ready&&servoOn)} onChange={e=>{setServoX(Number(e.target.value));engine.current?.queueServo();}}/></label>
    <label>俯仰 {servoY}°<input id="servo-y" type="range" min="85" max="185" value={servoY} disabled={!(manual&&ready&&servoOn)} onChange={e=>{setServoY(Number(e.target.value));engine.current?.queueServo();}}/></label>
   </div><div><h3>手动电机</h3><label><input id="prop-confirm" type="checkbox" checked={propConfirmed} onChange={e=>setPropConfirmed(e.target.checked)}/> 已拆除桨叶，可进行电机测试</label><label>速度指令 {motor}%<input id="motor-speed" type="range" min="0" max="100" value={motor} disabled={!(manual&&ready&&armed)} onChange={e=>{setMotor(Number(e.target.value));engine.current?.queueMotor();}}/></label>
    <div className="real-actions"><button className="button primary compact" disabled={!(ready&&manual&&propConfirmed&&!armed&&!arming&&!opBusy)} onClick={()=>invoke('电机已按当前速度启动。','startMotor')}>启动电机</button><button className="button secondary compact" disabled={!(armed||arming||tele?.outputs?.esc_signal_enabled||snap?.operation==='motor')} onClick={()=>invoke('电机已停止。','stopMotor')}>停止电机</button></div>
   </div><div><h3>姿态模式风扇</h3><label><input id="auto-motor-enable" type="checkbox" checked={!!snap?.autoFanWanted} disabled={!!snap?.autoThrottleWanted||!!tele?.control?.tilt_throttle?.enabled||!(fresh&&attitude&&((ready&&world&&!opBusy)||snap?.autoFanWanted))} onChange={()=>invoke('风扇许可已更新。','toggleAutoMotor')}/> 允许独立手动调速</label>
    <label>速度指令 {autoMotor}%<input id="auto-motor-speed" type="range" min="0" max="100" value={autoMotor} disabled={!!tele?.control?.tilt_throttle?.enabled||!(ready&&attitude&&tele?.control?.auto_motor_allowed&&snap?.autoFanWanted&&armed&&!opBusy)} onChange={e=>{setAutoMotor(Number(e.target.value));engine.current?.queueAutoMotor();}}/></label>
    <div className="real-actions"><button className="button primary compact" disabled={!!snap?.autoThrottleWanted||!!tele?.control?.tilt_throttle?.enabled||!(ready&&attitude&&world&&tele?.control?.auto_motor_allowed&&snap?.autoFanWanted&&!armed&&!arming&&!opBusy)} onClick={()=>invoke('风扇已启用。','startAutoMotor')}>启动风扇</button><button className="button secondary compact" disabled={!(attitude&&(armed||arming||snap?.operation==='auto-motor'))} onClick={()=>invoke('风扇已停止。','stopMotor')}>停止风扇</button></div>
   </div></div>
   <details><summary>姿态回中联动自动油门</summary><label><input id="tilt-throttle-enable" type="checkbox" checked={!!snap?.autoThrottleWanted} disabled={!(fresh&&attitude&&(snap?.autoThrottleWanted||(ready&&world&&!opBusy&&!outputActive&&!snap?.autoFanWanted&&tele?.control?.tilt_throttle)))} onChange={()=>invoke('自动油门许可已更新。','toggleTiltThrottle')}/> 允许倾斜自动油门</label><div className="real-input-grid">{throttleFields.map(([id,label,value])=><label key={id}>{label}<input id={id} type="number" defaultValue={value} step="0.1" disabled={opBusy||outputActive}/></label>)}</div><p className="field-note">设备回传：倾斜 {fresh?format(tele?.control?.tilt_throttle?.tilt_deg,1):'—'}° · 目标 {fresh?format(tele?.control?.tilt_throttle?.target_pct,1):'—'}% · 输出 {fresh?format(tele?.control?.tilt_throttle?.output_pct,1):'—'}%</p><button className="button secondary compact" disabled={!(armed||arming||snap?.autoThrottleWanted)} onClick={()=>invoke('电机已停止。','stopMotor')}>只停止电机</button><p className="field-note">允许后还需手动开始姿态回中；自动油门与独立调速互斥。</p></details>
   <details><summary>PID 与云台映射</summary><table className="real-pid"><thead><tr><th>轴</th><th>P / °</th><th>I / °</th><th>D / °</th><th>输出 / °</th></tr></thead><tbody>{(['x','y'] as const).map(axis=><tr key={axis}><td>{axis.toUpperCase()}</td>{(['p_deg','i_deg','d_deg','output_deg'] as const).map(term=><td key={term}>{fresh?format(tele?.control?.pid?.[axis]?.[term]):'—'}</td>)}</tr>)}</tbody></table><form id="pid-form" onSubmit={e=>{e.preventDefault();void applyPid();}}><div className="real-input-grid">{pidFields.map(([id,label])=><label key={id}>{label}<input id={id} type="number" step="0.01" value={pidValues[id]??0} onChange={e=>{setPidDirty(true);setPidValues(v=>({...v,[id]:Number(e.target.value)}));}}/></label>)}{(['x','y'] as const).map(axis=><label key={axis}>{axis.toUpperCase()} 修正方向<select id={'sign-'+axis} value={pidValues['sign-'+axis]??1} onChange={e=>{setPidDirty(true);setPidValues(v=>({...v,['sign-'+axis]:Number(e.target.value)}));}}><option value={1}>+1</option><option value={-1}>−1</option></select></label>)}</div><button className="button secondary compact" type="submit" disabled={!(fresh&&!outputActive&&!tele?.calibrating&&!opBusy&&tele?.control?.pid)}>应用 PID 与映射</button><p className="field-note">设备输出停止后应用；以设备回传确认结果，重启后恢复固件默认。</p></form></details>
  </section>
  <section className="panel real-records"><div className="real-heading"><div><h2>真实数据记录</h2><p>JSONL 保留完整原始遥测；CSV 导出常用字段。</p></div></div>{records.length?<div className="real-record-list">{records.map(r=><div key={r.id}><span><b>{r.id}</b><small>{r.transport.toUpperCase()} · {r.count} 帧 · {r.status==='recording'?'采集中':r.status==='stopped'?'已结束':r.status==='disconnected'?'连接中断':r.status}</small></span><div><a href={`/api/real/records/${encodeURIComponent(r.id)}/jsonl`}><Download size={13}/> JSONL</a><a href={`/api/real/records/${encodeURIComponent(r.id)}/csv`}><Download size={13}/> CSV</a></div></div>)}</div>:<p className="field-note">尚无真实采集记录。</p>}</section>
 </div>;
}
