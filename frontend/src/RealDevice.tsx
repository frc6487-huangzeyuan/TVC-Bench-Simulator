import {useCallback,useEffect,useRef,useState} from 'react';
import {Activity,Download,Square,Usb,Wifi} from 'lucide-react';
import {CartesianGrid,Line,LineChart,ResponsiveContainer,Tooltip,XAxis,YAxis} from 'recharts';
import ServoCalibrationPanel,{type ServoCalibration} from './ServoCalibrationPanel';
import ServoTravelPanel,{type ServoTravel} from './ServoTravelPanel';
import './real-device.css';
import Scene from './Scene';
import {deviceFrame,deviceServoAngles,deviceTilt} from './devicePose';
import type {DeviceModelBody} from './devicePose';

type Engine=Record<string,(...args:any[])=>any>;
type Packet={v:number;type:string;seq:number;t_us:number;boot:string;firmware:string;state:string;fault:string;warning?:string;calibrating:boolean;operating_mode:string;
 control_imu?:any;imus?:any[];imu2?:any;control?:any;outputs?:any;world?:any;config_rev?:number};
type Snapshot={mode:string|null;connecting:boolean;tele:Packet|null;lastRx:number;operation:string;stopping:number;heartbeat:boolean;autoFanWanted:boolean;autoThrottleWanted:boolean;mountBaseline?:boolean;mountTilted?:boolean;mountError?:string;mountAngle?:number|null};
type Entry={id:string;status:string;transport:string;count:number;started_utc:string;firmware?:string};
type Live={packet:Packet;transport:'usb'|'wifi';received:number};
type Point={time:number;x:number;y:number};
type CgServo={enabled:boolean;command_us:number;direction:-1|0|1;max_us?:number};
const format=(n:unknown,d=2)=>typeof n==='number'&&Number.isFinite(n)?n.toFixed(d):'—';
const vector=(v:unknown,d=2)=>Array.isArray(v)&&v.length===3?v.map(n=>format(n,d)).join(' / '):'—';
async function request(path:string,body?:unknown){const r=await fetch('/api/real'+path,body===undefined?undefined:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const data=await r.json();if(!r.ok)throw new Error(typeof data.detail==='string'?data.detail:'请求失败');return data;}

const pidFields=[['kp-x','X · Kp'],['ki-x','X · Ki'],['kd-x','X · Kd'],['kp-y','Y · Kp'],['ki-y','Y · Ki'],['kd-y','Y · Kd'],['max-tilt','最大修正 / °'],['azimuth-zero','方位零位 / °'],['trim-x','X 配平偏转 / °'],['trim-y','Y 配平偏转 / °']] as const;
const throttleFields=[['throttle-base','基础油门 / %',0],['throttle-gain','倾斜增益 / %每度',1],['throttle-rate-gain','角速度增益 / %每（°/s）',0.02],['throttle-max','油门上限 / %',30],['throttle-deadband','倾斜死区 / °',2],['throttle-ramp','变化速度 / %每秒',10]] as const;
const torqueFields=[
 ['full_torque_nm','满油门推力 × 有效力臂 / N·m',0.001,1000],
 ['base_pct','基准油门 / %',0.1,100],['max_pct','油门上限 / %',0.1,100],
 ['kp_x','X · Kp',0.0001,100],['ki_x','X · Ki',0,100],['kd_x','X · Kd',0,100],
 ['kp_y','Y · Kp',0.0001,100],['ki_y','Y · Ki',0,100],['kd_y','Y · Kd',0,100],
] as const;
const feedforwardFields=['X 配平 / N·m','Y 配平 / N·m','X·X','X·Y','Y·X','Y·Y'];
const mountDirections=[
 [0,'↑ +X','沿内轴轴承连线，朝 Type-C 较近的一端','up'],
 [1,'↓ −X','沿内轴轴承连线，朝 Type-C 较远的一端','down'],
 [2,'← +Y','沿外轴轴承连线，朝图示左侧','left'],
 [3,'→ −Y','沿外轴轴承连线，朝图示右侧','right'],
] as const;

export default function RealDevice({backendConnected,onActivityChange}:{backendConnected:boolean;onActivityChange:(active:boolean)=>void}){
 const engine=useRef<Engine|null>(null),socket=useRef<WebSocket|null>(null),liveRef=useRef<Live|null>(null),recordId=useRef(''),trendKey=useRef('');
 const [model,setModel]=useState<DeviceModelBody[]|null>(null),[modelError,setModelError]=useState('');
 const [snap,setSnap]=useState<Snapshot|null>(null),[live,setLive]=useState<Live|null>(null),[now,setNow]=useState(performance.now()),[points,setPoints]=useState<Point[]>([]);
 const [records,setRecords]=useState<Entry[]>([]),[recording,setRecording]=useState(false),[busy,setBusy]=useState(false),[error,setError]=useState(''),[notice,setNotice]=useState('');
 const [servoX,setServoX]=useState(180),[servoY,setServoY]=useState(0),[motor,setMotor]=useState(25),[autoMotor,setAutoMotor]=useState(0),[propConfirmed,setPropConfirmed]=useState(false);
 const [showServoCalibration,setShowServoCalibration]=useState(false);
 const [mountDirection,setMountDirection]=useState(0);
 const [torqueValues,setTorqueValues]=useState<Record<string,string>>({});
 const [feedforward,setFeedforward]=useState<string[]>(Array(6).fill('0'));
 const [targetX,setTargetX]=useState('0'),[targetY,setTargetY]=useState('0');
 const [pidDirty,setPidDirty]=useState(false),[pidValues,setPidValues]=useState<Record<string,number>>({'kp-x':.6,'ki-x':0,'kd-x':.03,'kp-y':.6,'ki-y':0,'kd-y':.03,'max-tilt':15,'azimuth-zero':180,'sign-x':1,'sign-y':-1});
 const refresh=()=>request('/records').then(setRecords).catch(e=>setError(e.message));
 useEffect(()=>{
  const abort=new AbortController();
  fetch('/api/real/model',{signal:abort.signal}).then(async r=>{
   if(!r.ok)throw new Error(`台架模型读取失败（HTTP ${r.status}）`);
   setModel(await r.json());
  }).catch(e=>{if(!abort.signal.aborted)setModelError(e.message);});
  return()=>abort.abort();
 },[]);
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
  return()=>{disposed=true;clearInterval(timer);window.removeEventListener('tvc-native-state',onState);window.removeEventListener('tvc-native-notice',onNotice);window.removeEventListener('tvc-native-disconnected',onDisconnected);window.removeEventListener('tvc-native-telemetry',onTelemetry);socket.current?.close();socket.current=null;const stopping=engine.current?.deactivate();engine.current=null;void Promise.resolve(stopping).finally(()=>onActivityChange(false));};
 },[]);
 const fresh=!!live&&now-live.received<2500,tele=fresh?snap?.tele:null,connected=!!snap?.mode;
 const servoTravel=tele?.control?.servo_travel as ServoTravel|undefined;
 const releaseServoTravel=useCallback(async()=>{await engine.current?.releaseServoTravel();},[]);
 const travelActive=!!servoTravel&&servoTravel.axis>=0;
 const commandBusy=!!snap?.operation||!!snap?.stopping,opBusy=commandBusy||travelActive;
 useEffect(()=>{onActivityChange(connected||recording||!!snap?.connecting);},[connected,recording,snap?.connecting,onActivityChange]);
 useEffect(()=>{if(!backendConnected&&connected){setError('本地引擎已断开，已请求停止设备输出并断开连接。');void engine.current?.disconnect(true);}},[backendConnected,connected]);
 const outputActive=!!(tele?.outputs?.servo_enabled||tele?.outputs?.esc_armed||tele?.outputs?.esc_arming);
 const ready=!!(fresh&&tele&&!tele.fault&&!tele.calibrating&&tele.control_imu?.valid),manual=tele?.operating_mode==='MANUAL',attitude=tele?.operating_mode==='ATTITUDE';
 const world=!!tele?.world?.valid,auto=!!tele?.control?.auto_enabled,armed=!!tele?.outputs?.esc_armed,arming=!!tele?.outputs?.esc_arming,servoOn=!!tele?.outputs?.servo_enabled&&!auto&&!travelActive;
 const torque=tele?.control?.thrust_torque;
 const torqueReady=ready&&attitude&&world&&tele?.control?.mount_ready&&tele?.control?.tilt_valid;
 const targetsValid=[targetX,targetY].every(v=>v.trim()!==''&&Number.isFinite(Number(v))&&Math.abs(Number(v))<=20);
 const cg=tele?.outputs?.cg_servo as CgServo|undefined,cgMoving=!!cg?.direction,cgJogging=snap?.operation==='cg-jog';
 const mountSupported=tele?.control?.mount_method==='GUIDED_TILT',mountSaved=tele?.control?.mount_reference_ready===true;
 const canMount=!!(fresh&&mountSupported&&attitude&&tele?.control_imu?.valid&&!opBusy&&!tele.calibrating&&!outputActive&&!cg?.enabled);
 const mountAngle=snap?.mountAngle,hasUpright=!!snap?.mountBaseline,hasLean=!!snap?.mountTilted;
 const mountDirectionHint=mountDirections[mountDirection][2];
 const mountHint=!fresh?'连接飞控后可标定。':!mountSupported?'引导式标定需要 2.5.9 或支持该功能的新版固件。':
  !attitude?'请先切换到姿态回中模式。':outputActive||cg?.enabled?'请先点击「全部停止」。':
  !tele?.control_imu?.valid?'等待当前 IMU 的有效数据。':tele.calibrating?'正在静止校准，请等待结束。':
  snap?.operation==='mount-save'?'正在等待飞控确认保存…':snap?.operation?.startsWith('mount-')?'正在采集，请保持静止…':snap?.mountError?snap.mountError:
  hasLean?'倾斜姿态已采集，请回到竖直，静止后保存。':hasUpright?'竖直姿态已采集，请沿所选方向倾斜。':
  mountSaved?'当前 IMU 已保存安装方向。可小幅向所选轴试倾，核对 X/Y 读数。':'让箭体竖直，静止后采集。';
 const canAdjustCg=cg?.max_us===2000&&ready&&!armed&&!arming&&!opBusy;
 const canJogCg=cg?.max_us===2000&&cg.enabled&&ready&&!armed&&!arming&&(!opBusy||cgJogging);
 const servoUs=tele?.outputs?.servo_us,rotation=tele?.world?.r_bw;
 const azimuthCenter=tele?.control?.azimuth_zero_deg??180;
 const azimuthLimits=tele?.outputs?.azimuth_limits_deg??[0,360];
 const servoCalibration=tele?.control?.servo_calibration as ServoCalibration|undefined;
 const servoPwm=tele?.outputs?.servo_pwm_us;
 const canCaptureServo=manual&&servoOn&&!opBusy&&Array.isArray(servoPwm)&&servoPwm.length===2&&servoPwm.every((v:unknown)=>typeof v==='number'&&v>=500&&v<=2500);
 const wasServoOn=useRef(false);
 useEffect(()=>{
  const enabled=manual&&servoOn;
  if(enabled&&!wasServoOn.current&&Array.isArray(servoUs)&&servoUs.length===2){
   setServoX(Math.round((servoUs[0]-500)*360/2000));
   setServoY(Math.round(((servoUs[1]-500)*270/2000-135)*10)/10);
  }
  wasServoOn.current=enabled;
 },[manual,servoOn,servoUs]);
 const tilt=deviceTilt(world,tele?.world?.r_bw),servo=travelActive?null:deviceServoAngles(tele?.outputs?.servo_deg,tele?.outputs?.servo_us);
 const frame=model?deviceFrame(model,tilt,servo):null;
 const transport=live?.transport==='usb'||snap?.mode==='serial'?'USB':'Wi-Fi';
 const linkStatus=snap?.connecting?'正在连接':!connected?'未连接':fresh?`${transport} 已连接`:live?'遥测超时':`等待 ${transport} 遥测`;
 const modelStatus=tilt?(servo?'实时倾斜与舵机指令':'实时倾斜 · 云台参考位'):servo?'舵机指令 · 箭体参考位':'参考姿态';
 useEffect(()=>{if(!snap?.autoFanWanted)setAutoMotor(0);},[snap?.autoFanWanted]);
 useEffect(()=>{const p=tele,c=p?.control,pid=c?.pid;if(!fresh||pidDirty||!pid?.x||!pid?.y)return;
  setPidValues({'kp-x':pid.x.kp,'ki-x':pid.x.ki,'kd-x':pid.x.kd,'kp-y':pid.y.kp,'ki-y':pid.y.ki,'kd-y':pid.y.kd,'max-tilt':pid.output_limit_deg,'azimuth-zero':c.azimuth_zero_deg,'sign-x':c.sign_x,'sign-y':c.sign_y,'trim-x':pid.trim_x_deg??0,'trim-y':pid.trim_y_deg??0});
 },[tele?.config_rev,fresh,pidDirty]);
 const invoke=(label:string,fn:string,...args:any[])=>{setError('');setNotice('');const e=engine.current;if(!e){setError('控制代码尚未加载。');return;}void e.run(label,()=>e[fn](...args));};
 const releaseCg=()=>{const e=engine.current;if(e)void e.run('',()=>e.releaseCgServo());};
 const cgJogButton=(direction:1|-1,label:string)=><button className="button secondary compact cg-jog" disabled={!canJogCg||(!cgJogging&&(direction>0?cg.command_us>=2000:cg.command_us<=500))}
  onPointerDown={e=>{if(!e.isPrimary||e.button!==0)return;e.currentTarget.setPointerCapture(e.pointerId);invoke('','jogCgServo',direction);}}
  onPointerUp={releaseCg} onPointerCancel={releaseCg} onLostPointerCapture={releaseCg} onBlur={releaseCg}
  onKeyDown={e=>{if(e.key!==' '&&e.key!=='Enter')return;e.preventDefault();if(!e.repeat)invoke('','jogCgServo',direction);}}
  onKeyUp={e=>{if(e.key===' '||e.key==='Enter'){e.preventDefault();releaseCg();}}}>{label}</button>;
 const applyPid=async()=>{setError('');try{if(!engine.current)throw new Error('控制代码尚未加载。');await engine.current.applyPid();setPidDirty(false);setNotice('PID 已由设备回传确认。');}catch(e){const message=e instanceof Error?e.message:String(e);if(engine.current?.responseNoticeType(e)==='warning')setNotice(message);else setError(message);}};
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
 return <div className="real-page">
  <div className="page-heading"><div><h1>真实飞控与数据采集</h1><p>通过 USB 或 Wi-Fi 连接飞控，查看姿态与执行器指令。</p></div></div>
  {error&&<p className="tuning-inline-error" role="alert">{error}</p>}{notice&&<p className="field-note" role="status">{notice}</p>}
  <div className="real-workbench">
   <figure className="panel real-scene" aria-label="真实飞控台架模型">
    {frame?<Scene frame={frame} deviceStatus={{text:modelStatus,live:!!(tilt||servo)}}/>:<div className="scene-wrap"><div className={`scene-loading${modelError?' error':''}`}>{modelError||'正在载入台架模型…'}</div></div>}
    <figcaption>台架显示姿态估计的倾斜，云台显示舵机指令角。舵机实际位置与推力需由实物测量。重心滑架显示参考位置。</figcaption>
   </figure>
   <section className="panel real-connection" aria-labelledby="real-connection-title">
    <div className="real-heading"><div><h2 id="real-connection-title">设备连接</h2><p>{tele?.firmware?`固件 ${tele.firmware}`:connected?'等待有效的设备数据':'Wi-Fi 连接前，请先加入 TVC-Bench 热点。'}</p></div><span className={`state-pill ${fresh?'':connected&&live?'error':'stopped'}`} role="status"><i/>{linkStatus}</span></div>
    <div className="real-connect-actions">
     {connected?<><button className="button secondary" onClick={()=>invoke('','disconnect')}>断开连接</button><button className="button danger" onClick={()=>invoke('舵机和电机已全部停止。','stopAll')}>全部停止</button></>:<><button className="button primary" disabled={!!snap?.connecting||!engine.current} onClick={()=>invoke('','connectUsb')}><Usb size={16}/>连接 USB</button><button className="button secondary" disabled={!!snap?.connecting||!engine.current} onClick={()=>invoke('','connectWifi')}><Wifi size={16}/>连接 Wi-Fi</button></>}
    </div>
    <div className="real-metrics">
     <div><span>飞控状态</span><b>{tele?.state??'—'}</b></div><div><span>故障 / 告警</span><b>{tele?(tele.fault||tele.warning||'无'):'—'}</b></div>
     <div><span>控制 IMU</span><b>{tele?.control?.source_id?'IMU'+tele.control.source_id:'—'}</b></div><div><span>电调 PWM 指令</span><b>{tele?`${format(tele.outputs?.esc_us,0)} µs`:'—'}</b></div>
     <div className="real-metric-wide"><span>舵机方位 / 俯仰指令</span><b>{servo?`${format(servo[0],1)}° / ${format(servo[1],1)}°`:'—'}</b></div>
    </div>
    <div className="real-capture"><p className="field-note">{recording?`正在采集 · ${recordId.current}`:'收到遥测后可开始采集。'}</p>{recording?<button className="button secondary" onClick={stop}><Square size={14}/>结束采集</button>:<button className="button secondary" onClick={start} disabled={!fresh||busy}><Activity size={14}/>{busy?'准备采集…':'开始采集'}</button>}</div>
   </section>
  </div>
  <section className="panel real-overview"><div className="real-heading"><div><h2>设备遥测</h2><p>显示最近 30 秒的姿态控制误差。</p></div></div>
   <div className="real-metrics"><div><span>姿态估计 Roll / Pitch / Yaw</span><b>{fresh&&world?(tele?.world?.euler_deg||[]).map((n:number)=>format(n,1)+'°').join(' / '):'—'}</b></div><div><span>控制源加速度 · g</span><b>{fresh&&tele?.control_imu?.valid?(tele.control_imu.a||[]).map((n:number)=>format(n,2)).join(' / '):'—'}</b></div><div><span>控制源角速度 · °/s</span><b>{fresh&&tele?.control_imu?.valid?(tele.control_imu.g||[]).map((n:number)=>format(n,1)).join(' / '):'—'}</b></div><div><span>PID 输出受限</span><b>{fresh&&tele?.control?.pid?(tele.control.pid.limited?'是':'否'):'—'}</b></div></div>
   <div className="real-chart">{points.length?<ResponsiveContainer width="100%" height="100%"><LineChart data={points}><CartesianGrid stroke="#34434d" strokeDasharray="3 6"/><XAxis dataKey="time" type="number" domain={['dataMin','dataMax']} tickFormatter={v=>format(v,1)+'s'} tick={{fill:'#8396a4',fontSize:10}}/><YAxis tick={{fill:'#8396a4',fontSize:10}} unit="°"/><Tooltip contentStyle={{background:'#222b33',border:'1px solid #3c4853',borderRadius:8}}/><Line name="X 控制误差" dataKey="x" stroke="#ddaa78" dot={false} isAnimationActive={false}/><Line name="Y 控制误差" dataKey="y" stroke="#78c4ca" dot={false} isAnimationActive={false}/></LineChart></ResponsiveContainer>:<div className="tuning-empty">等待真实飞控遥测；此处不显示仿真数据。</div>}</div>
  </section>
  <section className="panel imu-mount" aria-labelledby="imu-mount-title">
   <div className="real-heading"><div><h2 id="imu-mount-title">IMU 安装方向标定</h2><p>{tele?.control?.source_id?'当前控制源 · IMU'+tele.control.source_id:'每路 IMU 单独标定'}</p></div><span className="status-pill">{mountSaved?'方向已保存':!fresh?'等待设备':mountSupported?'待标定':'固件待更新'}</span></div>
   <div className="imu-mount-layout">
    <fieldset className="imu-mount-directions" disabled={opBusy||hasLean}>
     <legend>箭头倾斜方向 · 台架俯视</legend>
     <div className="imu-mount-compass">{mountDirections.map(([direction,label,detail,position])=>
      <label key={direction} className={position} title={detail}><input type="radio" name="mount-direction" checked={mountDirection===direction} onChange={()=>setMountDirection(direction)}/><span>{label}</span></label>)}
      <span className="imu-mount-pivot" aria-hidden="true">●</span>
     </div>
    </fieldset>
    <div>
     <p className="field-note">从上方看，把内轴轴承连线上靠近 Type-C 的一端对准图的上方。选择一侧，绕与所选连线垂直的另一根轴移动箭头。</p>
     <ol className="imu-mount-steps"><li>箭体竖直静止，采集参考。</li><li>{mountDirectionHint}，倾斜约 15° 后保持静止。可采集范围 8～30°。</li><li>回到竖直，保持静止后保存到当前 IMU。</li></ol>
     {hasUpright&&<p className="field-note">相对竖直倾角 <b>{format(mountAngle,1)}°</b></p>}
     {mountSaved&&tele?.control?.tilt_valid&&<p className="field-note">当前倾向 <b>X {format(tele.control.error_deg?.[1],1)}° · Y {format(tele.control.error_deg?.[0],1)}°</b></p>}
    </div>
   </div>
   <div className="real-actions">
    <button className="button secondary compact" disabled={!canMount||hasUpright} onClick={()=>invoke('','captureMount','upright')}>① 采集竖直</button>
    <button className="button secondary compact" disabled={!canMount||!hasUpright||hasLean} onClick={()=>invoke('','captureMount','leaned',mountDirection)}>② 采集倾斜</button>
    <button className="button primary compact" disabled={!canMount||!hasLean||mountAngle==null||mountAngle>3} onClick={()=>invoke('安装方向已由飞控确认保存。','captureMount','save')}>③ 回正并保存</button>
    {hasUpright&&<button className="button secondary compact" disabled={opBusy} onClick={()=>engine.current?.resetMount()}>重新采集</button>}
   </div>
   <p className="field-note" role="status">{mountHint}</p>
  </section>
  <section className="panel real-controls"><div className="real-heading"><div><h2>飞控操作</h2><p>设备的校准、模式切换与控制状态均以固件回传为准。</p></div><div className="real-actions"><button className="button secondary compact" disabled={!fresh||opBusy} onClick={()=>invoke('已切换到姿态模式。','setView','auto')}>姿态回中</button><button className="button secondary compact" disabled={!fresh||opBusy} onClick={()=>invoke('已切换到手动模式。','setView','manual')}>手动与行程</button></div></div>
   <div className="real-control-grid"><div><h3>姿态与 IMU</h3><p className="field-note">当前模式：{fresh?(attitude?'姿态回中':'手动与行程'):'等待设备'} · {mountSaved?'安装方向已按台架标定':`默认安装偏角 ${fresh?format(tele?.control?.mount_yaw_deg,1)+'°':'—'}`}</p>{fresh&&auto&&!armed&&!arming&&<p className="field-note" role="status">舵机正在根据实时姿态持续纠偏；电机尚未启动。</p>}
    <div className="real-actions"><button className="button secondary compact" disabled={!(fresh&&attitude&&!tele?.calibrating&&!outputActive&&!cg?.enabled&&!opBusy)} onClick={()=>invoke('正在校准，请保持竖直静止。','recalibrate')}>竖直静止校准</button><button className="button secondary compact" disabled={!(fresh&&tele?.control?.gravity_reference&&!tele?.calibrating&&!outputActive&&!cg?.enabled&&!opBusy)} onClick={()=>invoke('已恢复当前 IMU 的默认校准。','restoreCalibration')}>恢复默认校准</button><button className="button secondary compact" aria-expanded={showServoCalibration} aria-controls="axis-calibration" disabled={travelActive} onClick={()=>setShowServoCalibration(open=>!open)}>零位与行程标定</button><button className="button secondary compact" disabled={!(fresh&&!opBusy&&!outputActive&&!tele?.calibrating)} onClick={()=>invoke('已切换控制源。','switchImuSource',tele?.control?.source_id===1?2:1)}>切换至 IMU{tele?.control?.source_id===1?'2':'1'}</button></div>
    <p className="field-note">IMU1：{fresh?sensor(tele?.imus?.[0]):'等待数据'}<br/>加速度 / 角速度：{fresh&&tele?.imus?.[0]?.valid?vector(tele.imus[0].a):'—'} g · {fresh&&tele?.imus?.[0]?.valid?vector(tele.imus[0].g,1):'—'} °/s<br/>IMU2：{fresh?sensor(tele?.imu2):'等待数据'}<br/>加速度 / 角速度：{fresh&&tele?.imu2?.valid?vector(tele.imu2.a_g):'—'} g · {fresh&&tele?.imu2?.valid?vector(tele.imu2.g_dps,1):'—'} °/s</p>
    <div className="real-actions"><button className="button primary compact" disabled={!(ready&&attitude&&world&&tele?.control?.mount_ready&&tele?.control?.tilt_valid&&!auto&&(!armed&&!arming||tele?.control?.auto_motor_allowed)&&!opBusy)} onClick={()=>invoke('姿态回中已启动。','startAutomatic')}>{snap?.autoThrottleWanted?'开始回中 + 自动油门':torque?'仅舵机回中':'开始姿态回中'}</button><button className="button secondary compact" disabled={!(auto||snap?.autoFanWanted||snap?.autoThrottleWanted||armed||arming||snap?.operation==='auto'||snap?.operation==='auto-motor')} onClick={()=>invoke('自动控制已停止。','stopAutomatic')}>停止自动控制</button></div>
    {attitude&&torque&&<section>
     <h3>推力力矩稳姿</h3>
     <div className="real-input-grid">
      <label>目标 X 倾角 / °<input type="number" min="-20" max="20" step="0.1" value={targetX} onChange={e=>setTargetX(e.target.value)} disabled={opBusy}/></label>
      <label>目标 Y 倾角 / °<input type="number" min="-20" max="20" step="0.1" value={targetY} onChange={e=>setTargetY(e.target.value)} disabled={opBusy}/></label>
     </div>
     <p className="field-note">设备目标 {format(torque.target_deg?.[0],1)}° / {format(torque.target_deg?.[1],1)}° · 估计姿态 {format(torque.lean_deg?.[0],1)}° / {format(torque.lean_deg?.[1],1)}°</p>
     {torque.active&&<p className="field-note">请求力矩 {format(torque.requested_nm?.[0],3)} / {format(torque.requested_nm?.[1],3)} N·m · 目标油门 {format(torque.throttle_target_pct,1)}%{torque.limited?' · 输出受限':''}</p>}
     {torque.active&&torque.travel_limited&&<p className="field-note" role="status">目标力矩超出标定轴程，正在使用范围内的最接近解。</p>}
     {torque.azimuth_rewind&&<p className="field-note" role="status">方位正在回初始位，沿当前可用方向限幅纠偏。</p>}
     <form onSubmit={e=>{e.preventDefault();if(targetsValid&&torqueReady&&!cgMoving&&!outputActive&&!opBusy)
      invoke('推力力矩稳姿已启动。','startThrustTorque',{...Object.fromEntries(torqueFields.map(([key])=>[key,Number(torqueValues[key])])),feedforward:feedforward.map(Number)},Number(targetX),Number(targetY));}}>
      <details open={!torque.active&&!outputActive||undefined}><summary>推力标定与力矩 PID</summary>
       <p className="field-note">采用台架实测的推力、力臂和油门范围。P、I、D 的单位分别为 N·m/°、N·m/(°·s)、N·m·s/°。</p>
       <div className="real-input-grid">{torqueFields.map(([key,label,min,max])=><label key={key}>{label}<input required type="number" min={min} max={max} step="any" value={torqueValues[key]??''} disabled={outputActive||opBusy}
        onChange={e=>setTorqueValues(v=>({...v,[key]:e.target.value}))}/></label>)}</div>
       <details><summary>配平和重力补偿（可选）</summary><p className="field-note">矩阵行对应两路输出力矩，列对应 X/Y 机体倾角，单位 N·m/°。</p>
        <div className="real-input-grid">{feedforwardFields.map((label,i)=><label key={label}>{label}<input type="number" step="any" value={feedforward[i]}
         disabled={outputActive||opBusy} onChange={e=>setFeedforward(a=>a.map((value,j)=>i===j?e.target.value:value))}/></label>)}</div>
       </details>
      </details>
      <div className="real-actions">{torque.active?<button type="button" className="button primary compact" disabled={!targetsValid||opBusy}
       onClick={()=>invoke('目标倾角已由设备确认。','setAttitudeTarget',Number(targetX),Number(targetY))}>更新目标角</button>:
       <button type="submit" className="button primary compact" disabled={!torqueReady||cgMoving||outputActive||opBusy||!targetsValid}>启动推力稳姿</button>}
       <button type="button" className="button secondary compact" disabled={!(torque.prepared||snap?.operation==='torque')}
        onClick={()=>invoke('推力力矩稳姿已停止。','stopAutomatic')}>停止推力稳姿</button></div>
     </form>
    </section>}
    {fresh&&tele?.control?.gravity_reference&&<p className="field-note">重力参考 {tele.calibrating?'正在静止校准':tele.control.gravity_reference==='MANUAL'?'手动校准':'默认 9.8 m/s²'}{!tele.calibrating&&!world?' · 等待稳定采样':''}</p>}
   </div><div><h3>手动舵机</h3><div className="real-actions"><button className="button secondary compact" disabled={!(ready&&manual&&!servoOn&&!opBusy)} onClick={()=>invoke('舵机已启用。','enableServos')}>启用舵机</button><button className="button secondary compact" disabled={!(servoOn||snap?.operation==='servo')} onClick={()=>invoke('舵机已停止。','disableServos')}>停止舵机</button><button className="button secondary compact" disabled={!(manual&&ready&&servoOn)} onClick={()=>{setServoX(azimuthCenter);setServoY(0);invoke('已发送舵机回中指令。','centerServo');}}>机械回中</button></div>
    {servoCalibration&&<p className="field-note">中位输出刻度 {servoCalibration.azimuth_deg}° 方位 / {servoCalibration.elevation_deg}° 俯仰</p>}
    <label>方位 {servoX.toFixed(1)}°<input id="servo-x" type="range" min={azimuthLimits[0]} max={azimuthLimits[1]} step="0.2" value={servoX} disabled={!(manual&&ready&&servoOn&&!opBusy)} onInput={e=>{setServoX(e.currentTarget.valueAsNumber);engine.current?.queueServo();}} onPointerUp={()=>engine.current?.flushServo()}/></label>
    <label>俯仰 {servoY.toFixed(1)}°<input id="servo-y" type="range" min={Array.isArray(tele?.outputs?.elevation_limits_deg)?tele.outputs.elevation_limits_deg[0]-135:-20} max={Array.isArray(tele?.outputs?.elevation_limits_deg)?tele.outputs.elevation_limits_deg[1]-135:20} step="0.2" value={servoY} disabled={!(manual&&ready&&servoOn&&!opBusy)} onInput={e=>{setServoY(e.currentTarget.valueAsNumber);engine.current?.queueServo();}} onPointerUp={()=>engine.current?.flushServo()}/></label>
   </div><div className="cg-servo-panel"><h3>可动重心 · GPIO 7</h3>
    <p className="field-note">{!fresh?'连接飞控后可调节。':!cg?'需要支持重心舵机的固件。':cg.enabled?`位置 PWM ${cg.command_us} μs · ${cg.direction>0?'正向调节':cg.direction<0?'反向调节':'保持'}`:`启用后发送 ${cg.command_us} μs。`}</p>
    <div className="real-actions">
     <button className="button secondary compact" disabled={!canAdjustCg||cg.enabled} onClick={()=>invoke('重心舵机已启用。','enableCgServo')}>启用</button>
     {cgJogButton(1,'正转')}{cgJogButton(-1,'反转')}
     <button className="button secondary compact" disabled={!cgMoving&&!cgJogging} onClick={()=>invoke('已保持当前重心舵机指令。','stopCgServo')}>停止</button>
     <button className="button secondary compact" disabled={!cg?.enabled||opBusy||armed||arming} onClick={()=>invoke('重心舵机已停用。','stopCgServo',true)}>停用</button>
    </div>
    <p className="field-note">{cg&&cg.max_us!==2000?'更新固件后可使用 500～2000 μs 限位。':'范围 500～2000 μs。松开按键后保持当前指令。'}</p>
   </div><div><h3>手动电机</h3><label><input id="prop-confirm" type="checkbox" checked={propConfirmed} onChange={e=>setPropConfirmed(e.target.checked)}/> 已拆除桨叶，可进行电机测试</label><label>速度指令 {motor}%<input id="motor-speed" type="range" min="0" max="100" value={motor} disabled={!(manual&&ready&&armed)} onChange={e=>{setMotor(Number(e.target.value));engine.current?.queueMotor();}}/></label>
    <div className="real-actions"><button className="button primary compact" disabled={!(ready&&manual&&propConfirmed&&!armed&&!arming&&!opBusy&&!cgMoving)} onClick={()=>invoke('电机已按当前速度启动。','startMotor')}>启动电机</button><button className="button secondary compact" disabled={!(armed||arming||tele?.outputs?.esc_signal_enabled||snap?.operation==='motor')} onClick={()=>invoke('电机已停止。','stopMotor')}>停止电机</button></div>
   </div><div><h3>姿态模式风扇</h3><label><input id="auto-motor-enable" type="checkbox" checked={!!snap?.autoFanWanted} disabled={!!snap?.autoThrottleWanted||!!tele?.control?.tilt_throttle?.enabled||!(fresh&&attitude&&((ready&&world&&!opBusy)||snap?.autoFanWanted))} onChange={()=>invoke('风扇许可已更新。','toggleAutoMotor')}/> 允许独立手动调速</label>
    <label>速度指令 {autoMotor}%<input id="auto-motor-speed" type="range" min="0" max="100" value={autoMotor} disabled={!!tele?.control?.tilt_throttle?.enabled||!(ready&&attitude&&tele?.control?.auto_motor_allowed&&snap?.autoFanWanted&&armed&&!opBusy)} onChange={e=>{setAutoMotor(Number(e.target.value));engine.current?.queueAutoMotor();}}/></label>
    <div className="real-actions"><button className="button primary compact" disabled={!!snap?.autoThrottleWanted||!!tele?.control?.tilt_throttle?.enabled||!(ready&&attitude&&world&&tele?.control?.auto_motor_allowed&&snap?.autoFanWanted&&!armed&&!arming&&!opBusy)} onClick={()=>invoke('风扇已启用。','startAutoMotor')}>启动风扇</button><button className="button secondary compact" disabled={!(attitude&&(armed||arming||snap?.operation==='auto-motor'))} onClick={()=>invoke('风扇已停止。','stopMotor')}>停止风扇</button></div>
   </div></div>
   {(showServoCalibration||travelActive)&&<div className="servo-travel" id="axis-calibration" key={tele?.boot??'offline'}>
    <ServoTravelPanel travel={servoTravel} canStart={ready&&!opBusy&&!outputActive&&!cg?.enabled} canMove={ready} busy={commandBusy}
     reversed={[!!servoCalibration?.azimuth_reversed,!!servoCalibration?.elevation_reversed]}
     jog={async(axis,direction)=>{await engine.current!.jogServoTravel(axis,direction);}}
     release={releaseServoTravel}
     update={async(axis,action)=>{await engine.current!.updateServoTravel(axis,action);}}
     stop={async()=>{await engine.current!.stopAll();}}/>
    {!travelActive&&<details><summary>舵机方向和原始零位</summary>
     <ServoCalibrationPanel calibration={servoCalibration}
      canSave={!!fresh&&!opBusy&&!outputActive&&!cg?.enabled&&!tele?.calibrating}
      previewPwm={canCaptureServo?servoPwm:undefined}
      recordedZero={servoTravel?.saved_us.map(points=>!!points[1])}
      onSave={async settings=>{
       if(!engine.current)throw Error('控制代码尚未加载');
       await engine.current.saveServoCalibration(settings);
       setPidDirty(false);
      }}/>
    </details>}
   </div>}
   <details><summary>姿态回中联动自动油门</summary><label><input id="tilt-throttle-enable" type="checkbox" checked={!!snap?.autoThrottleWanted} disabled={!(fresh&&attitude&&(snap?.autoThrottleWanted||(ready&&world&&!opBusy&&!outputActive&&!snap?.autoFanWanted&&tele?.control?.tilt_throttle)))} onChange={()=>invoke('自动油门许可已更新。','toggleTiltThrottle')}/> 允许倾斜自动油门</label><div className="real-input-grid">{throttleFields.map(([id,label,value])=><label key={id}>{label}<input id={id} type="number" defaultValue={value} step="0.01" disabled={opBusy||outputActive}/></label>)}</div><p className="field-note">设备回传：倾斜 {fresh?format(tele?.control?.tilt_throttle?.tilt_deg,1):'—'}° · {tele?.control?.tilt_throttle?.rate_mode==='OUTWARD'?'外倾角速度':'横向角速度'} {fresh?format(tele?.control?.tilt_throttle?.rate_dps,1):'—'}°/s · 目标 {fresh?format(tele?.control?.tilt_throttle?.target_pct,1):'—'}% · 输出 {fresh?format(tele?.control?.tilt_throttle?.output_pct,1):'—'}%</p><button className="button secondary compact" disabled={!(armed||arming||snap?.autoThrottleWanted)} onClick={()=>invoke('电机已停止。','stopMotor')}>只停止电机</button><p className="field-note">勾选后点击「开始回中 + 自动油门」。静止且在倾斜死区内时只输出基础油门，基础油门为 0% 时电机保持停止。起转需要的最低油门由电调和电机决定。</p></details>
   <details><summary>PID 与云台映射</summary><p className="field-note">当前控制律：{!fresh?'等待遥测':tele?.control?.model_enabled===true?'模型换算（PID 项单位 °/s²）':tele?.control?.model_enabled===false?'角度 PID（PID 项单位 °）':'旧固件未报告'}。模型矩阵由固件配置，实物启用需要实测标定；模型模式的方向由矩阵决定。</p><table className="real-pid"><thead><tr><th>轴</th>{['P','I','D','输出'].map(label=><th key={label}>{label} / {fresh&&tele?.control?.model_enabled===true?'°/s²':'°'}</th>)}</tr></thead><tbody>{(['x','y'] as const).map(axis=><tr key={axis}><td>{axis.toUpperCase()}</td>{(['p_deg','i_deg','d_deg','output_deg'] as const).map(term=><td key={term}>{fresh?format(tele?.control?.pid?.[axis]?.[term]):'—'}</td>)}</tr>)}</tbody></table><form id="pid-form" onSubmit={e=>{e.preventDefault();void applyPid();}}><div className="real-input-grid">{pidFields.filter(([id])=>(id!=='azimuth-zero'||!servoCalibration)&&(!id.startsWith('trim-')||(tele?.control?.model_enabled===false&&Number.isFinite(tele.control?.pid?.trim_x_deg)))).map(([id,label])=><label key={id}>{label}<input id={id} type="number" step="0.01" value={pidValues[id]??0} onChange={e=>{setPidDirty(true);setPidValues(v=>({...v,[id]:Number(e.target.value)}));}}/></label>)}{(['x','y'] as const).map(axis=><label key={axis}>{axis.toUpperCase()} 修正方向<select id={'sign-'+axis} disabled={fresh&&tele?.control?.model_enabled===true} value={pidValues['sign-'+axis]??1} onChange={e=>{setPidDirty(true);setPidValues(v=>({...v,['sign-'+axis]:Number(e.target.value)}));}}><option value={1}>+1</option><option value={-1}>−1</option></select></label>)}</div><button className="button secondary compact" type="submit" disabled={!(fresh&&!outputActive&&!tele?.calibrating&&!opBusy&&tele?.control?.pid)}>应用 PID 与映射</button><p className="field-note">设备输出停止后应用。X/Y 修正方向用于机体纠偏，舵机转向在「方向标定」中设置。PID 参数在重启后恢复默认。</p></form></details>
  </section>
  <section className="panel real-records"><div className="real-heading"><div><h2>真实数据记录</h2><p>JSONL 保留完整原始遥测；CSV 导出常用字段。</p></div></div>{records.length?<div className="real-record-list">{records.map(r=><div key={r.id}><span><b>{r.id}</b><small>{r.transport.toUpperCase()} · {r.count} 帧 · {r.status==='recording'?'采集中':r.status==='stopped'?'已结束':r.status==='disconnected'?'连接中断':r.status}</small></span><div><a href={`/api/real/records/${encodeURIComponent(r.id)}/jsonl`}><Download size={13}/> JSONL</a><a href={`/api/real/records/${encodeURIComponent(r.id)}/csv`}><Download size={13}/> CSV</a></div></div>)}</div>:<p className="field-note">尚无真实采集记录。</p>}</section>
 </div>;
}
