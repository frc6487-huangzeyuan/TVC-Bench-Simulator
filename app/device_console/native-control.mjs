// TVC Studio 自有控制引擎；初始流程参考硬件工程 2.4.4，后续在 Studio 维护。
import {VERSION, encodeFrame, LineDecoder, isTelemetry, validateEscConfig, imu2Display, servoAngleToUs, servoUsToAngle, validatePidSettings, validateTiltThrottle, TRANSPORT_GRACE_MS} from './protocol.mjs';

const $=id=>document.getElementById(id);
function makeSession(){
  try{return Array.from(crypto.getRandomValues(new Uint8Array(10)),byte=>byte.toString(16).padStart(2,'0')).join('');}
  catch{return `${Date.now().toString(16)}${Math.random().toString(16).slice(2)}`.slice(0,20).padEnd(8,'0');}
}
const S={mode:null,port:null,reader:null,writer:null,ws:null,token:'',session:makeSession(),seq:0,boot:'',epoch:0,rev:0,tele:null,lastRx:0,pending:new Map(),generation:0,heartbeat:false,view:'auto',connecting:false,writeChain:Promise.resolve(),faultShown:'',deviceFirmware:'',action:0,operation:null,stopping:0,controlChain:Promise.resolve(),healthPending:false,lastKeepalive:0,autoFanWanted:false};
const ESC={stop_us:1500,min_us:1300,max_us:1700,hold_ms:3000,hz:50,confirmed:true};
// 预览模块只有 DOM 与数学接口，没有 command()/串口/WebSocket 的访问权限。
const attitudePreview={render(){}};
Object.assign(S,{trend:[],trendBoot:'',trendReference:0,pidDirty:false,autoThrottleWanted:false,faultSnapshot:null,warningShown:''});
const angleToUs=servoAngleToUs;
const usToAngle=(us,channel)=>servoUsToAngle(us,channel).toFixed(1);
const fresh=()=>!!S.tele&&performance.now()-S.lastRx<TRANSPORT_GRACE_MS;
const ready=()=>fresh()&&!S.tele.fault&&!S.tele.calibrating&&S.tele.control_imu?.valid;
const outputsActive=()=>!!(S.tele&&(S.tele.outputs.servo_enabled||S.tele.outputs.cg_servo?.enabled||S.tele.outputs.esc_armed||S.tele.outputs.esc_arming));

export const responseNoticeType=error=>error?.code==='RESPONSE_TIMEOUT'?'warning':'error';
function notice(message,type=''){window.dispatchEvent(new CustomEvent('tvc-native-notice',{detail:{message,type}}));}
const mountErrors={
  IMU_NOT_READY:'当前 IMU 没有新鲜采样，请检查连接与采样状态',
  MOUNT_POSE_RANGE:'设备拒绝标定数据，请重新采集竖直与倾斜姿态',
  MOUNT_RETURN_UPRIGHT:'请让箭体回到竖直并保持静止，再保存',
};
function describeReason(reason){if(reason==='FAULT_REQUIRES_RESTART')return `重新校准被当前故障拒绝；原始 fault：${S.tele?.fault||'未知，请查看原始故障码区域'}`;return ({ELEVATION_CENTER_AT_LIMIT:'俯仰中位必须留出双向行程，请远离端点或重新记录三点',ELEVATION_LIMIT:'俯仰命令超出设备当前限位，请检查固件版本',THROTTLE_NOT_READY:'自动油门尚未就绪',AUTO_THROTTLE_ACTIVE:'自动油门正在接管电机',THROTTLE_CONFIG:'自动油门参数无效',IMU_NOT_FOUND:'目标 IMU 未检测到，已保留原数据源',INVALID_IMU_SOURCE:'IMU 数据源编号无效',CONTROLLER_BUSY:'已有另一个控制页面连接，请先断开它',AUTH_REQUIRED:'无线认证失败',SESSION_MISMATCH:'连接会话已失效',STALE_COMMAND:'设备数据过期，请重新连接',MODE_LOCKED:'当前不在对应控制模式',NOT_READY:'设备尚未准备好',USE_SERVO_CALIBRATION:'方位零位请在方向标定中调整',SERVO_CALIBRATION_RANGE:'舵机标定参数超出范围',SERVO_CALIBRATION_SAVE:'设备保存舵机标定失败',SERVO_TRAVEL_MOVING:'请等待当前步进完成',SERVO_TRAVEL_ENDPOINTS:'请从零位分别移动到两个端点',SERVO_TRAVEL_ACTIVE:'请先结束行程标定',SERVO_TRAVEL_ZERO_LOCKED:'请用快速标定重新记录该轴零位',CALIBRATION_STORAGE:'设备无法保存或恢复 IMU 校准',STOP_OUTPUTS_FIRST:'请先停止全部输出',WORLD_NOT_READY:'世界坐标系尚未建立',CONFIG_MISMATCH:'电调配置已变化，请重试',OLD_EPOCH:'控制状态已变化，请重试'})[reason]||reason;}

function acceptPacket(p){
  if(!p||p.v!==VERSION||p.session!==S.session)return;
  if(p.type==='ack'){
    if(S.boot&&p.boot!==S.boot)return;
    if(typeof p.boot==='string')S.boot=p.boot;
    if(Number.isInteger(p.epoch))S.epoch=Math.max(S.epoch,p.epoch);
    if(Number.isInteger(p.config_rev))S.rev=Math.max(S.rev,p.config_rev);
    const waiter=S.pending.get(p.request_seq);
    if(waiter){clearTimeout(waiter.timer);S.pending.delete(p.request_seq);p.ok?waiter.resolve(p):waiter.reject(new Error(mountErrors[p.reason]||describeReason(p.reason)));}
  }else if(isTelemetry(p)){
    if(S.boot&&p.boot!==S.boot)return;
    S.boot=p.boot;S.epoch=Math.max(S.epoch,p.epoch);S.rev=Math.max(S.rev,p.config_rev);S.tele=p;S.lastRx=performance.now();recordTrend(p);if(S.mode==='serial'||S.mode==='wifi')window.dispatchEvent(new CustomEvent('tvc-native-telemetry',{detail:{packet:p,transport:S.mode==='serial'?'usb':'wifi'}}));
    if((p.fault||p.warning)&&(!S.faultSnapshot||S.faultSnapshot.packet.boot!==p.boot||S.faultSnapshot.packet.fault!==p.fault||S.faultSnapshot.packet.warning!==p.warning||S.faultShown!==p.fault||S.warningShown!==p.warning))S.faultSnapshot={packet:JSON.parse(JSON.stringify(p)),received:new Date().toISOString()};
    S.warningShown=p.warning;
    if(p.calibrating||p.operating_mode!=='ATTITUDE')resetAutoMotor();
    if(p.fault&&p.fault!==S.faultShown){
      S.faultShown=p.fault;resetAutoMotor();cancelActions();S.heartbeat=false;
      notice(faultDescription(p),'error');
    }
    if(!p.fault)S.faultShown='';
  }
  render();
}

function feedText(text){new LineDecoder().push(text,acceptPacket,()=>notice('收到损坏的数据帧','error'));}
const abort=()=>Object.assign(new Error('操作已取消'),{name:'AbortError'});
function ticket(){return {generation:S.generation,action:S.action,session:S.session};}
function check(t){if(t&&(t.generation!==S.generation||t.action!==S.action||t.session!==S.session||!S.mode||document.hidden))throw abort();}
function cancelActions(){
  mountError='';
  mountPose=null;
  ++S.action;S.operation=null;clearQueuedServo();clearTimeout(motorTimer);clearTimeout(autoMotorTimer);
  for(const [seq,p] of S.pending)if(p.guard){clearTimeout(p.timer);S.pending.delete(seq);p.reject(abort());}
}
async function operation(kind,fn){
  if(S.operation||S.stopping)throw abort();
  const t=ticket();S.operation={kind,t};render();
  try{check(t);return await fn(t);}catch(error){if(t.generation===S.generation&&t.action===S.action)S.heartbeat=outputsActive();throw error;}finally{if(S.operation?.t===t)S.operation=null;render();}
}
function rawWrite(frame,guard){
  const generation=S.generation,writer=S.writer,ws=S.ws,mode=S.mode;
  const send=()=>{if(generation!==S.generation||mode!==S.mode)throw abort();check(guard);return typeof frame==='function'?frame():frame;};
  if(mode==='wifi'&&ws?.readyState===WebSocket.OPEN){try{ws.send(send());return Promise.resolve();}catch(e){return Promise.reject(e);}}
  if(mode==='serial'&&writer){S.writeChain=S.writeChain.catch(()=>{}).then(()=>writer.write(new TextEncoder().encode(send())));return S.writeChain;}
  return Promise.reject(new Error('设备未连接'));
}
function sendCommand(cmd,data={},timeout=1800,guard){
  const seq=++S.seq,session=S.session,boot=S.boot;
  return new Promise((resolve,reject)=>{
    const timer=setTimeout(()=>{S.pending.delete(seq);reject(Object.assign(new Error('设备响应超时'),{code:'RESPONSE_TIMEOUT'}));},timeout);
    S.pending.set(seq,{resolve,reject,timer,guard});
    rawWrite(()=>{
      if(!S.pending.has(seq)||S.session!==session)throw abort();
      const packet={v:VERSION,type:'command',seq,session,cmd,...data};
      if(cmd==='hello'){if(S.token)packet.token=S.token;}else{
        packet.boot=boot;
        if(S.tele)packet.device_t_us=Math.round(S.tele.t_us+(performance.now()-S.lastRx)*1000);
      }
      return encodeFrame(packet);
    },guard).catch(error=>{clearTimeout(timer);S.pending.delete(seq);reject(error);});
  });
}
const immediate=new Set(['hello','heartbeat','keepalive','stop','servo_disable','esc_stop','auto_disable','auto_motor_disable','tilt_throttle_disable','cg_servo_hold','cg_servo_disable','servo_travel_hold']);
function command(cmd,data={},timeout=1800,guard){
  if(immediate.has(cmd))return sendCommand(cmd,data,timeout,guard);
  const t=guard||ticket();
  const work=S.controlChain.catch(()=>{}).then(async()=>{
    check(t);const result=await sendCommand(cmd,{...data,epoch:S.epoch},timeout,t);check(t);return result;
  });
  S.controlChain=work.catch(()=>{});return work;
}
async function run(label,fn){try{await fn();notice(label,'success');}catch(error){if(error.name!=='AbortError')notice(error.message||String(error),responseNoticeType(error));}finally{render();}}
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function waitTelemetry(test=()=>true,timeout=5000,guard){
  const generation=S.generation,end=performance.now()+timeout;
  while(performance.now()<end){if(generation!==S.generation)throw abort();check(guard);if(fresh()&&test(S.tele))return S.tele;await delay(50);}
  throw new Error('等待设备状态超时');
}
function resetSession(){
  resetAutoMotor();cancelActions();S.session=makeSession();S.seq=0;S.boot='';S.epoch=S.rev=0;S.tele=null;S.lastRx=0;S.faultShown='';S.warningShown='';
  S.writeChain=Promise.resolve();S.controlChain=Promise.resolve();S.healthPending=false;S.lastKeepalive=0;
}
function faultDescription(p){
  if(p.fault==='MOUNT_REFERENCE_REQUIRED')return '尚未保存当前 IMU 的安装方向。请完成两步安装方向标定，然后回到竖直做静止校准。';
  if(p.fault==='MOUNT_REZERO_REQUIRED')return '安装方向已保存。请把箭体恢复竖直静止，再点击“竖直静止校准”。';
  const m=p.control_imu||{};
  const link=`IMU${p.control.source_id} 本地 I2C，读取错误 ${m.read_errors??0}，丢帧 ${m.missed??0}`;
  const detail=`${link}；有效样本 ${m.calibration_samples??'—'}/300，剔除 ${m.calibration_rejected??'—'}，读取错误累计 ${m.read_errors??'—'}`;
  if(!p.fault&&p.warning==='HEARTBEAT_TIMEOUT')return `心跳超时告警：本项不停止输出，也不取消控制操作；心跳恢复后自动解除。最近心跳年龄 ${p.heartbeat_age_ms??'—'}ms。`;
  if(p.fault==='HEARTBEAT_TIMEOUT')return '当前主控将心跳超时作为停机故障处理。需要上传2.4.2主控，才能改为仅告警。';
  if(p.fault==='PID_INVALID')return 'PID 输入、参数或控制周期检查未通过，全部输出已停止。可能涉及控制间隔超过50ms、姿态输入非有限或指令范围异常；此总故障码还不能区分具体触发项。';
  if(p.fault==='CALIBRATION_STORAGE')return '设备无法写入 IMU 校准。';
  if(p.fault==='CALIBRATION_DATA')return `校准未通过（CALIBRATION_DATA）：${detail}。请保持静止，检查主IMU接线后重新建立坐标系；样本足够时还需检查数据波动。`;
  if(p.fault==='IMU_TIMEOUT')return `IMU${p.control.source_id} 无有效新样本，输出已停止。检查 SDA9、SCL10、供电及 ${p.control.source_id===2?'INT GPIO2、AD0 高电平':'INT GPIO8、AD0 接地'}。${detail}`;
  if(p.fault==='IMU_RESTART')return '传感器采集通道重启（IMU_RESTART），输出已停止。请保持竖直静止并重新校准。';
  if(p.fault==='MOUNT_CALIBRATION')return '安装校准未通过。请保持箭体竖直并检查传感器数据；水平安装后的轴向与安装偏角仍需确认。';
  if(p.fault==='ATTITUDE_INVALID')return '姿态数据无效，输出已停止。请检查当前 IMU 的数据，恢复后重新校准。';
  if(p.fault==='THROTTLE_INVALID')return '自动油门计算或控制周期异常，全部输出已停止。';
  if(p.fault==='PREPOSITION_MOVED')return '启动准备期间姿态变化过大或姿态无效，输出已停止；请固定台架后重新准备。';
  if(p.fault==='PREPOSITION_TIMEOUT')return '舵机预定位超时，输出已停止；请检查姿态、舵机及限速。';
  if(p.fault==='TORQUE_HEARTBEAT')return '推力稳姿接收不到 Studio 心跳，输出已停止。';
  if(p.fault==='TORQUE_IMU_STALE')return '推力稳姿超过 50 ms 未收到新姿态样本，输出已停止。';
  if(p.fault==='TORQUE_CONTROL_INVALID')return '推力稳姿计算失败，输出已停止。请核对 IMU 和控制周期。';
  if(p.fault==='PWM_INIT')return '舵机 PWM 输出失败，已停止控制。请检查开发板配置与引脚占用。';
  return `设备故障：${p.fault}`;
}
function displayedFault(){return S.tele&&(S.tele.fault||S.tele.warning)?S.tele:S.tele?null:S.faultSnapshot?.packet??null;}
function renderFault(){
  const p=displayedFault(),current=!!S.tele;
  $('device-fault-code').textContent=p?.fault||p?.warning||(current?'无故障':'等待连接');
  $('device-fault-code').className=p?(p.fault?'fault-active':'fault-warning'):'';
  $('device-fault-description').textContent=p?faultDescription(p):current?'设备当前 fault 为空。':'连接后持续显示设备 fault，不受操作提示覆盖。';
  $('device-fault-age').textContent=p?`${current?(fresh()?'当前设备回传':'连接数据已过期'):'保留的历史故障快照'} · 固件 ${p.firmware} · IMU${p.control.source_id} · ${p.state}`:current?'当前设备回传正常':'尚未收到设备遥测';
  $('copy-fault').disabled=!p;
  if(!p){$('fault-copy-text').hidden=true;$('fault-copy-text').value='';$('fault-copy-status').textContent='复制后可直接粘贴到聊天中。';}
}
async function copyFault(){
  const p=displayedFault();if(!p?.fault&&!p?.warning)return;
  const data={fault:p.fault,warning:p.warning||'',heartbeat_age_ms:p.heartbeat_age_ms,firmware:p.firmware,boot:p.boot,state:p.state,operating_mode:p.operating_mode,control_source:p.control.source_id,t_us:p.t_us,
    snapshot:!S.tele,telemetry_fresh:!!S.tele&&fresh(),first_received:S.faultSnapshot?.received,
    imu:p.control_imu,mount:{ready:p.control.mount_ready,yaw_deg:p.control.mount_yaw_deg,sensor_to_body:p.control.mount_matrix},world_valid:p.world.valid,pid:p.control.pid,outputs:p.outputs,
    note:p.fault?'这是故障后的遥测；PID复位后的dt可能为0，不能据此认定原始触发原因。':'当前为告警；控制不会因这项告警中断。'};
  const text=JSON.stringify(data,null,2),field=$('fault-copy-text');field.value=text;
  try{if(!navigator.clipboard?.writeText)throw Error('CLIPBOARD_UNAVAILABLE');await navigator.clipboard.writeText(text);field.hidden=true;$('fault-copy-status').textContent='故障信息已复制，可直接粘贴到聊天中。';}
  catch{field.hidden=false;field.focus?.();field.select?.();$('fault-copy-status').textContent='浏览器未允许直接复制，请选中文本框内容，按 Ctrl+C 复制。';}
}

async function beginSession(){
  await command('hello',{},2500);
  await waitTelemetry(()=>true,2500);
  notice('连接成功，设备输出保持停止。','success');
  await switchDeviceMode(S.view);
}
async function connectUsb(){
  if(S.connecting||S.mode)return;
  if(!('serial'in navigator)){notice('当前浏览器不支持USB串口，请使用Chrome或Edge。','error');return;}
  S.connecting=true;render();
  try{
    const port=await navigator.serial.requestPort();await port.open({baudRate:460800});
    resetSession();S.mode='serial';S.port=port;S.writer=port.writable.getWriter();S.generation++;
    const decoder=new LineDecoder();
    S.reader=port.readable.getReader();const reader=S.reader,generation=S.generation;
    (async()=>{try{while(S.mode==='serial'&&generation===S.generation){const {value,done}=await reader.read();if(done||generation!==S.generation)break;decoder.push(value,acceptPacket,()=>notice('USB数据帧校验失败','error'));}}catch(error){if(generation===S.generation)notice(`USB连接中断：${error.message}`,'error');}finally{if(generation===S.generation)void disconnect(false);}})();
    await beginSession();
  }catch(error){await closeTransport();notice(`USB连接失败：${error.message}`,responseNoticeType(error));}
  finally{S.connecting=false;render();}
}
async function connectWifi(){
  if(S.connecting||S.mode)return;S.connecting=true;render();
  try{
    const response=await fetch('/api/real/device',{cache:'no-store'});if(!response.ok)throw new Error(`设备信息 HTTP ${response.status}`);
    const info=await response.json();S.deviceFirmware=typeof info.firmware==='string'?info.firmware:'版本未知';render();if(info.protocol!==VERSION)throw new Error(`网页协议 ${VERSION} 与主控协议 ${info.protocol} 不一致；当前固件 ${S.deviceFirmware}，请上传本目录的主控工程`);
    resetSession();S.token=info.token;const ws=new WebSocket(info.ws_url);S.ws=ws;
    await new Promise((resolve,reject)=>{const timer=setTimeout(()=>reject(new Error('无线连接超时')),3000);ws.onopen=()=>{clearTimeout(timer);resolve();};ws.onerror=()=>{clearTimeout(timer);reject(new Error('WebSocket连接失败'));};});
    S.mode='wifi';S.generation++;
    const generation=S.generation;ws.onmessage=event=>{if(generation===S.generation)feedText(String(event.data));};
    ws.onclose=()=>{if(generation===S.generation&&S.mode==='wifi'){notice('Wi-Fi控制连接已断开','error');void disconnect(false);}};
    await beginSession();
  }catch(error){await closeTransport();notice(`Wi-Fi连接失败：${error.message}`,responseNoticeType(error));}
  finally{S.connecting=false;render();}
}
async function closeTransport(){
  const {mode,reader,writer,port,ws}=S;resetAutoMotor();
  cancelActions();S.mode=null;S.heartbeat=false;S.tele=null;S.lastRx=0;S.boot='';S.generation++;
  S.reader=S.writer=S.port=S.ws=null;S.writeChain=Promise.resolve();S.controlChain=Promise.resolve();window.dispatchEvent(new CustomEvent('tvc-native-disconnected'));
  for(const pending of S.pending.values()){clearTimeout(pending.timer);pending.reject(abort());}S.pending.clear();
  try{if(mode==='serial'&&reader){await reader.cancel();reader.releaseLock();}}catch{}
  try{if(mode==='serial'&&writer)writer.releaseLock();}catch{}
  try{if(mode==='serial'&&port)await port.close();}catch{}
  try{if(ws&&ws.readyState<2)ws.close();}catch{}render();
}
async function disconnect(sendStop=true){
  resetAutoMotor();cancelActions();S.heartbeat=false;++S.stopping;render();
  try{if(sendStop&&S.mode&&S.boot)try{await command('stop',{},600);}catch{}await closeTransport();notice('已断开，已请求设备停止输出。');}
  finally{--S.stopping;render();}
}
async function stopPart(cmd){
  cancelActions();if(cmd==='stop')S.heartbeat=false;
  const generation=S.generation,action=S.action;++S.stopping;render();
  try{
    if(S.mode&&S.boot){const ack=await command(cmd);await waitTelemetry(p=>p.epoch>=ack.epoch&&
      ((cmd==='stop'||cmd==='auto_disable')?!p.outputs.servo_enabled&&!p.outputs.cg_servo?.enabled&&!p.outputs.esc_armed&&!p.outputs.esc_arming:
       (cmd==='esc_stop'||cmd==='auto_motor_disable'||cmd==='tilt_throttle_disable')?!p.outputs.esc_armed&&!p.outputs.esc_arming:!p.outputs.servo_enabled),1500);}
  }finally{--S.stopping;if(generation===S.generation&&action===S.action)S.heartbeat=cmd!=='stop'&&outputsActive();render();}
}
async function stopAll(){resetAutoMotor();await stopPart('stop');}
async function stopAutomatic(){resetAutoMotor();await stopPart('auto_disable');}

async function switchDeviceMode(view){
  if(!S.mode||!S.tele)return;
  const target=view==='auto'?'ATTITUDE':'MANUAL';if(S.tele.operating_mode===target)return;
  resetAutoMotor();cancelActions();S.heartbeat=false;await operation('mode',async t=>{
    const ack=await command('set_mode',{mode:target},3500,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.operating_mode===target,2000,t);
  });
  if(target==='ATTITUDE'&&S.tele?.calibrating)notice('正在静止校准，约需3秒。','success');
  else notice(target==='ATTITUDE'?'已进入姿态模式。':'已进入手动控制。','success');
}
async function showPage(view){
  S.view=view;$('page-auto').hidden=view!=='auto';$('page-manual').hidden=view!=='manual';$('tab-auto').classList.toggle('active',view==='auto');$('tab-manual').classList.toggle('active',view==='manual');render();
  if(S.mode)await run(view==='auto'?'已切换到自动回正。':'已切换到手动控制。',()=>switchDeviceMode(view));
}

function servoValues(){return {x:angleToUs($('servo-x').value,0),y:angleToUs(Number($('servo-y').value)+135,1)};}
const SERVO_SEND_INTERVAL_MS=40;
let servoTimer=0,servoInFlight=false,lastServoSend=-Infinity;
let pendingServo=null;
const canSendServo=()=>S.mode&&!S.stopping&&!document.hidden&&ready()&&
  S.tele.operating_mode==='MANUAL'&&S.tele.outputs.servo_enabled;

function clearQueuedServo(){
  clearTimeout(servoTimer);
  servoTimer=0;
  pendingServo=null;
}

function sendQueuedServo(final=false){
  if(servoInFlight||!pendingServo)return;
  if(!canSendServo()){clearQueuedServo();return;}
  const wait=lastServoSend+SERVO_SEND_INTERVAL_MS-performance.now();
  if(!final&&wait>0){
    if(!servoTimer)servoTimer=setTimeout(()=>{
      servoTimer=0;
      sendQueuedServo();
    },wait);
    return;
  }
  const target=pendingServo;
  clearQueuedServo();
  servoInFlight=true;
  lastServoSend=performance.now();
  void command('servo',target,1800,ticket()).catch(error=>{
    if(error.name!=='AbortError')notice(error.message||String(error),responseNoticeType(error));
  }).finally(()=>{
    servoInFlight=false;
    sendQueuedServo();
  });
}

function queueServo(){
  if(!canSendServo())return;
  pendingServo=servoValues();
  sendQueuedServo();
}

export function flushServo(){sendQueuedServo(true);}
async function enableServos(){return operation('servo',async t=>{S.heartbeat=true;await command('heartbeat',{},1800,t);check(t);const ack=await command('servo_enable',{},1800,t);await waitTelemetry(p=>p.epoch>=ack.epoch&&p.outputs.servo_enabled,2000,t);});}
async function disableServos(){await stopPart('servo_disable');}

const motorUs=()=>Math.round(1500+Number($('motor-speed').value)*2);
async function startMotor(){return operation('motor',async t=>{
  if(!$('prop-confirm').checked)throw new Error('请先确认桨叶已经拆除');
  validateEscConfig(ESC);S.heartbeat=true;await command('heartbeat',{},1800,t);check(t);
  if(!S.tele?.outputs.esc_armed){
    await command('esc_config',ESC,1800,t);check(t);
    const ack=await command('esc_arm',{config_rev:S.rev},1800,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.outputs.esc_armed,5200,t);
  }
  await command('esc',{us:motorUs()},1800,t);
});}
async function stopMotor(){S.autoThrottleWanted=false;$('tilt-throttle-enable').checked=false;await stopPart('esc_stop');}
const throttleInputs={'throttle-base':'base_pct','throttle-gain':'gain_pct_deg','throttle-rate-gain':'rate_gain_pct_dps','throttle-max':'max_pct','throttle-deadband':'deadband_deg','throttle-ramp':'ramp_pct_s'};
function throttleSettings(){return validateTiltThrottle(Object.fromEntries(Object.entries(throttleInputs).map(([id,key])=>[key,Number($(id).value)])));}
async function toggleTiltThrottle(){
  const wanted=$('tilt-throttle-enable').checked;
  if(wanted&&S.autoFanWanted)throw Error('请先关闭独立手动调速');
  S.autoThrottleWanted=wanted;
  if(!wanted)await stopPart('tilt_throttle_disable');
}
async function startAutomatic(){return operation('auto',async t=>{
  const coupled=S.autoThrottleWanted,config=coupled?throttleSettings():null;
  S.heartbeat=true;await command('heartbeat',{},1800,t);check(t);
  try{
    if(coupled){
      if(!S.tele?.control?.tilt_throttle)throw Error('当前固件不支持倾斜自动油门，请更新主控');
      if(config.rate_gain_pct_dps>0&&typeof S.tele?.control?.tilt_throttle?.rate_gain_pct_dps!=='number')throw Error('主控固件不支持角速度油门，请先更新固件');
      await command('tilt_throttle_config',config,1800,t);check(t);
      await command('auto_motor_enable',{},1800,t);check(t);
      validateEscConfig(ESC);await command('esc_config',ESC,1800,t);check(t);
      const arm=await command('esc_arm',{config_rev:S.rev},1800,t);
      await waitTelemetry(p=>{if(p.fault)throw Error(faultDescription(p));return p.epoch>=arm.epoch&&p.outputs.esc_armed;},23000,t);check(t);
    }
    const ack=await command('auto_enable',coupled?{tilt_throttle:true}:{},1800,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.control?.auto_enabled&&(!coupled||p.control?.tilt_throttle?.enabled),2000,t);
  }catch(error){
    if(coupled&&t.generation===S.generation&&t.action===S.action)await command('tilt_throttle_disable').catch(()=>{});
    throw error;
  }
});}
async function sendAttitudeTarget(x,y,t){
  if(![x,y].every(n=>Number.isFinite(n)&&Math.abs(n)<=20))throw Error('目标倾角须在 −20°～+20° 内');
  const ack=await command('attitude_target',{x_deg:x,y_deg:y},1800,t);
  await waitTelemetry(p=>{
    const target=p.control?.thrust_torque?.target_deg;
    return p.epoch>=ack.epoch&&target?.length===2&&Math.abs(target[0]-x)<0.001&&Math.abs(target[1]-y)<0.001;
  },2000,t);
}
export async function setAttitudeTarget(x,y){
  return operation('torque-target',t=>sendAttitudeTarget(x,y,t));
}
export async function startThrustTorque(config,x,y){
  return operation('torque',async t=>{
    if(!fresh()||!S.tele?.control?.thrust_torque)throw Error('当前设备未回传推力力矩控制能力');
    if(outputsActive())throw Error('请先停止舵机和电机');
    const {feedforward,...tuning}=config;
    if(!Object.values(tuning).every(Number.isFinite)||!Array.isArray(feedforward)||feedforward.length!==6||
      !feedforward.every(n=>Number.isFinite(n)&&Math.abs(n)<=config.full_torque_nm)||!(config.full_torque_nm>0&&config.full_torque_nm<=1000&&
      config.base_pct>0&&config.base_pct<=config.max_pct&&config.max_pct<=100&&config.kp_x>0&&config.kp_y>0))
      throw Error('请填写有效的力矩标定、PID 增益与油门范围');
    S.heartbeat=true;
    try{
      await command('heartbeat',{},1800,t);check(t);
      await command('torque_config',config,1800,t);check(t);
      await sendAttitudeTarget(x,y,t);check(t);
      await command('auto_motor_enable',{},1800,t);check(t);
      validateEscConfig(ESC);
      await command('esc_config',ESC,1800,t);check(t);
      const arm=await command('esc_arm',{config_rev:S.rev,thrust_torque:true},1800,t);
      await waitTelemetry(p=>{if(p.fault)throw Error(faultDescription(p));return p.epoch>=arm.epoch&&p.outputs.esc_armed&&
        p.control?.thrust_torque?.prepared;},23000,t);check(t);
      const enable=await command('auto_enable',{thrust_torque:true},1800,t);
      await waitTelemetry(p=>p.epoch>=enable.epoch&&p.control?.thrust_torque?.active,2000,t);
    }catch(error){
      if(t.generation===S.generation&&t.action===S.action)await command('auto_disable').catch(()=>{});
      S.heartbeat=outputsActive();
      throw error;
    }
  });
}
async function recalibrate(){resetAutoMotor();cancelActions();S.heartbeat=false;return operation('calibrate',t=>command('calibrate',{},3500,t));}
let mountPose=null;
let mountError='';
const mountAvailable=()=>fresh()&&S.tele.control?.mount_method==='GUIDED_TILT'&&
  S.tele.operating_mode==='ATTITUDE'&&!S.tele.calibrating&&!outputsActive();
function activeMountPose(){
  return mountPose&&mountAvailable()&&mountPose.source===S.tele.control.source_id&&mountPose.epoch===S.epoch?mountPose:null;
}
const mountVector=value=>Array.isArray(value)&&value.length===3&&value.every(Number.isFinite);
function mountSample(p){
  const s=p.control_imu,age=p.t_us-s?.sample_t_us;
  return s?.valid&&s.source_id===p.control.source_id&&age>=0&&age<50000&&mountVector(s.a)?s:null;
}
function mountAngle(a,b){
  const norms=Math.hypot(...a)*Math.hypot(...b);
  if(norms<.5)return null;
  const cosine=a.reduce((sum,v,i)=>sum+v*b[i],0)/norms;
  return Math.acos(Math.max(-1,Math.min(1,cosine)))*180/Math.PI;
}
async function sampleMountPose(t,source,epoch){
  const rows=[],end=performance.now()+2200;
  let previous=-1;
  while(performance.now()<end){
    check(t);
    if(!mountAvailable()||S.tele.control.source_id!==source||S.epoch!==epoch)
      throw Error('连接、IMU 来源或输出状态发生变化，请重新采集竖直姿态');
    const s=mountSample(S.tele);
    if(s&&s.sample_t_us>previous&&mountVector(s.g)&&mountVector(s.bias)){
      previous=s.sample_t_us;
      rows.push([...s.a,...s.g.map((v,i)=>v+s.bias[i])]);
    }
    await delay(50);
  }
  check(t);
  if(!mountAvailable()||S.tele.control.source_id!==source||S.epoch!==epoch)
    throw Error('连接、IMU 来源或输出状态发生变化，请重新采集竖直姿态');
  if(rows.length<15)throw Error('有效采样不足，请检查 IMU 数据');
  const mean=Array.from({length:6},(_,k)=>rows.reduce((sum,r)=>sum+r[k],0)/rows.length);
  const variance=mean.map((m,k)=>rows.reduce((sum,r)=>sum+(r[k]-m)**2,0)/rows.length);
  const a=mean.slice(0,3),bias=mean.slice(3),norm=Math.hypot(...a);
  if(norm<.85||norm>1.15||variance.some((v,k)=>v>(k<3?.0025:4))||bias.some(v=>Math.abs(v)>=15))
    throw Error('采样期间不够静止，请保持姿态后重试');
  return {a,bias};
}
export function resetMount(){
  mountError='';
  mountPose=null;
  render();
}
async function captureMount(step,direction){
  if(!mountAvailable())throw Error('需连接支持安装方向标定的主控，切换到姿态模式并停止全部输出');
  mountError='';
  await operation('mount-'+step,async t=>{
    const {source_id:source}=S.tele.control,epoch=S.epoch;
    if(step==='upright'){
      mountPose=null;
      mountPose={source,epoch,...await sampleMountPose(t,source,epoch)};
      return;
    }
    const pose=activeMountPose();
    if(!pose)throw Error('请先采集当前 IMU 的竖直姿态');
    if(step==='leaned'){
      if(!Number.isInteger(direction)||direction<0||direction>3)throw Error('请选择倾斜方向');
      const {a}=await sampleMountPose(t,source,epoch);
      const angle=mountAngle(pose.a,a);
      if(angle===null||angle<8||angle>30)throw Error('倾斜角需为 8～30°，请调整后重试');
      pose.leaned=a;
      pose.direction=direction;
      return;
    }
    if(step!=='save'||!pose.leaned)throw Error('请先采集倾斜姿态');
    const s=mountSample(S.tele),angle=s&&mountAngle(pose.a,s.a);
    if(angle==null||angle>3)throw Error('请先让箭体回到竖直并保持静止');
    const ack=await command('mount_config',{source_id:source,direction:pose.direction,upright:pose.a,leaned:pose.leaned,bias:pose.bias},3500,t);
    mountPose=null;
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.config_rev>=ack.config_rev&&p.control.source_id===source&&
      p.control.mount_reference_ready===true,2500,t);
  }).catch(error=>{
    if(error.name!=='AbortError')mountError=error.message;
    throw error;
  });
}
async function switchImuSource(source){
  if(!S.mode||!fresh())throw new Error('请先连接主控');
  await stopAll();
  await operation('source',async t=>{
    const ack=await command('select_imu',{source},2500,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.control.source_id===source,2500,t);
  });
}
let motorTimer=0;
function queueMotor(){clearTimeout(motorTimer);if(S.tele?.outputs.esc_armed)motorTimer=setTimeout(()=>run('电机速度已更新。',()=>command('esc',{epoch:S.epoch,us:motorUs()})),100);}

let autoMotorTimer=0;
const autoMotorUs=()=>Math.round(1500+Number($('auto-motor-speed').value)*2);
function resetAutoMotor(){S.autoFanWanted=false;S.autoThrottleWanted=false;clearTimeout(autoMotorTimer);for(const id of ['tilt-throttle-enable','auto-motor-enable']){const e=$(id);if(e)e.checked=false;}const speed=$('auto-motor-speed');if(speed)speed.value=0;}
async function toggleAutoMotor(){
  const wanted=$('auto-motor-enable').checked;S.autoFanWanted=wanted;
  if(wanted&&S.autoThrottleWanted){S.autoFanWanted=false;throw Error('请先关闭倾斜自动油门');}
  if(!wanted){resetAutoMotor();await stopPart('auto_motor_disable');return;}
  $('auto-motor-speed').value=0;
  try{await operation('fan-optin',async t=>{
    S.heartbeat=true;await command('heartbeat',{},1800,t);check(t);
    const ack=await command('auto_motor_enable',{},1800,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.control?.auto_motor_allowed,2000,t);
  });}catch(error){if(error.name!=='AbortError')resetAutoMotor();throw error;}
}
async function startAutoMotor(){return operation('auto-motor',async t=>{
  if(S.autoThrottleWanted||S.tele?.control?.tilt_throttle?.enabled)throw Error('倾斜自动油门已接管，不能同时手动调速');
  if(!S.autoFanWanted||!S.tele?.control?.auto_motor_allowed||S.tele.operating_mode!=='ATTITUDE')throw new Error('请先勾选启用电机调速');
  validateEscConfig(ESC);S.heartbeat=true;await command('heartbeat',{},1800,t);check(t);
  try{
    if(!S.tele.outputs.esc_armed){
      await command('esc_config',ESC,1800,t);check(t);
      const ack=await command('esc_arm',{config_rev:S.rev},1800,t);
      await waitTelemetry(p=>{if(p.fault)throw Error(faultDescription(p));return p.epoch>=ack.epoch&&p.outputs.esc_armed&&p.control?.auto_motor_allowed;},23000,t);
    }
    check(t);if(!S.autoFanWanted)throw abort();await command('esc',{us:autoMotorUs()},1800,t);
  }catch(error){
    if(t.generation===S.generation&&t.action===S.action)await command('esc_stop').catch(()=>{});
    throw error;
  }
});}
async function sendAutoMotor(t=ticket()){
  if(!S.autoFanWanted||!S.tele?.control?.auto_motor_allowed||!S.tele?.outputs.esc_armed)return;
  check(t);await command('esc',{us:autoMotorUs()},1800,t);
}
function queueAutoMotor(){clearTimeout(autoMotorTimer);const t=ticket();if(S.tele?.outputs.esc_armed)autoMotorTimer=setTimeout(()=>run('风扇速度已更新。',()=>sendAutoMotor(t)),100);}

function renderImu2(packet,isFresh){
  const view=imu2Display(packet?.imu2,packet?.t_us,isFresh);
  $('imu2-status').textContent=view.status;$('imu2-status').classList.toggle('live',view.live);
  $('imu2-pitch').textContent=view.pitch;$('imu2-roll').textContent=view.roll;
  $('imu2-temp').textContent=view.temperature;$('imu2-rate').textContent=view.rate;
  $('imu2-accel').textContent=view.live?formatVector(packet.imu2?.a_g,3):'— / — / —';
  $('imu2-gyro').textContent=view.live?formatVector(packet.imu2?.g_dps,2):'— / — / —';
}
// IMU1（本地 0x68）是否参与 PID 由固件 control.source_id 决定。
function renderImu1(packet,isFresh){
  const s=packet?.imus?.[0],live=!!(isFresh&&s?.valid);
  const status={OK:'正常',STALE:'数据过期',WAIT_INT:'等待中断',READ_ERROR:'读取错误',NOT_FOUND:'未检测到'}[s?.status]||'异常';
  $('imu1-status').textContent=!packet?'等待连接':!isFresh?'数据过期':status;
  $('imu1-status').classList.toggle('live',live);
  $('imu1-age').textContent=live?`${Math.max(0,(packet.t_us-s.sample_t_us)/1000).toFixed(1)} ms`:'—';
  $('imu1-rate').textContent=live&&Number.isFinite(s.sample_hz)?`${s.sample_hz.toFixed(1)} Hz`:'—';
  $('imu1-temp').textContent=live&&Number.isFinite(s.temperature)?`${s.temperature.toFixed(1)} ℃`:'— ℃';
  $('imu1-uart').textContent=s?`${s.read_errors??0} / ${s.missed??0}`:'— / —';
  $('imu1-accel').textContent=live?formatVector(s.a,3):'— / — / —';
  $('imu1-gyro').textContent=live?formatVector(s.g,2):'— / — / —';
  $('imu1-role').textContent=packet?.control?.source_id===1?'当前控制源；原始轴数据（安装矩阵变换前）':'备用监测；不参与当前 PID';
}
const pidInputs={'kp-x':'kp_x','ki-x':'ki_x','kd-x':'kd_x','kp-y':'kp_y','ki-y':'ki_y','kd-y':'kd_y','max-tilt':'max_tilt_deg','azimuth-zero':'azimuth_zero_deg','sign-x':'sign_x','sign-y':'sign_y'};
function formatVector(values,digits){return Array.isArray(values)&&values.length===3?values.map(v=>Number.isFinite(v)?v.toFixed(digits):'—').join(' / '):'— / — / —';}
function settingsFromPacket(p){const c=p?.control,pid=c?.pid;if(!pid?.x||!pid?.y)return null;return {kp_x:pid.x.kp,ki_x:pid.x.ki,kd_x:pid.x.kd,kp_y:pid.y.kp,ki_y:pid.y.ki,kd_y:pid.y.kd,max_tilt_deg:pid.output_limit_deg,azimuth_zero_deg:c.azimuth_zero_deg,sign_x:c.sign_x,sign_y:c.sign_y,trim_x_deg:pid.trim_x_deg,trim_y_deg:pid.trim_y_deg};}
async function applyPid(){
  const settings=validatePidSettings(Object.fromEntries(Object.entries(pidInputs).map(([id,key])=>[key,
    key==='azimuth_zero_deg'&&S.tele?.control?.servo_calibration ? S.tele.control.azimuth_zero_deg : Number($(id).value)])));
  if(S.tele?.control?.model_enabled===false && Number.isFinite(S.tele.control?.pid?.trim_x_deg)){
    settings.trim_x_deg=Number($('trim-x').value);
    settings.trim_y_deg=Number($('trim-y').value);
    if(!Number.isFinite(Math.hypot(settings.trim_x_deg,settings.trim_y_deg)) ||
        Math.hypot(settings.trim_x_deg,settings.trim_y_deg)>Math.min(25,settings.max_tilt_deg))
      throw Error('配平偏转向量不能超过有效纠偏上限');
  }
  await operation('pid',async t=>{
    const ack=await command('pid_config',settings,2000,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&Object.entries(settings).every(([key,value])=>Math.abs(settingsFromPacket(p)?.[key]-value)<.0001),2500,t);
    S.pidDirty=false;
  });
}
function renderDetails(p,isFresh,connected,busy){
  const sensor=isFresh?p?.control_imu:null;
  const source=p?.control?.source_id??2;
  $('current-imu-title').textContent=source===2?'IMU2 · 主控本地 I2C · 0x69':'IMU1 · 主控本地 I2C · 0x68';
  $('current-error-label').textContent='读取错误 / 丢帧';
  $('primary-gyro').textContent=formatVector(sensor?.g,2);$('primary-accel').textContent=formatVector(sensor?.a,3);
  $('primary-temp').textContent=Number.isFinite(sensor?.temperature)?`${sensor.temperature.toFixed(1)} ℃`:'— ℃';
  $('primary-errors').textContent=sensor?`${sensor.read_errors??0} / ${sensor.missed??0}`:'— / —';
  $('primary-age').textContent=sensor?`${((p.t_us-sensor.sample_t_us)/1000).toFixed(1)} ms`:'等待数据';
  $('primary-rate').textContent=sensor&&Number.isFinite(sensor.sample_hz)?`${sensor.sample_hz.toFixed(1)} Hz`:'— Hz';
  $('primary-sample-age').textContent=sensor?`${Math.max(0,(p.t_us-sensor.sample_t_us)/1000).toFixed(1)} ms`:'— ms';
  $('primary-history').textContent=sensor?({LIVE:'实时',PREDICTED:'历史估算',HELD:'历史保持',UNAVAILABLE:'已失效'})[sensor.history_state]||'—':'—';
  $('primary-interface').textContent=sensor?`LOCAL_I2C / 0x${Number(sensor.address).toString(16)}`:'—';
  const pid=isFresh?p?.control?.pid:null;
  for(const axis of ['x','y'])for(const [cell,key] of [['p','p_deg'],['i','i_deg'],['d','d_deg'],['out','output_deg']])$('pid-'+axis+'-'+cell).textContent=Number.isFinite(pid?.[axis]?.[key])?pid[axis][key].toFixed(2):'—';
  $('pid-interval').textContent=Number.isFinite(pid?.dt_s)?`${(pid.dt_s*1000).toFixed(1)} ms`:'— ms';
  $('pid-limit-state').textContent=!pid?'等待':!p.control.auto_enabled?'未启动':pid.limited?'输出受限 · 限制积分':'正常';
  const settings=settingsFromPacket(p);
  if(isFresh&&settings&&!S.pidDirty&&document.activeElement?.closest('#pid-form')===null)for(const [id,key] of Object.entries(pidInputs))$(id).value=settings[key];
  $('apply-pid').disabled=!(connected&&isFresh&&!outputsActive()&&!p.calibrating&&!busy&&settings);
  $('pid-config-state').textContent=!connected?'连接设备后读取实际参数':!isFresh?'等待设备数据':S.pidDirty?'有待应用的修改':settings?'已读取设备实际参数':'固件未提供 PID 参数';
  drawTrend(isFresh);
}
function recordTrend(p){
  if(S.trendBoot!==p.boot||S.trendReference!==p.world.reference_id){S.trend=[];S.trendBoot=p.boot;S.trendReference=p.world.reference_id;}
  if(!p.world.valid||!p.control.tilt_valid||!p.control.error_deg?.every(Number.isFinite))return;
  const time=p.t_us/1e6,last=S.trend.at(-1);if(last&&time-last.t<.08)return;
  if(last&&time-last.t>.75)S.trend.push({t:time-.001,x:NaN,y:NaN});
  S.trend.push({t:time,x:p.control.error_deg[1],y:p.control.error_deg[0]});
  while(S.trend.length&&S.trend[0].t<time-30)S.trend.shift();
}
function drawTrend(isFresh){
  const canvas=$('trend-chart'),rect=canvas.getBoundingClientRect();if(rect.width<1)return;
  const dpr=Math.min(window.devicePixelRatio||1,2),w=Math.round(rect.width),h=210;
  if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);}
  const ctx=canvas.getContext('2d');ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  const left=42,right=w-12,top=12,bottom=h-25;
  const scale=Math.max(5,Math.ceil(Math.max(0,...S.trend.flatMap(s=>[s.x,s.y]).filter(Number.isFinite).map(Math.abs))/5)*5);
  ctx.font='11px system-ui';ctx.lineWidth=1;ctx.textAlign='right';
  for(let i=-2;i<=2;i++){let y=top+(2-i)*(bottom-top)/4;ctx.strokeStyle=i===0?'#456075':'#25374b';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(right,y);ctx.stroke();ctx.fillStyle='#8099b2';ctx.fillText(`${i*scale/2}°`,left-8,y+4);}
  ctx.textAlign='center';for(let i=0;i<=3;i++)ctx.fillText(`${i*10-30}s`,left+(right-left)*i/3,h-5);
  const end=S.trend.at(-1)?.t??0;
  for(const [key,color] of [['x','#4ce0ca'],['y','#78a7ff']]){ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;let started=false;for(const point of S.trend){if(!Number.isFinite(point[key])){started=false;continue;}const x=left+(1+(point.t-end)/30)*(right-left),y=top+(1-point[key]/scale)*(bottom-top)/2;if(started)ctx.lineTo(x,y);else{ctx.moveTo(x,y);started=true;}}ctx.stroke();}
  const history=S.tele?.control_imu?.history_state,name=`IMU${S.tele?.control?.source_id??2}`;
  $('chart-state').textContent=!S.trend.length?'等待真实设备数据':!isFresh?'连接中断 · 保留已接收曲线':history==='PREDICTED'?`${name} · 历史角速度估算（非新测量）`:history==='HELD'?`${name} · 保持历史估计，等待新样本`:`${name} · 倾斜角 °`;
}

function render(){
  const pose=activeMountPose(),sample=pose&&mountSample(S.tele);
  window.dispatchEvent(new CustomEvent('tvc-native-state',{detail:{
    mode:S.mode,connecting:S.connecting,tele:S.tele,lastRx:S.lastRx,operation:S.operation?.kind||'',stopping:S.stopping,
    heartbeat:S.heartbeat,autoFanWanted:S.autoFanWanted,autoThrottleWanted:S.autoThrottleWanted,
    mountBaseline:!!pose,mountTilted:!!pose?.leaned,mountError,mountAngle:sample?mountAngle(pose.a,sample.a):null,
  }}));
}
let nativeTimer=null;
function nativeTick(){
  if(S.mode&&S.boot&&!document.hidden&&fresh()){
    const generation=S.generation;
    if(S.heartbeat)command('heartbeat',{},800).catch(error=>{if(generation===S.generation)notice(`心跳回复异常：${error.message}`,responseNoticeType(error));});
    else if(!S.healthPending&&performance.now()-S.lastKeepalive>=1000){
      S.healthPending=true;S.lastKeepalive=performance.now();
      command('keepalive').catch(error=>{if(generation===S.generation)notice(error.message,responseNoticeType(error));}).finally(()=>{if(generation===S.generation)S.healthPending=false;});
    }
  }
  if(S.mode&&(S.heartbeat||S.operation)&&!fresh()){resetAutoMotor();cancelActions();S.heartbeat=false;void command('stop').catch(()=>{});notice('设备数据中断，已发送全部停止。','error');}
  render();
}
function nativeVisibility(){if(document.hidden){resetAutoMotor();const active=S.heartbeat||outputsActive()||S.operation;cancelActions();S.heartbeat=false;if(S.mode&&active)void stopAll().catch(()=>{});}}
function nativePageHide(){resetAutoMotor();cancelActions();S.heartbeat=false;if(S.mode&&S.boot)void command('stop',{},300).catch(()=>{});}
export function activate(){if(nativeTimer!==null)return;nativeTimer=setInterval(nativeTick,200);document.addEventListener('visibilitychange',nativeVisibility);window.addEventListener('pagehide',nativePageHide);render();}
export async function deactivate(){if(nativeTimer!==null){clearInterval(nativeTimer);nativeTimer=null;}document.removeEventListener('visibilitychange',nativeVisibility);window.removeEventListener('pagehide',nativePageHide);if(S.mode)await disconnect(true);}
export async function setView(view){S.view=view;if(S.mode)await switchDeviceMode(view);render();}
export async function saveServoCalibration(settings){
  await operation('servo-calibration',async t=>{
    if(!fresh()||!S.tele.control?.servo_calibration)throw Error('需要连接支持方向标定的固件');
    if(outputsActive()||S.tele.calibrating)throw Error('请先停止舵机和电机，并等待静止校准结束');
    const ack=await command('servo_calibrate',settings,2500,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.config_rev>=ack.config_rev&&
      Object.entries(settings).every(([key,value])=>key==='azimuth_deg'?Math.abs(p.control?.servo_calibration?.[key]-value)<0.051:p.control?.servo_calibration?.[key]===value),2500,t);
  });
}
export async function jogServoTravel(axis,direction){
  await operation('servo-travel-jog',async t=>{
    const travel=S.tele?.control?.servo_travel;
    if(!ready()||typeof travel?.jogging!=='boolean')throw Error('当前飞控无法连续调节轴程');
    try{
      if(travel.axis!==axis){
        if(travel.axis>=0||outputsActive())throw Error('请先停止全部输出');
        S.heartbeat=true;
        await command('heartbeat',{},1800,t);
        await command('servo_travel_start',{axis,explicit_endpoints:true},1800,t);
      }
      while(ready()){
        const sent=performance.now();
        // 只续期按住状态，飞控独立推进 PWM，无需等待舵机遥测。
        await command('servo_travel_jog',{axis,direction},230,t);
        await delay(Math.max(0,80-(performance.now()-sent)));
      }
    }finally{
      if(S.operation?.t===t)await command('servo_travel_hold',{},500).catch(()=>{});
    }
  });
}
export async function releaseServoTravel(){
  if(S.operation?.kind!=='servo-travel-jog')return;
  cancelActions();
  ++S.stopping;render();
  try{
    const ack=await command('servo_travel_hold',{},500);
    await waitTelemetry(p=>p.seq>ack.seq&&p.control?.servo_travel?.jogging===false,1000);
  }finally{--S.stopping;S.heartbeat=outputsActive();render();}
}
export async function updateServoTravel(axis,action){
  await operation('servo-travel',async t=>{
    const travel=S.tele?.control?.servo_travel;
    if(!fresh()||!travel||travel.axis!==axis)throw Error('当前轴的标定已结束，请重新开始');
    const capture=action==='min'||action==='max';
    const cmd=capture?'capture':action;
    const fields=capture?{endpoint:action}:{};
    const ack=await command('servo_travel_'+cmd,fields,2500,t);
    await waitTelemetry(p=>{
      const next=p.control?.servo_travel;
      if(!next||p.epoch<ack.epoch)return false;
      if(action==='save')return next.axis===-1&&!p.outputs.servo_enabled&&p.config_rev>=ack.config_rev&&
        [travel.min_us,travel.zero_us,travel.max_us].every((value,i)=>next.saved_us?.[travel.axis]?.[i]===value);
      if(next.axis!==axis||next.jogging||next.current_us!==next.target_us)return false;
      if(capture)return (action==='min'?next.min_us:next.max_us)===next.current_us;
      return action!=='zero'||next.zero_us===next.current_us;
    },2500,t);
    if(action==='save')S.heartbeat=false;
  });
}
export async function centerServo(){
  if(!fresh()||!S.tele.outputs.servo_enabled)throw Error('请先启用舵机');
  clearQueuedServo();
  const zero=S.tele.control?.azimuth_zero_deg??180;
  await command('servo',{x:angleToUs(zero,0),y:angleToUs(135,1)});
}
export async function restoreCalibration(){
  await operation('restore-calibration',async t=>{
    if(!fresh()||!S.tele.control?.gravity_reference)throw Error('当前固件不支持恢复默认校准');
    const ack=await command('restore_calibration',{},3500,t);
    await waitTelemetry(p=>p.epoch>=ack.epoch&&p.control?.gravity_reference==='DEFAULT',2500,t);
  });
}
export async function enableCgServo(){
  await operation('cg-enable',async t=>{
    const p=S.tele;
    const cg=p?.outputs?.cg_servo;
    if(!fresh()||cg?.max_us!==2000)throw Error('请更新至支持 500～2000 μs 限位的固件');
    if(p.outputs.esc_armed||p.outputs.esc_arming)throw Error('请先停止电机');
    if(p.fault||p.calibrating||!p.control_imu.valid)throw Error('设备尚未准备好');
    S.heartbeat=true;
    const ack=await command('cg_servo_enable',{},1800,t);
    await waitTelemetry(p=>p.seq>ack.seq&&p.outputs.cg_servo?.enabled,1500,t);
  });
}
export async function jogCgServo(direction){
  await operation('cg-jog',async t=>{
    const p=S.tele;
    if(!fresh()||!p.outputs.cg_servo?.enabled)throw Error('请先启用重心舵机');
    if(p.outputs.cg_servo.max_us!==2000)throw Error('请更新至支持 500～2000 μs 限位的固件');
    if(p.outputs.esc_armed||p.outputs.esc_arming)throw Error('请先停止电机');
    while(true){
      const cg=S.tele?.outputs.cg_servo;
      if(!fresh()||!cg?.enabled)return;
      if(direction>0?cg.command_us>=cg.max_us:cg.command_us<=500)return;
      const sent=performance.now();
      await command('cg_servo_jog',{direction},350,t);
      await delay(Math.max(0,120-(performance.now()-sent)));
      check(t);
    }
  });
}
export async function releaseCgServo(){
  if(S.operation?.kind==='cg-jog')await stopCgServo();
}
export async function stopCgServo(disable=false){
  if(S.operation?.kind==='cg-jog')cancelActions();
  ++S.stopping;render();
  try{
    const ack=await command(disable?'cg_servo_disable':'cg_servo_hold');
    await waitTelemetry(p=>{
      const cg=p.outputs.cg_servo;
      return p.seq>ack.seq&&cg&&(disable?!cg.enabled:cg.direction===0);
    },1500);
  }finally{--S.stopping;S.heartbeat=outputsActive();render();}
}
export {captureMount,run,connectUsb,connectWifi,disconnect,stopAll,stopAutomatic,recalibrate,switchImuSource,enableServos,disableServos,queueServo,startMotor,stopMotor,queueMotor,toggleTiltThrottle,startAutomatic,toggleAutoMotor,startAutoMotor,queueAutoMotor,applyPid};
