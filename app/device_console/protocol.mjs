export const VERSION=10, MAX_FRAME=8192;
export const TRANSPORT_GRACE_MS=2500;
const encoder=new TextEncoder();
export function crc32(input){const bytes=typeof input==='string'?encoder.encode(input):input;let c=0xffffffff;for(const b of bytes){c^=b;for(let i=0;i<8;i++)c=(c>>>1)^((c&1)?0xedb88320:0);}return((c^0xffffffff)>>>0).toString(16).padStart(8,'0');}
export function encodeFrame(packet){const body=JSON.stringify(packet);if(body.length>MAX_FRAME-10)throw new Error('FRAME_TOO_LONG');return `${body}\t${crc32(body)}\n`;}
export function decodeFrame(line){if(line.length>MAX_FRAME)throw new Error('FRAME_TOO_LONG');const split=line.lastIndexOf('\t');if(split<0||!/^[0-9a-f]{8}$/i.test(line.slice(split+1)))throw new Error('FRAME_FORMAT');const body=line.slice(0,split);if(crc32(body)!==line.slice(split+1).toLowerCase())throw new Error('CRC');const p=JSON.parse(body);if(!p||p.v!==VERSION||!Number.isSafeInteger(p.seq)||p.seq<1)throw new Error('VERSION_OR_SEQUENCE');return p;}
export class LineDecoder{
  constructor(){this.buffer='';this.decoder=new TextDecoder();this.discard=false;}
  push(bytes,onLine,onError){const chunk=typeof bytes==='string'?bytes:this.decoder.decode(bytes,{stream:true});for(const c of chunk){if(c==='\n'){if(this.discard)onError(new Error('FRAME_TOO_LONG'));else if(this.buffer.trim()&&!this.buffer.startsWith('#IMU1 ')){try{onLine(decodeFrame(this.buffer.replace(/\r$/,'')));}catch(e){onError(e);}}this.buffer='';this.discard=false;}else if(!this.discard){this.buffer+=c;if(this.buffer.length>MAX_FRAME){this.buffer='';this.discard=true;}}}}
}
const vector=v=>Array.isArray(v)&&v.length===3&&v.every(Number.isFinite);
export function servoAngleToUs(angle,channel,limits){
 const range=channel===0?360:270,n=Number(angle);
 const calibrated=Array.isArray(limits)&&limits.length===2&&limits.every(Number.isFinite)&&limits[0]<=limits[1];
 const [low,high]=calibrated?limits:channel===1?[100,170]:[0,360];
 if(!Number.isFinite(n)||n<low-1e-6||n>high+1e-6)
   throw new Error(!calibrated&&channel===1?'俯仰限位：−35～+35°':'舵机角度超出范围');
 return Math.round(500+Math.max(low,Math.min(high,n))*2000/range);
}
export function servoUsToAngle(us,channel){return (Number(us)-500)*(channel===0?360:270)/2000;}
export function validatePidSettings(c){
  for(const key of ['kp_x','ki_x','kd_x','kp_y','ki_y','kd_y'])if(!Number.isFinite(c[key])||c[key]<0||c[key]>(key.startsWith('kp')?10:2))throw new Error('PID 参数超出试调范围');
  if(!Number.isFinite(c.max_tilt_deg)||c.max_tilt_deg<1||c.max_tilt_deg>50)throw new Error('最大俯仰偏转须为1–50°');
  if(!Number.isFinite(c.azimuth_zero_deg)||c.azimuth_zero_deg<0||c.azimuth_zero_deg>360||![1,-1].includes(c.sign_x)||![1,-1].includes(c.sign_y))throw new Error('云台零位或方向设置不正确');
  return c;
}
export function validateTiltThrottle(c){
  if(c.rate_gain_pct_dps!==undefined&&(!Number.isFinite(c.rate_gain_pct_dps)||c.rate_gain_pct_dps<0||c.rate_gain_pct_dps>5))throw new Error('角速度增益必须为 0～5');
  if(!['base_pct','gain_pct_deg','max_pct','deadband_deg','ramp_pct_s'].every(k=>Number.isFinite(c[k]))||c.base_pct<0||c.base_pct>c.max_pct||c.max_pct<=0||c.max_pct>100||c.gain_pct_deg<0||c.gain_pct_deg>5||c.deadband_deg<0||c.deadband_deg>30||c.ramp_pct_s<1||c.ramp_pct_s>100)throw new Error('自动油门参数无效：基础值不能超过上限，增益 0～5%/°，死区 0～30°，变化速度 1～100%/秒');
  return c;
}
// 独立验证可选的监测数据，IMU2 不参与 IMU1 控制的启用判断。
export function imu2Display(m,nowUs,transportFresh){
  const empty=status=>({status,live:false,pitch:'—°',roll:'—°',temperature:'— ℃',rate:'— Hz'});
  if(!transportFresh)return empty(nowUs===undefined?'等待连接':'数据过期');
  if(m===undefined)return empty('未提供数据');
  if(!m||m.address!==0x69||typeof m.present!=='boolean'||typeof m.valid!=='boolean'||typeof m.tilt_valid!=='boolean'||!Number.isSafeInteger(m.sample_t_us)||m.sample_t_us<0||!Number.isSafeInteger(nowUs)||m.sample_t_us>nowUs)return empty('数据异常');
  if(!m.present)return empty(m.status==='NOT_FOUND'&&!m.valid&&!m.tilt_valid?'未检测到':'数据异常');
  if(!m.valid){const statuses={WAIT_INT:'等待中断',READ_ERROR:'读取异常',STALE:'数据过期'};return empty(!m.tilt_valid&&Object.hasOwn(statuses,m.status)?statuses[m.status]:'数据异常');}
  // 旧固件的监测超时为 50 ms；10 Hz 监测固件会明确提供
  // 其独立的 300 ms 超时值；IMU1 的历史窗口由主控单独报告。
  const timeoutUs=m.sample_timeout_us===undefined?50000:m.sample_timeout_us;
  if(!Number.isSafeInteger(timeoutUs)||timeoutUs<50000||timeoutUs>1000000)return empty('数据异常');
  if(m.status!=='OK'||nowUs-m.sample_t_us>=timeoutUs)return empty('数据过期');
  if(!Number.isFinite(m.temp_c)||!Number.isFinite(m.sample_hz)||m.sample_hz<0)return empty('数据异常');
  if(m.tilt_valid&&(!Number.isFinite(m.pitch_deg)||Math.abs(m.pitch_deg)>90||!Number.isFinite(m.roll_deg)||Math.abs(m.roll_deg)>180))return empty('数据异常');
  return {status:m.tilt_valid?'正常':'倾角不可用',live:true,
    pitch:m.tilt_valid?`${m.pitch_deg.toFixed(1)}°`:'—°',roll:m.tilt_valid?`${m.roll_deg.toFixed(1)}°`:'—°',
    temperature:`${m.temp_c.toFixed(1)} ℃`,rate:`${m.sample_hz.toFixed(1)} Hz`};
}
export function isTelemetry(p){
  if(p?.warning!==undefined&&typeof p.warning!=='string')return false;
  if(!p||p.v!==VERSION||p.imu_count!==1||p.type!=='telemetry'||typeof p.boot!=='string'||typeof p.session!=='string'||!Number.isSafeInteger(p.t_us)||p.t_us<0||!['DISARMED','SERVO_TEST','ARMING','ARMED','FAULT'].includes(p.state)||!['ATTITUDE','MANUAL'].includes(p.operating_mode)||typeof p.fault!=='string'||!Number.isInteger(p.epoch)||!Number.isInteger(p.config_rev)||typeof p.firmware!=='string'||typeof p.calibrating!=='boolean'||typeof p.calibrated!=='boolean')return false;
  const sensorValid=(s,address,source)=>{
    const timeout=s?.sample_timeout_us??50000;
    return s&&s.address===address&&s.source===source&&typeof s.valid==='boolean'&&typeof s.present==='boolean'&&vector(s.a)&&vector(s.g)&&Number.isFinite(s.pitch)&&Number.isFinite(s.roll)&&Number.isSafeInteger(s.sample_t_us)&&s.sample_t_us>=0&&s.sample_t_us<=p.t_us&&Number.isSafeInteger(timeout)&&timeout>=50000&&timeout<=2000000&&(!s.valid||(s.present&&p.t_us-s.sample_t_us<timeout));
  };
  const imu1Source=p.imus?.[0]?.source;
  if(!Array.isArray(p.imus)||p.imus.length!==1||!['LOCAL_I2C_IMU1','SUPERMINI_UART'].includes(imu1Source)||!sensorValid(p.imus[0],0x68,imu1Source))return false;
  const source=p.control?.source_id;
  if(![1,2].includes(source)||p.control_imu?.source_id!==source||!sensorValid(p.control_imu,source===1?0x68:0x69,source===1?imu1Source:'LOCAL_I2C_IMU2'))return false;
  const w=p.world;
  if(!w||w.frame!=='LOCAL_Z_UP'||typeof w.valid!=='boolean'||w.heading_absolute!==false||!Number.isSafeInteger(w.reference_id)||w.reference_id<0||typeof w.correction_active!=='boolean'||!Number.isFinite(w.reference_age_s)||w.reference_age_s<0)return false;
  if(typeof w.heading_defined!=='boolean'||![w.euler_deg,w.a_body_g,w.gyro_body_dps].every(vector))return false;
  const worldVector=v=>vector(v)||(!w.valid&&Array.isArray(v)&&v.length===3&&v.every(n=>n===null||Number.isFinite(n)));
  if(!worldVector(w.f_world_g)||!worldVector(w.linear_world_mps2))return false;
  if(!Array.isArray(w.q_bw)||w.q_bw.length!==4||!w.q_bw.every(Number.isFinite)||Math.abs(Math.hypot(...w.q_bw)-1)>.01)return false;
  if(!Array.isArray(w.r_bw)||w.r_bw.length!==9||!w.r_bw.every(Number.isFinite))return false;
  if(w.valid&&(!p.calibrated||!p.control_imu.valid||!p.control_imu.calibrated||w.reference_id<1))return false;
  if(!p.control||typeof p.control.auto_motor_allowed!=='boolean'||p.control.geometry!=='AZIMUTH_ELEVATION'||typeof p.control.mount_ready!=='boolean'||typeof p.control.tilt_valid!=='boolean')return false;
  const o=p.outputs;
  if(!o||typeof o.servo_enabled!=='boolean'||!Array.isArray(o.servo_us)||o.servo_us.length!==2||!o.servo_us.every(Number.isFinite)||typeof o.esc_armed!=='boolean'||typeof o.esc_arming!=='boolean'||typeof o.esc_configured!=='boolean'||typeof o.esc_signal_enabled!=='boolean'||!Number.isFinite(o.esc_us))return false;
  if(o.servo_range_deg?.[0]!==360||o.servo_range_deg?.[1]!==270)return false;
  const torque=p.control.thrust_torque;
  if(torque!==undefined){
    const pair=v=>Array.isArray(v)&&v.length===2&&v.every(Number.isFinite);
    if(!torque||!['active','prepared','limited'].every(k=>typeof torque[k]==='boolean')||
       !['target_deg','lean_deg','requested_nm','allocated_nm'].every(k=>pair(torque[k]))||
       !Number.isFinite(torque.throttle_target_pct)||torque.throttle_target_pct<0||torque.throttle_target_pct>100)return false;
    if(torque.azimuth_rewind!==undefined&&typeof torque.azimuth_rewind!=='boolean')return false;
    if(torque.active&&(!torque.prepared||!p.control.auto_enabled||!o.servo_enabled||!o.esc_armed||
       o.esc_arming||p.operating_mode!=='ATTITUDE'||p.fault||!w.valid))return false;
  }
  const cg=o.cg_servo;
  if(cg!==undefined){
    if(!cg||typeof cg.enabled!=='boolean'||![-1,0,1].includes(cg.direction)||!Number.isInteger(cg.command_us)||cg.command_us<500||cg.command_us>2500)return false;
    if(!cg.enabled&&cg.direction!==0)return false;
    if(cg.max_us!==undefined&&(!Number.isInteger(cg.max_us)||cg.max_us<=500||cg.max_us>2500||cg.command_us>cg.max_us))return false;
    if((cg.min_deg!==undefined||cg.max_deg!==undefined)&&
       (!Number.isFinite(cg.min_deg)||!Number.isFinite(cg.max_deg)||cg.min_deg>=cg.max_deg))return false;
  }
  if(o.esc_configured&&(!Number.isInteger(o.esc_stop_us)||o.esc_stop_us<900||o.esc_stop_us>2100||!Number.isInteger(o.esc_min_us)||o.esc_min_us<900||o.esc_min_us>=o.esc_stop_us||!Number.isInteger(o.esc_max_us)||o.esc_max_us<=o.esc_stop_us||o.esc_max_us>2100))return false;
  if((o.esc_armed||o.esc_arming)&&(!o.esc_configured||!o.esc_signal_enabled))return false;
  const tt=p.control.tilt_throttle;
  if(tt!==undefined){
    if(!tt||typeof tt.enabled!=='boolean'||!['tilt_deg','target_pct','output_pct'].every(k=>Number.isFinite(tt[k]))||tt.tilt_deg<0||tt.tilt_deg>180||tt.target_pct<0||tt.output_pct<0||tt.target_pct>tt.max_pct+.001||tt.output_pct>tt.max_pct+.001)return false;
    try{validateTiltThrottle(tt);}catch{return false;}
    if(tt.enabled&&(!p.control.auto_enabled||!o.servo_enabled||!o.esc_armed||o.esc_arming||p.operating_mode!=='ATTITUDE'||p.fault||!w.valid))return false;
  }
  return true;
}
export function validateEscConfig(c){for(const key of ['stop_us','min_us','max_us'])if(!Number.isInteger(c[key])||c[key]<900||c[key]>2100)throw new Error('脉宽必须是900–2100之间的整数');if(c.min_us>=c.stop_us||c.max_us<=c.stop_us)throw new Error('中位停止值必须位于反向端与正向端之间');if(!Number.isInteger(c.hold_ms)||c.hold_ms<1000||c.hold_ms>15000||c.hz!==50)throw new Error('保持时间须为1000–15000ms，仅支持50Hz PWM');if(c.confirmed!==true)throw new Error('请先确认已拆桨，并核对说明书');return c;}
export function canOperate(packet,ageMs){return !!(isTelemetry(packet)&&packet.operating_mode==='MANUAL'&&ageMs<TRANSPORT_GRACE_MS&&!packet.fault&&!packet.calibrating&&packet.control_imu.valid);}
export function controlLeaseValid(start,current){return start.generation===current.generation&&start.session===current.session&&current.heartbeatEnabled===true&&current.hidden===false;}
export class DemoDevice{
  constructor(session){this.session=session;this.boot='simulation-only';this.seq=0;this.epoch=0;this.rev=0;this.start=performance.now();this.fault='';this.operatingMode='ATTITUDE';this.servo=false;this.auto=false;this.autoMotorAllowed=false;this.x=1500;this.y=1500;this.config=null;this.armed=false;this.armingAt=0;this.us=0;this.signal=false;this.calibrateUntil=0;this.worldReady=false;this.referenceId=0;this.referenceAt=performance.now();this.yawZero=0;this.source=2;this.tiltThrottle={enabled:false,base_pct:0,gain_pct_deg:1,max_pct:30,deadband_deg:2,ramp_pct_s:10,rate_gain_pct_dps:0,rate_dps:0,tilt_deg:0,target_pct:0,output_pct:0};}
  stopServo(){if(this.tiltThrottle.enabled)this.stopEsc();this.servo=false;this.auto=false;this.epoch++;}
  stopEsc(){this.tiltThrottle.enabled=false;this.tiltThrottle.output_pct=0;this.armed=false;this.armingAt=0;this.epoch++;this.us=this.signal?(this.config?.stop_us??0):0;}
  stop(){this.autoMotorAllowed=false;this.stopServo();this.stopEsc();}
  motorAllowed(){return this.operatingMode==='MANUAL'||(this.operatingMode==='ATTITUDE'&&this.autoMotorAllowed&&this.worldReady);}
  command(p){let ok=true,reason='ACCEPTED';
    if(!['hello','heartbeat','keepalive','stop','servo_disable','esc_stop','auto_disable','auto_motor_disable','tilt_throttle_disable'].includes(p.cmd)&&p.epoch!==this.epoch){ok=false;reason='OLD_EPOCH';}
    else if(p.cmd==='hello'){this.stop();reason='HELLO';}else if(p.cmd==='keepalive')reason='KEEPALIVE';else if(p.cmd==='heartbeat')reason='HEARTBEAT';else if(p.cmd==='stop'){this.stop();reason='STOPPED';}else if(p.cmd==='clear_fault'){this.fault='';this.stop();}
    else if(p.cmd==='set_mode'){if(p.mode==='ATTITUDE'){this.stop();this.operatingMode='ATTITUDE';this.calibrateUntil=performance.now()+3000;this.worldReady=false;reason='MODE_ATTITUDE_CALIBRATING';}else if(p.mode==='MANUAL'){this.stop();this.operatingMode='MANUAL';this.calibrateUntil=0;if(['CALIBRATION_MOVED','CALIBRATION_DATA','HEADING_UNDEFINED'].includes(this.fault))this.fault='';reason='MODE_MANUAL';}else{ok=false;reason='INVALID_MODE';}}
    else if(p.cmd==='calibrate'){if(this.servo||this.armed||this.armingAt){ok=false;reason='STOP_OUTPUTS_FIRST';}else{this.autoMotorAllowed=false;this.epoch++;this.calibrateUntil=performance.now()+3000;this.worldReady=false;}}
    else if(p.cmd==='world_zero'){if(!this.worldReady||this.servo||this.armed||this.armingAt){ok=false;reason='WORLD_NOT_READY';}else{this.yawZero=15*Math.sin((performance.now()-this.start)/1000*.12);this.referenceId++;this.referenceAt=performance.now();this.epoch++;}}
    else if(p.cmd==='servo_enable'){if(this.operatingMode!=='MANUAL'){ok=false;reason='MODE_LOCKED';}else if(this.calibrateUntil||this.fault||this.servo){ok=false;reason='NOT_READY';}else{this.servo=true;this.auto=false;this.epoch++;this.x=this.y=1500;}}
    else if(p.cmd==='servo_disable'){if(this.operatingMode!=='MANUAL'){ok=false;reason='MODE_LOCKED';}else this.stopServo();}
    else if(p.cmd==='servo'){if(this.operatingMode!=='MANUAL'){ok=false;reason='MODE_LOCKED';}else if(!this.servo||p.epoch!==this.epoch||!Number.isInteger(p.x)||!Number.isInteger(p.y)||p.x<500||p.x>2500||p.y<500||p.y>2500){ok=false;reason='SERVO_NOT_READY';}else{this.x=p.x;this.y=p.y;}}
    else if(p.cmd==='esc_config'){try{if(!this.motorAllowed())throw new Error('MODE_LOCKED');validateEscConfig(p);if(this.armed||this.armingAt)throw new Error('STOP_ESC_FIRST');this.stopEsc();this.config={...p};this.rev++;this.signal=false;this.us=0;}catch(e){ok=false;reason=e.message;}}
    else if(p.cmd==='esc_arm'){if(!this.motorAllowed()){ok=false;reason='MODE_LOCKED';}else if(this.calibrateUntil||this.fault||!this.config||p.config_rev!==this.rev){ok=false;reason='NOT_CONFIGURED';}else{this.stopEsc();this.signal=true;this.us=this.config.stop_us;this.armingAt=performance.now()+this.config.hold_ms;}}
    else if(p.cmd==='esc'){if(this.tiltThrottle.enabled){ok=false;reason='AUTO_THROTTLE_ACTIVE';}else if(!this.motorAllowed()){ok=false;reason='MODE_LOCKED';}else if(!this.armed||p.epoch!==this.epoch||!Number.isInteger(p.us)||p.us<this.config.min_us||p.us>this.config.max_us){ok=false;reason='NOT_ARMED_OR_RANGE';}else this.us=p.us;}
    else if(p.cmd==='esc_stop'){this.stopEsc();}
    else if(p.cmd==='tilt_throttle_disable'){this.stopEsc();this.autoMotorAllowed=false;}
    else if(p.cmd==='tilt_throttle_config'){try{if(this.servo||this.armed||this.armingAt)throw Error('STOP_OUTPUTS_FIRST');validateTiltThrottle(p);for(const key of ['base_pct','gain_pct_deg','max_pct','deadband_deg','ramp_pct_s','rate_gain_pct_dps'])if(p[key]!==undefined)this.tiltThrottle[key]=p[key];this.tiltThrottle.enabled=false;this.epoch++;}catch(e){ok=false;reason=e.message;}}

    else if(p.cmd==='auto_motor_enable'){if(this.operatingMode!=='ATTITUDE'){ok=false;reason='MODE_LOCKED';}else if(!this.worldReady||this.fault||this.calibrateUntil){ok=false;reason='NOT_READY';}else{this.autoMotorAllowed=true;this.epoch++;}}
    else if(p.cmd==='auto_motor_disable'){if(this.operatingMode!=='ATTITUDE'){ok=false;reason='MODE_LOCKED';}else{this.autoMotorAllowed=false;this.stopEsc();}}
    else if(p.cmd==='auto_enable'){if(this.operatingMode!=='ATTITUDE'){ok=false;reason='MODE_LOCKED';}else if(!this.worldReady||this.fault||((this.armed||this.armingAt)&&!this.autoMotorAllowed)){ok=false;reason='NOT_READY';}else if(p.tilt_throttle&&(!this.armed||!this.autoMotorAllowed)){ok=false;reason='THROTTLE_NOT_READY';}else{this.tiltThrottle.enabled=!!p.tilt_throttle;this.servo=true;this.auto=true;this.epoch++;this.x=this.y=1500;}}
    else if(p.cmd==='auto_disable'){if(this.operatingMode!=='ATTITUDE'){ok=false;reason='MODE_LOCKED';}else this.stop();}
    else if(p.cmd==='select_imu'){if(![1,2].includes(p.source)){ok=false;reason='INVALID_IMU_SOURCE';}else if(p.source===this.source){reason='IMU_SOURCE_UNCHANGED';}else{this.stop();this.source=p.source;this.worldReady=false;this.calibrateUntil=performance.now()+3000;this.epoch++;reason='IMU_SOURCE_SELECTED';}}
    else{ok=false;reason='UNKNOWN_COMMAND';}
    return {v:VERSION,type:'ack',seq:++this.seq,request_seq:p.seq,session:this.session,boot:this.boot,ok,reason,epoch:this.epoch,config_rev:this.rev,firmware:'SIMULATION / 模拟'};
  }
  telemetry(){const now=performance.now();if(this.armingAt&&now>=this.armingAt){this.armingAt=0;this.armed=true;}const t=(now-this.start)/1000;
    if(this.calibrateUntil&&now>=this.calibrateUntil){this.calibrateUntil=0;this.worldReady=true;this.referenceId++;this.referenceAt=now;this.yawZero=15*Math.sin(t*.12);}
    const pitch=8*Math.sin(t*.75),roll=5*Math.sin(t*.5+.6),yaw=15*Math.sin(t*.12)-this.yawZero;
    const world=demoWorld(roll,pitch,yaw,this.worldReady,this.referenceId,(now-this.referenceAt)/1000);
    const sample=address=>({address,source:address===0x69?'LOCAL_I2C_IMU2':'LOCAL_I2C_IMU1',source_id:address===0x69?2:1,
      present:true,valid:true,calibrated:this.worldReady,sample_t_us:Math.max(0,Math.round(t*1e6)-650),
      samples:Math.round(t*200),errors:0,missed:0,read_errors:0,race_drops:0,pitch,roll,temperature:27,
      a:[-Math.sin(pitch*Math.PI/180),Math.sin(roll*Math.PI/180),Math.cos(pitch*Math.PI/180)*Math.cos(roll*Math.PI/180)],
      g:[2.5*Math.cos(t*.5+.6),6*Math.cos(t*.75),0],bias:[0,0,0],
      sample_timeout_us:2000000,fresh_window_us:50000,history_state:'LIVE',status:'OK',sample_hz:200,target_hz:200,
      uart_replies:address===0x68?Math.round(t*200):undefined,uart_bad_frames:0,uart_timeouts:0});
    const imu1=sample(0x68),controlImu=sample(this.source===1?0x68:0x69);
    controlImu.control_source=true;imu1.control_source=this.source===1;
    return {v:VERSION,type:'telemetry',seq:++this.seq,session:this.session,boot:this.boot,t_us:Math.round(t*1e6),firmware:'SIMULATION / 模拟',fault:this.fault,epoch:this.epoch,config_rev:this.rev,state:this.armingAt?'ARMING':this.armed?'ARMED':this.servo?'SERVO_TEST':'DISARMED',operating_mode:this.operatingMode,calibrating:now<this.calibrateUntil,calibrated:this.worldReady,imu_count:1,
      control_source:this.source,control_source_name:'IMU'+this.source,
      world,control:{auto_enabled:this.auto,auto_motor_allowed:this.autoMotorAllowed,error_deg:[roll,pitch],zero_deg:[0,0],source_id:this.source,source_address:this.source===1?0x68:0x69,tilt_throttle:{...this.tiltThrottle},geometry:'AZIMUTH_ELEVATION',mount_ready:this.worldReady,tilt_valid:this.worldReady},
      sample_hz:200,bad_frames:0,
      outputs:{servo_enabled:this.servo,servo_us:[this.x,this.y],servo_range_deg:[360,270],esc_configured:!!this.config,esc_signal_enabled:this.signal,esc_armed:this.armed,esc_arming:!!this.armingAt,esc_us:this.us,esc_stop_us:this.config?.stop_us??0,esc_min_us:this.config?.min_us??0,esc_max_us:this.config?.max_us??0},
      imus:[imu1],control_imu:controlImu,
      imu2:{address:0x69,present:true,valid:true,tilt_valid:true,sample_t_us:Math.max(0,Math.round(t*1e6)-500),sample_hz:200,target_hz:200,sample_timeout_us:300000,status:'OK',temp_c:27.4,pitch_deg:pitch,roll_deg:roll,a_g:[0,0,1],g_dps:[0,0,0],control_source:this.source===2}};

  }
}
// 四元数和矩阵约定与固件一致：机体坐标转换到局部世界坐标，Z 轴向上。
export function demoWorld(roll,pitch,yaw,valid=true,referenceId=1,referenceAge=0){
  const [r,p,y]=[roll,pitch,yaw].map(a=>a*Math.PI/360),cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
  const q=[cy*cp*cr+sy*sp*sr,cy*cp*sr-sy*sp*cr,cy*sp*cr+sy*cp*sr,sy*cp*cr-cy*sp*sr];
  const [w,x,zY,z]=q;
  const R=[1-2*(zY*zY+z*z),2*(x*zY-w*z),2*(x*z+w*zY),2*(x*zY+w*z),1-2*(x*x+z*z),2*(zY*z-w*x),2*(x*z-w*zY),2*(zY*z+w*x),1-2*(x*x+zY*zY)];
  return {valid,frame:'LOCAL_Z_UP',reference_id:referenceId,heading_absolute:false,heading_defined:Math.abs(Math.cos(pitch*Math.PI/180))>=.01,correction_active:true,reference_age_s:referenceAge,q_bw:q,r_bw:R,euler_deg:[roll,pitch,yaw],a_body_g:[R[6],R[7],R[8]],gyro_body_dps:[0,0,0],f_world_g:valid?[0,0,1]:[null,null,null],linear_world_mps2:valid?[0,0,0]:[null,null,null]};
}
export function isFreshCommandTime(sentUs,nowUs){return Number.isSafeInteger(sentUs)&&sentUs>=0&&sentUs<=nowUs+100000&&nowUs<=sentUs+500000;}
export function telemetryToCsv(records){
  const headers=['mode','received_iso','firmware','boot_id','t_us','operating_mode','state','fault','servo_enabled','servo_x_us','servo_y_us','esc_armed','esc_us'];
  for(const k of ['valid','sample_t_us','pitch_deg','roll_deg','ax_g','ay_g','az_g','gx_dps','gy_dps','gz_dps','errors','missed'])headers.push('imu1_'+k);
  headers.push('world_valid','world_reference_id','world_reference_age_s','heading_absolute','gravity_correction','world_roll_deg','world_pitch_deg','world_yaw_deg','qw','qx','qy','qz','body_ax_g','body_ay_g','body_az_g','body_gx_dps','body_gy_dps','body_gz_dps','world_fx_g','world_fy_g','world_fz_g','world_ax_mps2','world_ay_mps2','world_az_mps2');
  const quote=v=>'"'+String(v??'').replaceAll('"','""')+'"';
  const rows=records.map(p=>{const o=p.outputs,s=p.imus[0],w=p.world;return [p.mode,p.received,p.firmware,p.boot,p.t_us,p.operating_mode,p.state,p.fault,o.servo_enabled,...o.servo_us,o.esc_armed,o.esc_us,s.valid,s.sample_t_us,s.pitch,s.roll,...s.a,...s.g,s.errors,s.missed,w.valid,w.reference_id,w.reference_age_s,w.heading_absolute,w.correction_active,...w.euler_deg,...w.q_bw,...w.a_body_g,...w.gyro_body_dps,...w.f_world_g,...w.linear_world_mps2].map(quote).join(',');});
  return '\ufeff'+[headers.join(','),...rows].join('\r\n');
}
