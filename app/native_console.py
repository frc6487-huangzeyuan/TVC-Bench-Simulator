"""将原控制台的协议和操作流程接入 Studio 原生界面，不复制或修改固件源码。"""
from pathlib import Path


def native_control(project):
    path=Path(project)/'web'/'dist'/'app.mjs'
    if not path.is_file():raise ValueError('当前飞控工程没有原网页控制代码。')
    source=path.read_text(encoding='utf-8')
    changes={
      "import {createAttitudePreview} from './preview.mjs';\n":'',
      'const attitudePreview=createAttitudePreview({get:$,settings:()=>settingsFromPacket(S.tele)||{kp_x:.6,kp_y:.6,sign_x:1,sign_y:1,max_tilt_deg:15,azimuth_zero_deg:180}});':
        'const attitudePreview={render(){}};',
      "function notice(message,type=''){$('notice').textContent=message;$('notice').className=`notice ${type}`.trim();}":
        "function notice(message,type=''){window.dispatchEvent(new CustomEvent('tvc-native-notice',{detail:{message,type}}));}",
      "  if(location.hostname!=='192.168.4.1'){location.href='http://192.168.4.1/?ui=2.4.3';return;}\n":'',
      "fetch('/api/device',{cache:'no-store'})":"fetch('/api/real/device',{cache:'no-store'})",
      'S.tele=p;S.lastRx=performance.now();recordTrend(p);':
        "S.tele=p;S.lastRx=performance.now();recordTrend(p);if(S.mode==='serial'||S.mode==='wifi')window.dispatchEvent(new CustomEvent('tvc-native-telemetry',{detail:{packet:p,transport:S.mode==='serial'?'usb':'wifi'}}));",
      'S.reader=S.writer=S.port=S.ws=null;S.writeChain=Promise.resolve();S.controlChain=Promise.resolve();':
        "S.reader=S.writer=S.port=S.ws=null;S.writeChain=Promise.resolve();S.controlChain=Promise.resolve();window.dispatchEvent(new CustomEvent('tvc-native-disconnected'));",
      "function resetAutoMotor(){S.autoFanWanted=false;S.autoThrottleWanted=false;clearTimeout(autoMotorTimer);$('tilt-throttle-enable').checked=false;$('auto-motor-enable').checked=false;$('auto-motor-speed').value=0;}":
        "function resetAutoMotor(){S.autoFanWanted=false;S.autoThrottleWanted=false;clearTimeout(autoMotorTimer);for(const id of ['tilt-throttle-enable','auto-motor-enable']){const e=$(id);if(e)e.checked=false;}const speed=$('auto-motor-speed');if(speed)speed.value=0;}",
      "function updateLabels(){$('servo-x-value').textContent=`${$('servo-x').value}°`;$('servo-y-value').textContent=`${$('servo-y').value}°`;$('motor-speed-value').textContent=`${$('motor-speed').value}%`;$('auto-motor-speed-value').textContent=`${$('auto-motor-speed').value}%`;}":
        'function updateLabels(){}'
    }
    for old,new in changes.items():
        if source.count(old)!=1:raise ValueError('原网页控制代码已变化，请重新检查 Studio 接入。')
        source=source.replace(old,new,1)
    marker='function render(){'
    if source.count(marker)!=1 or source.count("$('connect-usb').addEventListener")!=1:
        raise ValueError('原网页控制代码结构已变化，请重新检查 Studio 接入。')
    source=source[:source.index(marker)]
    source+='''
function render(){window.dispatchEvent(new CustomEvent('tvc-native-state',{detail:{mode:S.mode,connecting:S.connecting,tele:S.tele,lastRx:S.lastRx,operation:S.operation?.kind||'',stopping:S.stopping,heartbeat:S.heartbeat,autoFanWanted:S.autoFanWanted,autoThrottleWanted:S.autoThrottleWanted}}));}
let nativeTimer=null;
function nativeTick(){
  if(S.mode&&S.boot&&!document.hidden&&fresh()){
    const generation=S.generation;
    if(S.heartbeat)command('heartbeat',{},800).catch(error=>{if(generation===S.generation)notice(`心跳回复异常：${error.message}`,'error');});
    else if(!S.healthPending&&performance.now()-S.lastKeepalive>=1000){
      S.healthPending=true;S.lastKeepalive=performance.now();
      command('keepalive').catch(error=>{if(generation===S.generation)notice(error.message,'error');}).finally(()=>{if(generation===S.generation)S.healthPending=false;});
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
export async function centerServo(){const x=$('servo-x'),y=$('servo-y');if(x)x.value='180';if(y)y.value='135';await sendServo();}
export {run,connectUsb,connectWifi,disconnect,stopAll,stopAutomatic,recalibrate,switchImuSource,enableServos,disableServos,queueServo,startMotor,stopMotor,queueMotor,toggleTiltThrottle,startAutomatic,toggleAutoMotor,startAutoMotor,queueAutoMotor,applyPid};
'''
    return source
