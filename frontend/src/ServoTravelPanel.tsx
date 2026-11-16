import {useEffect,useRef,useState,type PointerEvent} from 'react';

export type ServoTravel = {
 axis:number;
 current_us:number;
 target_us:number;
 min_us:number;
 zero_us:number;
 max_us:number;
 saved_us:[number[],number[]];
};

type TravelAction='step'|'zero'|'save';
const axes=['方位轴','俯仰轴'];

export default function ServoTravelPanel({travel,canStart,busy,start,update,stop}:{
 travel?:ServoTravel;
 canStart:boolean;
 busy:boolean;
 start:(axis:number)=>Promise<void>;
 update:(action:TravelAction,delta?:number)=>Promise<void>;
 stop:()=>Promise<void>;
}){
 const [message,setMessage]=useState('');
 const press=useRef<symbol|null>(null);
 const pending=useRef(false);
 const endMove=()=>{press.current=null;};
 useEffect(()=>endMove,[]);
 const run=async(action:()=>Promise<void>,success='')=>{
  if(pending.current)return false;
  pending.current=true;
  setMessage('');
  try{await action();setMessage(success);return true;}
  catch(error){setMessage(error instanceof Error?error.message:String(error));return false;}
  finally{pending.current=false;}
 };
 const hold=async(event:PointerEvent<HTMLButtonElement>,delta:number)=>{
  if(event.button!==0||press.current||pending.current)return;
  const id=Symbol();
  event.currentTarget.setPointerCapture(event.pointerId);
  press.current=id;
  let delay=280;
  while(press.current===id&&await run(()=>update('step',delta))){
   await new Promise(resolve=>setTimeout(resolve,delay));
   delay=90;
  }
 };
 const active=!!travel&&travel.axis>=0;
 const settled=!!travel&&travel.current_us===travel.target_us;
 const rangeReady=!!travel&&travel.min_us<travel.zero_us&&travel.zero_us<travel.max_us;
 return <section aria-labelledby="servo-travel-title">
  <h3 id="servo-travel-title">零位与行程快速标定</h3>
  {!travel?<p className="field-note">更新至飞控 2.5.2 后，可用按钮移动单轴并记录零位和行程。</p>:<>
   <p className="field-note">观察实物，用按钮移到机械零位并确认，再分别移动到两侧可用端点。设备记录的是输出脉宽（µs）。</p>
   {active?<>
    <p>{axes[travel.axis]} · 当前 {travel.current_us} µs{settled?'':` → ${travel.target_us} µs`}</p>
    <div className="real-actions servo-travel-jog">
     {[-20,-5,5,20].map(delta=><button key={delta} type="button" className="button secondary compact"
      aria-label={`移动 ${delta>0?'+':''}${delta} 微秒`}
      onPointerDown={event=>void hold(event,delta)} onPointerUp={endMove} onPointerCancel={endMove}
      onLostPointerCapture={endMove} onBlur={endMove}
      onClick={event=>{if(event.detail===0)void run(()=>update('step',delta));}}>{delta>0?'+':''}{delta} µs</button>)}
    </div>
    <p className="field-note">{travel.zero_us?`已记录 ${travel.min_us} / ${travel.zero_us} / ${travel.max_us} µs（最小 / 零位 / 最大）`:'请先移到机械零位。移动按钮可点按或按住。'}</p>
    <div className="real-actions">
     <button type="button" className="button secondary compact" disabled={busy||!settled} onClick={()=>void run(()=>update('zero'))}>
      {travel.zero_us?'重新记录零位':'记录当前为零位'}
     </button>
     <button type="button" className="button primary compact" disabled={busy||!settled||!rangeReady} onClick={()=>void run(()=>update('save'),'零位和两端行程已保存到飞控。')}>
      保存零位和行程
     </button>
     <button type="button" className="button secondary compact" onClick={()=>{endMove();void stop().then(()=>setMessage('已取消标定。')).catch(error=>setMessage(String(error)));}}>取消标定</button>
    </div>
   </>:<>
    <div className="real-actions">{axes.map((label,i)=><button key={label} type="button" className="button secondary compact"
     disabled={!canStart||busy} onClick={()=>void run(()=>start(i))}>标定{label}</button>)}</div>
    <p className="field-note">{axes.map((label,i)=>`${label} ${travel.saved_us[i]?.[1]?travel.saved_us[i].join(' / ')+' µs':'使用默认行程'}`).join(' · ')}</p>
   </>}
   <p className="field-note" role="status">{message}</p>
  </>}
 </section>;
}
