import {useCallback,useEffect,useRef,useState} from 'react';

export type ServoTravel = {
 axis:number;
 current_us:number;
 target_us:number;
 min_us:number;
 zero_us:number;
 max_us:number;
 saved_us:[number[],number[]];
 effective_us?:[number[],number[]];
 capture_endpoints?:boolean;
 explicit_endpoints?:boolean;
 jogging?:boolean;
};

type TravelAction='zero'|'min'|'max'|'save';
const axes=['方位轴','俯仰轴'];
const directions=[-1,1] as const;
const directionLabel=(direction:number)=>direction<0?'反向':'正向';

export default function ServoTravelPanel({travel,reversed,canStart,canMove,busy,jog,release,update,stop}:{
 travel?:ServoTravel;
 reversed:[boolean,boolean];
 canStart:boolean;
 canMove:boolean;
 busy:boolean;
 jog:(axis:number,direction:number)=>Promise<void>;
 release:()=>Promise<void>;
 update:(axis:number,action:TravelAction)=>Promise<void>;
 stop:()=>Promise<void>;
}){
 const [message,setMessage]=useState('');
 const press=useRef<{axis:number;direction:number}|null>(null);
 const pending=useRef(false);
 const endMove=useCallback(()=>{
  if(!press.current)return;
  press.current=null;
  void release().catch(error=>setMessage(String(error)));
 },[release]);
 useEffect(()=>{
  window.addEventListener('blur',endMove);
  return()=>{window.removeEventListener('blur',endMove);endMove();};
 },[endMove]);
 useEffect(()=>{if(!canMove)endMove();},[canMove]);

 const run=async(action:()=>Promise<void>,success='')=>{
  if(pending.current)return false;
  pending.current=true;
  setMessage('');
  try{await action();setMessage(success);return true;}
  catch(error){if(!(error instanceof Error&&error.name==='AbortError'))setMessage(error instanceof Error?error.message:String(error));return false;}
  finally{pending.current=false;}
 };
 const pwmDirection=(axis:number,direction:number)=>reversed[axis]?-direction:direction;
 const beginMove=(axis:number,direction:number)=>{
  if(press.current||pending.current)return;
  const held={axis,direction};
  press.current=held;
  void run(()=>jog(axis,pwmDirection(axis,direction))).finally(()=>{
   if(press.current===held)press.current=null;
  });
 };

 const active=travel?.axis??-1;
 const continuous=typeof travel?.jogging==='boolean';
 const settled=!!travel&&!travel.jogging&&travel.current_us===travel.target_us;
 const rangeReady=!!travel&&travel.min_us<travel.zero_us&&travel.zero_us<travel.max_us;
 return <section aria-labelledby="servo-travel-title">
  <h3 id="servo-travel-title">实物轴程标定</h3>
  {!travel?<p className="field-note">连接支持行程标定的飞控后可移动两轴。飞控 2.5.3 起支持逐端确认。</p>:<>
   <p className="field-note">{continuous?'按住正向或反向连续移动，松开保持当前位置。先记录零位，再记录两侧端点。':'请将飞控更新到 2.5.8 或更新版本后进行轴程标定。'}</p>
   <div className="servo-travel-axes">{axes.map((label,axis)=>{
    const saved=travel.saved_us[axis];
    const effective=travel.effective_us?.[axis];
    return <div className="servo-travel-axis" key={label}>
     <h4>{label}</h4>
     <div className="real-actions servo-travel-jog">{directions.map(direction=>{
      const held=press.current?.axis===axis&&press.current.direction===direction;
      const enabled=active===axis?canMove:active===-1&&canStart;
      return <button key={direction} type="button" className="button secondary compact"
       aria-label={label+directionLabel(direction)} disabled={!continuous||(!held&&(!enabled||busy))}
       onPointerDown={event=>{
        if(!event.isPrimary||event.button!==0||press.current||pending.current)return;
        event.currentTarget.setPointerCapture(event.pointerId);
        beginMove(axis,direction);
       }} onPointerUp={endMove}
       onPointerCancel={endMove} onLostPointerCapture={endMove} onBlur={endMove}
       onKeyDown={event=>{
        if(event.key!==' '&&event.key!=='Enter')return;
        event.preventDefault();
        if(!event.repeat)beginMove(axis,direction);
       }}
       onKeyUp={event=>{
        if(event.key!==' '&&event.key!=='Enter')return;
        event.preventDefault();endMove();
       }}>
       {directionLabel(direction)}
      </button>;
     })}</div>
     <p className="field-note">{saved?.[1]?'已保存 '+saved.join(' / ')+' µs':'尚未记录端点'}</p>
     {effective&&<p className="field-note">生效范围 {effective[0]}–{effective[2]} µs</p>}
    </div>;
   })}</div>
   {active>=0&&<>
    <p>{axes[active]} · 当前 {travel.current_us} µs{travel.jogging?' · 移动中':settled?'':' → '+travel.target_us+' µs'}</p>
    <p className="field-note">{travel.zero_us?
     '已记录 '+(travel.min_us<travel.zero_us?travel.min_us:'—')+' / '+travel.zero_us+' / '+(travel.max_us>travel.zero_us?travel.max_us:'—')+' µs（低端 / 零位 / 高端）'
     :'请先把轴移到机械零位。'}</p>
    {travel.zero_us&&!travel.explicit_endpoints&&<p className="field-note">当前固件记录本次点动经过的最小和最大脉宽。</p>}
    <div className="real-actions">
     <button type="button" className="button secondary compact" disabled={!canMove||busy||!settled} onClick={()=>void run(()=>update(active,'zero'))}>
      {travel.zero_us?'重新记录零位':'记录当前为零位'}
     </button>
     {travel.explicit_endpoints&&directions.map(direction=>{
      const high=pwmDirection(active,direction)>0;
      const onSide=high?travel.current_us>travel.zero_us:travel.current_us<travel.zero_us;
      return <button key={direction} type="button" className="button secondary compact"
       disabled={!canMove||busy||!settled||!travel.zero_us||!onSide}
       onClick={()=>void run(()=>update(active,high?'max':'min'),'已记录'+directionLabel(direction)+'端点。')}>
       记录{directionLabel(direction)}端点
      </button>;
     })}
     <button type="button" className="button primary compact" disabled={!canMove||busy||!settled||!rangeReady}
      onClick={()=>void run(()=>update(active,'save'),'零位和两个端点已写入飞控，重启后会自动应用。')}>
      保存零位和行程
     </button>
     <button type="button" className="button secondary compact" onClick={()=>{endMove();void stop().then(()=>setMessage('已取消标定。')).catch(error=>setMessage(String(error)));}}>取消标定</button>
    </div>
   </>}
   <p className="field-note">飞控会长期保存每个轴的标定。只有成功保存新的三点才会替换该轴记录，手动、自动和回位都使用生效范围。</p>
   <p className="field-note" role="status">{message}</p>
  </>}
 </section>;
}
