import {useState} from 'react';

export type ServoCalibration = {
 azimuth_deg:number;
 elevation_deg:number;
 azimuth_reversed:boolean;
 elevation_reversed:boolean;
};

export default function ServoCalibrationPanel({calibration,canSave,previewPwm,recordedZero,onSave}:{
 calibration?:ServoCalibration;
 canSave:boolean;
 previewPwm?:number[];
 recordedZero?:boolean[];
 onSave:(settings:ServoCalibration)=>Promise<void>;
}){
 const [draft,setDraft]=useState(calibration);
 const [saving,setSaving]=useState(false);
 const [message,setMessage]=useState('');
 const values=draft??calibration??{azimuth_deg:186.8,elevation_deg:175,azimuth_reversed:false,elevation_reversed:true};
 // 快速标定保存精确 PWM；方向修改要沿用设备当前回传的原始角度。
 const settings={...values,
  azimuth_deg:recordedZero?.[0]?calibration!.azimuth_deg:values.azimuth_deg,
  elevation_deg:recordedZero?.[1]?calibration!.elevation_deg:values.elevation_deg};
 const update=<K extends keyof ServoCalibration>(key:K,value:ServoCalibration[K])=>{
  setDraft({...values,[key]:value});
  setMessage('');
 };
 const save=async()=>{
  setSaving(true);
  setMessage('');
  try{
   await onSave(settings);
   setDraft(undefined);
   setMessage('设备已确认保存，重启后保留。');
  }catch(error){
   setMessage(error instanceof Error?error.message:String(error));
  }finally{setSaving(false);}
 };
 const capture=(axis:'azimuth'|'elevation',channel:number,range:number)=>{
  if(!previewPwm)return;
  const angle=(previewPwm[channel]-500)*range/2000;
  update(axis==='azimuth'?'azimuth_deg':'elevation_deg',axis==='azimuth'?Math.round(angle*10)/10:Math.round(angle));
 };
 return <form id="servo-calibration" className="servo-calibration" onSubmit={e=>{e.preventDefault();void save();}}>
  <h3>方向标定</h3>
  <p className="field-note">初始位置使用舵机原始角度刻度。可在手动模式用滑块找中位，再取当前输出。保存前停止舵机和电机。</p>
  <div className="real-input-grid">
   {([{axis:'azimuth',label:'方位舵机',min:0,max:360},{axis:'elevation',label:'俯仰舵机',min:140,max:210}] as const).map(({axis,label,min,max},channel)=>{
    const angle=axis==='azimuth'?'azimuth_deg':'elevation_deg';
    const direction=axis==='azimuth'?'azimuth_reversed':'elevation_reversed';
    return <fieldset key={axis} disabled={saving||!calibration}>
     <legend>{label}</legend>
     <label>初始位置 / °<input type="number" required min={min} max={max} step={axis==='azimuth'?0.1:1} value={Number.isNaN(settings[angle])?'':settings[angle]} disabled={recordedZero?.[channel]} onChange={e=>update(angle,e.currentTarget.valueAsNumber)}/></label>
     <button type="button" className="button secondary compact" disabled={!previewPwm||recordedZero?.[channel]} onClick={()=>capture(axis,channel,axis==='azimuth'?360:270)}>取{label.slice(0,2)}当前输出</button>
     <label>方向<select value={values[direction]?'reverse':'normal'} onChange={e=>update(direction,e.target.value==='reverse')}><option value="normal">正向</option><option value="reverse">反向</option></select></label>
    </fieldset>;
   })}
  </div>
  <div className="real-actions"><button className="button primary compact" type="submit" disabled={!calibration||!canSave||saving}>{saving?'正在保存…':'保存到设备'}</button></div>
  <p className="field-note" role="status">{message||(!calibration?'连接设备后读取标定。此功能需要 2.4.10 或更新的固件。':'保存后立即生效。手动滑块的中位对应两轴标定的初始位置。')}</p>
 </form>;
}
