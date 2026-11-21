import {Quaternion,Vector3,MathUtils} from 'three';
import type {Frame,Pose,Triple} from './types';

export type DeviceModelBody=Pose&{parent:string|null;axis:Triple|null};

function finiteVector(value:unknown,length:number):value is number[]{
  return Array.isArray(value)&&value.length===length&&value.every(Number.isFinite);
}

export function deviceTilt(valid:boolean,rotation:unknown):[number,number]|null{
  if(!valid||!finiteVector(rotation,9))return null;
  // r_bw 为机体到世界的行主序矩阵。第三行是世界 +Z 在机体系的方向。
  // 台架外轴绕世界 -Y，内轴在零位绕世界 +X，机体系零位绕世界 Z 转了 180°。
  // 用重力方向解算两根轴承的角度，世界航向变化不会扭转台架。
  const [x,y,z]=rotation.slice(6);
  const yz=Math.hypot(y,z);
  if(Math.hypot(x,y,z)<.5)return null;
  return [Math.atan2(-x,yz),yz<1e-8?0:Math.atan2(-y,z)];
}

export function deviceServoAngles(degrees:unknown,pwm:unknown):[number,number]|null{
  if(finiteVector(degrees,2))return [degrees[0],degrees[1]];
  // 旧协议 10 固件可只回传逻辑脉宽。映射沿用实验台的 180° / 135° 中位。
  if(finiteVector(pwm,2))return [(pwm[0]-500)*360/2000,(pwm[1]-500)*270/2000];
  return null;
}

export function deviceFrame(model:DeviceModelBody[],tilt:[number,number]|null,servo:[number,number]|null):Pick<Frame,'pose'|'thrust'>{
  const angleByBody:Record<string,number>={
    outer_frame:tilt?.[0]??0,
    rocket_body:tilt?.[1]??0,
    azimuth_stage:servo?MathUtils.degToRad(servo[0]-180):0,
    thrust_unit:servo?MathUtils.degToRad(servo[1]-135):0,
  };
  const joint=new Quaternion(),axis=new Vector3();
  const transforms=new Map<string,{position:Vector3;rotation:Quaternion}>();
  // 按父子关系装配。存在重心滑架时，它随箭体倾斜，保持在模型参考位置。
  const pose=model.map(body=>{
    const parent=body.parent?transforms.get(body.parent):undefined;
    const position=new Vector3().fromArray(body.position);
    const rotation=new Quaternion();
    if(parent){
      position.applyQuaternion(parent.rotation).add(parent.position);
      rotation.copy(parent.rotation);
    }
    const [w,x,y,z]=body.quaternion;
    rotation.multiply(joint.set(x,y,z,w));
    if(body.axis)rotation.multiply(joint.setFromAxisAngle(axis.fromArray(body.axis),angleByBody[body.name]));
    transforms.set(body.name,{position,rotation});
    return {name:body.name,position:position.toArray(),quaternion:[rotation.w,rotation.x,rotation.y,rotation.z]};
  });
  return {pose,thrust:0};
}
