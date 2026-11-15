// 纯几何预览：不包含串口、WebSocket、硬件命令或真实飞行动力学。
// R 使用行优先数组，v_world = R * v_body；机体 +Z 指向箭头。
const RAD=Math.PI/180,IDENTITY=[1,0,0,0,1,0,0,0,1];
const add=(a,b)=>a.map((v,i)=>v+b[i]),scale=(a,k)=>a.map(v=>v*k);
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));
export function transpose3(m){return [m[0],m[3],m[6],m[1],m[4],m[7],m[2],m[5],m[8]];}
export function multiply3(a,b){return Array.from({length:9},(_,i)=>{const r=Math.floor(i/3),c=i%3;return a[r*3]*b[c]+a[r*3+1]*b[c+3]+a[r*3+2]*b[c+6];});}
export function transform3(m,v){return [0,1,2].map(r=>m[3*r]*v[0]+m[3*r+1]*v[1]+m[3*r+2]*v[2]);}
export function determinant3(m){return m[0]*(m[4]*m[8]-m[5]*m[7])-m[1]*(m[3]*m[8]-m[5]*m[6])+m[2]*(m[3]*m[7]-m[4]*m[6]);}
export function axisMapping(axes){
  if(!Array.isArray(axes)||axes.length!==3||axes.some(n=>!Number.isInteger(n)||Math.abs(n)<1||Math.abs(n)>3)||new Set(axes.map(Math.abs)).size!==3)throw new Error('X、Y、Z 必须各使用一次，不能重复选轴。');
  const m=Array(9).fill(0);axes.forEach((n,r)=>m[r*3+Math.abs(n)-1]=Math.sign(n));return m;
}
export function eulerMatrix(roll,pitch,yaw){
  const [r,p,y]=[roll,pitch,yaw].map(v=>v*RAD),cr=Math.cos(r),sr=Math.sin(r),cp=Math.cos(p),sp=Math.sin(p),cy=Math.cos(y),sy=Math.sin(y);
  return [cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,-sp,cp*sr,cp*cr];
}
// 从原机体姿态先恢复传感器的世界朝向，再改变“传感器→预览机体”的对应关系。
export function remapAttitude(bodyToWorld,sensorToBody,previewSensorToBody){return multiply3(multiply3(bodyToWorld,sensorToBody),transpose3(previewSensorToBody));}
export function gimbalDirection(azimuth,elevation,zero=180){
  const a=(azimuth-zero)*RAD,t=(elevation-135)*RAD;
  return [Math.sin(t)*Math.cos(a),Math.sin(t)*Math.sin(a),Math.cos(t)];
}
// 与 GimbalControl.h 的静态 P 分量和几何选解一致；不冒充完整动态 PID 仿真。
export function staticGimbal(r,settings,current=180){
  const lean=[Math.atan2(-r[6],r[8])/RAD,Math.atan2(-r[7],r[8])/RAD];
  const vx=-lean[0]*settings.kp_x*settings.sign_x,vy=-lean[1]*settings.kp_y*settings.sign_y;
  const mag=Math.hypot(vx,vy),amount=Math.min(mag,settings.max_tilt_deg);
  if(mag<.15)return {azimuth:current,elevation:135,lean};
  const direction=Math.atan2(vy,vx)/RAD+settings.azimuth_zero_deg;
  let best=Infinity,azimuth=current,elevation=135;
  for(let flip=0;flip<2;flip++)for(let turn=-2;turn<=2;turn++){
    const candidate=direction+180*flip+360*turn,distance=Math.abs(candidate-current);
    if(candidate>=0&&candidate<=360&&distance<best){best=distance;azimuth=candidate;elevation=135+(flip?-amount:amount);}
  }
  return {azimuth,elevation,lean};
}
const validMatrix=m=>Array.isArray(m)&&m.length===9&&m.every(Number.isFinite);
const fmt=(n,d=1)=>Number.isFinite(n)?n.toFixed(d):'—';
const vector=v=>v.map(n=>fmt(n,3)).join(' / ');
const COLORS=['#ff8c87','#55ddbc','#7caeff'];

// SVG 三维投影，无 CDN、WebGL 或外部模型文件；适用于固件热点和离线单文件。
export function sceneSvg(r,azimuth,elevation,{camera=42,view='orbit',sensorMatrix=IDENTITY,zero=180,valid=true,mirrored=false}={}){
  const yaw=camera*RAD,pitch=(view==='top'?85:view==='side'?4:23)*RAD;
  const right=[Math.cos(yaw),Math.sin(yaw),0],depth=[-Math.sin(yaw)*Math.cos(pitch),Math.cos(yaw)*Math.cos(pitch),-Math.sin(pitch)],up=[-Math.sin(yaw)*Math.sin(pitch),Math.cos(yaw)*Math.sin(pitch),Math.cos(pitch)];
  const dot=(a,b)=>a.reduce((s,v,i)=>s+v*b[i],0);
  const project=p=>[430+dot(p,right)*135,282-dot(p,up)*135,dot(p,depth)];
  const points=ps=>ps.map(p=>project(p).slice(0,2).map(n=>n.toFixed(2)).join(',')).join(' ');
  const segment=(a,b,color,width=1,dash='')=>`<polyline points="${points([a,b])}" fill="none" stroke="${color}" stroke-width="${width}" ${dash?'stroke-dasharray="'+dash+'"':''}/>`;
  const text=(p,label,color,size=13)=>{const [x,y]=project(p);return `<text x="${x.toFixed(2)}" y="${y.toFixed(2)}" fill="${color}" font-size="${size}" font-family="system-ui,Microsoft YaHei" stroke="#0a1422" stroke-width="3" paint-order="stroke">${label}</text>`;};
  const arrow=(a,b,color,width=3)=>{const [x,y]=project(a),[u,v]=project(b),t=Math.atan2(v-y,u-x),d=10;
    return segment(a,b,color,width)+`<path d="M${u},${v} L${u-d*Math.cos(t-.4)},${v-d*Math.sin(t-.4)} L${u-d*Math.cos(t+.4)},${v-d*Math.sin(t+.4)} Z" fill="${color}"/>`;};
  let grid='';for(let i=-4;i<=4;i++){const n=i*.45;grid+=segment([-1.8,n,-1.2],[1.8,n,-1.2],'#24374a')+segment([n,-1.8,-1.2],[n,1.8,-1.2],'#24374a');}
  const origin=[-1.85,-.8,-1.2];let world='';
  for(let k=0;k<3;k++){const v=[0,0,0];v[k]=.52;world+=arrow(origin,add(origin,v),COLORS[k],2)+text(add(add(origin,v),[.04,0,.04]),`W+${'XYZ'[k]}`,COLORS[k],11);}
  const faces=[],body=p=>transform3(r,p);
  function face(ps,fill,opacity=1){const worldPs=ps.map(body);faces.push({z:worldPs.reduce((sum,p)=>sum+project(p)[2],0)/ps.length,markup:`<polygon points="${points(worldPs)}" fill="${fill}" fill-opacity="${opacity}" stroke="#26394e" stroke-width=".6"/>`});}
  const ring=(i,z,radius)=>[radius*Math.cos(i*Math.PI/8),radius*Math.sin(i*Math.PI/8),z];
  for(let i=0;i<16;i++){
    const shade=Math.round(145+38*Math.cos(i*Math.PI/8+.4));
    face([ring(i,-.85,.19),ring(i+1,-.85,.19),ring(i+1,1.05,.19),ring(i,1.05,.19)],`rgb(${shade},${shade+16},${shade+29})`);
    face([ring(i,1.05,.19),ring(i+1,1.05,.19),[0,0,1.58]],'#4cd2c1');
    face([ring(i,.32,.196),ring(i+1,.32,.196),ring(i+1,.51,.196),ring(i,.51,.196)],'#1d6a76');
    face([ring(i,-.85,.19),ring(i+1,-.85,.19),ring(i+1,-.96,.16),ring(i,-.96,.16)],'#293e52');
  }
  for(let i=0;i<4;i++){
    const t=i*Math.PI/2,cs=Math.cos(t),sn=Math.sin(t),fin=(radius,z)=>[radius*cs,radius*sn,z];
    face([fin(.18,-.22),fin(.5,-.85),fin(.18,-.85)],'#3e687f');
  }
  const dir=gimbalDirection(azimuth,elevation,zero),pivot=[0,0,-.99],rim=add(pivot,scale(dir,-.32));
  const normal=Math.hypot(dir[0],dir[1])>.01?[-dir[1],dir[0],0]:[1,0,0],norm=Math.hypot(...normal),u=scale(normal,1/norm),v=[dir[1]*u[2]-dir[2]*u[1],dir[2]*u[0]-dir[0]*u[2],dir[0]*u[1]-dir[1]*u[0]];
  const nozzle=(i,c,rad)=>add(c,add(scale(u,rad*Math.cos(i*Math.PI/6)),scale(v,rad*Math.sin(i*Math.PI/6))));
  for(let i=0;i<12;i++)face([nozzle(i,pivot,.08),nozzle(i+1,pivot,.08),nozzle(i+1,rim,.16),nozzle(i,rim,.16)],i%2?'#ba873d':'#e7b768');
  faces.sort((a,b)=>b.z-a.z);
  const showAxes=(transform,base,length,prefix)=>{
    let bits='';
    for(let k=0;k<3;k++){
      const axis=[0,0,0];axis[k]=prefix==='B'&&k===2?1.85:length;const end=add(base,transform3(transform,axis)),neg=add(base,scale(transform3(transform,axis),-.65));
      bits+=arrow(base,end,COLORS[k],prefix==='S'?1.8:2.5)+segment(base,neg,COLORS[k],1.5,'5 5')+text(end,`${prefix}+${'XYZ'[k]}`,COLORS[k],prefix==='S'?13:15)+text(neg,`${prefix}−${'XYZ'[k]}`,COLORS[k],12);
    }
    return bits;
  };
  let axes=showAxes(r,[0,0,0],1.1,'B');
  const sensorOrigin=body([.23,0,.73]),sensorWorld=multiply3(r,sensorMatrix);
  axes+=`<circle cx="${project(sensorOrigin)[0]}" cy="${project(sensorOrigin)[1]}" r="5" fill="#af8cff"/>`;
  // 将传感器朝向单独放在右侧，避免三个坐标系的正负标签叠在箭体上。
  axes+=`<g transform="translate(325,25)">${showAxes(sensorWorld,[0,0,0],.48,'S')}</g><text x="695" y="397" fill="#a9bad0" font-size="13" font-family="system-ui">传感器轴 S · 朝向示意</text>`;
  let thrust='';if(valid){const a=body(pivot),b=body(add(pivot,scale(dir,.85))),c=body(add(pivot,scale(dir,-.82)));
    const [bx,by]=project(b);
    thrust=arrow(a,b,'#ffd17c',4)+`<polyline points="${bx},${by} 650,165 698,165" fill="none" stroke="#ffd17c" stroke-width="1" opacity=".7"/><text x="705" y="170" fill="#ffd17c" font-size="14" font-family="system-ui">推力 +T（指令）</text>`+arrow(body(rim),c,'#ed9363',2)+text(c,'喷口 / 排气 −T','#ed9363',12);
  }
  const label=mirrored?'镜像坐标预览':valid?'箭头方向 = B+Z':'等待有效姿态 · 参考模型';
  return `<rect width="1000" height="530" fill="#0b1727"/>${grid}${world}<g opacity="${valid?1:.35}">${faces.map(f=>f.markup).join('')}${thrust}${axes}</g><text x="28" y="34" fill="#8da6bf" font-size="13" font-family="system-ui">${label}</text><text x="28" y="506" fill="#6d879e" font-size="12" font-family="system-ui">实线正方向 · 虚线负方向 · 拖动可旋转视角</text>`;
}

export function createAttitudePreview({get,settings,changed=()=>{}}){
  let lastPacket=null,lastFresh=false,custom=null,axes=[1,2,3],mappingNotice='',camera=42,drag=null,azimuth=180;
  const state={mode:'live'};
  const number=id=>Number(get(id).value);
  const output=(id,text)=>{get(id).textContent=text;};
  function render(packet=lastPacket,fresh=lastFresh){
    lastPacket=packet;lastFresh=fresh;
    const sim=state.mode==='simulation',w=packet?.world;
    const measured=!!(fresh&&w?.valid&&validMatrix(w.r_bw));
    const actualMount=validMatrix(packet?.control?.mount_matrix)?packet.control.mount_matrix:IDENTITY;
    const base=sim?eulerMatrix(number('sim-roll'),number('sim-pitch'),number('sim-yaw')):measured?w.r_bw:IDENTITY;
    const originalMount=sim?IDENTITY:actualMount,map=custom||originalMount;
    const valid=sim||measured,r=custom?remapAttitude(base,originalMount,map):base;
    const cfg=settings(),manual=get('sim-gimbal-mode').value==='manual';
    let az=180,el=135;
    if(sim){
      if(manual){az=number('sim-azimuth');el=number('sim-elevation');}
      else {const result=staticGimbal(r,cfg,azimuth);az=result.azimuth;el=result.elevation;azimuth=az;get('sim-azimuth').value=fmt(az);get('sim-elevation').value=fmt(el);}
    }else if(fresh&&packet?.outputs?.servo_us?.length===2){az=(packet.outputs.servo_us[0]-500)*360/2000;el=(packet.outputs.servo_us[1]-500)*270/2000;}
    for(const id of ['sim-roll','sim-pitch','sim-yaw','sim-gimbal-mode'])get(id).disabled=!sim;
    for(const id of ['sim-azimuth','sim-elevation'])get(id).disabled=!(sim&&manual);
    for(const id of ['sim-roll','sim-pitch','sim-yaw','sim-azimuth','sim-elevation'])output(id+'-value',`${fmt(number(id))}°`);
    get('preview-mode').value=state.mode;
    output('preview-status',sim?'本地模拟 · 不发送硬件命令':!packet?'等待设备':!fresh?'连接中断 · 姿态不可用':!w?.valid?'姿态未建立':`实测跟随 · IMU${packet.control.source_id}`);
    get('preview-status').className='badge '+(sim?'simulation-badge':valid?'live':'');
    output('preview-scope',sim?'当前图形来自本地模拟输入；真实硬件控制保持独立。':'箭体使用设备姿态；TVC 使用 PWM 角度指令，没有舵机位置反馈。');
    output('preview-model-note',sim?(manual?'手动几何预览 · 不模拟机械响应':'静态 P 指向试算 · 不含 I/D、运动限速、气动力或下降轨迹'):'指令几何预览 · 不表示实物已到位');
    output('preview-azimuth',valid?`${fmt(az)}°`:'—°');output('preview-elevation',valid?`${fmt(el)}°（偏转 ${fmt(el-135)}°）`:'—°');
    const dir=gimbalDirection(az,el,cfg.azimuth_zero_deg);
    output('preview-direction',valid?vector(dir):'— / — / —');
    output('preview-tilt',valid?`${fmt(Math.acos(clamp(r[8],-1,1))/RAD)}°`:'—°');
    output('axis-map-status',mappingNotice||(custom?(determinant3(map)>0?'自定义右手系 · 仅影响预览':'自定义左手系 / 镜像 · 仅影响预览'):'沿用设备安装坐标 · 仅预览'));
    get('axis-map-status').className='hint '+(mappingNotice?'axis-error':custom&&determinant3(map)<0?'axis-warning':'');
    const raw=sim?transform3(transpose3(originalMount),[base[6],base[7],base[8]]):fresh?packet?.control_imu?.a:null;
    output('axis-preview-accel',raw?vector(transform3(map,raw)):'— / — / —');
    output('axis-matrix',map.map(v=>Math.abs(v)<.0005?'0.000':v.toFixed(3)).reduce((rows,v,i)=>rows+v.padStart(7)+(i%3===2?'\n':' '),''));
    for(let k=0;k<3;k++)output('axis-correspondence-'+k,custom?`B${'XYZ'[k]} ← ${axes[k]>0?'+':'−'}S${'XYZ'[Math.abs(axes[k])-1]}`:`B${'XYZ'[k]} ← 设备安装矩阵第 ${k+1} 行`);
    get('attitude-scene').innerHTML=sceneSvg(r,az,el,{camera,view:get('preview-camera').value,sensorMatrix:map,zero:cfg.azimuth_zero_deg,valid,mirrored:determinant3(map)<0});
  }
  get('preview-mode').addEventListener('change',()=>{state.mode=get('preview-mode').value==='simulation'?'simulation':'live';render();changed();});
  for(const id of ['sim-roll','sim-pitch','sim-yaw','sim-gimbal-mode','sim-azimuth','sim-elevation','preview-camera'])get(id).addEventListener(id==='preview-camera'||id==='sim-gimbal-mode'?'change':'input',()=>render());
  get('axis-preview-form').addEventListener('submit',event=>{
    event.preventDefault();try{const next=[0,1,2].map(k=>number('map-axis-'+k));const matrix=axisMapping(next);axes=next;custom=matrix;mappingNotice='';}catch(error){mappingNotice=error.message;}render();
  });
  get('axis-reset').addEventListener('click',()=>{custom=null;axes=[1,2,3];mappingNotice='';axes.forEach((n,k)=>get('map-axis-'+k).value=String(n));render();});
  get('sim-upright').addEventListener('click',()=>{for(const id of ['sim-roll','sim-pitch','sim-yaw'])get(id).value=0;get('sim-azimuth').value=180;get('sim-elevation').value=135;azimuth=180;render();});
  const scene=get('attitude-scene');
  scene.addEventListener('pointerdown',event=>{drag={x:event.clientX,camera};scene.setPointerCapture?.(event.pointerId);});
  scene.addEventListener('pointermove',event=>{if(drag){camera=drag.camera+(event.clientX-drag.x)*.4;render();}});
  scene.addEventListener('pointerup',()=>{drag=null;});scene.addEventListener('pointercancel',()=>{drag=null;});
  // 单独的图形刷新没有硬件副作用；实测输入仍只来自主控制台传入的遥测包。
  return {render,get mode(){return state.mode;}};
}
