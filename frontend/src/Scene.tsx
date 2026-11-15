import {useEffect,useRef,useState} from 'react';
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {Crosshair,RotateCcw,Scan,Grid2X2,Move3D,MoveHorizontal,LoaderCircle} from 'lucide-react';
import type {Frame,Pose,BodyPreview,Triple} from './types';

const colors:Record<string,number>={base:0x76818a,outer_frame:0x4a555e,rocket_body:0x238b94,azimuth_stage:0x29323c,thrust_unit:0xc27d50,balance_carriage:0xe0ac50};
type PropellerGeometry={pivot_m:Triple;shaft_axis:Triple;elevation_axis:Triple};
type Overlay={body:BodyPreview;totalCom:Triple;showInertia:boolean};
export default function Scene({frame,overlay}:{frame:(Pick<Frame,'pose'|'thrust'>&Partial<Frame>)|null;overlay?:Overlay}){
  const host=useRef<HTMLDivElement>(null),latest=useRef(frame),reset=useRef(()=>{}),focusElevation=useRef(()=>{}),
    focusBalance=useRef(()=>{}),showBalance=useRef(false),gridToggle=useRef(()=>{}),wireToggle=useRef(()=>{});
  const [loading,setLoading]=useState(true),[error,setError]=useState(''),[grid,setGrid]=useState(true),[wire,setWire]=useState(false);
  latest.current=frame;
  const overlayRef=useRef(overlay);overlayRef.current=overlay;
  useEffect(()=>{
    if(!host.current)return;
    const el=host.current;let disposed=false,animation=0;
    let renderer:THREE.WebGLRenderer;
    try{renderer=new THREE.WebGLRenderer({antialias:true,alpha:true,powerPreference:'high-performance'});}catch{setError('三维渲染不可用，请检查浏览器 WebGL 支持。');setLoading(false);return;}
    renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
    renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.25;el.appendChild(renderer.domElement);
    const scene=new THREE.Scene();scene.fog=new THREE.FogExp2(0x171d23,.23);
    const camera=new THREE.PerspectiveCamera(36,1,.005,20);camera.up.set(0,0,1);
    const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.minDistance=.18;controls.maxDistance=3;
    controls.maxPolarAngle=Math.PI*.89;controls.target.set(0,0,.27);
    const home=()=>{showBalance.current=false;camera.position.set(.9,-1.12,.9);controls.target.set(0,0,.26);controls.update();};home();reset.current=home;
    scene.add(new THREE.HemisphereLight(0xbfd8eb,0x222b30,3));
    const key=new THREE.DirectionalLight(0xffe6d0,3.5);key.position.set(1,-1,2);key.castShadow=true;
    key.shadow.mapSize.set(2048,2048);Object.assign(key.shadow.camera,{left:-.8,right:.8,top:.8,bottom:-.8,near:.1,far:5});key.shadow.bias=-.0003;scene.add(key);
    const rim=new THREE.DirectionalLight(0x7bc7d7,2);rim.position.set(-1,.5,1);scene.add(rim);
    const floor=new THREE.Mesh(new THREE.PlaneGeometry(200,200),new THREE.MeshStandardMaterial({color:0x1a2229,roughness:.95,metalness:.05}));
    floor.position.z=-.002;floor.receiveShadow=true;scene.add(floor);
    const gridHelper=new THREE.GridHelper(4,80,0x50616d,0x2b3944);gridHelper.rotation.x=Math.PI/2;gridHelper.position.z=-.001;
    (gridHelper.material as THREE.Material).transparent=true;(gridHelper.material as THREE.Material).opacity=.24;scene.add(gridHelper);gridToggle.current=()=>gridHelper.visible=!gridHelper.visible;
    const meshes:THREE.Mesh[]=[],bodies:Record<string,THREE.Group>={};
    const loader=new STLLoader();let count=0,shell:THREE.Mesh<THREE.BufferGeometry,THREE.MeshStandardMaterial>|undefined;
    const loaded=()=>{if(++count===Object.keys(colors).length+2)setLoading(false);};
    const loadingError=()=>{if(!disposed){setError('模型网格读取失败，请检查本地仿真服务。');setLoading(false);}};
    Object.entries(colors).forEach(([name,color])=>{
      const body=new THREE.Group();body.visible=false;bodies[name]=body;scene.add(body);
      const meshName=name==='thrust_unit'?'thrust_housing':name==='rocket_body'?'rocket_display':name;
      loader.load(`/meshes/${meshName}.stl`,geometry=>{
        if(disposed){geometry.dispose();return;}
        geometry.computeVertexNormals();
        const material=new THREE.MeshStandardMaterial({color,roughness:name==='rocket_body'?.4:.48,metalness:name==='rocket_body'?.12:.55});
        const mesh=new THREE.Mesh(geometry,material);mesh.castShadow=true;mesh.receiveShadow=true;
        if(name==='rocket_body')shell=mesh;
        meshes.push(mesh);body.add(mesh);loaded();
      },undefined,loadingError);
    });
    loader.load('/meshes/balance_rack.stl',geometry=>{
      if(disposed){geometry.dispose();return;}
      geometry.computeVertexNormals();
      const rack=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:0x9caec0,roughness:.48,metalness:.4}));
      meshes.push(rack);bodies.rocket_body.add(rack);loaded();
    },undefined,loadingError);
    const rotor=new THREE.Group(),shaft=new THREE.Vector3(),hinge=new THREE.Vector3();
    const thrustBody=bodies.thrust_unit;
    thrustBody.add(rotor);
    fetch('/meshes/propeller_scene.json').then(response=>{
      if(!response.ok)throw Error('propeller geometry missing');
      return response.json() as Promise<PropellerGeometry>;
    }).then(info=>loader.loadAsync('/meshes/propeller.stl').then(geometry=>{
      if(disposed){geometry.dispose();return;}
      geometry.computeVertexNormals();
      const blade=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:0xbde6e7,roughness:.38,metalness:.12}));
      rotor.position.fromArray(info.pivot_m);
      shaft.fromArray(info.shaft_axis).normalize();hinge.fromArray(info.elevation_axis).normalize();
      blade.castShadow=true;meshes.push(blade);rotor.add(blade);loaded();
    })).catch(loadingError);
    focusElevation.current=()=>{
      showBalance.current=false;
      const shaftWorld=shaft.clone().applyQuaternion(thrustBody.quaternion);
      const hingeWorld=hinge.clone().applyQuaternion(thrustBody.quaternion);
      controls.target.copy(thrustBody.position).lerp(rotor.getWorldPosition(new THREE.Vector3()),.55);
      camera.position.copy(controls.target).addScaledVector(hingeWorld,.35).addScaledVector(shaftWorld,-.12);
      controls.update();
    };
    focusBalance.current=()=>{
      const carriage=bodies.balance_carriage;
      if(!carriage.visible)return;
      showBalance.current=true;
      new THREE.Box3().setFromObject(carriage).getCenter(controls.target);
      camera.position.copy(controls.target).add(new THREE.Vector3(.22,-.37,.26));
      controls.update();
    };
    wireToggle.current=()=>meshes.forEach(m=>{(m.material as THREE.MeshStandardMaterial).wireframe=!(m.material as THREE.MeshStandardMaterial).wireframe;});
    const arrow=new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(0,0,.15),.1,0xe4a26e,.015,.008);scene.add(arrow);arrow.visible=false;
    const center=new THREE.Mesh(new THREE.SphereGeometry(.008,18,12),new THREE.MeshBasicMaterial({color:0xe1ac76,depthTest:false}));center.renderOrder=10;scene.add(center);
    const total=new THREE.Mesh(new THREE.SphereGeometry(.006,16,12),new THREE.MeshBasicMaterial({color:0x79c6c5,depthTest:false}));total.renderOrder=11;scene.add(total);
    const ellipsoid=new THREE.Mesh(new THREE.SphereGeometry(1,24,16),new THREE.MeshBasicMaterial({color:0xe1ac76,transparent:true,opacity:.18,wireframe:true,depthTest:false}));ellipsoid.renderOrder=9;scene.add(ellipsoid);
    const cgAxes=new THREE.AxesHelper(.07);scene.add(cgAxes);
    const resize=new ResizeObserver(()=>{const w=el.clientWidth,h=el.clientHeight;renderer.setSize(w,h);camera.aspect=w/Math.max(h,1);camera.updateProjectionMatrix();});resize.observe(el);
    let run:string|undefined,fromTurns=0,toTurns=0,shownTurns=0,receivedAt=0;
    const draw=(now:number)=>{
      if(disposed)return;
      const f=latest.current;
      f?.pose.forEach((p:Pose)=>{const body=bodies[p.name];if(body){body.position.fromArray(p.position);body.quaternion.set(p.quaternion[1],p.quaternion[2],p.quaternion[3],p.quaternion[0]);body.visible=true;}});
      const turns=f?.propeller_turns??0;
      if(f?.status!=='running'||run!==f.run_id||turns<toTurns){
        shownTurns=fromTurns=toTurns=turns;run=f?.run_id;
      }else{
        if(turns!==toTurns){fromTurns=shownTurns;toTurns=turns;receivedAt=now;}
        shownTurns=THREE.MathUtils.lerp(fromTurns,toTurns,Math.min(1,(now-receivedAt)/40));
      }
      if(shaft.lengthSq()>0)rotor.quaternion.setFromAxisAngle(shaft,shownTurns*2*Math.PI);
      arrow.visible=!!f&&f.thrust>.03;
      if(f?.thrust_origin&&f?.thrust_direction){
        const dir=new THREE.Vector3().fromArray(f.thrust_direction).normalize();
        arrow.position.fromArray(f.thrust_origin);
        arrow.setDirection(dir);arrow.setLength(.035+Math.min(f.thrust/12,.8)*.16,.018,.009);
      }
      const o=overlayRef.current;center.visible=cgAxes.visible=!!o;ellipsoid.visible=!!o?.showInertia;
      const displayedCom=showBalance.current?f?.balance?.com_world:undefined;
      if(shell)shell.visible=!showBalance.current;
      total.visible=!!o||!!displayedCom;
      if(displayedCom)total.position.fromArray(displayedCom);
      if(o){
        center.position.fromArray(o.body.com_world);total.position.fromArray(o.totalCom);cgAxes.position.copy(center.position);
        ellipsoid.position.copy(center.position);const q=o.body.inertia_quaternion;ellipsoid.quaternion.set(q[1],q[2],q[3],q[0]);
        const moments=o.body.inertia_kg_m2,scale=moments.map((v,i)=>Math.sqrt(Math.max(1e-8,2.5*(moments[(i+1)%3]+moments[(i+2)%3]-v)/o.body.mass_kg)));
        ellipsoid.scale.fromArray(scale);
      }
      controls.update();renderer.render(scene,camera);animation=requestAnimationFrame(draw);
    };draw(performance.now());
    return()=>{disposed=true;cancelAnimationFrame(animation);resize.disconnect();controls.dispose();scene.traverse(o=>{if(o instanceof THREE.Mesh){o.geometry.dispose();(Array.isArray(o.material)?o.material:[o.material]).forEach(m=>m.dispose());}});renderer.dispose();renderer.domElement.remove();};
  },[]);
  return <div className="scene-wrap"><div className="scene-canvas" ref={host}/>
    <div className="scene-label"><span className="live-dot"/>{overlay?'质量分布':'物理场景'}<span className="scene-label-muted">{overlay?'COM · INERTIA':'MUJOCO · 1 kHz'}</span></div>
    <div className="scene-tools"><button title="恢复视角" onClick={()=>reset.current()}><RotateCcw size={16}/></button><button title="俯仰特写" aria-label="俯仰特写" disabled={loading||!!error} onClick={()=>focusElevation.current()}><Crosshair size={16}/></button>
      {!overlay&&<button title="重心滑架特写（隐藏箭体外壳）" aria-label="重心滑架特写" disabled={loading||!!error||!frame?.balance} onClick={()=>focusBalance.current()}><MoveHorizontal size={16}/></button>}
      <button title="网格" className={grid?'selected':''} onClick={()=>{gridToggle.current();setGrid(!grid);}}><Grid2X2 size={16}/></button><button title="线框" className={wire?'selected':''} onClick={()=>{wireToggle.current();setWire(!wire);}}><Scan size={16}/></button></div>
    {!overlay&&frame?.servo_deg&&frame.pwm&&<div className="scene-servo-readout" aria-label="俯仰关节角度">
      <span>俯仰实际 <b>{(frame.servo_deg[1]-135).toFixed(1)}°</b></span>
      <span>指令 <b>{((frame.pwm[1]-1500)*270/2000).toFixed(1)}°</b></span>
    </div>}
    <div className="scene-bottom"><span><Move3D size={14}/>拖动旋转 · 滚轮缩放</span><span>6 个刚体 <i/> 5 个关节 <i/> SI 单位</span></div>
    {loading&&<div className="scene-loading"><LoaderCircle className="spin" size={25}/><span>载入你的台架模型</span></div>}
    {error&&<div className="scene-loading error">{error}</div>}
    <div className="axis-key"><b className="axis-x">X</b><b className="axis-y">Y</b><b className="axis-z">Z</b></div>
  </div>;
}
