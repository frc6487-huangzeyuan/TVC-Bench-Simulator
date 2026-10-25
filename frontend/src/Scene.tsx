import {useEffect,useRef,useState} from 'react';
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import {RotateCcw,Scan,Grid2X2,Move3D,LoaderCircle} from 'lucide-react';
import type {Frame,Pose} from './types';

const colors:Record<string,number>={base:0x76818a,outer_frame:0x4a555e,rocket_body:0x238b94,azimuth_stage:0x29323c,thrust_unit:0xc27d50};
export default function Scene({frame}:{frame:Frame|null}){
  const host=useRef<HTMLDivElement>(null),latest=useRef(frame),reset=useRef(()=>{}),gridToggle=useRef(()=>{}),wireToggle=useRef(()=>{});
  const [loading,setLoading]=useState(true),[error,setError]=useState(''),[grid,setGrid]=useState(true),[wire,setWire]=useState(false);
  latest.current=frame;
  useEffect(()=>{
    if(!host.current)return;
    const el=host.current;let disposed=false,animation=0;
    let renderer:THREE.WebGLRenderer;
    try{renderer=new THREE.WebGLRenderer({antialias:true,alpha:true,powerPreference:'high-performance'});}catch{setError('三维渲染不可用，请检查浏览器 WebGL 支持。');setLoading(false);return;}
    renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
    renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.25;el.appendChild(renderer.domElement);
    const scene=new THREE.Scene();scene.fog=new THREE.FogExp2(0x171d23,.23);
    const camera=new THREE.PerspectiveCamera(36,1,.005,20);camera.up.set(0,0,1);
    const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.minDistance=.4;controls.maxDistance=3;
    controls.maxPolarAngle=Math.PI*.89;controls.target.set(0,0,.27);
    const home=()=>{camera.position.set(.9,-1.12,.9);controls.target.set(0,0,.26);controls.update();};home();reset.current=home;
    scene.add(new THREE.HemisphereLight(0xbfd8eb,0x222b30,3));
    const key=new THREE.DirectionalLight(0xffe6d0,3.5);key.position.set(1,-1,2);key.castShadow=true;
    key.shadow.mapSize.set(2048,2048);Object.assign(key.shadow.camera,{left:-.8,right:.8,top:.8,bottom:-.8,near:.1,far:5});key.shadow.bias=-.0003;scene.add(key);
    const rim=new THREE.DirectionalLight(0x7bc7d7,2);rim.position.set(-1,.5,1);scene.add(rim);
    const floor=new THREE.Mesh(new THREE.PlaneGeometry(200,200),new THREE.MeshStandardMaterial({color:0x1a2229,roughness:.95,metalness:.05}));
    floor.position.z=-.002;floor.receiveShadow=true;scene.add(floor);
    const gridHelper=new THREE.GridHelper(4,80,0x50616d,0x2b3944);gridHelper.rotation.x=Math.PI/2;gridHelper.position.z=-.001;
    (gridHelper.material as THREE.Material).transparent=true;(gridHelper.material as THREE.Material).opacity=.24;scene.add(gridHelper);gridToggle.current=()=>gridHelper.visible=!gridHelper.visible;
    const meshes:Record<string,THREE.Mesh>={};const loader=new STLLoader();let count=0;
    Object.entries(colors).forEach(([name,color])=>loader.load(`/meshes/${name}.stl`,geometry=>{
      if(disposed){geometry.dispose();return;}
      geometry.computeVertexNormals();const material=new THREE.MeshStandardMaterial({color,roughness:name==='rocket_body'?.4:.48,metalness:name==='rocket_body'?.12:.55});
      const mesh=new THREE.Mesh(geometry,material);mesh.castShadow=true;mesh.receiveShadow=true;mesh.visible=false;meshes[name]=mesh;scene.add(mesh);
      if(++count===5)setLoading(false);
    },undefined,()=>{if(!disposed){setError('模型网格读取失败，请检查本地仿真服务。');setLoading(false);}}));
    wireToggle.current=()=>Object.values(meshes).forEach(m=>{(m.material as THREE.MeshStandardMaterial).wireframe=!(m.material as THREE.MeshStandardMaterial).wireframe;});
    const arrow=new THREE.ArrowHelper(new THREE.Vector3(0,0,1),new THREE.Vector3(0,0,.15),.1,0xe4a26e,.015,.008);scene.add(arrow);arrow.visible=false;
    const resize=new ResizeObserver(()=>{const w=el.clientWidth,h=el.clientHeight;renderer.setSize(w,h);camera.aspect=w/Math.max(h,1);camera.updateProjectionMatrix();});resize.observe(el);
    const draw=()=>{
      if(disposed)return;
      const f=latest.current;
      f?.pose.forEach((p:Pose)=>{const m=meshes[p.name];if(m){m.position.fromArray(p.position);m.quaternion.set(p.quaternion[1],p.quaternion[2],p.quaternion[3],p.quaternion[0]);m.visible=true;}});
      arrow.visible=!!f&&f.thrust>.03;
      if(f?.thrust_origin&&f?.thrust_direction){
        const dir=new THREE.Vector3().fromArray(f.thrust_direction).normalize();
        arrow.position.fromArray(f.thrust_origin);
        arrow.setDirection(dir);arrow.setLength(.035+Math.min(f.thrust/12,.8)*.16,.018,.009);
      }
      controls.update();renderer.render(scene,camera);animation=requestAnimationFrame(draw);
    };draw();
    return()=>{disposed=true;cancelAnimationFrame(animation);resize.disconnect();controls.dispose();scene.traverse(o=>{if(o instanceof THREE.Mesh){o.geometry.dispose();(Array.isArray(o.material)?o.material:[o.material]).forEach(m=>m.dispose());}});renderer.dispose();renderer.domElement.remove();};
  },[]);
  return <div className="scene-wrap"><div className="scene-canvas" ref={host}/>
    <div className="scene-label"><span className="live-dot"/>物理场景<span className="scene-label-muted">MUJOCO · 1 kHz</span></div>
    <div className="scene-tools"><button title="恢复视角" onClick={()=>reset.current()}><RotateCcw size={16}/></button><button title="网格" className={grid?'selected':''} onClick={()=>{gridToggle.current();setGrid(!grid);}}><Grid2X2 size={16}/></button><button title="线框" className={wire?'selected':''} onClick={()=>{wireToggle.current();setWire(!wire);}}><Scan size={16}/></button></div>
    <div className="scene-bottom"><span><Move3D size={14}/>拖动旋转 · 滚轮缩放</span><span>5 个刚体 <i/> 4 个关节 <i/> SI 单位</span></div>
    {loading&&<div className="scene-loading"><LoaderCircle className="spin" size={25}/><span>载入你的台架模型</span></div>}
    {error&&<div className="scene-loading error">{error}</div>}
    <div className="axis-key"><b className="axis-x">X</b><b className="axis-y">Y</b><b className="axis-z">Z</b></div>
  </div>;
}
