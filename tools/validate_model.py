"""核对 CAD 保存姿态、URDF/MJCF 一致性、惯量和关节，并渲染预览。"""
import json,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

sys.stdout.reconfigure(encoding='utf-8')
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from app.model_loader import load_model
meta=json.loads((ROOT/'reports/model_metadata.json').read_text(encoding='utf-8'))
urdf=ROOT/'urdf/tvc_bench.urdf'; mjcf=ROOT/'mjcf/tvc_bench.xml'
um=load_model(urdf); mm=load_model(mjcf)
ud=mujoco.MjData(um);md=mujoco.MjData(mm)
assert um.nq==mm.nq==5 and mm.nv==5
assert [mm.joint(i).name for i in range(mm.njnt)]==meta['joint_order']
assert len(ET.parse(urdf).findall('link'))==6
assert len(ET.parse(urdf).findall("joint[@type='continuous']"))==2
assert len(ET.parse(urdf).findall("joint[@type='revolute']"))==2
assert len(ET.parse(urdf).findall("joint[@type='prismatic']"))==1
max_pose_error=0.
for q in [np.zeros(5),meta['cad_saved_pose_qpos'],[.13,-.17,.7,.2,.02],[-.2,.1,-1.3,-.3,-.02]]:
    ud.qpos[:]=q;md.qpos[:]=q;mujoco.mj_forward(um,ud);mujoco.mj_forward(mm,md)
    for name in meta['inertials']:
        ui=um.body(name).id;mi=mm.body(name).id
        e=max(np.max(abs(ud.xpos[ui]-md.xpos[mi])),np.max(abs(ud.xmat[ui]-md.xmat[mi])))
        max_pose_error=max(max_pose_error,float(e));assert e<1e-8,(name,e)

# 所有关键帧都必须重建真实 CAD 的保存位置，而不只是“看起来差不多”。
md.qpos[:]=meta['cad_saved_pose_qpos'];mujoco.mj_forward(mm,md)
cad_error=0.
for name,anchor in meta['link_anchor_world_m'].items():
    i=mm.body(name).id
    e=max(np.max(abs(md.xpos[i]-anchor)),np.max(abs(md.xmat[i].reshape(3,3)-np.eye(3))))
    cad_error=max(cad_error,float(e));assert e<1e-8,(name,e)
for name,properties in meta['inertials'].items():
    i=np.array(properties['inertia_kg_m2']);eig=np.linalg.eigvalsh(i)
    assert eig.min()>0 and eig.max()<=sum(eig)-eig.max()+1e-10
    assert properties['mass_kg']>0

md.qpos[:]=0;mujoco.mj_forward(mm,md)
body_site=mm.site('body_reference').id
upright_error=float(np.linalg.norm(md.site_xmat[body_site].reshape(3,3)[:,2]-[0,0,1]))
assert upright_error<1e-8,upright_error
thrust=mm.body('thrust_unit').id
dir_world=md.xmat[thrust].reshape(3,3)@meta['thrust_direction_in_link']
assert np.linalg.norm(dir_world-[0,0,1])<1e-8

# 单独转动每个关节，检查父刚体不动、子刚体绕指定轴运动。
for j,child in enumerate(['outer_frame','rocket_body','azimuth_stage','thrust_unit']):
    md.qpos[:]=0;mujoco.mj_forward(mm,md)
    parent=mm.body(child).parentid[0]
    before=md.xmat[parent].copy();child_before=md.xmat[mm.body(child).id].copy()
    md.qpos[j]=.1;mujoco.mj_forward(mm,md)
    assert np.max(abs(md.xmat[parent]-before))<1e-10
    assert np.linalg.norm(md.xmat[mm.body(child).id]-child_before)>.1

# 滑架位移只沿机体 +X；固定齿条合在箭体上，滑架位移不改变其位姿。
md.qpos[:]=0;mujoco.mj_forward(mm,md)
rocket=mm.body('rocket_body').id;carriage=mm.body('balance_carriage').id
before=md.xpos[carriage].copy();body_before=md.xpos[rocket].copy()
com_before=md.subtree_com[rocket].copy()
direction=md.site_xmat[body_site].reshape(3,3)[:,0].copy()
md.qpos[mm.joint('cg_slide').qposadr[0]]=.02;mujoco.mj_forward(mm,md)
assert np.allclose(md.xpos[carriage]-before,.02*direction,atol=1e-9)
assert np.allclose(md.xpos[rocket],body_before,atol=1e-9)
expected=.02*mm.body_mass[carriage]/mm.body_subtreemass[rocket]*direction
assert np.allclose(md.subtree_com[rocket]-com_before,expected,atol=1e-9)
mujoco.mj_resetData(mm,md);md.qpos[:2]=[.03,-.02]
for _ in range(1000):mujoco.mj_step(mm,md)
assert np.isfinite(md.qpos).all() and np.isfinite(md.qvel).all()
report={'urdf_load':True,'mjcf_load':True,'link_count':6,'joint_count':5,
        'passive_bearing_axes':2,'finite_tvc_axes':2,'balance_slide':True,'cad_leaf_instances':80,
        'cad_saved_pose_max_error':cad_error,'urdf_mjcf_max_pose_error':max_pose_error,
        'upright_axis_error':upright_error,'independent_joint_motion':True,
        'one_second_gravity_integration_finite':True,
        'mass_calibrated':False,'flight_controller_connected':False}
(ROOT/'reports/validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')

renderer=mujoco.Renderer(mm,height=1000,width=1400)
cam=mujoco.MjvCamera();cam.type=mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:]=[0,0,.23];cam.distance=1.05;cam.azimuth=135;cam.elevation=-23
option=mujoco.MjvOption();option.geomgroup[:]=1
from PIL import Image
for name,q in [('upright',np.zeros(5)),('cad_saved_pose',meta['cad_saved_pose_qpos']),
               ('joint_motion',np.r_[np.radians([12,-10,30,15]),.02])]:
    md.qpos[:]=q;mujoco.mj_forward(mm,md)
    renderer.update_scene(md,camera=cam,scene_option=option)
    Image.fromarray(renderer.render()).save(ROOT/'reports'/f'{name}.png')
renderer.close()
print(json.dumps(report,ensure_ascii=False,indent=2))
