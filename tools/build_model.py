"""把 CAD 提取结果归并成双轴台架 + 两轴 TVC 的 URDF/MJCF。

CAD 变换按行向量存储，转成列向量时必须转置；已用 SolidWorks MathPoint 实测核对。
零位由两条台架转轴及箭体长轴计算，保存姿态另存为 keyframe，便于回查。
"""
import csv,json,math,sys
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import trimesh

sys.stdout.reconfigure(encoding='utf-8')
ROOT=Path(__file__).resolve().parents[1]
D=json.loads((ROOT/'reports/cad_extract.json').read_text(encoding='utf-8'))
MODELS={m['id']:m for m in D['models']}
C={c['name']:c for c in D['components']}
W=np.array([[0,1,0],[0,0,-1],[-1,0,0]],float) # CAD -X 为台架竖直向上。

def transform(c):
    a=np.array(c['transform']); t=np.eye(4)
    t[:3,:3]=a[:9].reshape(3,3).T*a[12];t[:3,3]=a[9:12]
    return t
def mate(name,mid=None):
    records=D['mates'] if mid is None else MODELS[mid]['mates']
    return next(m for m in records if m['name']==name)
def axis_record(name,mid=None):
    v=np.array(mate(name,mid)['entities'][0]['params'],float)
    return v[:3],v[3:6]/np.linalg.norm(v[3:6])
def rot(axis,q):return Rotation.from_rotvec(np.asarray(axis)*q).as_matrix()
def nums(v):return ' '.join(f'{float(x):.12g}' for x in np.asarray(v).reshape(-1))
def add(p,tag,**attrs):return ET.SubElement(p,tag,{k:str(v) for k,v in attrs.items()})
def save_xml(root,path):
    ET.indent(root,space='  '); ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

# 两条同心配合轴的交点就是台架旋转中心；不用包围盒估计支点。
p1,a1=axis_record('同心2'); p2,a2=axis_record('同心5')
ts=np.linalg.lstsq(np.column_stack((a1,-a2)),p2-p1,rcond=None)[0]
pivot=(p1+a1*ts[0]+p2+a2*ts[1])/2
motor=transform(C['01-单轴电机组装-2'])
pa,aa=axis_record('同心3',12);pe,ae=axis_record('同心34',12)
pe=pe-ae*np.dot(pe-pa,ae)
az_cad=motor[:3,:3]@pa+motor[:3,3]
el_cad=motor[:3,:3]@pe+motor[:3,3]
az_axis=W@motor[:3,:3]@aa
el_axis=W@motor[:3,:3]@ae
nose=W@motor[:3,:3]@np.array([-1.,0,0])
axis1,axis2=W@a1,W@a2
base_cad=np.array([max((transform(c)@np.r_[v,1]) [0]
                           for c in D['components'] if c.get('type')==1
                           for v in np.array(MODELS[c['model_id']]['bounds'])),pivot[1],pivot[2]])
# 仅使用固定外框的真实顶点确定地面，不把倾斜箭体误判成底座。
floor_points=[]
for c in D['components']:
    if c.get('type')==1 and not c['name'].startswith(('01-','03-框架2','03-重力')) and ('型材' in c['name']):
        mesh=trimesh.load_mesh(ROOT/'meshes'/MODELS[c['model_id']]['mesh'])
        floor_points.append(trimesh.transform_points(mesh.vertices,transform(c)))
base_cad[0]=np.concatenate(floor_points)[:,0].max()
def world(p):return W@(np.asarray(p)-base_cad)

anchors={'base':np.zeros(3),'outer_frame':world(pivot),'rocket_body':world(pivot),
         'azimuth_stage':world(az_cad),'thrust_unit':world(el_cad)}
solution=least_squares(lambda q:(rot(axis1,q[0])@rot(axis2,q[1])@nose)[:2],[0.,0.],xtol=1e-14,ftol=1e-14,gtol=1e-14)
upright=solution.x
assert (rot(axis1,upright[0])@rot(axis2,upright[1])@nose)[2]>.999999
thrust_p,thrust_a=axis_record('同心25',12)
thrust_dir=W@motor[:3,:3]@thrust_a
tilt_correction=math.atan2(np.dot(el_axis,np.cross(thrust_dir,nose)),np.dot(thrust_dir,nose))
assert np.linalg.norm(rot(el_axis,tilt_correction)@thrust_dir-nose)<1e-6

def group(c):
    name=c['name']
    if name.startswith('03-框架2-1/') or name in ['03-75mm 10mm-1','03-75mm 10mm-2']:
        return 'outer_frame'
    if name.startswith('01-单轴电机组装-2/'):
        rest=name.split('/',1)[1]
        if any(rest.startswith(s) for s in ['01-X2212无刷电机-','01-螺旋桨-','01-无刷电机头-',
               '01-双轴舵机臂-','01-螺旋桨外壳-','main-horn-']):return 'thrust_unit'
        if '/______.step_1.STEP-' in rest:return 'thrust_unit'
        if any(rest.startswith(s) for s in ['01-旋转平台-','01-金属主舵盘-','01-M3 20尼龙套-',
                                           '01-45扭力PWM双轴舵机-']):return 'azimuth_stage'
        return 'rocket_body'
    if name.startswith(('03-重力模块','03-10mm光杆','02-连接件','05-电子部分','04-直齿条','04-舵机齿轮连接','06-火箭顶锥')):
        return 'rocket_body'
    return 'base'

parameter_path=ROOT/'mass_parameters.json'
if not parameter_path.exists():
    parameter_path.write_text(json.dumps({
      'petg_density_kg_m3':1270,'aluminium_density_kg_m3':2700,
      'printed_cad_volume_factor':1.0,
      'petg_model_ids':[16,18,23,25,26,27,37,41,42,43,44,45,46],
      'aluminium_model_ids':[0,1,7,10,11],
      'measured_part_mass_kg':{},
      'note':'PETG 和铝型材由用户确认；密度是标称值。打印结构按 CAD 实体体积计算，未校正填充。未列入材质的硬件保留原 CAD 质量，仍待称重。可按 model_id 在 measured_part_mass_kg 中填写实测质量。'
    },ensure_ascii=False,indent=2),encoding='utf-8')
params=json.loads(parameter_path.read_text(encoding='utf-8'))
meshes={k:[] for k in anchors}; items={k:[] for k in anchors}; manifest=[]
for c in D['components']:
    if c.get('type')!=1:continue
    m=MODELS[c['model_id']]; g=group(c);t=transform(c)
    mesh=trimesh.load_mesh(ROOT/'meshes'/m['mesh'],process=False)
    mesh.vertices=(mesh.vertices@t[:3,:3].T+t[:3,3]-base_cad)@W.T-anchors[g]
    meshes[g].append(mesh)
    raw=m['mass'];scale=1.;basis='CAD default; hardware mass unverified'
    if m['id'] in params['petg_model_ids']:
        scale=params['petg_density_kg_m3']/raw['density']*params['printed_cad_volume_factor'];basis='PETG nominal density; CAD solid volume'
    elif m['id'] in params['aluminium_model_ids']:
        scale=params['aluminium_density_kg_m3']/raw['density'];basis='aluminium nominal density'
    measured=params['measured_part_mass_kg'].get(str(m['id']))
    if measured is not None:
        assert measured>0;scale=measured/raw['mass'];basis='measured mass; CAD mass distribution'
    mass=raw['mass']*scale
    r=W@t[:3,:3]
    com=world(t[:3,:3]@np.array(raw['com'])+t[:3,3])-anchors[g]
    inertia=r@np.array(raw['inertia']).reshape(3,3)@r.T*scale
    items[g].append((mass,com,inertia))
    manifest.append({'component':c['name'],'model_id':m['id'],'source_path':m['path'],
                     'link':g,'mass_kg':mass,'mass_basis':basis,'raw_mesh':m['mesh']})

joints=[('stand_outer','base','outer_frame',axis1,float(upright[0]),'continuous'),
        ('stand_inner','outer_frame','rocket_body',axis2,float(upright[1]),'continuous'),
        ('tvc_azimuth','rocket_body','azimuth_stage',az_axis,0.,'revolute'),
        ('tvc_elevation','azimuth_stage','thrust_unit',el_axis,tilt_correction,'revolute')]
colors={'base':[.65,.68,.72,1],'outer_frame':[.42,.46,.52,1],
        'rocket_body':[.08,.43,.68,1],'azimuth_stage':[.12,.16,.22,1],'thrust_unit':[.9,.4,.09,1]}
robot=ET.Element('robot',name='tvc_bearing_bench')
robot.append(ET.Comment('SI units. Two passive bearing axes, two finite-range TVC axes. Hardware masses and servo torque not calibrated.'))
compiler=add(add(robot,'mujoco'),'compiler',discardvisual='false',fusestatic='false',balanceinertia='false')
inertials={}
for g in anchors:
    merged=trimesh.util.concatenate(meshes[g]);merged.export(ROOT/'meshes'/f'{g}.stl')
    total=sum(a[0] for a in items[g]);com=sum(a[0]*a[1] for a in items[g])/total
    tensor=np.zeros((3,3))
    for mass,point,i in items[g]:
        off=point-com;tensor+=i+mass*(np.dot(off,off)*np.eye(3)-np.outer(off,off))
    assert np.linalg.eigvalsh(tensor).min()>0
    inertials[g]={'mass_kg':total,'com_m':com.tolist(),'inertia_kg_m2':tensor.tolist(),
                  'component_count':len(items[g]),'triangle_count':len(merged.faces)}
    link=add(robot,'link',name=g);ine=add(link,'inertial')
    add(ine,'origin',xyz=nums(com),rpy='0 0 0');add(ine,'mass',value=f'{total:.12g}')
    add(ine,'inertia',**{key:f'{tensor[a,b]:.12g}' for key,(a,b) in
                       {'ixx':(0,0),'ixy':(0,1),'ixz':(0,2),'iyy':(1,1),'iyz':(1,2),'izz':(2,2)}.items()})
    visual=add(link,'visual');add(visual,'origin',xyz='0 0 0',rpy='0 0 0')
    add(add(visual,'geometry'),'mesh',filename=f'../meshes/{g}.stl',scale='1 1 1')
    add(add(visual,'material',name=g+'_material'),'color',rgba=nums(colors[g]))
for name,parent,child,axis,offset,kind in joints:
    j=add(robot,'joint',name=name,type=kind);add(j,'parent',link=parent);add(j,'child',link=child)
    add(j,'origin',xyz=nums(anchors[child]-anchors[parent]),rpy=nums(Rotation.from_matrix(rot(axis,offset)).as_euler('xyz')))
    add(j,'axis',xyz=nums(axis));add(j,'dynamics',damping='0',friction='0')
    if kind=='revolute':
        limit=math.pi if name=='tvc_azimuth' else math.radians(50)
        velocity=math.radians(54 if name=='tvc_azimuth' else 40.5)
        # 未提供扭矩实测值，effort=0 明确不宣称执行器动力学已标定。
        add(j,'limit',lower=f'{-limit:.12g}',upper=f'{limit:.12g}',effort='0',velocity=f'{velocity:.12g}')
urdf_path=ROOT/'urdf/tvc_bench.urdf';save_xml(robot,urdf_path)
model=mujoco.MjModel.from_xml_path(str(urdf_path))
mujoco.mj_saveLastXML(str(ROOT/'work/imported_urdf.xml'),model)
scene=ET.parse(ROOT/'work/imported_urdf.xml').getroot()
scene.set('model','TVC bearing bench - geometry validated, mass provisional')
comp=scene.find('compiler');comp.set('meshdir','../meshes')
for mesh in scene.findall('asset/mesh'):mesh.set('file',Path(mesh.get('file')).name)
# mj_saveLastXML 默认只保留有限有效数字。把精确的 CAD 参数写回，避免中间序列化损失精度。
for g,properties in inertials.items():
    b=next(b for b in scene.findall('.//body') if b.get('name')==g)
    ine=b.find('inertial');ine.attrib.clear();tensor=np.array(properties['inertia_kg_m2'])
    ine.set('pos',nums(properties['com_m']));ine.set('mass',f"{properties['mass_kg']:.12g}")
    ine.set('fullinertia',nums([tensor[0,0],tensor[1,1],tensor[2,2],tensor[0,1],tensor[0,2],tensor[1,2]]))
for name,parent,child,axis,offset,kind in joints:
    b=next(b for b in scene.findall('.//body') if b.get('name')==child)
    b.set('pos',nums(anchors[child]-anchors[parent]))
    b.set('quat',nums(Rotation.from_matrix(rot(axis,offset)).as_quat()[[3,0,1,2]]))
    j=b.find('joint');j.set('axis',nums(axis))
    if kind=='revolute':
        limit=math.pi if name=='tvc_azimuth' else math.radians(50)
        j.set('range',nums([-limit,limit]))
add(scene,'option',timestep='0.001',gravity='0 0 -9.80665',integrator='implicitfast')
visual=add(scene,'visual');add(visual,'global',offwidth='1600',offheight='1200')
stat=scene.find('statistic')
if stat is None:stat=add(scene,'statistic')
stat.set('center',nums(world(pivot)));stat.set('extent','.7')
wb=scene.find('worldbody');add(wb,'light',pos='1 -1 2',dir='-1 1 -2',diffuse='.8 .8 .8')
add(wb,'geom',name='floor',type='plane',size='1 1 .02',rgba='.88 .9 .92 1',contype='0',conaffinity='0')
for geom in scene.findall('.//geom'):
    geom.set('contype','0');geom.set('conaffinity','0')
    if geom.get('name')!='floor':geom.set('group','1')
thrust_body=next(b for b in scene.findall('.//body') if b.get('name')=='thrust_unit')
add(thrust_body,'site',name='thrust_point',pos=nums(world(motor[:3,:3]@thrust_p+motor[:3,3])-anchors['thrust_unit']),size='.003',rgba='1 .15 .1 1')
body=next(b for b in scene.findall('.//body') if b.get('name')=='rocket_body')
# 机体系 Z 沿箭头；IMU 实际安装外参仍由现有固件配置给定。
body_z=nose;body_x=az_axis.copy() # 后面用独立的横向参考构建正交坐标。
body_x=W@motor[:3,:3]@np.array([0.,0.,1.])
body_y=np.cross(body_z,body_x);brot=np.column_stack((body_x,body_y,body_z))
quat=Rotation.from_matrix(brot).as_quat()[[3,0,1,2]]
add(body,'site',name='body_reference',pos='0 0 0',quat=nums(quat),size='.003',rgba='.1 1 .1 1')
sensors=add(scene,'sensor');add(sensors,'gyro',name='ideal_gyro',site='body_reference');add(sensors,'accelerometer',name='ideal_accel',site='body_reference')
keyframes=add(scene,'keyframe');add(keyframes,'key',name='upright',qpos='0 0 0 0')
cad_q=[-float(upright[0]),-float(upright[1]),0.,-tilt_correction]
add(keyframes,'key',name='cad_saved_pose',qpos=nums(cad_q))
(ROOT/'mjcf').mkdir(exist_ok=True);save_xml(scene,ROOT/'mjcf/tvc_bench.xml')
meta={'source':D['source'],'coordinate_mapping':'world=(CAD_Y,-CAD_Z,-CAD_X), ground at bottom extrusion',
      'cad_origin_m':base_cad.tolist(),'pivot_cad_m':pivot.tolist(),'pivot_world_m':world(pivot).tolist(),
      'joint_order':[j[0] for j in joints],'cad_saved_pose_qpos':cad_q,
      'upright_corrections_rad':[float(upright[0]),float(upright[1]),0.,tilt_correction],
      'thrust_direction_in_link':thrust_dir.tolist(),'body_z_in_link':nose.tolist(),
      'link_anchor_world_m':{k:v.tolist() for k,v in anchors.items()},'inertials':inertials,
      'mass_calibrated':False,'servo_torque_calibrated':False,'self_collision_enabled':False,
      'balance_carriage':'fixed at CAD saved position; outside two-axis stand scope'}
(ROOT/'reports/model_metadata.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
(ROOT/'reports/component_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
with (ROOT/'reports/component_manifest.csv').open('w',encoding='utf-8-sig',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(manifest[0]));writer.writeheader();writer.writerows(manifest)
print('URDF',urdf_path,'links',len(anchors),'joints',len(joints),'parts',len(manifest))
print('MJCF',ROOT/'mjcf/tvc_bench.xml')
print('UPRIGHT CORRECTIONS degrees',np.degrees(meta['upright_corrections_rad']))
print('LINKS',json.dumps(inertials,ensure_ascii=False))
