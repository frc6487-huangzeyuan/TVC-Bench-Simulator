"""验证参数编辑器到物理计算的链路；不覆盖现有实验、设置或源模型。"""
import json,queue,sys,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from pydantic import ValidationError
from fastapi.testclient import TestClient
from app import simulation as sim
from app.compiler import current_project,build_is_current
from app.server import app
from app import server


def main():
    checks=[]
    base=sim.Experiment(initial_x_deg=0,initial_y_deg=0,realtime=False)
    original=sim.parameter_preview(base)
    for name in sim.NAMES[1:]:
        cfg=base.model_dump();cfg['body_overrides']={name:{'cg_offset_mm':[20,-30,40]}}
        changed=sim.parameter_preview(sim.Experiment(**cfg))
        a=next(b for b in original['bodies'] if b['name']==name)
        b=next(b for b in changed['bodies'] if b['name']==name)
        np.testing.assert_allclose(np.array(b['com_body_mm'])-a['com_body_mm'],[20,-30,40],atol=1e-8)
    checks.append('all four body COM offsets follow zero-pose body XYZ, including trimmed TVC stages')

    cfg=base.model_dump();cfg['body_overrides']={'rocket_body':{'mass_kg':1.5}}
    changed=sim.parameter_preview(sim.Experiment(**cfg));a=original['bodies'][1];b=changed['bodies'][1]
    assert b['mass_kg']==1.5
    np.testing.assert_allclose(b['inertia_kg_m2'],np.array(a['inertia_kg_m2'])*1.5/a['mass_kg'])
    checks.append('absolute mass override rescales default principal inertia and aggregate mass')
    moments=[.02,.018,.012];cfg['body_overrides']['rocket_body']['inertia_kg_m2']=moments
    changed=sim.parameter_preview(sim.Experiment(**cfg));np.testing.assert_allclose(changed['bodies'][1]['inertia_kg_m2'],moments)
    checks.append('explicit principal inertia is independent of mass scaling')

    m,d,_=sim.make_model(base);matrix=np.zeros((m.nv,m.nv));mujoco.mj_fullM(m,d,matrix)
    for override in [{'inertia_kg_m2':moments},{'cg_offset_mm':[40,0,100]}]:
        cfg=base.model_dump();cfg['body_overrides']={'rocket_body':override}
        mm,dd,_=sim.make_model(sim.Experiment(**cfg));other=np.zeros_like(matrix);mujoco.mj_fullM(mm,dd,other)
        assert not np.allclose(matrix,other)
        assert np.min(np.linalg.eigvalsh(other))>0
    checks.append('inertia and COM edits change the positive-definite generalized mass matrix')

    bad=[{'body_overrides':{'rocket_body':{'inertia_kg_m2':[1,1,3]}}},
         {'body_overrides':{'base':{'mass_kg':1}}},
         {'thrust_curve':[{'rpm':0,'thrust_n':0},{'rpm':0,'thrust_n':1}]},
         {'thrust_curve':[{'rpm':0,'thrust_n':0},{'rpm':100,'thrust_n':2},{'rpm':200,'thrust_n':1}]},
         {'thrust_mode':'rpm_curve','max_rpm':30000},
         {'body_overrides':{'rocket_body':{'mass_kg':float('nan')}}}]
    for value in bad:
        try:sim.Experiment(**value)
        except ValidationError:continue
        raise AssertionError(f'invalid parameters accepted: {value}')
    checks.append('invalid inertia, body names, curve ordering, coverage and nonfinite values are rejected')

    build=current_project()['build'];assert build_is_current(build),'compile the current controller first'
    cfg=base.model_dump();cfg.update(duration_s=1,initial_x_deg=5,throttle_pct=50,thrust_mode='rpm_curve',max_rpm=20000,
        thrust_curve=[{'rpm':0,'thrust_n':0},{'rpm':10000,'thrust_n':2},{'rpm':20000,'thrust_n':8}])
    old_root=sim.ROOT
    with tempfile.TemporaryDirectory(prefix='tvc-parameters-') as temporary:
        # 仿真只把 runs 写入临时目录；模型和其他资源继续指向原仓库。
        root=Path(temporary);(root/'runs').mkdir();(root/'mjcf').symlink_to(old_root/'mjcf',target_is_directory=True)
        (root/'meshes').symlink_to(old_root/'meshes',target_is_directory=True)
        sim.ROOT=root
        try:
            histories=[]
            for i,extra in enumerate([{}, {'thrust_curve':[{'rpm':0,'thrust_n':0},{'rpm':10000,'thrust_n':4},{'rpm':20000,'thrust_n':16}]},
                                       {'body_overrides':{'rocket_body':{'mass_kg':1.5,'cg_offset_mm':[20,0,20]}}},
                                       {'body_overrides':{'rocket_body':{'inertia_kg_m2':moments}}},
                                       {'servo_speed_deg_s':5}]):
                params=sim.Experiment(**(cfg|extra));out=queue.Queue()
                sim.run_experiment(params.model_dump(),build,str(i),out,queue.Queue())
                packets=[];done=None
                while not out.empty():
                    packet=out.get()
                    if packet['kind']=='frame':packets.append(packet)
                    if packet['kind']=='done':done=packet
                assert packets and done['status']!='error',done
                for p in packets:np.testing.assert_allclose(p['thrust'],sim.thrust_at_rpm(params,p['rpm']),atol=1e-10)
                saved=json.loads((root/'runs'/str(i)/'run.json').read_text(encoding='utf-8'))
                assert saved['config']==params.model_dump(mode='json')
                histories.append(packets)
            assert histories[1][-1]['thrust']>histories[0][-1]['thrust']*1.9
            for packets in histories[2:]:
                assert not np.isclose(packets[-1]['x'],histories[0][-1]['x'],atol=1e-5)
        finally:sim.ROOT=old_root
    checks+=['RPM curve controls actual thrust and is recorded with RPM telemetry',
             'mass/COM, inertia and servo speed each change the simulated trajectory']

    with TestClient(app) as client:
        response=client.post('/api/parameters/preview',json=cfg);assert response.status_code==200,response.text
        assert len(response.json()['bodies'])==4
        assert client.post('/api/parameters/preview',json=base.model_dump()|bad[0]).status_code==422
        # 保存到临时工程状态；不覆盖正在使用的用户配置。
        with tempfile.TemporaryDirectory(prefix='tvc-settings-') as temporary:
            root=Path(temporary);(root/'work').mkdir()
            (root/'controllers').symlink_to(old_root/'controllers',target_is_directory=True)
            previous=server.ROOT;server.ROOT=root
            try:
                saved=client.post('/api/parameters/save',json=cfg);assert saved.status_code==200,saved.text
                restored=client.get('/api/bootstrap').json()['config']
                assert restored==sim.Experiment(**cfg).model_dump(mode='json')
            finally:server.ROOT=previous
    checks.append('preview API accepts valid data and rejects nonphysical inertia')
    checks.append('saved curves and body properties survive bootstrap reload without changing user settings')
    report={'passed':len(checks),'checks':checks,'mass_calibrated':False}
    print(json.dumps(report,ensure_ascii=False,indent=2))
    (old_root/'reports/parameter_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':main()
