"""对本次实际接入做集成验证，不连接硬件，不修改原飞控源码。"""
import ctypes as ct,json,queue,sys,time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from app.compiler import ROOT,current_project,build_project,build_is_current,save_state,DEFAULT_PROJECT
from app.simulation import Experiment,make_model,lean,run_experiment,SimulationManager
import httpx

def main():
    checks=[];initial=current_project()
    settings=ROOT/'work/studio_settings.json';original_settings=settings.read_bytes() if settings.exists() else None
    cfg=Experiment(initial_x_deg=12,initial_y_deg=-8)
    m,d,_=make_model(cfg)
    import numpy as np
    actual=lean(d.site_xmat[m.site('body_reference').id].reshape(3,3))
    assert np.max(abs(actual-[12,-8]))<1e-6,actual
    checks.append('initial X/Y lean matches requested body angles')
    native=initial['build'];assert build_is_current(native)
    def run(name,build):
        q=queue.Queue();run_experiment(Experiment(duration_s=2,realtime=False).model_dump(),build,name,q,queue.Queue())
        records=[];done=None
        while not q.empty():
            p=q.get()
            if p['kind']=='frame':records.append(p)
            elif p['kind']=='done':done=p
        assert records and done and done['status']!='error',done
        assert all(0<=p['tilt']<=180 for p in records)
        return records,done
    a,done=run('.verification-native',native)
    b,_=run('.verification-repeat',native)
    assert len(a)==len(b)
    assert max(abs(x['x']-y['x']) for x,y in zip(a,b))<1e-10
    assert any(abs(p['pwm'][0]-1500)>0 or abs(p['pwm'][1]-1500)>0 for p in a)
    assert any(abs(p['servo_deg'][0]-180)>0.1 or abs(p['servo_deg'][1]-135)>.1 for p in a)
    checks+=['original C++ PWM affects simulated joint positions','fixed seed produces identical trajectories','overall tilt metric stays in 0..180 degrees']
    try:
        custom=build_project(ROOT/'controllers/example',persist=False)['build']
        c,_=run('.verification-custom',custom)
        assert custom['fingerprint']!=native['fingerprint']
        assert abs(c[-1]['x']-a[-1]['x'])>.01
        checks.append('separate custom C++ controller compiles and changes physical trajectory')
        invalid=ROOT/'work/invalid-controller';invalid.mkdir(exist_ok=True)
        (invalid/'sim_controller.cpp').write_text('this is invalid C++',encoding='utf-8')
        try:build_project(invalid);raise AssertionError('invalid C++ unexpectedly built')
        except ValueError as e:assert '编译失败' in str(e)
        checks.append('invalid C++ produces compiler diagnostics')
        custom_file=ROOT/'controllers/example/sim_controller.cpp'
        before=custom_file.read_bytes()
        try:
            custom_file.write_bytes(before+b'\n// verification: changed source\n')
            assert not build_is_current(custom)
        finally:custom_file.write_bytes(before)
        checks.append('source changes invalidate compiled snapshot')
        modular=ROOT/'work/modular-controller';(modular/'include').mkdir(parents=True,exist_ok=True);(modular/'src').mkdir(exist_ok=True)
        text=(ROOT/'controllers/example/sim_controller.cpp').read_text(encoding='utf-8')
        text='#include "extra.h"\n'+text.replace('config=*c;','config=*c;config.kp[0]=extra_gain(config.kp[0]);')
        (modular/'sim_controller.cpp').write_text(text,encoding='utf-8')
        (modular/'include/extra.h').write_text('double extra_gain(double);',encoding='utf-8')
        (modular/'src/gain.cpp').write_text('#include "extra.h"\ndouble extra_gain(double x){return x*1.2;}',encoding='utf-8')
        (modular/'sim_project.json').write_text(json.dumps({'sources':['src/gain.cpp'],'include_dirs':['include']}),encoding='utf-8')
        multi=build_project(modular,persist=False)['build'];assert build_is_current(multi)
        checks.append('multi-file C++ adapter links declared translation units')
        crash=ROOT/'work/crash-controller';crash.mkdir(exist_ok=True)
        text=(ROOT/'controllers/example/sim_controller.cpp').read_text(encoding='utf-8')
        text='#include <cstdlib>\n'+text.replace('  constexpr double rad=', '  std::exit(27);\n  constexpr double rad=')
        (crash/'sim_controller.cpp').write_text(text,encoding='utf-8')
        crash_build=build_project(crash,persist=False)['build'];mgr=SimulationManager()
        crash_id=mgr.start(Experiment(duration_s=1,realtime=False),crash_build)['run_id']
        until=time.monotonic()+8
        while time.monotonic()<until and mgr.poll()['status'] not in ['error','completed']:time.sleep(.05)
        result=mgr.poll();assert result['status']=='error' and '27' in result['message'],result
        assert (ROOT/'runs'/crash_id/'summary.json').is_file();mgr.close()
        import shutil
        shutil.move(str(ROOT/'runs'/crash_id),str(ROOT/'reports'/('isolated-controller-exit-'+crash_id)))
        checks.append('controller process exit preserves error report without stopping the app')
    finally:pass
    with httpx.Client(base_url='http://127.0.0.1:8765',timeout=20) as client:
        cfg=Experiment(duration_s=20,initial_x_deg=0,initial_y_deg=0).model_dump()
        start=client.post('/api/simulation/start',json=cfg);start.raise_for_status();run_id=start.json()['run_id']
        def wait_status(states,timeout=6):
            until=time.monotonic()+timeout
            while time.monotonic()<until:
                result=client.get('/api/simulation/state').json()
                if result['status'] in states:return result
                if result['status']=='error':raise AssertionError(result)
                time.sleep(.04)
            raise AssertionError('expected state '+str(states))
        wait_status(['running']);client.post('/api/simulation/action',json={'action':'pause'}).raise_for_status()
        paused=wait_status(['paused']);time.sleep(.2)
        assert abs(client.get('/api/simulation/state').json()['time']-paused['time'])<.001
        client.post('/api/simulation/action',json={'action':'resume'}).raise_for_status();wait_status(['running'])
        client.post('/api/simulation/action',json={'action':'disturb','x':0,'y':.15}).raise_for_status()
        time.sleep(.3);client.post('/api/simulation/action',json={'action':'stop'}).raise_for_status()
        finished=wait_status(['stopped']);assert finished['summary']['sample_count']>0
        csv=client.get('/api/runs/'+run_id+'/csv');csv.raise_for_status();assert 'time' in csv.text
        record=client.get('/api/runs/'+run_id).json();assert record['run']['build']['fingerprint']==native['fingerprint']
        checks+=['worker starts without a premature exit status','pause freezes simulation time','resume and disturbance commands reach worker','stop persists run parameters and controller hash','CSV download contains recorded telemetry']
    if original_settings is not None:settings.write_bytes(original_settings)
    report={'checks':checks,'passed':len(checks),'controller_fingerprint':native['fingerprint'],'http_run_id':run_id}
    (ROOT/'reports/studio_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
