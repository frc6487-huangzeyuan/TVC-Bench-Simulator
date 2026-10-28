import asyncio,json,threading
from contextlib import asynccontextmanager
from pathlib import Path
from fastapi import FastAPI,HTTPException,Request,WebSocket,WebSocketDisconnect
from fastapi.responses import FileResponse,Response,JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from .compiler import ROOT,DEFAULT_PROJECT,current_project,save_state,scan_project,read_source,build_project,build_is_current
from .simulation import Experiment,SimulationManager,META,parameter_preview
from .tuning import TuningManager,TuningRequest,GAINS,ACTIVE
from .real_recording import RealRecorder,export_csv
from .real_transport import device_info,proxy_device,allowed_local_origin
from .native_console import native_control

manager=None
tuner=None
recorder=RealRecorder(ROOT/'real_runs')
record_socket=None
execution_lock=threading.RLock()
@asynccontextmanager
async def lifespan(app):
    global manager,tuner
    manager=SimulationManager();tuner=TuningManager()
    yield
    if recorder.status()['status']=='recording':recorder.stop('interrupted')
    tuner.close();manager.close()
app=FastAPI(title='TVC Studio',lifespan=lifespan)

class ProjectRequest(BaseModel):path:str
class SourceRequest(BaseModel):project:str;file:str
class AdapterRequest(BaseModel):content:str
class ActionRequest(BaseModel):action:str;x:float=0;y:float=.15
class TrainingSelection(BaseModel):id:str
class RecordStart(BaseModel):transport:str

def wrap_error(fn):
    try:return fn()
    except (ValueError,FileNotFoundError,PermissionError) as e:raise HTTPException(400,str(e))

@app.get('/api/bootstrap')
def bootstrap():
    state=current_project();state['build_current']=build_is_current(state.get('build'))
    settings=ROOT/'work/studio_settings.json'
    cfg=Experiment(**json.loads(settings.read_text(encoding='utf-8'))).model_dump() if settings.exists() else Experiment().model_dump()
    return {'version':'0.1.0','project':state['project'],'build':state.get('build'),
            'build_current':state['build_current'],'config':cfg,'model':META,
            'frame':manager.poll(),'root':str(ROOT),'abi_header':(ROOT/'controllers/sim_api.h').read_text(encoding='utf-8')}

@app.post('/api/project/scan')
def project_scan(body:ProjectRequest):
    project=wrap_error(lambda:scan_project(body.path));old=current_project()
    state={'project':project,'build':old.get('build') if old['project']['path']==project['path'] else None}
    save_state(state);return state|{'build_current':build_is_current(state.get('build'))}

@app.post('/api/project/source')
def project_source(body:SourceRequest):return {'content':wrap_error(lambda:read_source(body.project,body.file))}

@app.post('/api/project/build')
def project_build(body:ProjectRequest):
    return wrap_error(lambda:build_project(body.path))|{'build_current':True}

@app.get('/api/adapter/template')
def template():return {'content':(ROOT/'controllers/example/sim_controller.cpp').read_text(encoding='utf-8')}

@app.post('/api/adapter/save')
def save_adapter(body:AdapterRequest):
    if len(body.content)>500000:raise HTTPException(400,'适配器源码过大。')
    path=ROOT/'controllers/custom';path.mkdir(exist_ok=True)
    (path/'sim_controller.cpp').write_text(body.content,encoding='utf-8')
    project=scan_project(path);save_state({'project':project,'build':None});return {'project':project,'build':None,'build_current':False}

@app.post('/api/simulation/start')
def start(cfg:Experiment):
    with execution_lock:
        if tuner.poll()['status'] in ACTIVE:raise HTTPException(400,'请先停止强化学习训练，再开始实验。')
        state=current_project()
        if not build_is_current(state.get('build')):raise HTTPException(400,'源码尚未编译，或编译后已有改动。请先编译接入当前工程。')
        result=wrap_error(lambda:manager.start(cfg,state['build']))
        (ROOT/'work/studio_settings.json').write_text(json.dumps(cfg.model_dump(),ensure_ascii=False,indent=2),encoding='utf-8')
        return result

@app.post('/api/tuning/start')
def tuning_start(body:TuningRequest):
    with execution_lock:
        if manager.poll()['status'] in ['running','paused','starting']:raise HTTPException(400,'请先停止实验，再开始训练。')
        state=current_project();build=state.get('build')
        if not build_is_current(build):raise HTTPException(400,'请先编译接入当前工程。')
        if build['adapter']!='tvc':raise HTTPException(400,'当前调参器仅支持确认会使用六项 PID 增益的 TVC 适配器。')
        return wrap_error(lambda:tuner.start(body,build))

@app.get('/api/tuning/state')
def tuning_state():return tuner.poll()

@app.get('/api/tuning/history')
def tuning_history():
    records=[]
    for path in sorted((ROOT/'tuning').glob('*/result.json'),reverse=True)[:40]:
        try:
            item=json.loads(path.read_text(encoding='utf-8'))
            if item.get('id')!=path.parent.name or item.get('status') in ACTIVE:continue
            candidate=(item.get('validation') or {}).get('candidate') or {}
            records.append({'id':item['id'],'status':item.get('status'),'created_at':item.get('created_at'),
                            'stable_count':candidate.get('stable_count'),'case_count':candidate.get('case_count')})
        except (OSError,ValueError,TypeError):continue
    return records

@app.post('/api/tuning/select')
def tuning_select(body:TrainingSelection):
    with execution_lock:return wrap_error(lambda:tuner.select(body.id))

def local_request(request:Request):
    if request.headers.get('host') not in ('127.0.0.1:8765','localhost:8765'):
        raise HTTPException(403,'仅允许本地控制台访问。')
    origin=request.headers.get('origin')
    if origin and origin not in ('http://127.0.0.1:8765','http://localhost:8765'):
        raise HTTPException(403,'仅允许本地控制台访问。')

@app.get('/api/real/device')
async def real_device(request:Request):
    local_request(request);info=await device_info()
    return JSONResponse({**info,'ws_url':f"ws://{request.headers['host']}/api/real/ws"},headers={'Cache-Control':'no-store'})

@app.websocket('/api/real/ws')
async def real_socket(ws:WebSocket):await proxy_device(ws)

@app.post('/api/real/record/start')
def real_record_start(body:RecordStart,request:Request):
    local_request(request);return wrap_error(lambda:recorder.start(body.transport))

@app.get('/api/real/record/status')
def real_record_status(request:Request):
    local_request(request);return recorder.status()

@app.post('/api/real/record/stop')
def real_record_stop(request:Request):
    local_request(request);return wrap_error(lambda:recorder.stop())

@app.websocket('/api/real/record/ws/{record_id}')
async def real_record_socket(ws:WebSocket,record_id:str):
    global record_socket
    if not allowed_local_origin(ws) or recorder.status().get('id')!=record_id or recorder.status()['status']!='recording' or record_socket:
        await ws.close(code=1008);return
    record_socket=record_id;reason='disconnected'
    try:
        await ws.accept()
        while True:
            message=await ws.receive_json()
            if not isinstance(message,dict):raise ValueError('采集数据格式不正确。')
            if message.get('type')=='stop':reason='stopped';break
            if message.get('type')!='telemetry':raise ValueError('不支持的数据类型。')
            recorder.append(message.get('transport'),message.get('packet'))
    except WebSocketDisconnect:pass
    except (ValueError,TypeError,KeyError) as exc:
        reason='error'
        try:await ws.send_json({'type':'error','message':str(exc)})
        except RuntimeError:pass
    finally:
        if recorder.status().get('id')==record_id and recorder.status()['status']=='recording':recorder.stop(reason)
        record_socket=None
        try:await ws.close()
        except RuntimeError:pass

@app.get('/api/real/records')
def real_records(request:Request):
    local_request(request);items=[]
    for path in sorted((ROOT/'real_runs').glob('*/metadata.json'),reverse=True):
        try:items.append(json.loads(path.read_text(encoding='utf-8')))
        except (OSError,ValueError):continue
    return items

@app.get('/api/real/records/{record_id}/{format}')
def real_record_download(record_id:str,format:str,request:Request):
    local_request(request)
    if Path(record_id).name!=record_id or record_id in ('.','..') or format not in ('jsonl','csv'):
        raise HTTPException(404,'未找到记录。')
    folder=ROOT/'real_runs'/record_id;source=folder/'telemetry.jsonl'
    if not source.is_file():raise HTTPException(404,'未找到记录。')
    if recorder.status().get('id')==record_id and recorder.status()['status']=='recording':
        raise HTTPException(400,'请先结束采集再导出。')
    path=source if format=='jsonl' else export_csv(folder)
    return FileResponse(path,filename=f'{record_id}.{format}',media_type='text/csv' if format=='csv' else 'application/x-ndjson')

@app.get('/api/tuning/comparison/{training_id}')
def tuning_comparison(training_id:str):
    if Path(training_id).name!=training_id or training_id in ('.','..'):
        raise HTTPException(404,'未找到训练对比记录。')
    path=ROOT/'tuning'/training_id/'comparison.json'
    if not path.is_file():raise HTTPException(404,'该训练没有三维对比记录。')
    return FileResponse(path,media_type='application/json')

@app.post('/api/tuning/stop')
def tuning_stop():return wrap_error(tuner.stop)

@app.post('/api/tuning/apply')
def tuning_apply(cfg:Experiment,stop_running:bool=False):
    with execution_lock:
        result=tuner.poll()
        if result['status'] in ACTIVE:raise HTTPException(400,'请先停止训练，再载入候选参数。')
        gains=result.get('best_gains')
        if result['status'] not in ['completed','stopped'] or not result.get('evaluated') or not isinstance(gains,dict) or set(gains)!=set(GAINS):
            raise HTTPException(400,'尚无已评估的候选参数可载入。请先训练至少一组参数。')
        state=current_project();build=state.get('build')
        if not build_is_current(build) or build['fingerprint']!=result['build']['fingerprint']:
            raise HTTPException(400,'当前控制器与训练快照不一致，请重新训练。')
        ignored=set(GAINS)|{'duration_s','initial_x_deg','initial_y_deg','realtime','seed'}
        if cfg.model_dump(exclude=ignored)!=Experiment(**result['request']['experiment']).model_dump(exclude=ignored):
            raise HTTPException(400,'模型或控制约束已改变，请用新工况重新训练。')
        updated=Experiment(**(cfg.model_dump()|gains))
        if manager.poll()['status'] in ['running','paused','starting']:
            if not stop_running:raise HTTPException(400,'实验正在运行，请使用“停止实验并载入候选”。')
            manager.close();manager.poll()
        (ROOT/'work/studio_settings.json').write_text(json.dumps(updated.model_dump(),ensure_ascii=False,indent=2),encoding='utf-8')
        return {'config':updated.model_dump(),'recommended':bool(result.get('recommended')),
                'message':'候选 PID 已载入仿真配置' if result.get('recommended') else '候选 PID 已载入仿真配置；独立验证未通过，仍需试验'}

@app.post('/api/parameters/preview')
def preview_parameters(cfg:Experiment):
    return wrap_error(lambda:parameter_preview(cfg))

@app.post('/api/parameters/save')
def save_parameters(cfg:Experiment):
    if manager.poll()['status'] in ['running','paused','starting']:
        raise HTTPException(400,'请先停止实验，再保存模型参数。')
    parameter_preview(cfg)
    (ROOT/'work/studio_settings.json').write_text(json.dumps(cfg.model_dump(),ensure_ascii=False,indent=2),encoding='utf-8')
    return {'config':cfg.model_dump()}

@app.post('/api/simulation/action')
def action(body:ActionRequest):
    if body.action not in ['pause','resume','stop','disturb']:raise HTTPException(400,'未知实验操作。')
    if abs(body.x)>2 or abs(body.y)>2:raise HTTPException(400,'扰动力矩范围为 ±2 N·m。')
    wrap_error(lambda:manager.command(body.action,x=body.x,y=body.y));return {'ok':True}

@app.get('/api/simulation/state')
def state():return manager.poll()

@app.post('/api/shutdown')
def shutdown():
    if recorder.status()['status']=='recording':raise HTTPException(400,'真实数据采集中，请先结束采集。')
    tuner.close();manager.close()
    server=getattr(app.state,'uvicorn_server',None)
    if server is not None:server.should_exit=True
    return {'ok':True}

@app.websocket('/ws')
async def frames(ws:WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json(manager.poll());await asyncio.sleep(.04)
    except WebSocketDisconnect:pass

@app.get('/api/runs')
def runs():
    return [json.loads(p.read_text(encoding='utf-8')) for p in sorted((ROOT/'runs').glob('*/summary.json'),reverse=True) if not p.parent.name.startswith('.')][:100]

@app.get('/api/runs/{run_id}')
def run_detail(run_id:str):
    p=ROOT/'runs'/run_id
    if Path(run_id).name!=run_id or not (p/'summary.json').is_file():raise HTTPException(404,'未找到实验记录。')
    return {'summary':json.loads((p/'summary.json').read_text(encoding='utf-8')),
            'run':json.loads((p/'run.json').read_text(encoding='utf-8')),
            'telemetry':json.loads((p/'telemetry.json').read_text(encoding='utf-8'))}

@app.get('/api/runs/{run_id}/csv')
def csv_export(run_id:str):
    p=ROOT/'runs'/run_id/'telemetry.csv'
    if Path(run_id).name!=run_id or not p.is_file():raise HTTPException(404,'该实验没有可导出的采样数据。')
    return FileResponse(p,media_type='text/csv',filename=f'tvc-{run_id}.csv')

@app.get('/device-console/{asset}')
def device_console(asset:str):
    project=current_project()['project']['path']
    if asset=='native-control.mjs':
        text=wrap_error(lambda:native_control(project))
    elif asset in ('protocol.mjs','preview.mjs'):
        path=Path(project)/'web'/'dist'/asset
        if not path.is_file():raise HTTPException(404,'当前飞控工程没有网页协议代码。')
        text=path.read_text(encoding='utf-8')
    else:raise HTTPException(404,'未找到控制代码。')
    return Response(text,media_type='application/javascript',headers={'Cache-Control':'no-store','X-Content-Type-Options':'nosniff'})

app.mount('/meshes',StaticFiles(directory=ROOT/'meshes'),name='meshes')
DIST=ROOT/'frontend/dist'
if DIST.is_dir():app.mount('/',StaticFiles(directory=DIST,html=True),name='studio')
