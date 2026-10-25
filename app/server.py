import asyncio,json
from contextlib import asynccontextmanager
from pathlib import Path
from fastapi import FastAPI,HTTPException,WebSocket,WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from .compiler import ROOT,DEFAULT_PROJECT,current_project,save_state,scan_project,read_source,build_project,build_is_current
from .simulation import Experiment,SimulationManager,META

manager=None
@asynccontextmanager
async def lifespan(app):
    global manager
    manager=SimulationManager()
    yield
    manager.close()
app=FastAPI(title='TVC Studio',lifespan=lifespan)

class ProjectRequest(BaseModel):path:str
class SourceRequest(BaseModel):project:str;file:str
class AdapterRequest(BaseModel):content:str
class ActionRequest(BaseModel):action:str;x:float=0;y:float=.15

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
    state=current_project()
    if not build_is_current(state.get('build')):raise HTTPException(400,'源码尚未编译，或编译后已有改动。请先编译接入当前工程。')
    result=wrap_error(lambda:manager.start(cfg,state['build']))
    (ROOT/'work/studio_settings.json').write_text(json.dumps(cfg.model_dump(),ensure_ascii=False,indent=2),encoding='utf-8')
    return result

@app.post('/api/simulation/action')
def action(body:ActionRequest):
    if body.action not in ['pause','resume','stop','disturb']:raise HTTPException(400,'未知实验操作。')
    if abs(body.x)>2 or abs(body.y)>2:raise HTTPException(400,'扰动力矩范围为 ±2 N·m。')
    wrap_error(lambda:manager.command(body.action,x=body.x,y=body.y));return {'ok':True}

@app.get('/api/simulation/state')
def state():return manager.poll()

@app.post('/api/shutdown')
def shutdown():
    manager.close()
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

app.mount('/meshes',StaticFiles(directory=ROOT/'meshes'),name='meshes')
DIST=ROOT/'frontend/dist'
if DIST.is_dir():app.mount('/',StaticFiles(directory=DIST,html=True),name='studio')
