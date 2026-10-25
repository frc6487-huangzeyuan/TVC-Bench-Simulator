"""读取用户工程，固定源码快照，并编译成具有明确 ABI 的本地控制器。"""
import hashlib,json,os,re,shutil,subprocess,time,uuid
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_PROJECT=Path(r'C:\Projects\代码\TVC_Electronics')
GXX=Path(r'C:\msys64\ucrt64\bin\g++.exe')
STATE=ROOT/'work/studio_project.json'
EXTS={'.ino','.h','.hpp','.cpp','.c','.cc'}
SKIP={'.git','.venv','node_modules','build','dist','backups','tests'}

def scan_project(path):
    p=Path(path).expanduser().resolve()
    if not p.is_dir():raise ValueError('工程目录不存在。请填写包含飞控源码的文件夹。')
    files=[]
    for base,dirs,names in os.walk(p):
        dirs[:]=[d for d in dirs if d not in SKIP and not d.startswith('.')]
        for name in sorted(names):
            f=Path(base)/name
            if f.suffix.lower() in EXTS and f.stat().st_size<2_000_000:
                files.append({'path':f.relative_to(p).as_posix(),'bytes':f.stat().st_size})
        if len(files)>1000:break
    tvc=all((p/n).is_file() for n in ['GimbalControl.h','Attitude.h','AutoLevelPid.h','ServoLimits.h','HistoryAttitude.h'])
    custom=(p/'sim_controller.cpp').is_file()
    version=''
    if (p/'bench_config.h').is_file():
        match=re.search(r'FW_VERSION\[\]\s*=\s*"([^"]+)"',(p/'bench_config.h').read_text(encoding='utf-8-sig'))
        version=match.group(1) if match else ''
    result={'path':str(p),'name':p.name,'files':files,'file_count':len(files),
            'adapter':'tvc' if tvc else 'custom' if custom else None,'firmware':version,
            'scope':'原工程 Attitude + GimbalPid' if tvc else '自定义 C++ 仿真接口' if custom else '源码可读；需添加 sim_controller.cpp 接口',
            'supported':bool(tvc or custom)}
    return result

def read_source(project,relative):
    p=Path(project).resolve();f=(p/relative).resolve()
    if not f.is_relative_to(p) or f.suffix.lower() not in EXTS:raise ValueError('请选择工程内的 C/C++ 源码文件。')
    if f.stat().st_size>2_000_000:raise ValueError('该文件超过编辑器读取上限。')
    return f.read_text(encoding='utf-8-sig',errors='replace')

def current_project():
    if STATE.exists():return json.loads(STATE.read_text(encoding='utf-8'))
    return {'project':scan_project(DEFAULT_PROJECT),'build':None}

def save_state(state):STATE.write_text(json.dumps(state,ensure_ascii=False,indent=2),encoding='utf-8')

def build_is_current(build):
    if not build or not Path(build['dll']).is_file():return False
    for entry in build['files']:
        name=entry['file']
        if name=='sim_api.h':p=ROOT/'controllers/sim_api.h'
        elif name=='sim_controller.cpp' and build['adapter']=='tvc':p=ROOT/'controllers/tvc_adapter.cpp'
        else:p=Path(build['project_path'])/name
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=entry['sha256']:return False
    return True

def build_project(path,persist=True):
    project=scan_project(path)
    if not project['supported']:raise ValueError('此工程需要仿真适配器。可先打开接口示例，使用 sim_controller.cpp 接入控制循环。Arduino 外设与网络代码不直接在电脑执行。')
    if not GXX.is_file():raise ValueError('未找到本机 C++ 编译器：'+str(GXX))
    build_id=time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]
    folder=ROOT/'work/controllers'/build_id;source=folder/'source';source.mkdir(parents=True)
    p=Path(path).resolve();translation_units=['sim_controller.cpp'];include_dirs=[p]
    if project['adapter']=='custom' and (p/'sim_project.json').is_file():
        custom=json.loads((p/'sim_project.json').read_text(encoding='utf-8-sig'))
        translation_units=list(dict.fromkeys(translation_units+custom.get('sources',[])))
        include_dirs+=[p/n for n in custom.get('include_dirs',[])]
        if len(translation_units)>30:raise ValueError('单个适配工程最多支持 30 个 C/C++ 编译单元。')
        if any(not f.resolve().is_relative_to(p) for f in include_dirs):raise ValueError('头文件搜索目录必须在工程内部。')
    queue=[p/'GimbalControl.h',p/'HistoryAttitude.h'] if project['adapter']=='tvc' else [p/n for n in translation_units]
    copied={}
    while queue:
        f=queue.pop(0).resolve()
        if f in copied:continue
        if not f.is_relative_to(p):raise ValueError('源码引用位于工程目录外：'+str(f))
        rel=f.relative_to(p);dest=source/rel;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dest)
        copied[f]={'file':rel.as_posix(),'sha256':hashlib.sha256(f.read_bytes()).hexdigest()}
        for name in re.findall(r'^\s*#\s*include\s*"([^"]+)"',f.read_text(encoding='utf-8-sig',errors='replace'),re.M):
            if name=='sim_api.h':continue
            dep=f.parent/name
            if not dep.is_file():dep=next((d/name for d in include_dirs if (d/name).is_file()),p/name)
            if not dep.is_file():raise ValueError('缺少头文件：'+name)
            queue.append(dep)
    api=ROOT/'controllers/sim_api.h';shutil.copy2(api,source/'sim_api.h')
    if project['adapter']=='tvc':shutil.copy2(ROOT/'controllers/tvc_adapter.cpp',source/'sim_controller.cpp')
    dll=folder/'controller.dll'
    command=[str(GXX),'-std=c++17','-O2','-shared','-static','-static-libgcc','-static-libstdc++']
    for d in include_dirs:command+=['-I',str(source/d.resolve().relative_to(p))]
    command += [str(source/n) for n in translation_units]+['-o',str(dll)]
    env=os.environ.copy();env['PATH']=str(GXX.parent)+os.pathsep+env.get('PATH','')
    start=time.perf_counter()
    try:result=subprocess.run(command,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=60,env=env,creationflags=0x08000000)
    except subprocess.TimeoutExpired:raise ValueError('C++ 编译超过 60 秒，已结束编译进程。请检查适配工程。')
    logs=(result.stdout+'\n'+result.stderr).strip()
    (folder/'build.log').write_text(logs,encoding='utf-8')
    if result.returncode:raise ValueError('C++ 编译失败\n'+logs[-10000:])
    files=list(copied.values())
    if project['adapter']=='custom' and (p/'sim_project.json').is_file():
        shutil.copy2(p/'sim_project.json',source/'sim_project.json')
        files.append({'file':'sim_project.json','sha256':hashlib.sha256((p/'sim_project.json').read_bytes()).hexdigest()})
    files.extend({'file':n,'sha256':hashlib.sha256((source/n).read_bytes()).hexdigest()} for n in ['sim_api.h']+(['sim_controller.cpp'] if project['adapter']=='tvc' else []))
    fingerprint=hashlib.sha256(json.dumps(files,sort_keys=True).encode()).hexdigest()
    build={'id':build_id,'dll':str(dll),'fingerprint':fingerprint,'files':files,'seconds':round(time.perf_counter()-start,3),
           'compiled_at':time.strftime('%Y-%m-%d %H:%M:%S'),'scope':project['scope'],'adapter':project['adapter'],'log':logs,'project_path':str(p)}
    (folder/'manifest.json').write_text(json.dumps(build,ensure_ascii=False,indent=2),encoding='utf-8')
    if persist:save_state({'project':project,'build':build})
    return {'project':project,'build':build}
