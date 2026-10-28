"""启动本地仿真服务，并在桌面应用窗口中打开界面。"""
import multiprocessing,os,subprocess,sys,threading,time,urllib.request,webbrowser
from pathlib import Path
ROOT=Path(__file__).resolve().parent

def open_window():
    url='http://127.0.0.1:8765'
    for _ in range(80):
        try:
            with urllib.request.urlopen(url+'/api/bootstrap',timeout=1) as r:
                if r.status==200:break
        except Exception:time.sleep(.2)
    browser=next((p for p in [Path(r'C:\Program Files\Google\Chrome\Application\chrome.exe'),
        Path(r'C:\Program Files (x86)\Google\Chrome\Application\chrome.exe'),
        Path.home()/'AppData/Local/Google/Chrome/Application/chrome.exe'] if p.is_file()),None)
    if browser:
        subprocess.Popen([str(browser),'--app='+url,'--window-size=1580,1000','--user-data-dir='+str(ROOT/'work/chrome-profile')],creationflags=0x08000000)
    else:webbrowser.open(url)

if __name__=='__main__':
    multiprocessing.freeze_support();os.chdir(ROOT)
    if sys.stdout is None or sys.stderr is None:
        log=(ROOT/'reports/studio-server.log').open('a',encoding='utf-8',buffering=1)
        sys.stdout=log;sys.stderr=log
    try:
        with urllib.request.urlopen('http://127.0.0.1:8765/api/bootstrap',timeout=.5) as r:
            if r.status==200:
                if '--no-browser' not in sys.argv:open_window()
                sys.exit(0)
    except Exception:pass
    if '--no-browser' not in sys.argv:threading.Thread(target=open_window,daemon=True).start()
    import uvicorn
    from app.server import app
    server=uvicorn.Server(uvicorn.Config(app,host='127.0.0.1',port=8765,log_level='warning'))
    app.state.uvicorn_server=server
    server.run()
