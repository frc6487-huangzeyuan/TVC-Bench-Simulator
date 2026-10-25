"""备份后保存用户确认的舵机/舵盘引用修复，不修改零件材料和尺寸。"""
import hashlib,json,shutil,sys
from datetime import datetime
from pathlib import Path
import pythoncom,win32com.client

sys.stdout.reconfigure(encoding='utf-8')
ROOT=Path(__file__).resolve().parents[1]
def val(o,n):
    x=getattr(o,n)
    return x() if callable(x) and not hasattr(x,'_oleobj_') else x
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
pythoncom.CoInitialize()
sw=win32com.client.GetActiveObject('SldWorks.Application')
top=sw.ActiveDoc
assert val(top,'GetPathName')==r'C:\Projects\机械部分\04-总装配.SLDASM'
docs=[val(c,'GetModelDoc2') for c in top.GetComponents(False) if c.Name2=='03-重力模块舵机-5']+[top]
back=ROOT/'backups'/datetime.now().strftime('%Y%m%d-%H%M%S')
back.mkdir(parents=True)
report=[]
for doc in docs:
    path=Path(val(doc,'GetPathName'))
    deps=sw.GetDocumentDependencies2(str(path),False,True,False)
    backup=back/path.name
    shutil.copy2(path,backup)
    entry={'path':str(path),'backup':str(backup),'before_sha256':sha(path),'dependencies_before':list(deps or [])}
    if val(doc,'IsOpenedReadOnly'):
        assert doc.SetReadOnlyState(False), '无法解除当前会话只读状态'
    err=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
    warn=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
    ok=doc.Save3(1,err,warn)
    entry.update(ok=bool(ok),error=err.value,warning=warn.value,after_sha256=sha(path),
                 dependencies_after=list(sw.GetDocumentDependencies2(str(path),False,True,False) or []))
    report.append(entry)
    print(path.name,ok,err.value,warn.value,flush=True)
    assert ok and err.value==0,entry
(ROOT/'reports'/'reference_repairs.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('BACKUP',back)
