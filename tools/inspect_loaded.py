import json,sys
from pathlib import Path
import pythoncom,win32com.client
from probe_sw import value
sys.stdout.reconfigure(encoding='utf-8')
sw=win32com.client.GetActiveObject('SldWorks.Application')
doc=sw.ActiveDoc
print('MAIN',value(doc,'GetPathName'))
for c in doc.GetComponents(False):
    if '/' not in c.Name2 and '重力' in c.Name2:
        m=value(c,'GetModelDoc2')
        for obj in [doc,m]:
            print('STATE',value(obj,'GetPathName'),'dirty',value(obj,'GetSaveFlag'))
        print('DISK DEPS',sw.GetDocumentDependencies2(value(m,'GetPathName'),False,True,False))
        break
for c in doc.GetComponents(False):
    if c.Name2=='01-单轴电机组装-2':
        util=value(sw,'GetMathUtility'); util._FlagAsMethod('CreatePoint'); pt=util.CreatePoint(win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,(.01,.02,.03)))
        print('INPUT_POINT',list(pt.ArrayData)); pt._FlagAsMethod('MultiplyTransform')
        out=pt.MultiplyTransform(c.Transform2)
        print('SW_POINT',list(out.ArrayData))
        print('TRANSFORM',list(c.Transform2.ArrayData))
        break
