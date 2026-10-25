"""只读检查 SolidWorks API 连接及总装配，不保存 CAD 文档。"""
import json
import sys
from pathlib import Path

import pythoncom
import win32com.client
sys.stdout.reconfigure(encoding="utf-8")

ROOT = Path(__file__).resolve().parents[1]
def value(obj, name):
    item = getattr(obj, name)
    return item() if callable(item) and not hasattr(item, "_oleobj_") else item

pythoncom.CoInitialize()
try:
    sw = win32com.client.GetActiveObject("SldWorks.Application")
except Exception:
    sw = win32com.client.Dispatch("SldWorks.Application")
print("CONNECTED", repr(sw), flush=True)
print("REV", value(sw,"RevisionNumber"), flush=True)
sw.Visible = True
print("DOCS", value(sw,"GetDocumentCount"), flush=True)
if len(sys.argv) > 1:
    err = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    warn = win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)
    doc = sw.ActiveDoc
    if doc is None or value(doc,"GetPathName").lower() != sys.argv[1].lower():
        doc = sw.OpenDoc6(sys.argv[1], 2, 3, "", err, warn)  # silent + read-only
    print("OPEN", err.value, warn.value, repr(doc), flush=True)
    if doc is not None:
        print("TITLE", value(doc,"GetTitle"), "PATH", value(doc,"GetPathName"), flush=True)
        items = []
        for c in doc.GetComponents(False) or []:
            m = value(c,"GetModelDoc2")
            t = c.Transform2
            items.append({"name": c.Name2, "path": value(c,"GetPathName"),
                          "suppression": value(c,"GetSuppression"),
                          "fixed": value(c,"IsFixed"), "config": c.ReferencedConfiguration,
                          "transform": list(t.ArrayData) if t else None,
                          "loaded": m is not None})
        (ROOT / "reports" / "components.json").write_text(
            json.dumps(items, ensure_ascii=False, indent=2), encoding="utf-8")
        print("COMPONENTS", len(items), flush=True)
        for item in items:
            print(item["name"], item["suppression"], item["loaded"], flush=True)
