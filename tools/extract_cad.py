"""从只读打开的 SolidWorks 总装配提取几何、变换、配合和 CAD 质量属性。

坐标和质量统一保留 SolidWorks API 的 SI 单位；此脚本不保存源文件。
"""
import json
import sys
from pathlib import Path

import numpy as np
import pythoncom
import trimesh
import win32com.client

sys.stdout.reconfigure(encoding="utf-8")
ROOT = Path(__file__).resolve().parents[1]

def val(obj, name):
    x = getattr(obj, name)
    return x() if callable(x) and not hasattr(x, "_oleobj_") else x

def attempt(fn):
    try:
        return fn()
    except Exception as e:
        return {"error": str(e)}

def mates(model):
    found = []
    def visit(f, sub=False):
        while f is not None:
            typ = val(f,"GetTypeName2")
            if typ == "MateGroup":
                visit(val(f,"GetFirstSubFeature"), True)
            elif "Mate" in typ:
                m = val(f,"GetSpecificFeature2")
                if m is not None:
                    record = {"name": f.Name, "feature_type": typ,
                              "type": attempt(lambda: m.Type),
                              "alignment": attempt(lambda: m.Alignment), "entities": []}
                    count = attempt(lambda: val(m,"GetMateEntityCount"))
                    if isinstance(count,int):
                        for i in range(count):
                            try:
                                e = m.MateEntity(i)
                                c = e.ReferenceComponent
                                record["entities"].append({"component": c.Name2 if c else None,
                                     "type": e.ReferenceType2, "params": list(e.EntityParams)})
                            except Exception as ex:
                                record["entities"].append({"error":str(ex)})
                    found.append(record)
            f = val(f,"GetNextSubFeature" if sub else "GetNextFeature")
    visit(val(model,"FirstFeature"))
    return found

pythoncom.CoInitialize()
sw = win32com.client.GetActiveObject("SldWorks.Application")
doc = sw.ActiveDoc
print("SOURCE", val(doc,"GetPathName"), flush=True)
sw.FrameState = 1
attempt(lambda: val(doc,"ViewZoomtofit2"))
components = list(doc.GetComponents(False) or [])
records, models, mesh_ids = [], {}, {}
for c in components:
    model = val(c,"GetModelDoc2")
    if model is None:
        records.append({"name":c.Name2,"missing":True,"path":val(c,"GetPathName")})
        continue
    path = val(c,"GetPathName")
    cfg = c.ReferencedConfiguration
    key = (path,cfg)
    t = c.Transform2
    rec = {"name":c.Name2,"path":path,"config":cfg,"fixed":val(c,"IsFixed"),
           "suppression":val(c,"GetSuppression"), "transform":list(t.ArrayData) if t else None,
           "type":val(model,"GetType"), "parent":c.Name2.rpartition('/')[0] or None}
    if key not in models:
        idx = len(models)
        item = {"path":path,"config":cfg,"id":idx,"type":rec["type"],
                "units":attempt(lambda:list(val(model,"GetUnits"))),
                "mates":attempt(lambda:mates(model))}
        if rec["type"] == 1:
            mp = val(model.Extension,"CreateMassProperty")
            mp.UseSystemUnits = True
            item["mass"] = {"mass":mp.Mass,"com":list(mp.CenterOfMass),
                          "inertia":list(mp.GetMomentOfInertia(0)),
                          "volume":mp.Volume,"density":mp.Density}
            item["appearance"] = attempt(lambda:list(model.MaterialPropertyValues))
            db = win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_BSTR, "")
            item["material"] = attempt(lambda:model.GetMaterialPropertyName2(cfg,db))
            try:
                triangles = np.asarray(model.GetTessTriangles(True),dtype=float).reshape(-1,3)
                mesh = trimesh.Trimesh(vertices=triangles, faces=np.arange(len(triangles)).reshape(-1,3),process=False)
                mesh_name = f"part_{idx:03d}.stl"
                mesh.export(ROOT/"meshes"/mesh_name)
                item["mesh"] = mesh_name
                item["bounds"] = mesh.bounds.tolist()
                item["triangles"] = len(mesh.faces)
            except Exception as ex:
                item["mesh_error"] = str(ex)
        models[key] = item
        print("MODEL",idx,Path(path).name,"type",rec["type"],"mesh",item.get("triangles",item.get("mesh_error","assembly")),flush=True)
    rec["model_id"] = models[key]["id"]
    records.append(rec)
result = {"source":val(doc,"GetPathName"),"revision":val(sw,"RevisionNumber"),
          "components":records,"models":list(models.values()),"mates":mates(doc)}
(ROOT/"reports"/"cad_extract.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
print("DONE",len(records),"components",len(models),"models",len(result["mates"]),"top-level mates",flush=True)
