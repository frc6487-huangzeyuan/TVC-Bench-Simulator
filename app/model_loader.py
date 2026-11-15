"""通过内存加载模型资源，避免 MuJoCo 在 Windows 中文路径下读取失败。"""
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco

ROOT = Path(__file__).resolve().parents[1]

def load_model(path=ROOT / 'mjcf/tvc_bench.xml'):
    xml = path.read_text(encoding='utf-8')
    tree = ET.fromstring(xml)
    files = {Path(name).name for mesh in tree.findall('.//mesh')
             if (name:=mesh.get('file') or mesh.get('filename'))}
    assets = {name: (ROOT / 'meshes' / name).read_bytes() for name in files}
    return mujoco.MjModel.from_xml_string(xml, assets)
