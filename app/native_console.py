"""Studio 自有的真实飞控控制引擎与协议资源。"""
from pathlib import Path

ASSETS=Path(__file__).resolve().parent/'device_console'

def native_control(project=None):
    # 保留调用签名；独立网页不再影响 Studio 的控制入口。
    return (ASSETS/'native-control.mjs').read_text(encoding='utf-8')

def console_asset(name):
    if name not in ('protocol.mjs','preview.mjs'):
        raise ValueError('未找到控制资源。')
    return (ASSETS/name).read_text(encoding='utf-8')
