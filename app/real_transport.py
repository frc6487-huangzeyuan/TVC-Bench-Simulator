"""本地控制台到原飞控 Wi-Fi 接口的透明转发。"""
import asyncio
import httpx
import websockets
from fastapi import HTTPException,WebSocket,WebSocketDisconnect

DEVICE_HTTP='http://192.168.4.1/api/device'
DEVICE_WS='ws://192.168.4.1:81/ws'
DEVICE_ORIGIN='http://192.168.4.1'


async def device_info():
    try:
        # 仅访问原固件固定地址；不使用系统代理，也不接受任意外部地址。
        async with httpx.AsyncClient(timeout=3,trust_env=False) as client:
            response=await client.get(DEVICE_HTTP);response.raise_for_status();info=response.json()
        if not isinstance(info,dict) or info.get('ws_url')!=DEVICE_WS:
            raise ValueError('设备返回的 WebSocket 地址不符合原固件接口。')
        return info
    except (httpx.HTTPError,ValueError) as exc:
        raise HTTPException(503,'无法读取飞控信息，请确认电脑已连接 TVC-Bench 热点。') from exc


def allowed_local_origin(ws:WebSocket):
    return ws.headers.get('origin') in ('http://127.0.0.1:8765','http://localhost:8765')


async def proxy_device(ws:WebSocket):
    if not allowed_local_origin(ws):await ws.close(code=1008);return
    await ws.accept()
    try:
        async with websockets.connect(DEVICE_WS,origin=DEVICE_ORIGIN,proxy=None,open_timeout=4,
                                      close_timeout=2,max_size=8192,compression=None) as device:
            async def upstream():
                while True:
                    message=await ws.receive_text()
                    if len(message.encode('utf-8'))>8192:raise ValueError('命令帧过长')
                    await device.send(message)
            async def downstream():
                async for message in device:
                    if not isinstance(message,str):raise ValueError('设备发送了非文本帧')
                    await ws.send_text(message)
            tasks=[asyncio.create_task(upstream()),asyncio.create_task(downstream())]
            done,pending=await asyncio.wait(tasks,return_when=asyncio.FIRST_COMPLETED)
            for task in pending:task.cancel()
            await asyncio.gather(*pending,return_exceptions=True)
            for task in done:
                try:task.result()
                except (WebSocketDisconnect,websockets.exceptions.ConnectionClosed):pass
    except (WebSocketDisconnect,websockets.exceptions.ConnectionClosed):pass
    except (OSError,TimeoutError,ValueError,websockets.exceptions.WebSocketException):
        try:await ws.close(code=1011,reason='无法连接飞控')
        except RuntimeError:pass
