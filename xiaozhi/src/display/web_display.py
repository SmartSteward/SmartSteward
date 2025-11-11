import asyncio
import json
import webbrowser
from typing import Dict, Any, Callable, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
import uvicorn
import os
import socket

from src.display.base_display import BaseDisplay


class WebDisplay(BaseDisplay):
    def __init__(self, host: str = "127.0.0.1", port: int = 0):
        super().__init__()
        self.host = host
        self.port = port if port != 0 else self._find_free_port()
        self.callbacks: Dict[str, Callable] = {}
        self.connected_clients = set()
        self.browser_opened = False
        
        # 创建 FastAPI 应用
        self.app = FastAPI(title="XiaoZhi Web UI", version="1.0.0")
        
        # 挂载静态文件目录（用于存放前端页面）
        static_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "static")
        if os.path.exists(static_dir):
            self.app.mount("/static", StaticFiles(directory=static_dir), name="static")
            # 添加根路径路由来服务 10.html
            @self.app.get("/")
            async def get_index():
                from starlette.responses import FileResponse
                return FileResponse(os.path.join(static_dir, "10.html"))
            
            # 添加专门的情绪显示页面
            @self.app.get("/emotion")
            async def get_emotion():
                from starlette.responses import FileResponse
                return FileResponse(os.path.join(static_dir, "emotion.html"))
        else:
            # 如果没有静态文件目录，提供一个简单的根路径
            @self.app.get("/")
            async def get_index():
                return {"message": "XiaoZhi Web UI Server"}
            
            @self.app.get("/emotion")
            async def get_emotion():
                return {"message": "XiaoZhi Emotion Display"}

        # 注册路由
        self._register_routes()
        
        # 启动服务器的任务
        self.server_task = None

    def _find_free_port(self):
        """查找可用的端口"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind(('', 0))
            s.listen(1)
            port = s.getsockname()[1]
        return port

    def _register_routes(self):
        """注册 FastAPI 路由"""
        
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.connected_clients.add(websocket)
            
            try:
                while True:
                    data = await websocket.receive_text()
                    # 处理来自前端的消息
                    await self._handle_frontend_message(data, websocket)
            except WebSocketDisconnect:
                self.connected_clients.remove(websocket)
        
        @self.app.get("/health")
        async def get_health():
            return {"status": "ok"}

        @self.app.post("/emotion/{emotion}")
        async def update_emotion_endpoint(emotion: str):
            """提供 HTTP API 来更新表情"""
            await self.update_emotion(emotion)
            return {"status": "success", "emotion": emotion}

    async def _handle_frontend_message(self, data: str, websocket: WebSocket):
        """处理来自前端的消息"""
        try:
            message = json.loads(data)
            action = message.get("action")
            payload = message.get("payload", {})
            
            if action in self.callbacks:
                callback = self.callbacks[action]
                if asyncio.iscoroutinefunction(callback):
                    await callback(**payload)
                else:
                    callback(**payload)
            elif action == "send_text":
                # 处理用户输入的文本
                text = payload.get("text", "")
                if text and "send_text" in self.callbacks:
                    callback = self.callbacks["send_text"]
                    if asyncio.iscoroutinefunction(callback):
                        await callback(text)
                    else:
                        callback(text)
        except Exception as e:
            self.logger.error(f"Error handling frontend message: {e}")

    async def _broadcast_message(self, message: Dict[str, Any]):
        """向所有连接的客户端广播消息"""
        if not self.connected_clients:
            return
            
        disconnected_clients = set()
        for client in self.connected_clients:
            try:
                await client.send_text(json.dumps(message, ensure_ascii=False))
            except WebSocketDisconnect:
                disconnected_clients.add(client)
            except Exception as e:
                self.logger.error(f"Error sending message to client: {e}")
                disconnected_clients.add(client)
        
        # 移除已断开的连接
        self.connected_clients -= disconnected_clients

    async def update_text(self, text: str):
        """更新文本显示"""
        await self._broadcast_message({
            "type": "text_update",
            "data": {
                "text": text
            }
        })

    async def update_emotion(self, emotion: str):
        """更新表情显示"""
        # 定义支持的情绪状态映射，将各种情绪词映射到标准表情
        emotion_mapping = {
            # 基本情绪
            'happy': 'happy',
            'joy': 'happy',
            'joyful': 'happy',
            'pleased': 'happy',
            'delighted': 'happy',
            'laughing': 'happy',
            'funny': 'happy',
            'cool': 'happy',
            'loving': 'happy',
            'excited': 'happy',
            'cheerful': 'happy',
            'pleasure': 'happy',
            'glad': 'happy',
            'smiling': 'happy',
            
            'sad': 'sad',
            'upset': 'sad',
            'disappointed': 'sad',
            'depressed': 'sad',
            'unhappy': 'sad',
            'crying': 'sad',
            'cry': 'sad',
            'tearful': 'sad',
            'grieving': 'sad',
            'sorrowful': 'sad',
            
            'angry': 'angry',
            'mad': 'angry',
            'furious': 'angry',
            'irritated': 'angry',
            'annoyed': 'angry',
            'rage': 'angry',
            'enraged': 'angry',
            'outraged': 'angry',
            'resentful': 'angry',
            
            'surprised': 'surprised',
            'shocked': 'surprised',
            'amazed': 'surprised',
            'astonished': 'surprised',
            'stunned': 'surprised',
            'bewildered': 'surprised',
            'confused': 'surprised',
            'disbelief': 'surprised',
            
            'neutral': 'neutral',
            'calm': 'neutral',
            'normal': 'neutral',
            'default': 'neutral',
            'indifferent': 'neutral',
            'bored': 'neutral',
            'thoughtful': 'neutral',
            'pensive': 'neutral',
        }
        
        # 将情绪词标准化，如果不在映射中则使用默认状态
        normalized_emotion = emotion_mapping.get(emotion.lower(), 'neutral')
        
        # 将情绪更新消息广播到所有连接的客户端
        # 消息格式包含类型和数据，其中数据包含具体的情绪状态
        print(f"广播情绪更新: {emotion} (标准化为: {normalized_emotion})")  # 添加调试日志
        await self._broadcast_message({
            "type": "emotion_update",
            "data": {
                "emotion": normalized_emotion
            }
        })

    async def update_status(self, status: str, connected: bool = False):
        """更新状态显示"""
        await self._broadcast_message({
            "type": "status_update",
            "data": {
                "status": status,
                "connected": connected
            }
        })

    async def update_button_status(self, text: str):
        """更新按钮状态"""
        await self._broadcast_message({
            "type": "button_status_update",
            "data": {
                "text": text
            }
        })

    async def set_callbacks(
        self,
        press_callback: Optional[Callable] = None,
        release_callback: Optional[Callable] = None,
        mode_callback: Optional[Callable] = None,
        auto_callback: Optional[Callable] = None,
        abort_callback: Optional[Callable] = None,
        send_text_callback: Optional[Callable] = None,
    ):
        """设置回调函数"""
        self.callbacks = {
            "press": press_callback,
            "release": release_callback,
            "mode": mode_callback,
            "auto_toggle": auto_callback,
            "abort": abort_callback,
            "send_text": send_text_callback,
        }
        # 过滤掉 None 值
        self.callbacks = {k: v for k, v in self.callbacks.items() if v is not None}

    async def start(self):
        """启动 Web 服务器"""
        config = uvicorn.Config(
            self.app,
            host=self.host,
            port=self.port,
            log_level="info"
        )
        server = uvicorn.Server(config)
        
        # 在后台运行服务器
        self.server_task = asyncio.create_task(server.serve())
        self.logger.info(f"Web UI started at http://{self.host}:{self.port}")
        
        # 等待服务器启动后自动打开浏览器
        asyncio.create_task(self._open_browser())

    async def _open_browser(self):
        """延迟打开浏览器以确保服务器已启动"""
        await asyncio.sleep(1)
        if not self.browser_opened:
            webbrowser.open(f"http://{self.host}:{self.port}/")
            self.browser_opened = True

    async def close(self):
        """关闭显示"""
        if self.server_task and not self.server_task.done():
            self.server_task.cancel()
            try:
                await self.server_task
            except asyncio.CancelledError:
                pass


# 用于测试的简单应用
app = FastAPI()

@app.get("/")
async def root():
    return {"message": "XiaoZhi Web UI"}

if __name__ == "__main__":
    uvicorn.run("web_display:app", host="127.0.0.1", port=8000, reload=True)