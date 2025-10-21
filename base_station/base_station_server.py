import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from typing import Dict, List

app = FastAPI()

# HTMLテンプレートを読み込む設定
app.mount("/static", StaticFiles(directory="static"), name="static") # この行を追加
templates = Jinja2Templates(directory="static")

# 接続中のロボットとフロントエンドを管理するための辞書
# { "robot_id": {"robot": WebSocket | None, "frontend": WebSocket | None} }
connections: Dict[str, Dict[str, WebSocket | None]] = {}


# --- HTTP エンドポイント ---

@app.get("/", response_class=HTMLResponse)
async def get_index(request: Request):
    """
    ブラウザからのアクセス時にindex.htmlを返すエンドポイント
    """
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/api/robots", response_model=List[str])
async def get_robot_list():
    """
    現在接続中のロボットのIDリストをJSONで返すAPIエンドポイント
    """
    # "robot"キーにWebSocket接続が確立されているIDのみをリストアップ
    connected_robots = [
        robot_id for robot_id, conn in connections.items() if conn.get("robot")
    ]
    return connected_robots


# --- WebSocket エンドポイント (変更なし) ---

@app.websocket("/ws/robot/{robot_id}")
async def websocket_robot_endpoint(websocket: WebSocket, robot_id: str):
    await websocket.accept()
    if robot_id not in connections:
        connections[robot_id] = {"robot": None, "frontend": None}
    connections[robot_id]["robot"] = websocket
    print(f"Robot '{robot_id}' connected.")

    try:
        # ロボットからのデータはフロントエンドに中継するだけ
        # データ形式(バイナリ/JSON)を問わず受信してそのまま転送
        while True:
            data = await websocket.receive_bytes() # receive_bytes()で両方受け取れる
            frontend_ws = connections[robot_id].get("frontend")
            if frontend_ws:
                await frontend_ws.send_bytes(data)

    except WebSocketDisconnect:
        print(f"Robot '{robot_id}' disconnected.")
        if robot_id in connections:
            # フロントエンドにも切断を通知したい場合はここで処理を追加できる
            del connections[robot_id]

@app.websocket("/ws/frontend/{robot_id}")
async def websocket_frontend_endpoint(websocket: WebSocket, robot_id: str):
    await websocket.accept()
    if robot_id not in connections:
        # ロボットが未接続でもUIは接続できるようにしておく
        connections[robot_id] = {"robot": None, "frontend": None}
    connections[robot_id]["frontend"] = websocket
    print(f"Frontend for '{robot_id}' connected.")

    try:
        while True:
            # フロントエンドからのコマンドをロボットに中継
            command = await websocket.receive_json()
            robot_ws = connections[robot_id].get("robot")
            if robot_ws:
                await robot_ws.send_json(command)

    except WebSocketDisconnect:
        print(f"Frontend for '{robot_id}' disconnected.")
        # フロントエンドの接続が切れてもロボットの接続は維持
        connections[robot_id]["frontend"] = None

if __name__ == "__main__":
    import uvicorn
    # 基地局PCのIPアドレス(192.168.137.1)で待ち受ける
    uvicorn.run(app, host="192.168.137.1", port=8000)