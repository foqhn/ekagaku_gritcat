import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from typing import Dict

app = FastAPI()


# API: 接続中のロボット一覧を返す
@app.get("/api/robots")
async def get_connected_robots():
    # connections に登録されていて、かつ robot ウェブソケットが接続されているものを返す
    return [rid for rid, info in connections.items() if info.get("robot") is not None]

# 接続中のロボットとフロントエンドを管理するための辞書
# { "robot_id": {"robot": WebSocket, "frontend": WebSocket | None} }
connections: Dict[str, Dict[str, WebSocket | None]] = {}

@app.websocket("/ws/robot/{robot_id}")
async def websocket_robot_endpoint(websocket: WebSocket, robot_id: str):
    await websocket.accept()
    if robot_id not in connections:
        connections[robot_id] = {"robot": None, "frontend": None}
    connections[robot_id]["robot"] = websocket
    print(f"Robot '{robot_id}' connected.")

    try:
        while True:
            # ロボットからデータ（映像やIMU）を受信
            data = await websocket.receive_bytes()
            frontend_ws = connections[robot_id].get("frontend")
            if frontend_ws:
                # 対応するフロントエンドにデータを転送
                await frontend_ws.send_bytes(data)

    except WebSocketDisconnect:
        print(f"Robot '{robot_id}' disconnected.")
        # 接続が切れたら管理リストから削除
        if robot_id in connections:
            del connections[robot_id]

@app.websocket("/ws/frontend/{robot_id}")
async def websocket_frontend_endpoint(websocket: WebSocket, robot_id: str):
    await websocket.accept()
    if robot_id not in connections:
        connections[robot_id] = {"robot": None, "frontend": None}
    connections[robot_id]["frontend"] = websocket
    print(f"Frontend for '{robot_id}' connected.")

    try:
        while True:
            # フロントエンドからコマンドを受信
            command = await websocket.receive_json()
            robot_ws = connections[robot_id].get("robot")
            if robot_ws:
                # 対応するロボットにコマンドを転送
                await robot_ws.send_json(command)

    except WebSocketDisconnect:
        print(f"Frontend for '{robot_id}' disconnected.")
        # フロントエンドの接続が切れてもロボットの接続は維持
        connections[robot_id]["frontend"] = None

if __name__ == "__main__":
    import uvicorn
    # 基地局PCのIPアドレスを指定
    uvicorn.run(app, host="0.0.0.0", port=8000)