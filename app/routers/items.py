from fastapi import APIRouter, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
import asyncio
import threading

from app.ros_node import (
    latest_imu_msg, latest_image_msg, imu_lock, image_lock, imu_to_dict, has_cv_bridge
)

router = APIRouter()
templates = Jinja2Templates(directory="app/templates")


@router.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})


@router.websocket("/sensors/imu")
async def websocket_imu_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            with imu_lock:
                imu_data = imu_to_dict(latest_imu_msg)

            if imu_data:
                await websocket.send_json(imu_data)

            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        print("IMU client disconnected")
    except Exception as e:
        print(f"An error occurred in IMU websocket: {e}")


@router.websocket("/sensors/image")
async def websocket_image_endpoint(websocket: WebSocket):
    await websocket.accept()
    # If cv_bridge or proper ROS image is not available, we just send a small text message
    try:
        while True:
            image_bytes = None
            with image_lock:
                img = latest_image_msg

            if img is not None and has_cv_bridge():
                # Delegates conversion to ros_node to keep this file lightweight
                try:
                    image_bytes = img_to_jpeg_bytes(img)
                except Exception as e:
                    print(f"Image conversion error: {e}")

            if image_bytes:
                await websocket.send_bytes(image_bytes)
            else:
                # send a tiny heartbeat to keep connection alive
                await websocket.send_text("no-image")

            await asyncio.sleep(0.033)
    except WebSocketDisconnect:
        print("Image client disconnected")
    except Exception as e:
        print(f"Unexpected error in Image websocket: {e}")


def img_to_jpeg_bytes(img_msg):
    """Helper to convert an Image message to JPEG bytes by delegating to ros_node if available."""
    # import locally to avoid circular import at module import time
    from app.ros_node import image_msg_to_jpeg_bytes

    return image_msg_to_jpeg_bytes(img_msg)
