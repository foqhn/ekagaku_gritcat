"""FastAPI entrypoint - mounts routers, static files and starts the ROS thread."""
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
import threading
import uvicorn

# Import router and ROS runner
from app.routers import items as items_router
from app.ros_node import run_ros_node

app = FastAPI()

# Mount static files (served at /static)
app.mount("/static", StaticFiles(directory="app/static"), name="static")

# Include routers
app.include_router(items_router.router)


if __name__ == "__main__":
    ros_thread = threading.Thread(target=run_ros_node, daemon=True)
    ros_thread.start()
    # Run uvicorn on this module's app
    uvicorn.run(app, host="0.0.0.0", port=8000, reload=True)