from pydantic import BaseModel
from typing import Dict


class IMUHeader(BaseModel):
    stamp: Dict[str, int]
    frame_id: str


class IMUModel(BaseModel):
    header: IMUHeader
    orientation: Dict[str, float]
    angular_velocity: Dict[str, float]
    linear_acceleration: Dict[str, float]
