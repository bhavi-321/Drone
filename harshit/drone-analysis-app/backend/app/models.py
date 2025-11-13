from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime

class TelemetryPoint(BaseModel):
    t: datetime
    lat: float
    lon: float
    alt: float
    battery: Optional[float]

class Mission(BaseModel):
    mission_id: str
    start_time: datetime
    end_time: Optional[datetime]
    summary: dict
    telemetry: List[TelemetryPoint] = []

class Drone(BaseModel):
    drone_id: str
    owner: str
    last_seen: Optional[datetime]
    missions: List[Mission] = []