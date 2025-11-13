# from pydantic import BaseModel
# from typing import Optional

# class Telemetry(BaseModel):
#     timestamp: float
#     latitude: Optional[float] = None
#     longitude: Optional[float] = None
#     altitude: Optional[float] = None
#     velocity: Optional[float] = None
#     battery: Optional[float] = None
#     mode: Optional[str] = None
#     armed: Optional[bool] = None
#     drone_id: Optional[str] = "drone_001"
from pydantic import BaseModel

class Telemetry(BaseModel):
    latitude: float
    longitude: float
    altitude: float
    velocity: float
    battery: float
    timestamp: str
