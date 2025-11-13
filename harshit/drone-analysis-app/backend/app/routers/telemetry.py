from fastapi import APIRouter, HTTPException
from ..db import db
from pydantic import BaseModel
from datetime import datetime

router = APIRouter(prefix='/api/telemetry')

class TelemetryIn(BaseModel):
    drone_id: str
    time: datetime
    lat: float
    lon: float
    alt: float
    battery: float = None

@router.post('/ingest')
async def ingest(t: TelemetryIn):
    doc = {
    'drone_id': t.drone_id,
    't': t.time,
    'lat': t.lat,
    'lon': t.lon,
    'alt': t.alt,
    'battery': t.battery,
    }
    await db.telemetry.insert_one(doc)
# optionally publish to MQTT here
    return {'ok': True}

@router.get("/telemetry/latest")
async def get_latest_telemetry():
    data = await db.telemetry.find_one(sort=[("_id", -1)])
    if data:
        data["_id"] = str(data["_id"])  # convert ObjectId to string
        return data
    return {"message": "No telemetry data found"}
