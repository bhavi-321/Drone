from fastapi import FastAPI
from app.mqtt_client import start_mqtt
from app.routers import telemetry

app = FastAPI()

app.include_router(telemetry.router)

@app.on_event("startup")
def startup_event():
    start_mqtt()
