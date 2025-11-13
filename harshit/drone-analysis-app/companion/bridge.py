# companion/bridge.py
import time
import requests
import os
from pymavlink import mavutil

FASTAPI_HTTP = os.getenv('FASTAPI_HTTP', 'http://backend:8000')
MAVLINK_URL = os.getenv('MAVLINK_URL', 'udp:0.0.0.0:14550')

master = mavutil.mavlink_connection(MAVLINK_URL)

while True:
    msg = master.recv_match(blocking=True, timeout=1)
    if not msg:
        continue
    if msg.get_type() == 'GLOBAL_POSITION_INT':
        payload = {
'drone_id': os.getenv('DRONE_ID', 'drone001'),
'time': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
'lat': msg.lat/1e7,
'lon': msg.lon/1e7,
'alt': msg.relative_alt/1000.0,
'battery': None
        }
        try:
            requests.post(f"{FASTAPI_HTTP}/api/telemetry/ingest", json=payload, timeout=2)
        except Exception as e:
            print('POST failed', e)