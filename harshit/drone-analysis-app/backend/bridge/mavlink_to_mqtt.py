from pymavlink import mavutil
import paho.mqtt.client as mqtt
import json

# --- Connect to your local ArduPilot SITL or real drone ---
# Example for SITL:
mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# --- MQTT setup ---
mqtt_client = mqtt.Client("drone_mavlink_bridge")


mqtt_client.connect("localhost", 1883, 60)  # 'localhost' because broker runs on your Mac via Docker
mqtt_client.loop_start()

print(" MAVLink → MQTT Bridge Started")

while True:
    msg = mav.recv_match(blocking=True)
    if not msg:
        continue

    if msg.get_type() == "GLOBAL_POSITION_INT":
        data = {
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "alt": msg.alt / 1000.0,
            "relative_alt": msg.relative_alt / 1000.0,
            "time_boot_ms": msg.time_boot_ms
        }
        mqtt_client.publish("drone/telemetry", json.dumps(data))
        print(f" Sent telemetry: {data}")

    elif msg.get_type() == "HEARTBEAT":
        data = {
            "type": msg.type,
            "autopilot": msg.autopilot,
            "base_mode": msg.base_mode,
            "system_status": msg.system_status
        }
        mqtt_client.publish("drone/status", json.dumps(data))
