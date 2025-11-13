# import os
# import json
# import paho.mqtt.client as mqtt
# import asyncio
# from app.db import db
# from app.models import Telemetry


# # Read environment variables
# MQTT_BROKER = os.getenv("MQTT_BROKER", "mosquitto")
# MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
# MQTT_USERNAME = os.getenv("MQTT_USERNAME")
# MQTT_PASSWORD = os.getenv("MQTT_PASSWORD")

# # ---Create MQTT client---
# client = mqtt.Client(client_id="drone_backend_bridge")

# # ---Optional authentication---

# if MQTT_USERNAME and MQTT_PASSWORD:
#     client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

# # --- CALLBACKS ---

# def on_connect(client, userdata, flags, rc):
#     if rc == 0:
#         print(" Connected to MQTT Broker:", MQTT_BROKER)
#         # Subscribe to drone telemetry topic
#         client.subscribe("drone/telemetry")
#         client.subscribe("drone/status")
#     else:
#         print(" MQTT Connection failed with code:", rc)

# async def save_to_db(payload):
#     try:
#         telemetry = Telemetry(**payload)
#         await db.telemetry.insert_one(telemetry.dict())
#         print(f"Saved telemetry: {telemetry.dict()}")
#     except Exception as e:
#         print(f"DB save error: {e}")

# def on_message(client, userdata, msg):
#     try:
#         payload = json.loads(msg.payload.decode())
#         print(f"Received from MQTT: {payload}")
#         asyncio.create_task(save_to_db(payload))
#     except Exception as e:
#         print(f"Error handling MQTT message: {e}")


# client.on_connect = on_connect
# client.on_message = on_message

# # --- START CONNECTION ---
# def start_mqtt():
#     print(f" Connecting to broker {MQTT_BROKER}:{MQTT_PORT}")
#     client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
#     client.loop_start()
import paho.mqtt.client as mqtt

broker_address = "localhost"
topic = "drone/telemetry"

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT Broker:", rc)
    client.subscribe(topic)

def on_message(client, userdata, message):
    print(f"Message from {topic}: {message.payload.decode()}")

def start_mqtt():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker_address, 1883, 60)
    client.loop_start()
