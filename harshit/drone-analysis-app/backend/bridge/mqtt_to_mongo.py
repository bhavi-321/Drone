import json
import paho.mqtt.client as mqtt
from pymongo import MongoClient

# MongoDB setup
mongo_client = MongoClient("mongodb://localhost:27017/")
db = mongo_client["drone_data"]
telemetry_collection = db["telemetry"]

# MQTT setup
broker = "localhost"
topic = "drone/telemetry"

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker!")
    client.subscribe(topic)

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        print("Received message:", payload)
        telemetry_collection.insert_one(payload)
        print("Inserted into MongoDB!")
    except Exception as e:
        print("Error processing message:", e)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker, 1883, 60)
client.loop_forever()
