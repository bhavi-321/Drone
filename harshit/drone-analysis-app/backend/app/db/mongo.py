# from motor.motor_asyncio import AsyncIOMotorClient
# from pymongo.server_api import ServerApi
# import os

# MONGO_URI = os.getenv("MONGO_URI", "mongodb://localhost:27017")
# MONGO_DB_NAME = os.getenv("MONGO_DB_NAME", "drone_data")

# client = AsyncIOMotorClient(MONGO_URI, server_api=ServerApi("1"))
# db = client[MONGO_DB_NAME]
from pymongo import MongoClient
import os
from dotenv import load_dotenv

load_dotenv()

MONGO_URI = os.getenv("MONGO_URI", "mongodb://localhost:27017/")
client = MongoClient(MONGO_URI)
db = client["drone_data"]
