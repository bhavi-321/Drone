from motor.motor_asyncio import AsyncIOMotorClient
from os import getenv

MONGO_URI = getenv('MONGO_URI', 'mongodb://localhost:27017/drone')
client = AsyncIOMotorClient(MONGO_URI)
db = client.get_default_database()