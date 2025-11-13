from datetime import datetime, timedelta
from jose import jwt
from os import getenv

JWT_SECRET = getenv('JWT_SECRET', 'change_me')
ALGORITHM = 'HS256'

def create_access_token(data: dict, expires_minutes: int = 60*24*7):
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(minutes=expires_minutes)
    to_encode.update({"exp": expire})
    encoded = jwt.encode(to_encode, JWT_SECRET, algorithm=ALGORITHM)
    return encoded