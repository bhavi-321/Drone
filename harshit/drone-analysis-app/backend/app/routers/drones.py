from fastapi import APIRouter

router = APIRouter(prefix="/drones", tags=["drones"])

@router.get("/")
def list_drones():
    return {"message": "Drone routes active"}
