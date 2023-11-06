from typing import List
import pydantic

class CarAControlSignal(pydantic.BaseModel):
    target_vel:List[float]=[]
    direction:int=None

class CarAState(pydantic.BaseModel):
    motor_count:int=2
    vels:List[float]=[]
    encoders:List [int]=[]
    direction:int