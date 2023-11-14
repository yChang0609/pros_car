from typing import List
import pydantic

CARA_CONTROL = 'CarA_Control'
class TwoWheelAndServoControlSignal(pydantic.BaseModel):
    target_vel:List[float]=[]
    direction:int=None

class TwoWheelAndServoState(pydantic.BaseModel):
    motor_count:int=2
    vels:List[float]=[]
    encoders:List [int]=[]
    direction:int

class TwoWheelControlSignal(pydantic.BaseModel):
    target_vel:List[float]=[]
class TwoWheelState(pydantic.BaseModel):
    motor_count:int=2
    vels:List[float]=[]
    encoders:List [int]=[]
    
