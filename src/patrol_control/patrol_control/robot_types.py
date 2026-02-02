from enum import Enum

class RobotState(Enum):
    IDLE = 'IDLE'
    PATROL_STARTED = 'PATROL_STARTED'
    RETURN_HOME = 'RETURN_HOME'
    STOPPED = 'STOPPED'

    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'

    def to_string(self) -> str:
            return self.name
    
class RobotCmd(Enum):
    START_PATROL = 'START_PATROL'
    RETURN_HOME = 'RETURN_HOME'
    STOP = 'STOP'

    def to_string(self) -> str:
        return self.name