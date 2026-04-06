from enum import Enum

class RobotState(Enum):
    IDLE = 'IDLE'
    START_PATROL = 'START_PATROL'
    RETURN_HOME = 'RETURN_HOME'
    STOP = 'STOP'
    CANCELLED = 'CANCELLED'
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
    
class TaskResult(Enum):
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"

    def to_string(self) -> str:
            return self.name
    