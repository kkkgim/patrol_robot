from enum import Enum

class RobotState(Enum):
    IDLE = 'IDLE'
    PATROL_STARTED = 'PATROL_STARTED'
    RETURN_HOME = 'RETURN_HOME'
    STOPPED = 'STOPPED'
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
    
STATE_COMMAND_MAP = {
    RobotState.PATROL_STARTED: RobotCmd.START_PATROL,
    RobotState.RETURN_HOME: RobotCmd.RETURN_HOME,
    RobotState.STOPPED: RobotCmd.STOP,
}
