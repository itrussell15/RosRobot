import enum

class Registers(enum.IntEnum):
    MOVE = 0x01
    STOP = 0x02
    SERVO = 0x03

class DriveStatus(enum.Enum):
    STARTING = 0
    IN_MOTION = 1
    STOPPED = 2

    @classmethod
    def has_key(cls, name):
        return name in cls.__members__

class DriveState(enum.Enum):
    OPERATIONAL = 0
    ERROR = 1

class Servos(enum.IntEnum):
    TOP = 1
    BOTTOM = 2
