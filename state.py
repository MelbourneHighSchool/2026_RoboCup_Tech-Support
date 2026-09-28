from enum import Enum, StrEnum


class BotMode(Enum):
    DEFENCE = 1
    GOALIE = 2
    STRIKER = 3


class GameState(StrEnum):
    AUTO = ""
    STARTING = "STARTING"
    BLOCKED = "BLOCKED"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"


class StartupStage(StrEnum):
    OTHER = "OTHER"
    LIDAR = "LIDAR"
    HARDWARE = "HARDWARE"
    CAMERA = "CAMERA"


class HealthState(StrEnum):
    UNKNOWN = "?"
    READY = "+"
    FAILED = "!"


class FusionMode(StrEnum):
    OFF = "off"
    DIAGNOSTIC = "diagnostic"
    ACTIVE = "active"


class FusionSource(StrEnum):
    CAMERA = "camera"
    FUSED = "fused"

