from dataclasses import dataclass
from typing import List, Optional

@dataclass
class Point3D:
    x: float
    y: float
    z: float

@dataclass
class CableParameters:
    stiffness: float
    damping: float
    min_length: float
    max_length: float
    min_tension: float
    max_tension: float
    diameter: float

@dataclass
class PlatformParameters:
    mass: float
    com_offset: List[float]  # [x, y, z]
    inertia: List[List[float]]  # 3x3 matrix
    attachment_points: List[Point3D]

@dataclass
class PillarParameters:
    mass: float
    com_offset: List[float]
    inertia: List[List[float]]
    attachment_points: List[Point3D]
    dimensions: List[float]  # [width, length, height]

@dataclass
class WorkspaceParameters:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float
    min_roll: float
    max_roll: float
    min_pitch: float
    max_pitch: float
    min_yaw: float
    max_yaw: float

@dataclass
class RobotConfiguration:
    robot_name: str
    num_cables: int
    platform: PlatformParameters
    frame: List[PillarParameters]
    cables: List[CableParameters]
    workspace_limits: WorkspaceParameters