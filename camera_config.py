"""Fixed identities and endpoints for the two OAK cameras."""

from dataclasses import dataclass


@dataclass(frozen=True)
class CameraConfig:
    role: str
    address: str
    pub_port: int
    ctl_port: int


CAMERAS = {
    "docking_camera": CameraConfig("docking_camera", "192.168.50.21", 5556, 5566),
    "cutting_camera": CameraConfig("cutting_camera", "192.168.50.22", 5557, 5567),
}


def camera_choices():
    return tuple(CAMERAS)
