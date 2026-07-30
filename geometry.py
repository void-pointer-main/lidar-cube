import numpy as np
from dataclasses import dataclass, field

@dataclass
class Lidar:
    position: np.ndarray
    rotation: np.ndarray
    rays: np.ndarray = None
    resolution: int = 8
    fov_deg: float = 60

    def __post_init__(self):
        if self.rays is None:
            self.rays = self.generate_rays_simple(self.resolution, self.fov_deg)

    @staticmethod
    def generate_rays_simple(resolution, fov_deg):
        rays = np.zeros((resolution**2, 3))

        center = (resolution - 1) / 2
        angle_step_deg = fov_deg/resolution

        for i in range(resolution):
            for j in range(resolution):
                idx = i * resolution + j

                x_angle = (j - center) * np.deg2rad(angle_step_deg)
                y_angle = (center - i) * np.deg2rad(angle_step_deg)

                ray = np.array([
                    np.tan(x_angle),
                    np.tan(y_angle),
                    1.0
                ])

                rays[idx] = ray / np.linalg.norm(ray)

        return rays

    def points(self, distances):
        local = self.rays * np.array(distances)[:, None]
        return local @ self.rotation.T + self.position


@dataclass
class Display:
    position: np.ndarray
    rotation: np.ndarray
    resolution: int = 8
    width: float = 64
    pixel_vectors: np.ndarray = None
    intensity: np.ndarray = None
    height: np.ndarray = None
    velocity: np.ndarray = None

    def __post_init__(self):
        if self.pixel_vectors is None:
            self.pixel_vectors = self.generate_pixel_vectors(self.resolution, self.width) @ self.rotation.T + self.position

        if self.intensity is None:
            self.intensity = np.zeros(
                (self.resolution, self.resolution),
                dtype=np.float32
            )

        if self.height is None:
            self.height = np.zeros(
                (self.resolution, self.resolution),
                dtype=np.float32
            )

        if self.velocity is None:
            self.velocity = np.zeros(
                (self.resolution, self.resolution),
                dtype=np.float32
            )


    @staticmethod
    def generate_pixel_vectors(resolution, width):
        pixel_vectors = np.zeros((resolution, resolution, 3))
        increment = width/(resolution-1)
        for r in range(resolution):
            for c in range(resolution):
                pixel_vectors[r][c] = np.array([(c+0.5)*increment, -(r+0.5)*increment, 0])

        return pixel_vectors
    
