from serial_frame_receiver import CSVReceiver
from scipy.spatial.transform import Rotation
import numpy as np
from geometry import Lidar
import matplotlib.pyplot as plt

lidars = []

NUM_ROWS = NUM_COLS = 8
CUBE_UNIT = 33.5

receiver = CSVReceiver("COM45", decoder=int)

lidars.append(
    Lidar(
        position=np.array([0, CUBE_UNIT, CUBE_UNIT*2]),
        rotation=Rotation.from_euler("XYZ", [0, 45, -90], degrees=True).as_matrix(),
        resolution=NUM_ROWS
    )
)

# Create figure
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Initial empty point cloud
scatter = ax.scatter([], [], [], s=30)

# Set cube limits (example: 500 mm cube)
ax.set_xlim(-500, 500)
ax.set_ylim(-500, 500)
ax.set_zlim(-500, 500)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")


def update_points(points):

    scatter._offsets3d = (
        points[:, 0],
        points[:, 1],
        points[:, 2]
    )

    plt.draw()
    plt.pause(0.016)


# Example fake lidar data
while True:

    payload = receiver.poll()
    
    if payload is not None and len(payload) == NUM_ROWS * NUM_COLS:

        points = lidars[0].points(payload)

        update_points(points)
