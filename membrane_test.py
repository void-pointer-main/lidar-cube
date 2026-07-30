import pygame
from serial_frame_receiver import CSVReceiver
from geometry import Lidar, Display
import numpy as np
from scipy.spatial.transform import Rotation
import time

NUM_ROWS = NUM_COLS = 8
CUBE_UNIT = 33.5

lidars = [
    Lidar(
        position=np.array([0, CUBE_UNIT, CUBE_UNIT*2]),
        rotation=Rotation.from_euler("XYZ", [0, 45, -90], degrees=True).as_matrix(),
        resolution=NUM_ROWS
    ),
    Lidar(
        position=np.array([-CUBE_UNIT, 2*CUBE_UNIT, 2*CUBE_UNIT]),
        rotation=Rotation.from_euler("XYZ", [-45, 0, 180], degrees=True).as_matrix(),
        resolution=NUM_ROWS
    )
]

displays = [
    Display(
        position=np.array([0, 0, CUBE_UNIT*2]),
        rotation=Rotation.from_euler("XYZ", [90, 90, 0], degrees=True).as_matrix(),
        resolution=NUM_ROWS
    ),
    Display(
        position=np.array([-CUBE_UNIT*2, CUBE_UNIT*2, CUBE_UNIT*2]),
        rotation=Rotation.from_euler("XYZ", [90, -90, 0], degrees=True).as_matrix(),
        resolution=NUM_ROWS
    ),
    Display(
        position=np.array([-CUBE_UNIT*2, 0, CUBE_UNIT*2]),
        rotation=Rotation.from_euler("XYZ", [90, 0, 0], degrees=True).as_matrix(),
        resolution=NUM_ROWS
    ),
    Display(
        position=np.array([0, CUBE_UNIT*2, CUBE_UNIT*2]),
        rotation=Rotation.from_euler("XYZ", [90, 180, 0], degrees=True).as_matrix(),
        resolution=NUM_ROWS
    ),
    Display(
        position=np.array([-CUBE_UNIT*2, 0, CUBE_UNIT*2]),
        rotation=Rotation.from_euler("XYZ", [0, 0, 90], degrees=True).as_matrix(),
        resolution=NUM_ROWS
    ),
    Display(
        position=np.array([0, 0, 0]),
        rotation=Rotation.from_euler("XYZ", [0, 180, 90], degrees=True).as_matrix(),
        resolution=NUM_ROWS
    )
]

CELL_SIZE = 20
SCREEN_SIZE = CELL_SIZE*NUM_COLS
SCREEN_POSITIONS = [
    (SCREEN_SIZE, SCREEN_SIZE),
    (SCREEN_SIZE, SCREEN_SIZE*3),
    (SCREEN_SIZE, 0),
    (SCREEN_SIZE, SCREEN_SIZE*2),
    (0, SCREEN_SIZE),
    (SCREEN_SIZE*2, SCREEN_SIZE)
]

WIN_WIDTH = SCREEN_SIZE*4
WIN_HEIGHT = SCREEN_SIZE*3

MAX_INTENSITY = 5.
def draw_display(screen, display, charges, start_r, start_c, cell_size, dt):
    for r in range(display.resolution):
        for c in range(display.resolution):
            point = display.pixel_vectors[r][c]
            display.intensity[r][c] = intensity_magnitude_at_point(point, charges, gain=2,exponent=2)

            # intensity = np.clip(intensity, 0., MAX_INTENSITY)/MAX_INTENSITY
            # pygame.draw.rect(screen, hsv_to_rgb(intensity, 1., 1., 1.), (c*cell_size+start_c, r*cell_size+start_r, cell_size, cell_size))
    
    p = np.pad(display.height, 1, mode="constant")
    lap = (
            p[:-2,1:-1] +
            p[2:,1:-1] +
            p[1:-1,:-2] +
            p[1:-1,2:] -
            4*p[1:-1,1:-1]
    )

    # print(display.height)

    display.velocity += 100 * lap * dt
    display.velocity += 0.35 * display.intensity
    display.velocity *= 0.9
    display.height += display.velocity * dt

    for r in range(display.resolution):
        for c in range(display.resolution):
            h = np.clip(display.height[r][c], 0., MAX_INTENSITY)/MAX_INTENSITY
            pygame.draw.rect(screen, hsv_to_rgb(h, 1., 1., 1.), (c*cell_size+start_c, r*cell_size+start_r, cell_size, cell_size))
    

def intensity_magnitude_at_point(point, charges, gain=1, exponent=2):
    r = point - charges
    dist = np.linalg.norm(r, axis=1)

    field = gain * r / dist[:, None]**exponent

    E = field.sum(axis=0)

    return np.linalg.norm(E)

# https://stackoverflow.com/questions/24852345/hsv-to-rgb-color-conversion I was lazy
scalar = float # a scale value (0.0 to 1.0)
def hsv_to_rgb( h:scalar, s:scalar, v:scalar, a:scalar ) -> tuple:
    a = int(255*a)
    if s:
        if h == 1.0: h = 0.0
        i = int(h*6.0); f = h*6.0 - i
        
        w = int(255*( v * (1.0 - s) ))
        q = int(255*( v * (1.0 - s * f) ))
        t = int(255*( v * (1.0 - s * (1.0 - f)) ))
        v = int(255*v)
        
        if i==0: return (v, t, w, a)
        if i==1: return (q, v, w, a)
        if i==2: return (w, v, t, a)
        if i==3: return (w, q, v, a)
        if i==4: return (t, w, v, a)
        if i==5: return (v, w, q, a)
    else: v = int(255*v); return (v, v, v, a)

pygame.init()

# Window setup
width, height = WIN_WIDTH, WIN_HEIGHT
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Electric Field Test")

clock = pygame.time.Clock()

receiver = CSVReceiver("COM45", decoder=int)

prev_time = time.time()

displays[0].height[4][4] = 1.

running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    payload = receiver.poll()

    if payload is not None and len(payload) == NUM_ROWS * NUM_COLS * 2:

        dt = time.time() - prev_time
        print(dt)
        charges = np.vstack([
            lidars[0].points(payload[:NUM_ROWS * NUM_COLS]),
            lidars[1].points(payload[NUM_ROWS * NUM_COLS:])
        ])
        for i in range(len(displays)-5):
            draw_display(screen, displays[i], charges, SCREEN_POSITIONS[i][0], SCREEN_POSITIONS[i][1], CELL_SIZE, dt)

        prev_time = time.time()

        pygame.display.flip()   # update display

    # Limit FPS
    clock.tick(500)

pygame.quit()
