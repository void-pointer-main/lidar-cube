import pygame
from copy import deepcopy
import serial
from serial_frame_receiver import CSVReceiver

NUM_SCREENS = 6
NUM_ROWS = NUM_COLS = 8
N = NUM_ROWS-1

# screen indexes
FRONT = 0
BACK = 1
LEFT = 2
RIGHT = 3
TOP = 4
BOTTOM = 5

# current and next gen arrays for a cross cube mapping
lebensraum = [[[False for _ in range(NUM_COLS)] for _ in range(NUM_ROWS)] for _ in range(NUM_SCREENS)]
tmp_lbnsrm = deepcopy(lebensraum)

initial_front = [
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 0, 0],
    [0, 1, 1, 1, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
]
for r in range(NUM_ROWS):
    for c in range(NUM_COLS):
        lebensraum[0][r][c] = bool(initial_front[r][c])


neighbour_dirs = [
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, -1),
    (0, 1),
    (1, -1),
    (1, 0),
    (1, 1)
]

def update_next_gen(lebensraum, tmp_lbnsrm):
    for s in range(len(lebensraum)):
        for r in range(len(lebensraum[s])):
            for c in range(len(lebensraum[s][r])):
                cell_cnt = 0
                for dr, dc in neighbour_dirs:
                    cell_cnt += int(cell_is_alive(s, r+dr, c+dc))
                tmp_lbnsrm[s][r][c] = cell_will_be_alive(cell_cnt, lebensraum[s][r][c])

def cell_will_be_alive(cell_cnt, alive):
    if cell_cnt < 2 or cell_cnt > 3:
        return False
    if cell_cnt == 3:
        return True
    return alive


# we allow peaking out of array bounds to neighbours
def cell_is_alive(screen, row, col):
    if (row == -1 and col == -1) or (row == NUM_ROWS and col == NUM_COLS) or (row == -1 and col == NUM_COLS) or (row == NUM_ROWS and col == -1): # degenerate case
        return False
    match screen:
        case 0: # TOP
            if row == -1:
                return lebensraum[TOP][N][col]
            elif row == NUM_ROWS:
                return lebensraum[BOTTOM][0][col]
            elif col == -1:
                return lebensraum[LEFT][row][N]
            elif col == NUM_COLS:
                return lebensraum[RIGHT][row][0]
            else:
                return lebensraum[FRONT][row][col]
        case 1: # BACK
            if row == -1:
                return lebensraum[TOP][0][N-col]
            elif row == NUM_ROWS:
                return lebensraum[BOTTOM][0][N-col]
            elif col == -1:
                return lebensraum[RIGHT][row][N]
            elif col == NUM_COLS:
                return lebensraum[LEFT][row][0]
            else:
                return lebensraum[BACK][row][col]
        case 2: # LEFT
            if row == -1:
                return lebensraum[TOP][col][0]
            elif row == NUM_ROWS:
                return lebensraum[BOTTOM][N-col][0]
            elif col == -1:
                return lebensraum[BACK][row][N]
            elif col == NUM_COLS:
                return lebensraum[FRONT][row][0]
            else:
                return lebensraum[LEFT][row][col]
        case 3: # RIGHT
            if row == -1:
                return lebensraum[TOP][N-col][N]
            elif row == NUM_ROWS:
                return lebensraum[BOTTOM][col][N]
            elif col == -1:
                return lebensraum[FRONT][row][N]
            elif col == NUM_COLS:
                return lebensraum[BACK][row][0]
            else:
                return lebensraum[RIGHT][row][col]
        case 4: # TOP
            if row == -1:
                return lebensraum[BACK][0][N-col]
            elif row == NUM_ROWS:
                return lebensraum[FRONT][0][col]
            elif col == -1:
                return lebensraum[LEFT][0][row]
            elif col == NUM_COLS:
                return lebensraum[RIGHT][0][N-row]
            else:
                return lebensraum[TOP][row][col]
        case 5: # BOTTOM
            if row == -1:
                return lebensraum[FRONT][N][col]
            elif row == NUM_ROWS:
                return lebensraum[BACK][N][N-col]
            elif col == -1:
                return lebensraum[LEFT][N][N-row]
            elif col == NUM_COLS:
                return lebensraum[RIGHT][N][row]
            else:
                return lebensraum[BOTTOM][row][col]
        case _:
            return False

def draw_lebensraum(screen, lebensraum, start_r, start_c, cell_size):
    for r in range(len(lebensraum)):
        for c in range(len(lebensraum[r])):
            if lebensraum[r][c]:
                pygame.draw.rect(screen, 'white', (c*cell_size+start_c, r*cell_size+start_r, cell_size, cell_size))


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

pygame.init()

# Window setup
width, height = WIN_WIDTH, WIN_HEIGHT
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("game o' life test")

clock = pygame.time.Clock()

receiver = CSVReceiver("COM45", decoder=int)

running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    payload = receiver.poll()

    if payload is not None and len(payload) == NUM_ROWS * NUM_COLS*2:
        # Update logic here
        update_next_gen(lebensraum, tmp_lbnsrm)
        # swap next generation into current generation
        lebensraum, tmp_lbnsrm = tmp_lbnsrm, lebensraum

        for r in range(NUM_ROWS):
            for c in range(NUM_COLS):
                if payload[r*NUM_COLS + c]:
                    lebensraum[0][r][c] = True

        # Draw
        screen.fill((0, 0, 0))  # clear screen

        # Draw things here
        for s in range(len(lebensraum)):
            start_r, start_c = SCREEN_POSITIONS[s]
            draw_lebensraum(screen, lebensraum[s], start_r, start_c, CELL_SIZE)

        pygame.display.flip()   # update display

    # Limit FPS
    clock.tick(500)

pygame.quit()