from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

SQUARE_DIM = 5
CELL_DIM = 0.2
grid_height = int(2*SQUARE_DIM/CELL_DIM)
grid_width = grid_height


import numpy as np
RADIUS = 1.0


SQUARE_DIM = 5
CELL_DIM = 0.2
grid_height = int(2*SQUARE_DIM/CELL_DIM)
grid_width = grid_height
grid =  np.zeros([grid_height, grid_width])

# TODO add if risk_zone == True

def make_world():

    fig = plt.figure(figsize=(7,7))

    plt.xlim(-SQUARE_DIM, SQUARE_DIM)
    plt.ylim(-SQUARE_DIM, SQUARE_DIM)

    ax = fig.add_subplot(1, 1, 1)

    # # Major ticks every 20, minor ticks every 5
    # major_ticks = np.arange(-SQUARE_DIM, SQUARE_DIM, 1)
    # minor_ticks = np.arange(-SQUARE_DIM, SQUARE_DIM, CELL_DIM)

    # ax.set_xticks(major_ticks)
    # ax.set_xticks(minor_ticks, minor=True)
    # ax.set_yticks(major_ticks)
    # ax.set_yticks(minor_ticks, minor=True)

    # # And a corresponding grid
    # ax.grid(which='both')

    # # Or if you want different settings for the grids:
    # ax.grid(which='minor', alpha=0.2)
    # ax.grid(which='major', alpha=0.5)


    # obstacles

    # GOAL 1
    for row in range(0, 5):
        for column in range(5, 10):
            grid[row][column] = 1

    # GOAL 2
    for row in range(20, 25):
        for column in range(45, 50):
            grid[row][column] = 1

    # GOAL 3
    for row in range(40, 45):
        for column in range(45, 50):
            grid[row][column] = 1

    # GOAL 4
    for row in range(0, 5):
        for column in range(20, 25):
            grid[row][column] = 1

    # Wall 1
    for row in range(0, 20):
        for column in range(18, 21):
            grid[row][column] = 1

    # Wall 2
    for row in range(0, 20):
        for column in range(24, 27):
            grid[row][column] = 1


    #for row in range(25,30):
        #for column in range(35,37):
            #grid[row][column] = 1

    #for row in range(30,35):
        #for column in range(35,40):
            #grid[row][column] = 1

    #for row in grid:

    # The bottom and left rectangle coordinates

    # draw obstacles
    currentAxis = plt.gca()
    for column in range(grid_height):
        for row in range(grid_height):
            if grid[row][column] == 1:
                currentAxis.add_patch(Rectangle((column*CELL_DIM - SQUARE_DIM, SQUARE_DIM - CELL_DIM*(row + 1)), CELL_DIM, CELL_DIM,
                          alpha=1.0, facecolor='black'))


    # Risk Zone
    #currentAxis.add_patch(Rectangle(( 0.0, -5.0), 1.0, 10, alpha=0.8, facecolor='red'))

    return grid, fig


if __name__ == '__main__':
    make_world()