# Main and helper function

from PIL import Image
import numpy as np
from trapezoidalDecomposition import TrapezoidalDecomposition
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt


def load_map(file_path, resolution_scale):
    ''' Load map from an image and return a 2D binary numpy array
        where 0 represents obstacles and 1 represents free space
    '''
    # Load the image with grayscale
    img = Image.open(file_path).convert('L')
    # Rescale the image
    size_x, size_y = img.size
    new_x, new_y  = int(size_x*resolution_scale), int(size_y*resolution_scale)
    img = img.resize((new_x, new_y), Image.ANTIALIAS)

    map_array = np.asarray(img, dtype='uint8')

    # Get bianry image
    threshold = 240 #230
    map_array = 1 * (map_array > threshold)

    # Result 2D numpy array
    return map_array

def draw_path(grid, title, lines = list(), vertices = list()):
    # Visualization of the found path using matplotlib
    fig, ax = plt.subplots(1)
    ax.margins()
    # Draw map
    row = len(grid)     # map size
    col = len(grid[0])  # map size
    for i in range(row):
        for j in range(col):
            if grid[i][j]:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='w',facecolor='w'))  # free space
            else:
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='w',facecolor='k'))  # obstacle
    # Draw path
    # for x, y in path:
    #     ax.add_patch(Rectangle((y-0.5, x-0.5),1,1,edgecolor='k',facecolor='b'))          # path
    # ax.add_patch(Rectangle((start[1]-0.5, start[0]-0.5),1,1,edgecolor='k',facecolor='g'))# start
    # ax.add_patch(Rectangle((goal[1]-0.5, goal[0]-0.5),1,1,edgecolor='k',facecolor='r'))  # goal
    # Graph settings

    for line in lines:
        for pt in line:
            x = pt[0]
            y = pt[1]
            ax.add_patch(Rectangle((y - 0.5, x - 0.5), 1, 1, edgecolor='k', facecolor='b'))

    for vert in vertices:
        x = vert[0]
        y = vert[1]
        ax.add_patch(Rectangle((y - 0.5, x - 0.5), 1, 1, edgecolor='k', facecolor='r'))


    plt.title(title)
    plt.axis('scaled')
    plt.gca().invert_yaxis()
    plt.show()


if __name__ == "__main__":
    # Load the map
    start = (200, 75)
    goal  = (30, 250)
    map_array = load_map("HospitalMapCleaned_filledin_cropped.png", 0.1)
    # map_array = load_map("test_map_simple.png", 0.1)
    draw_path(map_array,"Hospital Array")

    trap_decomp_graph = TrapezoidalDecomposition(map_array)
    vertices, line_lists_for_boundaries = trap_decomp_graph.create_trapezoids() # trap_decomp_graph = TrapezoidalDecomposition(map_array)
    draw_path(map_array,"Hospital Array",line_lists_for_boundaries, vertices)


    # # Planning class
    # PRM_planner = PRM(map_array)
    # RRT_planner = RRT(map_array, start, goal)
    #
    # # Search with PRM
    # PRM_planner.sample(n_pts=1000, sampling_method="uniform")
    # PRM_planner.search(start, goal)
    # PRM_planner.sample(n_pts=1000, sampling_method="random")
    # PRM_planner.search(start, goal)
    # PRM_planner.sample(n_pts=2000, sampling_method="gaussian")
    # PRM_planner.search(start, goal)
    # PRM_planner.sample(n_pts=20000, sampling_method="bridge")
    # PRM_planner.search(start, goal)
    #
    # # Search with RRT and RRT*
    # RRT_planner.RRT(n_pts=1000)
    # RRT_planner.RRT_star(n_pts=2000)