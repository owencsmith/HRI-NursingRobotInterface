# Main and helper function

import os.path
from PIL import Image
import numpy as np
import networkx as nx
import copy

from trapezoidalDecomposition import TrapezoidalDecomposition
from visibility_map import VisibilityMap
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt

def load_map(file_path, resolution_scale):
    ''' Load map from an image and return a 2D binary numpy array
        where 0 represents obstacles and 1 represents free space
    '''
    # Load the image with grayscale
    script_dir = os.path.dirname(os.path.abspath(__file__))
    img = Image.open(os.path.join(script_dir, file_path)).convert('L')

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

# Adds 1 pixel of padding around all obstacles so that all vertices are created at
#   a free point on the original map
def pad_map(map):

    # make a copy to store the padded data in
    padded_map = copy.deepcopy(map)

    # get the map size
    size_row = map.shape[0]
    size_col = map.shape[1]

    for r in range(0, size_row):
        for c in range(0, size_col):
            for rp in range(-1,2):
                for cp in range(-1,2):
                    if (map[r,c] == 1) and ((r+rp > 0) and (c+cp > 0) and (r+rp < size_row) and (c+cp < size_col)):
                        if map[r+rp, c+cp] == 0:
                            padded_map[r,c] = 0

    return padded_map

def draw_path(grid, title, lines = list(), vertices = list(), centers = list(), visibility_map=None):
    
    # Create plot
    fig, ax = plt.subplots(1)
    
    # Draw map
    img = 255 * np.dstack((grid, grid, grid))
    ax.imshow(img)

    # Optional draw TD lines
    for line in lines:
        for pt in line:
            x = pt[0]
            y = pt[1]
            ax.add_patch(Rectangle((y - 0.5, x - 0.5), 1, 1, edgecolor='w', facecolor='b'))

    # Optional draw TD vertices
    for vert in vertices:
        x = vert[0]
        y = vert[1]
        ax.plot(y, x, markersize=10, marker='+', color='g')
        #ax.add_patch(Rectangle((y - 0.5, x - 0.5), 1, 1, edgecolor='w', facecolor='g'))

    # Optional draw TD centers
    for cent in centers:
        if cent is not None:
            x = cent[0]
            y = cent[1]
            ax.add_patch(Rectangle((y - 0.5, x - 0.5), 1, 1, edgecolor='w', facecolor='r'))

    # Optional draw visibility map
    if visibility_map is not None:
        node_pos = np.array(visibility_map.nodes)[:, [1, 0]]
        pos = dict( zip( range( len(visibility_map.nodes) ), node_pos) )
        nx.draw(visibility_map.graph, pos, node_size=3, node_color='y', edge_color='y', ax=ax)

    # Show plot
    plt.title(title)
    plt.axis('scaled')
    plt.show()

if __name__ == "__main__":

    '''
    Map Generation
    '''

    # Load the map
    original_map = load_map("HospitalMapCleaned_filledin_cropped_noholes.png", 0.3)
    
    # Display the initial map
    #draw_path(original_map,"2D Hospital Map")

    # Add padding to the map
    padded_map = pad_map(original_map)

    # Display the padded map
    #draw_path(padded_map,"Padded 2D Hospital Map")

    '''
    Trapezoidal Decomposition and Guard Placement
    '''

    # Run the trapeziodal decomposition algorithm
    td = TrapezoidalDecomposition(padded_map)
    centers, vertices = td.find_centers()

    # Display the TD centers and vertices
    draw_path(original_map, "Trapezoidal Decomposition Centers", vertices=vertices, centers=centers)

    '''
    Create the visibility graph
    '''

    # try using just the centers and see if the graph is fully connected
    vm = VisibilityMap(original_map, centers) 
    if nx.number_connected_components(vm.graph) > 1:
        # there are disconnected sections, add the vertices as nodes
        print("There are %d sections of the graph, adding vertices" %(nx.number_connected_components(vm.graph)))
        vm = VisibilityMap(original_map, centers + vertices)
    if nx.number_connected_components(vm.graph) > 1:
        # there are still disconnected sections, exiting
        print("There are now %d sections of the graph, exiting" %(nx.number_connected_components(vm.graph)))
        raise NotImplementedError("There are still disconnected sections of the visibility map")

    draw_path(original_map, "Visibility Map", visibility_map=vm)