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

'''
Miscelaneous Helpers
'''

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

'''
Guard Class
'''

class Guard:
    def __init__(self, position):
        self.position = position
        self.items_to_search_for = []
        self.being_searched = False

    def needs_searching(self):
        if ((len(self.items_to_search_for) > 0) and (not self.being_searched)):
            return True
        return False

    def get_distance(self, robot_pos):
        '''
        return:
            euclidean distance between two points
        '''
        #TODO: this doesn't take into account any obstacles
        return np.sqrt(np.power(self.position[0]-robot_pos[0],2)+np.power(self.position[1]-robot_pos[1],2))

'''
Search Coordinator Class
'''

class SearchCoordinator:
    def __init__(self, map_image, map_scale=0.3):
        self.original_map = load_map(map_image , map_scale)
        self.padded_map = pad_map(self.original_map)

        self.td = TrapezoidalDecomposition(self.padded_map)
        
        self.centers = self.td.find_centers()

        # Create list of guard objects
        self.guard_list = []
        for c in self.centers:
            self.guard_list.append(Guard(c))

        self.search_list = []

    def draw_guards(self):
        draw_path(self.original_map, "Trapezoidal Decomposition Centers", centers=self.centers)

    def start_search(self, item_id):
        if item_id not in self.search_list:
            self.search_list.append(item_id)
            for g in self.guard_list:
                g.items_to_search_for.append(item_id)
        else:
            print("item \"%s\" already being searched for, not adding again" %(item_id))

    #return the guard object to search
    def get_guard_to_search(self, robot_pos):
        closest_guard = None
        shortest_dist = float('inf')
        for g in self.guard_list:
            d = g.get_distance(robot_pos)
            if ((d < shortest_dist) and (g.needs_searching())):
                shortest_dist = d
                closest_guard = g

        closest_guard.being_searched = True
        return closest_guard

    def mark_guard_searched(self, guard, items_found=[]):
        for item in items_found:
            self.search_list.remove(item)
            for g in self.guard_list:
                if item in g.items_to_search_for:
                    g.items_to_search_for.remove(item)

        guard.items_to_search_for = []
        guard.being_searched = False

    def reassign_guard(self, guard):
        guard.being_searched = False
        

'''
MAIN
'''

if __name__ == "__main__":

    sc = SearchCoordinator("HospitalMapCleaned_filledin_cropped_noholes.png")
    #sc.draw_guards()

    # Start search for scissors
    sc.start_search("scissors")
    sc.start_search("advil")
    sc.start_search("bandages")
    sc.start_search("advil")

    print(sc.search_list)

    # Robot1 gets a guard to search
    g1 = sc.get_guard_to_search((50,100))
    print("\nItems for robot1 to search for at guard %s" %(str(g1.position)))
    print(g1.items_to_search_for)

    # Robot2 gets a guard to search
    g2 = sc.get_guard_to_search((50,100)) #robots started at the same spot
    print("\nItems for robot2 to search for at guard %s" %(str(g2.position)))
    print(g2.items_to_search_for)

    # Robot3 gets a guard to search
    g3 = sc.get_guard_to_search((50,100)) #robots started at the same spot
    print("\nItems for robot3 to search for at guard %s" %(str(g3.position)))
    print(g3.items_to_search_for)

    # Robot1 finds nothing
    sc.mark_guard_searched(g1)
    print("\nItems for to search for at guard %s" %(str(g1.position)))
    print(g1.items_to_search_for)

    # Robot2 finds scissors
    sc.mark_guard_searched(g2, ["scissors"])
    print("\nSearch List")
    print(sc.search_list)
    print("Items for to search for at guard %s" %(str(g2.position)))
    print(g2.items_to_search_for)

    # Robot3 needs to go recharge and can't keep searching
    sc.reassign_guard(g3)
    print("\nDoes guard3 need to be searched still:")
    print(g3.needs_searching())