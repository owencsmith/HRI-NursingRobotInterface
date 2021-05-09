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
import random

'''
Miscelaneous Helpers
'''

def load_map(file_path, resolution_scale):
    ''' Load map from an image and return a 2D binary numpy array
        where 0 represents obstacles and 1 represents free space
    '''
    # Load the image with grayscale
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        img = Image.open(os.path.join(script_dir, file_path)).convert('L')
    except:
        print("Image open failed")

    # Rescale the image
    size_x, size_y = img.size
    new_x, new_y = int(size_x*resolution_scale), int(size_y*resolution_scale)
    img = img.resize((new_x, new_y), Image.ANTIALIAS)

    map_array = np.asarray(img, dtype='uint8')

    # Get bianry image
    threshold = 240 #230
    map_array = 1 * (map_array > threshold)

    # Result 2D numpy array
    return map_array


def resize_to_orig(resolution_scale, pt):
    old_res = 1/resolution_scale

    cx = int(pt[0]*old_res)
    cy = int(pt[1]*old_res)

    return (cx,cy)


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
            ax.add_patch(Rectangle((y - 0.5, x - 0.5), 1, 1, edgecolor='b', facecolor='b'))

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
            ax.add_patch(Rectangle((y - 0.5, x - 0.5), 1, 1, edgecolor='r', facecolor='r'))

    #GRAPH ROBOTS POSITIONS (ex ros output "robot trina2_3 trap graph position is 89, 44" plug in (89 - 0.5, 44 - 0.5) into Rectangle
    ax.add_patch(Rectangle((89 - 0.5, 37 - 0.5), 1, 1, edgecolor='b', facecolor='b'))
    ax.add_patch(Rectangle((89 - 0.5, 94 - 0.5), 1, 1, edgecolor='b', facecolor='b'))
    ax.add_patch(Rectangle((89 - 0.5, 88 - 0.5), 1, 1, edgecolor='b', facecolor='b'))

    #GRAPH THEIR GUARDS (ex ros output is "guard: [88, 91]" plug in (88 - 0.5, 91 - 0.5) into Rectangle
    ax.add_patch(Rectangle((87 - 0.5, 44 - 0.5), 1, 1, edgecolor='y', facecolor='y'))
    ax.add_patch(Rectangle((99 - 0.5, 105 - 0.5), 1, 1, edgecolor='y', facecolor='y'))
    ax.add_patch(Rectangle((90 - 0.5, 85 - 0.5), 1, 1, edgecolor='y', facecolor='y'))


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
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.items_to_search_for = []
        self.being_searched = False

    def needs_searching(self):
        if (len(self.items_to_search_for) > 0) and (not self.being_searched):
            return True
        return False

    def get_distance(self, robot_x, robot_y):
        '''
        return:
            euclidean distance between two points
        '''
        #TODO: this doesn't take into account any obstacles
        return np.sqrt(np.power(self.x-robot_x, 2)+np.power(self.y-robot_y, 2))

'''
Search Coordinator Class
'''

class SearchCoordinator:
    def __init__(self, map_image, map_scale=0.3):
        self.map_scale = map_scale
        self.original_map = load_map(map_image, map_scale)
        self.padded_map = np.flipud(pad_map(self.original_map))

        self.width, self.height = np.shape(self.padded_map)

        self.td = TrapezoidalDecomposition(self.padded_map)
        
        self.centers = self.td.find_centers()

        # Create list of guard objects
        self.guard_list = []
        for c in self.centers:
            self.guard_list.append(Guard(c[1], c[0]))
        self.search_list = []

        # Dictionary to hold item locations
        self.item_location_dict = {}

    def get_width_and_height(self):
        # return self.width, self.height
        return len(self.padded_map), len(self.padded_map[0])

    def draw_guards(self):
        draw_path(np.flipud(self.original_map), "Trapezoidal Decomposition Centers", centers=self.centers)

    def draw_guards_gone_to(self, guard):
        guards = list()
        guards.append([guard.y, guard.x])
        draw_path(self.original_map, "Guard", vertices=guards)

    def get_num_nodes_to_search(self):
        i = 0
        for g in self.guard_list:
            if g.needs_searching():
                i=i+1

        return i

    def start_search(self, item_ids_list):
        for item_id in item_ids_list:
            if item_id not in self.search_list:
                self.search_list.append(item_id)
                for g in self.guard_list:
                    g.items_to_search_for.append(item_id)
                print("SC: started new search for \"%s\", %d/%d nodes left to search (some may still be in progress)" %(item_id, self.get_num_nodes_to_search(), len(self.guard_list)))
                print("SC: current search list %s" %(str(self.search_list)))
                #place the item at a random guard
                location = random.choice(self.guard_list)
                self.item_location_dict[item_id] = location

            else:
                print("SC: item \"%s\" already being searched for, not adding again" %(item_id))

    #return the guard object to search
    def get_guard_to_search(self, robot_pos):
        """
        Returns the nearest guard object that still needs to be searched
        :param robot_pos: an iterable containing [robot_x, robot_y]
        :return: The guard object, its [x,y] position
        """
        closest_guard = None
        shortest_dist = float('inf')
        for g in self.guard_list:
            d = g.get_distance(robot_pos[0], robot_pos[1])
            # print("guard: " + str(g) + " need searching? " + str(g.needs_searching()))
            if (d < shortest_dist) and (g.needs_searching()):
                shortest_dist = d
                closest_guard = g

        if(closest_guard is not None):
            closest_guard.being_searched = True
            return closest_guard, [closest_guard.x, closest_guard.y] #resize_to_orig(self.map_scale, closest_guard.position)

        print("SC: no nodes to search for, returning None")
        return None, None


    def get_closest_guard(self, robot_pos):
        closest_guard = None
        shortest_dist = float('inf')
        for g in self.guard_list:
            d = g.get_distance(robot_pos[0], robot_pos[1])
            # print("guard: " + str(g) + " need searching? " + str(g.needs_searching()))
            if (d < shortest_dist) and (g.needs_searching()):
                shortest_dist = d
                closest_guard = g

        return closest_guard


    def mark_guard_searched(self, guard, items_found=[]):

        items = []
        items = items + items_found
        
        # this is the temp check for the item at the randomly selected guard
        for item in self.item_location_dict:
            if self.item_location_dict[item] == guard:
                items.append(item)

        
        for item in items:
            self.search_list.remove(item)
            del self.item_location_dict[item]
            for g in self.guard_list:
                if item in g.items_to_search_for:
                    g.items_to_search_for.remove(item)


        guard.items_to_search_for = []
        guard.being_searched = False

        print("SC: found \"%s\" at node, %d/%d nodes left to search (some may still be in progress)" %(items, self.get_num_nodes_to_search(), len(self.guard_list)))

    def reassign_guard(self, guard):
        guard.being_searched = False
        print("SC: node set to be reassigned, %d/%d nodes left to search (some may still be in progress)" %(self.get_num_nodes_to_search(), len(self.guard_list)))
        

'''
MAIN - for testing, not ROS 
'''

if __name__ == "__main__":
    sc = SearchCoordinator("HospitalMapCleaned_filledin_black_border_clean2.png",0.3)
    sc.draw_guards()

    # Start search for scissors
    items_list = ["scissors","advil","bandages","advil"]

    sc.start_search(items_list)
    
    # Robot1 gets a guard to search
    g1,p = sc.get_guard_to_search((50,100))
    #print("\nItems for robot1 to search for at guard %s" %(str(g1.position)))
    print(g1.items_to_search_for)
    
    # Robot2 gets a guard to search
    g2,p = sc.get_guard_to_search((50,100)) #robots started at the same spot
    #print("\nItems for robot2 to search for at guard %s" %(str(g2.position)))
    print(g2.items_to_search_for)
    
    # Robot3 gets a guard to search
    g3,p = sc.get_guard_to_search((50,100)) #robots started at the same spot
    #print("\nItems for robot3 to search for at guard %s" %(str(g3.position)))
    print(g3.items_to_search_for)
    
    # Robot1 finds nothing
    sc.mark_guard_searched(g1)
    #print("\nItems for to search for at guard %s" %(str(g1.position)))
    print(g1.items_to_search_for)
    
    # Robot2 finds scissors
    sc.mark_guard_searched(g2, ["scissors"])
    print("\nSearch List")
    print(sc.search_list)
    #print("Items for to search for at guard %s" %(str(g2.position)))
    print(g2.items_to_search_for)
    
    # Robot3 needs to go recharge and can't keep searching
    sc.reassign_guard(g3)
    print("\nDoes guard3 need to be searched still:")
    print(g3.needs_searching())