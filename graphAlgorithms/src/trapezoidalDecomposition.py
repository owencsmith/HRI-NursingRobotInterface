import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import math


class TrapezoidalDecomposition:

    def __init__(self,map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]
        self.graph = nx.Graph()

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        string_ints = [str(i) for i in range(len(self.samples))]
        pos = dict(zip(range(len(self.samples)), node_pos))
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y', ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12, node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12, node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()


    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''

        # get the difference in the row and column values
        diff_row = p2[0] - p1[0]
        diff_col = p2[1] - p1[1]

        # in this implementation, I find which difference is larger.
        # the larger difference becomes the increment by 1, and the smaller
        # one is a fraction of that increment.  This allows the check to move
        # along the hypotenuse between 2 points (the edge) but travel in increments
        # of 1 since the graph's occupancy exists in a discrete space not continuous

        if abs(diff_col) > abs(diff_row):
            inc_total = abs(diff_col)
            increment_row_by = diff_row / abs(diff_col)
            increment_col_by = math.copysign(1, diff_col)
        elif abs(diff_col) < abs(diff_row):
            inc_total = abs(diff_row)
            increment_row_by = math.copysign(1, diff_row)
            increment_col_by = diff_col / abs(diff_row)
        else:
            inc_total = abs(diff_row)
            increment_row_by = math.copysign(1, diff_row)
            increment_col_by = math.copysign(1, diff_col)

        # increments through the path from one node to the other
        for i in range(inc_total):
            check_row = p1[0] + i * increment_row_by
            check_col = p1[1] + i * increment_col_by

            # if any of the increments are an obstacle, return false if there is a collision
            if self.map_array[int(check_row), int(check_col)] == 0:
                return False

    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''

        




        

