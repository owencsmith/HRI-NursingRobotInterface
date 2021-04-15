import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import math
from scipy import spatial


class TrapezoidalDecomposition:

    def __init__(self, map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]
        self.graph = nx.Graph()
        self.samples = list()

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
        pos = dict(zip(range(len(self.samples)), node_pos))
        # pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        # pos['goal'] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y', ax=ax)

        # If found a path
        # if self.path:
        #     # add temporary start and goal edge to the path
        #     final_path_edge = list(zip(self.path[:-1], self.path[1:]))
        #     nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
        #     nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        # nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12, node_color='g')
        # nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12, node_color='r')

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
            True if two points can be connected
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
        for i in range(int(inc_total)):
            check_row = p1[0] + i * increment_row_by
            check_col = p1[1] + i * increment_col_by

            # if any of the increments are an obstacle, return false if there is a collision
            if self.map_array[int(check_row), int(check_col)] == 0:
                return False

        return True

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

    def create_trapezoids(self):

        obstacles_dict = dict()
        line_list_for_boundaries = list()
        vertices = list()

        for r in range(1,self.size_row-1):
            for c in range(1,self.size_col-1):
                if self.map_array[r,c] == 0:
                    if self.is_vertex((r,c),0,self.size_row,self.size_col):
                        vertices.append((r,c))

        # return vertices

        for vertice in vertices:
            # get left and right lines from vertices
            row = vertice[0]
            col = vertice[1]
            left_line = list()
            right_line = list()
            #left
            if col!=0:
                for c in range(col-1,-1,-1):
                    if self.map_array[row,c] == 1:
                        left_line.append((row,c))
                    else:
                        line_list_for_boundaries.append(left_line)
                        break

            if col!=self.size_col-1:
                for c in range(col+1,self.size_col):
                    if self.map_array[row,c] == 1:
                        right_line.append((row,c))
                    else:
                        line_list_for_boundaries.append(right_line)
                        break

        return vertices, line_list_for_boundaries


    def find_centers(self, updated_map_array):

        self.map_array = updated_map_array

        visited = list()
        centers = list()

        for r in range(1, self.size_row - 1):
            for c in range(1, self.size_col - 1):
                if self.map_array[r,c] == 1 and [r,c] not in visited:
                    q = list()
                    q.append([r,c])
                    center, visited = self.iterative_bfs(self.map_array,q,visited,0,self.size_row-1,self.size_col-1)
                    centers.append(center)

        return centers

    def iterative_bfs(self, grid, queue, visited, min_dimension, max_dimension_row,
                      max_dimension_col):

        this_bfs_visited = list()

        while queue:

            # FIFO popping
            node = queue.pop(0)
            visited.append(node)
            this_bfs_visited.append(node)

            queue, parent_dict = self.four_connected(node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, grid, dict())

        min_row = self.size_row
        min_col = self.size_col
        max_row = 0
        max_col = 0

        for point in this_bfs_visited:
            if point[0] < min_row:
                min_row = point[0]
            if point[0] > max_row:
                max_row = point[0]
            if point[1] < min_col:
                min_col = point[1]
            if point[1] > max_col:
                max_col = point[1]

        mid_pt_row = (min_row+max_row)/2
        mid_pt_col = (min_col+max_col)/2

        if min_col==min_dimension:
            return None, visited
        elif max_col==self.size_col-1:
            return None,visited
        elif min_row==min_dimension:
            return None, visited
        elif max_row==self.size_row-1:
            return None, visited
        else:
            return (mid_pt_row, mid_pt_col), visited


    # def create_visibility_graph(self, centers):
    #
    #     weighted_pairs = list()
    #
    #     for check_node_index in range(len(centers)):
    #         for other_index in range(len(centers)):
    #             if check_node_index!=other_index and centers[check_node_index] is not None and centers[other_index] is not None:
    #                 if self.check_collision(centers[check_node_index],centers[other_index]):
    #                     already_in_pairs = False
    #                     for pair in weighted_pairs:
    #                         if (pair[0] == centers[check_node_index] and pair[1] == centers[other_index]) or (pair[1] == centers[check_node_index] and pair[0] == centers[other_index]):
    #                             already_in_pairs = True
    #
    #                     if not already_in_pairs:
    #                         weighted_pairs.append((centers[check_node_index],centers[other_index],self.dis(centers[check_node_index],centers[other_index])))


    def create_visibility_graph(self, centers):

        for c in centers:
            if c is not None:
                self.samples.append(c)

        points = np.array(self.samples)

        kdtree = spatial.KDTree(points)
        pairs_initial = kdtree.query_pairs(math.sqrt(self.size_row**2+self.size_col**2))
        pairs = []

        for (i, j) in pairs_initial:
            p1_i = self.samples[i]
            p2_i = self.samples[j]
            if self.check_collision(p1_i, p2_i):
                pairs.append((i, j, self.dis(p1_i, p2_i)))

        points_id = range(0, len(points))
        self.graph.add_nodes_from(points_id)
        self.graph.add_weighted_edges_from(pairs)


    def clear_lines_from_map(self, origional_map_array):
        # map = self.map_array
        # for r in range(self.size_row):
        #     for c in range(self.size_col):
        #         if map[r,c] == 0.5:
        #             map[r,c] = 1.0
        #
        # self.map_array = map

        self.map_array = origional_map_array


    def is_vertex(self, node, min_dimension, max_dimension_row, max_dimension_col):

        row = node[0]
        col = node[1]
        free_counter = 0
        obstacle_counter = 0

        right_free_counter = 0
        right_obstacle_counter = 0
        left_free_counter = 0
        left_obstacle_counter = 0
        vert_free_counter = 0
        vert_obstacle_counter = 0

        if node[0] == min_dimension and node[1] == min_dimension:

            node_to_add = [min_dimension, min_dimension + 1]
            if self.map_array[node_to_add[0],node_to_add[1]] == 1:
                free_counter+=1
                right_free_counter+=1
            else:
                obstacle_counter+=1
                right_obstacle_counter+=1

            node_to_add = [min_dimension + 1, min_dimension]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1

            node_to_add = [min_dimension+1, min_dimension+1]
            if self.map_array[node_to_add[0],node_to_add[1]] == 1:
                free_counter+=1
                right_free_counter+=1
            else:
                obstacle_counter+=1
                right_obstacle_counter+=1


        elif node[0] == max_dimension_row and node[1] == max_dimension_col:

            node_to_add = [max_dimension_row, max_dimension_col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1

            node_to_add = [max_dimension_row-1, max_dimension_col-1]
            if self.map_array[node_to_add[0],node_to_add[1]] == 1:
                free_counter+=1
                left_free_counter+=1
            else:
                obstacle_counter+=1
                left_obstacle_counter+=1

            node_to_add = [max_dimension_row - 1, max_dimension_col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1


        elif node[0] == min_dimension and node[1] == max_dimension_col:

            node_to_add = [min_dimension + 1, max_dimension_col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1

            node_to_add = [min_dimension+1, max_dimension_col-1]
            if self.map_array[node_to_add[0],node_to_add[1]] == 1:
                free_counter+=1
                left_free_counter+=1
            else:
                obstacle_counter+=1
                left_obstacle_counter+=1

            node_to_add = [min_dimension, max_dimension_col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1


        elif node[0] == max_dimension_row and node[1] == min_dimension:

            node_to_add = [max_dimension_row, min_dimension + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter+=1
            else:
                obstacle_counter += 1
                right_obstacle_counter+=1

            node_to_add = [max_dimension_row-1, min_dimension+1]
            if self.map_array[node_to_add[0],node_to_add[1]] == 1:
                free_counter+=1
                right_free_counter+=1
            else:
                obstacle_counter+=1
                right_obstacle_counter+=1

            node_to_add = [max_dimension_row - 1, min_dimension]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1


        elif node[0] == min_dimension:

            node_to_add = [row, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter+=1
            else:
                obstacle_counter += 1
                right_obstacle_counter+=1

            node_to_add = [row+1, col+1]
            if self.map_array[node_to_add[0],node_to_add[1]] == 1:
                free_counter+=1
                right_free_counter+=1
            else:
                obstacle_counter+=1
                right_obstacle_counter+=1

            node_to_add = [row + 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1

            node_to_add = [row+1, col-1]
            if self.map_array[node_to_add[0],node_to_add[1]] == 1:
                free_counter+=1
                left_free_counter+=1
            else:
                obstacle_counter+=1
                left_obstacle_counter+=1

            node_to_add = [row, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1


        elif node[1] == min_dimension:

            node_to_add = [row, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter+=1
            else:
                obstacle_counter += 1
                right_obstacle_counter+=1

            node_to_add = [row+1,col+1]
            if self.map_array[node_to_add[0],node_to_add[1]] == 1:
                free_counter+=1
                right_free_counter+=1
            else:
                obstacle_counter+=1
                right_obstacle_counter+=1

            node_to_add = [row + 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1

            node_to_add = [row - 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1

            node_to_add = [row - 1, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter+=1
            else:
                obstacle_counter += 1
                right_obstacle_counter+=1


        elif node[0] == max_dimension_row:

            node_to_add = [row, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter+=1
            else:
                obstacle_counter += 1
                right_obstacle_counter+=1

            node_to_add = [row, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1

            node_to_add = [row-1,col-1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1

            node_to_add = [row - 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1

            node_to_add = [row-1,col+1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter+=1
            else:
                obstacle_counter += 1
                right_obstacle_counter+=1


        elif node[1] == max_dimension_col:

            node_to_add = [row + 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1

            node_to_add = [row+1,col-1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1

            node_to_add = [row, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1

            node_to_add = [row-1,col-1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1

            node_to_add = [row - 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1


        else:

            node_to_add = [row, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter+=1
            else:
                obstacle_counter += 1
                right_obstacle_counter+=1

            node_to_add = [row+1,col+1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter+=1
            else:
                obstacle_counter += 1
                right_obstacle_counter+=1

            node_to_add = [row + 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1

            node_to_add = [row+1,col-1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1

            node_to_add = [row, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1

            node_to_add = [row-1,col-1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter+=1
            else:
                obstacle_counter += 1
                left_obstacle_counter+=1

            node_to_add = [row - 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter+=1
            else:
                obstacle_counter += 1
                vert_obstacle_counter+=1

            node_to_add = [row-1,col+1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter+=1
            else:
                obstacle_counter += 1
                right_obstacle_counter+=1


        if abs(right_free_counter-left_free_counter)>1:
        # if right_free_counter!=left_free_counter:
            ratio = free_counter/(free_counter+obstacle_counter)
            if ratio>=0.5:
                return True
            else:
                return False
        else:
            return False


    def four_connected(self, node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, grid, parent_dict):

        row = node[0]
        col = node[1]

        if node[0] == min_dimension and node[1] == min_dimension:

            node_to_add = [min_dimension, min_dimension + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)

            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [min_dimension + 1, min_dimension]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[0] == max_dimension_row and node[1] == max_dimension_col:
            node_to_add = [max_dimension_row, max_dimension_col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [max_dimension_row - 1, max_dimension_col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[0] == min_dimension and node[1] == max_dimension_col:

            node_to_add = [min_dimension + 1, max_dimension_col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [min_dimension, max_dimension_col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[0] == max_dimension_row and node[1] == min_dimension:

            node_to_add = [max_dimension_row, min_dimension + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [max_dimension_row - 1, min_dimension]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[0] == min_dimension:

            node_to_add = [row, col + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row + 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row, col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[1] == min_dimension:

            node_to_add = [row, col + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row + 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row - 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[0] == max_dimension_row:

            node_to_add = [row, col + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row, col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row - 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[1] == max_dimension_col:
            node_to_add = [row + 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row, col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row - 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        else:

            node_to_add = [row, col + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row + 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row, col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row - 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] ==1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

        return queue, parent_dict




