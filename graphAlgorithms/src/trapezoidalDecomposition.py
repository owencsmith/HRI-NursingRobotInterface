import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import math

class Trapezoid:

    def __init__(self, coord1, coord2, coord3, coord4):
        self.coord1 = coord1 # tuple of (row, col)
        self.coord2 = coord2
        self.coord3 = coord3
        self.coord4 = coord4

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

    def create_trapezoids(self):

        obstacles_dict = dict()
        line_list_for_boundaries = list()

        for r in range(self.size_row-1):
            c=0
            while c < self.size_col: #for c in range(self.size_col-1):
                print("row: " + str(r) + " col: "+ str(c))
                if c!=0 and c!=self.size_col-1:
                    if self.map_array[r,c-1] == 1 and self.map_array[r,c] == 0:
                        this_line_list = list()
                        this_line_list.append((r, c))
                        last_sub_col = 0
                        for sub_col in range(c, self.size_col):
                            if self.map_array[r, sub_col] == 1: #used to be 1 !!!!!!!!!!!!! working kinda
                                if obstacles_dict.get(r) is None:
                                    new_list = list()
                                    new_list.append(this_line_list)
                                    obstacles_dict[r] = new_list
                                else:
                                    existing_list = obstacles_dict.get(r)
                                    existing_list.append(this_line_list)
                                    obstacles_dict[r] = existing_list
                                last_sub_col = sub_col
                                break
                            else:
                                this_line_list.append((r,sub_col))

                            if sub_col==self.size_col-1 and last_sub_col==0:
                                last_sub_col=sub_col

                        # if len(this_line_list)>0:
                        if len(this_line_list) > 1:
                            this_line_first_pt = this_line_list[0]
                            this_line_last_pt = this_line_list[-1]
                            obs_above = False
                            obs_below = True
                            if obstacles_dict.get(r-1) is not None:
                                obs_list = obstacles_dict.get(r-1)
                                for obs in obs_list:
                                    if len(obs)>1:
                                        first_pt = obs[0]
                                        last_pt = obs[-1]

                                        if first_pt[1] == this_line_first_pt[1] and last_pt[1] == this_line_last_pt[1]:
                                            obs_above = True
                                            break

                            # else:
                            #     this_line_first_pt = this_line_list.pop(0)
                            #     obs_above = False
                            #     obs_below = True
                            #     if obstacles_dict.get(r - 1) is not None:
                            #         obs_list = obstacles_dict.get(r - 1)
                            #         for obs in obs_list:
                            #             first_pt = obs[0]
                            #
                            #             if first_pt == this_line_first_pt:
                            #                 obs_above = True
                            #                 break

                            if r<self.size_row:
                                for pt in this_line_list:
                                    pt_col = pt[1]
                                    if self.map_array[r+1, pt_col] == 0:
                                        obs_below = False
                                        break


                            if (not obs_above) or (obs_above and not obs_below):
                                left_boundary_line = list()
                                right_boundary_line = list()
                                # make line left and right
                                for line_column in range(c,0,-1):
                                    if self.map_array[r,line_column] != 0:
                                        left_boundary_line.append((r,line_column))
                                    else:
                                        break

                                for line_column in range(last_sub_col,self.size_col):
                                    if self.map_array[r,line_column] != 0:
                                        right_boundary_line.append((r,line_column))
                                    else:
                                        break

                                if len(left_boundary_line) != 0:
                                    line_list_for_boundaries.append(left_boundary_line)
                                if len(right_boundary_line) != 0:
                                    line_list_for_boundaries.append(right_boundary_line)

                        # elif obs_above and not obs_below:

                        c = last_sub_col
                c+=1


        return line_list_for_boundaries



        # Dict for obstacles - key is row, value is list of obstacles
        # Obstacles are lists of points (in a line bc its row by row)
        # Lines are lists of points
        #
        # Create obstacles
        #
        # Go row by row
        #     If a cell is an obstacle
        #       record points until switches back to free space.
        #       Add that obstacle to obstacle dict with row as key
        #
        #       Check row above it for SAME COLUMN obstacle -- as in same start and end points (pop(0) and pop() from list for first and last)
        #
        #       If above it obstacle does not exist
        #           Extend line left and right of the obstacle until hits next obstacle - record line pts
        #       Else if above obstacle exists but below doesnt (check map array ahead)
        #           Extend line left and right of the obstacle until hits next obstacle - record line pts
        #
        #        Now in for loop jump ahead to current col being the col at the end of the obstacle
        #


import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import math


class Trapezoid:

    def __init__(self, coord1, coord2, coord3, coord4):
        self.coord1 = coord1  # tuple of (row, col)
        self.coord2 = coord2
        self.coord3 = coord3
        self.coord4 = coord4


class TrapezoidalDecomposition:

    def __init__(self, map_array):
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
        queue = list()
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

        return (mid_pt_row, mid_pt_col), visited





        #     c = 0
        #     while c < self.size_col:  # for c in range(self.size_col-1):
        #         print("row: " + str(r) + " col: " + str(c))
        #         if c != 0 and c != self.size_col - 1:
        #             if self.map_array[r, c - 1] == 1 and self.map_array[r, c] == 0:
        #                 this_line_list = list()
        #                 this_line_list.append((r, c))
        #                 last_sub_col = 0
        #                 for sub_col in range(c, self.size_col):
        #                     if self.map_array[r, sub_col] == 1:  # used to be 1 !!!!!!!!!!!!! working kinda
        #                         if obstacles_dict.get(r) is None:
        #                             new_list = list()
        #                             new_list.append(this_line_list)
        #                             obstacles_dict[r] = new_list
        #                         else:
        #                             existing_list = obstacles_dict.get(r)
        #                             existing_list.append(this_line_list)
        #                             obstacles_dict[r] = existing_list
        #                         last_sub_col = sub_col
        #                         break
        #                     else:
        #                         this_line_list.append((r, sub_col))
        #
        #                     if sub_col == self.size_col - 1 and last_sub_col == 0:
        #                         last_sub_col = sub_col
        #
        #                 # if len(this_line_list)>0:
        #                 if len(this_line_list) > 1:
        #                     this_line_first_pt = this_line_list[0]
        #                     this_line_last_pt = this_line_list[-1]
        #                     obs_above = False
        #                     obs_below = True
        #                     if obstacles_dict.get(r - 1) is not None:
        #                         obs_list = obstacles_dict.get(r - 1)
        #                         for obs in obs_list:
        #                             if len(obs) > 1:
        #                                 first_pt = obs[0]
        #                                 last_pt = obs[-1]
        #
        #                                 if first_pt[1] == this_line_first_pt[1] and last_pt[1] == this_line_last_pt[1]:
        #                                     obs_above = True
        #                                     break
        #
        #                     # else:
        #                     #     this_line_first_pt = this_line_list.pop(0)
        #                     #     obs_above = False
        #                     #     obs_below = True
        #                     #     if obstacles_dict.get(r - 1) is not None:
        #                     #         obs_list = obstacles_dict.get(r - 1)
        #                     #         for obs in obs_list:
        #                     #             first_pt = obs[0]
        #                     #
        #                     #             if first_pt == this_line_first_pt:
        #                     #                 obs_above = True
        #                     #                 break
        #
        #                     if r < self.size_row:
        #                         for pt in this_line_list:
        #                             pt_col = pt[1]
        #                             if self.map_array[r + 1, pt_col] == 0:
        #                                 obs_below = False
        #                                 break
        #
        #                     if (not obs_above) or (obs_above and not obs_below):
        #                         left_boundary_line = list()
        #                         right_boundary_line = list()
        #                         # make line left and right
        #                         for line_column in range(c, 0, -1):
        #                             if self.map_array[r, line_column] != 0:
        #                                 left_boundary_line.append((r, line_column))
        #                             else:
        #                                 break
        #
        #                         for line_column in range(last_sub_col, self.size_col):
        #                             if self.map_array[r, line_column] != 0:
        #                                 right_boundary_line.append((r, line_column))
        #                             else:
        #                                 break
        #
        #                         if len(left_boundary_line) != 0:
        #                             line_list_for_boundaries.append(left_boundary_line)
        #                         if len(right_boundary_line) != 0:
        #                             line_list_for_boundaries.append(right_boundary_line)
        #
        #                 # elif obs_above and not obs_below:
        #
        #                 c = last_sub_col
        #         c += 1
        #
        # return line_list_for_boundaries

        # Dict for obstacles - key is row, value is list of obstacles
        # Obstacles are lists of points (in a line bc its row by row)
        # Lines are lists of points
        #
        # Create obstacles
        #
        # Go row by row
        #     If a cell is an obstacle
        #       record points until switches back to free space.
        #       Add that obstacle to obstacle dict with row as key
        #
        #       Check row above it for SAME COLUMN obstacle -- as in same start and end points (pop(0) and pop() from list for first and last)
        #
        #       If above it obstacle does not exist
        #           Extend line left and right of the obstacle until hits next obstacle - record line pts
        #       Else if above obstacle exists but below doesnt (check map array ahead)
        #           Extend line left and right of the obstacle until hits next obstacle - record line pts
        #
        #        Now in for loop jump ahead to current col being the col at the end of the obstacle
        #

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


    # def create_trapezoids(self):
    #
    #     obstacles_dict = dict()
    #     line_list_for_boundaries = list()
    #
    #     for r in range(self.size_row - 1):
    #         c = 0
    #         while c < self.size_col:  # for c in range(self.size_col-1):
    #             print("row: " + str(r) + " col: " + str(c))
    #             if c != 0 and c != self.size_col - 1:
    #                 if self.map_array[r, c - 1] == 1 and self.map_array[r, c] == 0:
    #                     this_line_list = list()
    #                     this_line_list.append((r, c))
    #                     last_sub_col = 0
    #                     for sub_col in range(c, self.size_col):
    #                         if self.map_array[r, sub_col] == 1:  # used to be 1 !!!!!!!!!!!!! working kinda
    #                             if obstacles_dict.get(r) is None:
    #                                 new_list = list()
    #                                 new_list.append(this_line_list)
    #                                 obstacles_dict[r] = new_list
    #                             else:
    #                                 existing_list = obstacles_dict.get(r)
    #                                 existing_list.append(this_line_list)
    #                                 obstacles_dict[r] = existing_list
    #                             last_sub_col = sub_col
    #                             break
    #                         else:
    #                             this_line_list.append((r, sub_col))
    #
    #                         if sub_col == self.size_col - 1 and last_sub_col == 0:
    #                             last_sub_col = sub_col
    #
    #                     # if len(this_line_list)>0:
    #                     if len(this_line_list) > 1:
    #                         this_line_first_pt = this_line_list[0]
    #                         this_line_last_pt = this_line_list[-1]
    #                         obs_above = False
    #                         obs_below = True
    #                         if obstacles_dict.get(r - 1) is not None:
    #                             obs_list = obstacles_dict.get(r - 1)
    #                             for obs in obs_list:
    #                                 if len(obs) > 1:
    #                                     first_pt = obs[0]
    #                                     last_pt = obs[-1]
    #
    #                                     if first_pt[1] == this_line_first_pt[1] and last_pt[1] == this_line_last_pt[1]:
    #                                         obs_above = True
    #                                         break
    #
    #                         # else:
    #                         #     this_line_first_pt = this_line_list.pop(0)
    #                         #     obs_above = False
    #                         #     obs_below = True
    #                         #     if obstacles_dict.get(r - 1) is not None:
    #                         #         obs_list = obstacles_dict.get(r - 1)
    #                         #         for obs in obs_list:
    #                         #             first_pt = obs[0]
    #                         #
    #                         #             if first_pt == this_line_first_pt:
    #                         #                 obs_above = True
    #                         #                 break
    #
    #                         if r < self.size_row:
    #                             for pt in this_line_list:
    #                                 pt_col = pt[1]
    #                                 if self.map_array[r + 1, pt_col] == 0:
    #                                     obs_below = False
    #                                     break
    #
    #                         if (not obs_above) or (obs_above and not obs_below):
    #                             left_boundary_line = list()
    #                             right_boundary_line = list()
    #                             # make line left and right
    #                             for line_column in range(c, 0, -1):
    #                                 if self.map_array[r, line_column] != 0:
    #                                     left_boundary_line.append((r, line_column))
    #                                 else:
    #                                     break
    #
    #                             for line_column in range(last_sub_col, self.size_col):
    #                                 if self.map_array[r, line_column] != 0:
    #                                     right_boundary_line.append((r, line_column))
    #                                 else:
    #                                     break
    #
    #                             if len(left_boundary_line) != 0:
    #                                 line_list_for_boundaries.append(left_boundary_line)
    #                             if len(right_boundary_line) != 0:
    #                                 line_list_for_boundaries.append(right_boundary_line)
    #
    #                     # elif obs_above and not obs_below:
    #
    #                     c = last_sub_col
    #             c += 1
    #
    #     return line_list_for_boundaries
    #
    #     # Dict for obstacles - key is row, value is list of obstacles
    #     # Obstacles are lists of points (in a line bc its row by row)
    #     # Lines are lists of points
    #     #
    #     # Create obstacles
    #     #
    #     # Go row by row
    #     #     If a cell is an obstacle
    #     #       record points until switches back to free space.
    #     #       Add that obstacle to obstacle dict with row as key
    #     #
    #     #       Check row above it for SAME COLUMN obstacle -- as in same start and end points (pop(0) and pop() from list for first and last)
    #     #
    #     #       If above it obstacle does not exist
    #     #           Extend line left and right of the obstacle until hits next obstacle - record line pts
    #     #       Else if above obstacle exists but below doesnt (check map array ahead)
    #     #           Extend line left and right of the obstacle until hits next obstacle - record line pts
    #     #
    #     #        Now in for loop jump ahead to current col being the col at the end of the obstacle
    #     #

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




