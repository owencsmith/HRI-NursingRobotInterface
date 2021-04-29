import numpy as np
import matplotlib
# import math
import scipy
import copy


class TrapezoidalDecomposition:

    def __init__(self, map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]
        print(self.size_col)
        print(self.size_row)
        print(np.__version__)
        print(scipy.__version__)
        print(matplotlib.__version__)

    def create_trapezoids(self):

        line_list_for_boundaries = list()
        vertices = list()
        count = 0
        count2 = 0
        for r in range(1, self.size_row - 1):
            for c in range(1, self.size_col - 1):
                if self.map_array[r, c] == 0:
                    count += 1
                    if self.is_vertex((r, c), 0, self.size_row, self.size_col):
                        count2 += 1
                        vertices.append((r, c))
        print("If count1: " + str(count) + " If count2: " + str(count2))
        # return vertices

        for vertice in vertices:
            # get left and right lines from vertices
            row = vertice[0]
            col = vertice[1]
            left_line = list()
            right_line = list()
            # left
            if col != 0:
                for c in range(col - 1, -1, -1):
                    if self.map_array[row, c] == 1:
                        left_line.append((row, c))
                    else:
                        line_list_for_boundaries.append(left_line)
                        break

            if col != self.size_col - 1:
                for c in range(col + 1, self.size_col):
                    if self.map_array[row, c] == 1:
                        right_line.append((row, c))
                    else:
                        line_list_for_boundaries.append(right_line)
                        break

        self.vertices = vertices
        self.line_list_for_boundaries = line_list_for_boundaries
        # return vertices, line_list_for_boundaries

    def find_centers(self):

        # Find the vertuces and edges of the trapezoids
        self.create_trapezoids()
        print("Trap Decomp Vertices: " + str(len(self.vertices)))

        # Draw the trapezoid boundaries in the map
        updated_map_array = copy.deepcopy(self.map_array)
        for line in self.line_list_for_boundaries:
            for pt in line:
                updated_map_array[pt[0], pt[1]] = 0.5

        # List for visited points and center points
        visited = list()
        centers = list()

        # For each point that is not visited use bfs to find the entire area and return the center
        for r in range(1, self.size_row - 1):
            for c in range(1, self.size_col - 1):
                if updated_map_array[r, c] == 1 and [r, c] not in visited:
                    q = list()
                    q.append([r, c])
                    center, visited = self.iterative_bfs(updated_map_array, q, visited, 0, self.size_row - 1,
                                                         self.size_col - 1)
                    if center is not None:
                        centers.append((int(center[0]), int(center[1])))

        self.centers = centers
        return self.centers

    def iterative_bfs(self, grid, queue, visited, min_dimension, max_dimension_row,
                      max_dimension_col):

        this_bfs_visited = list()

        while queue:
            # FIFO popping
            node = queue.pop(0)
            visited.append(node)
            this_bfs_visited.append(node)

            queue, parent_dict = self.four_connected(node, visited, min_dimension, max_dimension_row, max_dimension_col,
                                                     queue, grid, dict())

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

        mid_pt_row = (min_row + max_row) / 2
        mid_pt_col = (min_col + max_col) / 2

        if min_col == min_dimension:
            return None, visited
        elif max_col == self.size_col - 1:
            return None, visited
        elif min_row == min_dimension:
            return None, visited
        elif max_row == self.size_row - 1:
            return None, visited
        else:
            return (mid_pt_row, mid_pt_col), visited

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
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

            node_to_add = [min_dimension + 1, min_dimension]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1

            node_to_add = [min_dimension + 1, min_dimension + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1


        elif node[0] == max_dimension_row and node[1] == max_dimension_col:

            node_to_add = [max_dimension_row, max_dimension_col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [max_dimension_row - 1, max_dimension_col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [max_dimension_row - 1, max_dimension_col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1


        elif node[0] == min_dimension and node[1] == max_dimension_col:

            node_to_add = [min_dimension + 1, max_dimension_col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1

            node_to_add = [min_dimension + 1, max_dimension_col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [min_dimension, max_dimension_col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1


        elif node[0] == max_dimension_row and node[1] == min_dimension:

            node_to_add = [max_dimension_row, min_dimension + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

            node_to_add = [max_dimension_row - 1, min_dimension + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

            node_to_add = [max_dimension_row - 1, min_dimension]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1


        elif node[0] == min_dimension:

            node_to_add = [row, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

            node_to_add = [row + 1, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

            node_to_add = [row + 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1

            node_to_add = [row + 1, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [row, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1


        elif node[1] == min_dimension:

            node_to_add = [row, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

            node_to_add = [row + 1, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

            node_to_add = [row + 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1

            node_to_add = [row - 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1

            node_to_add = [row - 1, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1


        elif node[0] == max_dimension_row:

            node_to_add = [row, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

            node_to_add = [row, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [row - 1, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [row - 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1

            node_to_add = [row - 1, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1


        elif node[1] == max_dimension_col:

            node_to_add = [row + 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1

            node_to_add = [row + 1, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [row, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [row - 1, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [row - 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1


        else:

            node_to_add = [row, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

            node_to_add = [row + 1, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

            node_to_add = [row + 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1

            node_to_add = [row + 1, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [row, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [row - 1, col - 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                left_free_counter += 1
            else:
                obstacle_counter += 1
                left_obstacle_counter += 1

            node_to_add = [row - 1, col]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                vert_free_counter += 1
            else:
                obstacle_counter += 1
                vert_obstacle_counter += 1

            node_to_add = [row - 1, col + 1]
            if self.map_array[node_to_add[0], node_to_add[1]] == 1:
                free_counter += 1
                right_free_counter += 1
            else:
                obstacle_counter += 1
                right_obstacle_counter += 1

        if abs(right_free_counter - left_free_counter) > 1:
            # if right_free_counter!=left_free_counter:
            ratio = float(free_counter) / float((free_counter + obstacle_counter))
            if ratio >= 0.5:
                return True
            else:
                return False
        else:
            return False

    def four_connected(self, node, visited, min_dimension, max_dimension_row, max_dimension_col, queue, grid,
                       parent_dict):

        row = node[0]
        col = node[1]

        if node[0] == min_dimension and node[1] == min_dimension:

            node_to_add = [min_dimension, min_dimension + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)

            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [min_dimension + 1, min_dimension]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[0] == max_dimension_row and node[1] == max_dimension_col:
            node_to_add = [max_dimension_row, max_dimension_col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [max_dimension_row - 1, max_dimension_col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[0] == min_dimension and node[1] == max_dimension_col:

            node_to_add = [min_dimension + 1, max_dimension_col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [min_dimension, max_dimension_col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[0] == max_dimension_row and node[1] == min_dimension:

            node_to_add = [max_dimension_row, min_dimension + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [max_dimension_row - 1, min_dimension]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[0] == min_dimension:

            node_to_add = [row, col + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row + 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row, col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[1] == min_dimension:

            node_to_add = [row, col + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row + 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row - 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[0] == max_dimension_row:

            node_to_add = [row, col + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row, col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row - 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        elif node[1] == max_dimension_col:
            node_to_add = [row + 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row, col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row - 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node


        else:

            node_to_add = [row, col + 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row + 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row, col - 1]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

            node_to_add = [row - 1, col]
            if node_to_add not in queue and grid[node_to_add[0]][node_to_add[1]] == 1 and node_to_add not in visited:
                parent_dict[str(node_to_add)] = node
                queue.append(node_to_add)
            # elif node_to_add in queue:
            #     parent_dict[str(node_to_add)] = node

        return queue, parent_dict
