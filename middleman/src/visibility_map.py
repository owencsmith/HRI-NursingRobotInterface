import numpy as np
import networkx as nx
from scipy import spatial

class VisibilityMap:

    def __init__(self, map, nodes):

        self.map = map
        self.nodes = nodes
        self.graph = nx.Graph()

        self.formatted_nodes = []; i = 0
        for n in self.nodes:
            self.formatted_nodes.append((i, {"pos": n}))
            i = i+1

        self.edges = []
        for n1 in self.formatted_nodes:
            for n2 in self.formatted_nodes:
                if ( (not self.check_collision(n1[1]["pos"],n2[1]["pos"])) and
                     (not (n1 == n2) ) ):
                    self.edges.append((n1[0], n2[0], self.dist(n1[1]["pos"],n2[1]["pos"])))

        self.graph.add_nodes_from(self.formatted_nodes)
        self.graph.add_weighted_edges_from(self.edges)


    def dist(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]
        return:
            euclidean distance between two points
        '''
        
        return np.sqrt(np.power(point1[0]-point2[0],2)+np.power(point1[1]-point2[1],2))

    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]
        return:
            True if there are obstacles between two points
        '''
        
        b = self.bresenham(p1[1],p1[0],p2[1],p2[0])
        for p in b:
            if self.map[p[1],p[0]] == 0:
                return True

        return False

    def bresenham(self, x0, y0, x1, y1):  
        '''
        Bresenham line algorithm
        http://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#Python
        '''

        pixel_list = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                pixel_list.append((x,y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                pixel_list.append((x,y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy        
        pixel_list.append((x,y))

        return pixel_list


        