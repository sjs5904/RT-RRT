import math
import random

import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, pi

class RRT:

    class Node:

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.parent = None

        def path_x(self):
            if self.parent:
                return [self.x, self.parent.x]
            else:
                return []

        def path_y(self):
            if self.parent:
                return [self.y, self.parent.y]
            else:
                return []

    class Edge:

        def __init__(self, fnode, tnode):
            self.fromx = fnode.x
            self.fromy = fnode.y
            self.tox = tnode.x
            self.toy = tnode.y

    def __init__(self, start, goal, obstacle_list, wall_list, rand_area,
                 expand_dis=0.2, goal_sample_rate=5, max_iter=500, node_list = []):
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_randx = rand_area[0]
        self.max_randx = rand_area[1]
        self.min_randy = rand_area[2]
        self.max_randy = rand_area[3]
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.wall_list = wall_list
        self.node_list = node_list

    def planning(self):

        self.node_list = [self.start]
        for i in range(self.max_iter):
            
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list, self.wall_list):
                self.node_list.append(new_node)

            if i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end, self.expand_dis)
                if self.check_collision(final_node, self.obstacle_list, self.wall_list):
                    return self.generate_final_course(len(self.node_list) - 1)

            if i % 5:
                self.draw_graph(rnd_node)

        return None

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        if extend_length > d:
            extend_length = d
        
        new_node.x += extend_length * math.cos(theta)
        new_node.y += extend_length * math.sin(theta)

        new_node.parent = from_node
        
        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        
        return path
    
    def generate_final_course2(self, goal_ind):
        path = []
        node = self.node_list[goal_ind]
        while node.parent:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])
        
        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) >= self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_randx, self.max_randx),
                            random.uniform(self.min_randy, self.max_randy))
        else:
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x(), node.path_y(), "-g")
        
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        for (sx,sy,ex,ey) in self.wall_list:
            x1, y1 = [sx, ex], [sy, ey]
            plt.plot(x1, y1, "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([self.min_randx, self.max_randx, self.min_randy, self.max_randy])
        plt.grid(True)
        plt.pause(0.001)

    @staticmethod
    def plot_circle(x, y, size, color="-k"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    def getAngle(self, a, b, c):
        
        ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
        if ang < 0:
            ang = ang + 180

        if ang < 0:
            ang = ang + 180

        if ang > 180:
            ang = ang - 180

        if ang > 180:
            ang = ang - 180
            
        return ang

    def get_nearest_point_on_edge(self, edge, node):
        a = edge.fromy - edge.toy
        b = -(edge.fromx - edge.tox)
        c = edge.fromx*edge.toy - edge.tox*edge.fromy
        if self.getAngle((edge.fromx,edge.fromy),(edge.tox,edge.toy),(node.x,node.y)) < 90 and self.getAngle((node.x,node.y),(edge.fromx,edge.fromy),(edge.tox,edge.toy)) < 90:
            temp = -1 * (a * node.x + b * node.y + c) / (a * a + b * b)
            x = temp * a + node.x
            y = temp * b + node.y
            return x, y
        elif math.hypot((edge.fromx - node.x),(edge.fromy - node.y)) > math.hypot((edge.tox - node.x),(edge.toy - node.y)):
            return edge.tox, edge.toy
        return edge.fromx, edge.fromy

    def get_edge_dist(self, edge, node):
        a = edge.fromy - edge.toy
        b = -(edge.fromx - edge.tox)
        c = edge.fromx*edge.toy + edge.tox*edge.fromy
        if self.getAngle((edge.fromx,edge.fromy),(edge.tox,edge.toy),(node.x,node.y)) < 90 and self.getAngle((edge.tox,edge.toy),(edge.fromx,edge.fromy),(node.x,node.y)) < 90:
            return abs(a*node.x+b*node.y+c)/math.hypot(a,b)
        return min(math.hypot(node.x-edge.fromx,node.y-edge.fromy), math.hypot(node.x-edge.tox,node.y-edge.toy))

    def get_nearest_node_index(self, rnd_node):
        dlist = [math.sqrt((node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y)
                 ** 2) for node in self.node_list]
        minind = dlist.index(min(dlist))

        return minind

    def check_collision(self, node, obstacleList, linearObstacleList):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x()]
            dy_list = [oy - y for y in node.path_y()]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= size ** 2:
                return False
            
            firstx = node.path_x()[0]
            secondx = node.path_x()[1]
            firsty = node.path_y()[0]
            secondy = node.path_y()[1]

            edge = self.Edge(self.Node(firstx, firsty), self.Node(secondx, secondy))

            footx, footy = self.get_nearest_point_on_edge(edge, self.Node(ox, oy))
            dist = (footx - ox)*(footx - ox) + (footy - oy)*(footy - oy)

            if dist <= size ** 2:
                return False
                
        for (sx,sy,ex,ey) in linearObstacleList:
            firstx = node.path_x()[0]
            secondx = node.path_x()[1]
            firsty = node.path_y()[0]
            secondy = node.path_y()[1]

            A = [firstx, firsty]
            B = [secondx, secondy]

            C = [sx, sy]
            D = [ex, ey]

            if self.line_intersection((A, B), (C, D)):
                return False

        return True

    @staticmethod
    def line_intersection(line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
           return False
        
        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div

        if (x > max(line1[0][0], line1[1][0]) or x < min(line1[0][0], line1[1][0]) or
            y > max(line1[0][1], line1[1][1]) or y <min(line1[0][1], line1[1][1]) or
            x > max(line2[0][0], line2[1][0]) or x < min(line2[0][0], line2[1][0]) or
            y > max(line2[0][1], line2[1][1]) or y <min(line2[0][1], line2[1][1])):
            return False
        
        return True

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta


def plot_circle(x, y, size, color="-k"):
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)
    
def main():
    sx=-2.0
    sy=-0.5
    gx=2.0
    gy=-0.5
    
    obstacleList = [
        (0, -1, 0.9)
    ]
    linearObstacleList = [
        (0, 0.3, 0, 0.9),
        (-2, 0.5, 2, 0.5)
    ]
    
    rrt = RRT(start=[sx, sy],
              goal=[gx, gy],
              rand_area=[-3, 3, -1, 1],
              obstacle_list=obstacleList,
              wall_list=linearObstacleList)
    path = rrt.planning()

    if path:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01) 
        plt.show()
    


if __name__ == '__main__':
    main()
