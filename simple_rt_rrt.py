import math
import os
import sys
import time

import matplotlib.pyplot as plt

try:
    from rrt import RRT
except ImportError:
    raise


class RRTStar(RRT):

    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)

        def cost(self):
            if not self.parent:
                return 0.0
            cost = self.get_path_cost()
            node = self.parent
            while node.parent:
                cost = cost + node.get_path_cost()
                node = node.parent
            return cost

        def get_path_cost(self):
            path_x = self.path_x()
            path_y = self.path_y()
            dx = path_x[1] - path_x[0]
            dy = path_y[1] - path_y[0]
            return math.hypot(dx,dy)

    def __init__(self, start, goal, obstacle_list, wall_list, rand_area,
                 expand_dis=0.2,
                 goal_sample_rate=5,
                 connect_circle_dist=0.3,
                 node_list = [],
                 kmax = 100
                 ):
        super().__init__(start, goal, obstacle_list, wall_list,
                         rand_area, expand_dis, goal_sample_rate, node_list)
        self.node_list = node_list
        self.start_time = time.time()
        self.kmax = kmax
        self.connect_circle_dist = connect_circle_dist

    def planning(self):
        if not self.node_list:
            self.node_list = [self.start]
        
        start_ind = self.get_node_ind_xy(self.start.x, self.start.y)
        if self.node_list[start_ind].parent:
            self.change_root(start_ind)
            
        qs = []
        i = 0
        
        while time.time() - self.start_time < 0.5:
            
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(rnd)
            nearest_node = self.node_list[nearest_ind]
            
            new_node = self.steer(nearest_node, rnd, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list, self.wall_list):
                near_inds = self.find_near_nodes(new_node)
                if len(near_inds) < self.kmax:
                    new_node = self.choose_parent(new_node, near_inds)
                    if new_node and (not self.check_node_in_list(new_node.x, new_node.y)):
                        self.node_list.append(new_node)
                        self.rewire(new_node, near_inds)

            if i % 5 == 0:
                self.draw_graph(rnd)
            i = i + 1
        
        last_index = self.search_best_goal_node()
        if last_index:
            path = self.generate_final_course(last_index)
            next_ind = self.get_next_root_ind(path)
            return True, path, self.node_list, next_ind

        path = self.generate_final_course2(self.get_node_closest_to_end())
        next_ind = self.get_next_root_ind(path)
        return False, path, self.node_list, next_ind

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None
    
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list, self.wall_list):
                costs.append(t_node.cost())
            else:
                costs.append(float("inf"))
        min_cost = min(costs)

        if min_cost == float("inf"):
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        
        return new_node            

    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_dis]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.end)
            if self.check_collision(t_node, self.obstacle_list, self.wall_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost() for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost() == min_cost:
                end_ind = self.get_node_ind_xy(self.end.x, self.end.y)
                if end_ind:
                    self.node_list[end_ind].parent = self.node_list[i]
                return i

        return None

    def find_near_nodes(self, new_node):
        dist_list = [(node.x - new_node.x) ** 2 +
                     (node.y - new_node.y) ** 2 for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= self.connect_circle_dist  ** 2]
        return near_inds

    def check_node_in_list(self, x, y):
        for node in self.node_list:
            if node.x == x and node.y == y:
                return True
        return False

    def get_node_ind_xy(self, x, y):
        for i in range(0,len(self.node_list)):
            node = self.node_list[i]
            if node.x == x and node.y == y:
                return i
        return None
    
    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost() + d
    
    def get_node_closest_to_end(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        min_ind = dist_to_goal_list.index(min(dist_to_goal_list))
        return min_ind
        
    def find_next_nodes_of(self, cnode, node_list):
        next_inds = [node_list.index(i) for i in node_list if i.parent and i.parent.x == cnode[0] and i.parent.y == cnode[1]]
        return next_inds

    def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.Node(near_node.x,near_node.y)
            edge_node.parent = new_node
            if not edge_node:
                continue

            no_collision = self.check_collision(edge_node, self.obstacle_list, self.wall_list)
            cost_improved = edge_node.cost() < near_node.cost()

            if no_collision and cost_improved:
                self.node_list[i].parent = new_node

    def change_root(self, next_root_ind):
        current_root_ind = self.get_node_ind_xy(self.node_list[next_root_ind].parent.x, self.node_list[next_root_ind].parent.y)
        self.node_list[next_root_ind].parent =  None
        self.node_list[current_root_ind].parent = self.node_list[next_root_ind]

    
        
    def get_next_root_ind(self, path):
        next_ind = 0
        if len(path) > 2:
            next_root_x = path[-2][0]
            next_root_y = path[-2][1]
            next_ind = self.get_node_ind_xy(next_root_x,next_root_y)
        elif len(path) == 2:
            next_root_x = path[1][0]
            next_root_y = path[1][1]
            next_ind = self.get_node_ind_xy(next_root_x,next_root_y)
        else: 
            next_root_x = path[0][0]
            next_root_y = path[0][1]
            next_ind = self.get_node_ind_xy(next_root_x,next_root_y)
        return next_ind
    

def main():
    
    sx=-2.0
    sy=-0.5
    gx=3.0
    gy=-0.5

    gx2 = -gx
    
    obstacleList = [
        (0, -1, 0.94)
    ]
    linearObstacleList = [
        (0, 0.3, 0, 0.9),
        (-2, 0.5, 1, 0.5)
    ]
    
    starttime = time.time()
    
    t = 20.0
    expand = 0.24
    rewire = expand * 1.5
    nodelist = []

    
    rrt_star = RRTStar(start=[sx, sy],
                      goal=[gx, gy],
                      rand_area=[-3.5, 3.5, -1, 1],
                      obstacle_list=obstacleList,
                      wall_list=linearObstacleList,
                      expand_dis = expand,
                      connect_circle_dist = rewire,
                      node_list = [] )
    found, path, nodelist, next_ind = rrt_star.planning()
    
    while time.time() - starttime < t:
        if time.time() - starttime > t/3:
            gx = gx2
        rrt_star = RRTStar(start=[nodelist[next_ind].x, nodelist[next_ind].y],
                      goal=[gx, gy + (time.time() - starttime)/ 15.0],
                      rand_area=[-3.5, 3.5, -1, 1],
                      obstacle_list=obstacleList,
                      wall_list=linearObstacleList,
                      expand_dis = expand,
                      connect_circle_dist = rewire,
                      node_list = nodelist )
        found, path, nodelist, next_ind = rrt_star.planning()
        
    endtime = time.time()
    print(endtime - starttime)
    
    if found:
        if path:        
            rrt_star.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r', alpha = 0.5)
            plt.grid(True)
            plt.pause(0.01)
            plt.show()
    else:
        print("path not found")
    


if __name__ == '__main__':
    main()
