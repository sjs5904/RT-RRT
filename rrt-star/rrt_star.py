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
            self.cost = 0.0

    def __init__(self, start, goal, obstacle_list, wall_list, rand_area,
                 expand_dis=0.2,
                 goal_sample_rate=10,
                 max_iter=100,
                 connect_circle_dist=0.3,
                 node_list = [],
                 kmax = 30
                 ):
        super().__init__(start, goal, obstacle_list, wall_list,
                         rand_area, expand_dis, goal_sample_rate, max_iter, node_list)

        self.start_time = time.time()
        self.kmax = kmax
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])

    def planning(self, search_until_max_iter=True):
        
        if not self.node_list:
            self.node_list = [self.start]
        for i in range(self.max_iter):
            
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            nearest_node = self.node_list[nearest_ind]
            
            new_node = self.steer(nearest_node, rnd, self.expand_dis)

            if self.check_collision(new_node, self.obstacle_list, self.wall_list):
                near_inds = self.find_near_nodes(new_node)
                if len(near_inds) < self.kmax:
                    new_node = self.choose_parent(new_node, near_inds) # choose parent with minimum cost
                    if new_node and not self.check_node_in_list(new_node.x, new_node.y):
                        self.node_list.append(new_node)
                        #self.rewire(new_node, near_inds)

            if i % 10 == 0:
                self.draw_graph(rnd)

            if time.time() - self.start_time > 0.5 and new_node:
                last_index = self.search_best_goal_node()
                if last_index:
                    return self.generate_final_course(last_index), self.node_list

        #print("max iteration")
        
        last_index = self.search_best_goal_node()
        if last_index:
            return self.generate_final_course(last_index), self.node_list

        return self.generate_final_course(self.get_node_closest_to_end()), self.node_list

    def choose_parent(self, new_node, near_inds):
        if not near_inds:
            return None
    
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.obstacle_list, self.wall_list):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))
        min_cost = min(costs)

        if min_cost == float("inf"):
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.parent = self.node_list[min_ind]
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_dis]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.obstacle_list, self.wall_list):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
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
    
    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d
    
    def get_node_closest_to_end(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        min_ind = dist_to_goal_list.index(min(dist_to_goal_list))
        return min_ind
        
    def find_next_nodes_of(self, cnode, node_list):
        next_inds = [node_list.index(i) for i in node_list if i.parent and i.parent.x == cnode[0] and i.parent.y == cnode[1]]
        return next_inds

    def rewire_random_node(self, qr):
        

    
'''   def rewire(self, new_node, near_inds):
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.obstacle_list, self.wall_list)
            cost_improved = near_node.cost > edge_node.cost

            if no_collision and cost_improved:
                self.node_list[i] = edge_node
                self.propagate_cost_to_leaves(new_node)

    def change_root(self, new_root_ind, node_list):

    def get_next_root(self, goal_ind, node_list):
#   '''

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)
    

def main():
    
    sx=-2.0
    sy=-0.5
    gx=1.5
    gy=0.75
    
    obstacleList = [
        (0, -1, 0.94)
    ]
    linearObstacleList = [
        (0, 0.3, 0, 0.9),
        (-2, 0.5, 2, 0.5)
    ]
    
    starttime = time.time()
    
    t = 8
    expand = 0.2
    rewire = expand * 2
    nodelist = []

    
    rrt_star = RRTStar(start=[sx, sy],
                      goal=[gx, gy],
                      rand_area=[-3, 3, -1.5, 1.5],
                      obstacle_list=obstacleList,
                      wall_list=linearObstacleList,
                      expand_dis = expand,
                      connect_circle_dist = rewire )
    path, nodelist = rrt_star.planning()
    while time.time() - starttime < t:
        print(rrt_star.find_next_nodes_of([sx,sy], nodelist))
        rrt_star = RRTStar(start=[nodelist[1].x, nodelist[1].y],
                      goal=[gx, gy],
                      rand_area=[-3, 3, -1, 1],
                      obstacle_list=obstacleList,
                      wall_list=linearObstacleList,
                      expand_dis = expand,
                      connect_circle_dist = rewire,
                      node_list = nodelist )
        path, nodelist = rrt_star.planning()
    
    endtime = time.time()
    print(endtime - starttime)
    if path:        
        rrt_star.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(0.01)
        plt.show()


if __name__ == '__main__':
    main()
