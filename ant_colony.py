#!/usr/bin/env python

import numpy as np

class AntColony:
    ''' Class used for handling
        the behaviour of the whole
        ant colony '''
    class Ant:
        ''' Class used for handling the ant's individual behaviour '''
        def __init__(self, start_node_pos, final_node_pos):
            self.start_pos = start_node_pos
            self.actual_node = start_node_pos
            self.final_node = final_node_pos
            self.visited_nodes = []
            self.final_node_reached = False
            self.remember_visited_node(start_node_pos)

        def remember_visited_node(self, node_pos):
            ''' Appends the visited node to the list of visited nodes '''
            if isinstance(node_pos, list):
                node_pos = tuple(node_pos)  # 如果是列表，转换为元组
            self.visited_nodes.append(node_pos)

        def move_ant(self, node_to_visit):
            ''' Moves ant to the selected node '''
            if isinstance(node_to_visit, list):
                node_to_visit = tuple(node_to_visit)  # 如果是列表，转换为元组
            self.actual_node = node_to_visit
            self.remember_visited_node(node_to_visit)

        def get_visited_nodes(self):
            ''' Returns the list of visited nodes '''
            return self.visited_nodes

        def is_final_node_reached(self):
            ''' Checks if the ant has reached the final destination '''
            if isinstance(self.actual_node, list):
                actual = tuple(self.actual_node)
            else:
                actual = self.actual_node
                
            if isinstance(self.final_node, list):
                final = tuple(self.final_node)
            else:
                final = self.final_node
                
            if actual == final:
                self.final_node_reached = True

        def enable_start_new_path(self):
            ''' Enables a new path search setting the final_node_reached variable to false '''
            self.final_node_reached = False

        def setup_ant(self):
            ''' Clears the list of visited nodes, it stores the first one and selects the first one as initial'''
            self.visited_nodes = []
            self.remember_visited_node(self.start_pos)
            self.actual_node = self.start_pos

    def __init__(self, in_map, n_ants, iterations, evaporation_factor, pheromone_adding_constant):
        self.map = in_map
        self.n_ants = n_ants
        self.iterations = iterations
        self.evaporation_factor = evaporation_factor
        self.pheromone_adding_constant = pheromone_adding_constant
        self.paths = []
        self.ants = self.create_ants()
        self.best_result = []
        self.res = []
        self.shortest_route = []

    # 初始化n_ants只蚂蚁，每只蚂蚁都记录了始、末位置，当前位置，访问过的位置 和 是否到达目的地的flag
    def create_ants(self):
        ''' Creates a list contain in the total number of ants specified in the initial node '''
        ants = []
        for i in range(self.n_ants):
            ants.append(self.Ant(self.map.initial_node, self.map.final_node))
        return ants
    #在map-nodes_array中选择候选地点


    def select_next_node(self, actual_node):
        ''' Randomly selects the next node to visit based on pheromone levels '''
        # Compute the total sum of the pheromone of each edge
        total_pheromone = 0
        edges_list = []
        
        for edge in actual_node.edges:
            total_pheromone += edge['Pheromone']
            edges_list.append(edge)
            
        # Calculate probability of each edge
        p = []
        for edge in edges_list:
            if total_pheromone > 0:
                probability = edge['Pheromone'] / total_pheromone
            else:
                probability = 1.0 / len(edges_list)
            edge['Probability'] = probability
            p.append(probability)
            
        # Clear probability values
        cumsum_p = np.cumsum(p)
        random_value = np.random.random()
        
        # Return the node based on the probability of the solutions
        for i, cum_prob in enumerate(cumsum_p):
            if random_value <= cum_prob:
                return edges_list[i]['FinalNode']
                
        # If no node was selected (shouldn't happen), return the last one
        return edges_list[-1]['FinalNode']


    def sort_paths(self):
        ''' Sorts the paths based on their Euclidean distance '''
        # 按照欧几里得距离排序
        # self.paths.sort(key=self.calculate_euclidean_distance)
        # 按照路径时间步长度排序
        self.paths.sort(key=len)

    # def pheromone_update(self):
    #     ''' Updates the pheromone level of the each of the trails and sorts the paths by length '''
    #     # Sort the list according to the size of the lists
    #     self.sort_paths()
        
    #     # Update pheromone levels for all edges
    #     for i in range(len(self.map.nodes_array)):
    #         for j in range(len(self.map.nodes_array[i])):
    #             node = self.map.nodes_array[i][j]
    #             for edge in node.edges:
    #                 # Evaporation
    #                 edge['Pheromone'] = (1 - self.evaporation_factor) * edge['Pheromone']
                    
    #                 # Add new pheromone from ants that used this edge
    #                 for path in self.paths:
    #                     for k in range(len(path)-1):
    #                         if path[k] == node.node_pos and path[k+1] == edge['FinalNode']:
    #                             # 计算当前节点和下一个节点之间的欧几里得距离
    #                             current_node = path[k]
    #                             next_node = path[k+1]
    #                             distance = np.sqrt((next_node[0] - current_node[0])**2 + 
    #                                              (next_node[1] - current_node[1])**2)
    #                             # 根据距离更新pheromone
    #                             edge['Pheromone'] += self.pheromone_adding_constant / distance
    def pheromone_update(self):
        ''' Updates the pheromone level of the each of the trails and sorts the paths by length '''
        # Sort the list according to the size of the lists
        self.sort_paths()
        
        # Update pheromone levels for all edges
        for i in range(len(self.map.nodes_array)):
            for j in range(len(self.map.nodes_array[i])):
                node = self.map.nodes_array[i][j]
                for edge in node.edges:
                    # Evaporation
                    edge['Pheromone'] = (1 - self.evaporation_factor) * edge['Pheromone']
                    
                    # Add new pheromone from ants that used this edge
                    for path in self.paths:
                        for k in range(len(path)-1):
                            if path[k] == node.node_pos and path[k+1] == edge['FinalNode']:
                                edge['Pheromone'] += self.pheromone_adding_constant / len(path)
    def empty_paths(self):
        ''' Empty the list of paths '''
        self.paths = []

    def add_to_path_results(self, path_withloop):
        ''' Appends the path to the results path list'''
        self.paths.append(path_withloop)

    def get_coincidence_indices(self,path_withloop, element):
        ''' Gets the indices of the coincidences of elements in the path '''
        result = []
        offset = -1
        while True:
            try:
                offset = path_withloop.index(element, offset+1)  # 可能产生异常的代码块
                # 在offset中从 offset+1 位置开始寻找重复的element的位置
            except ValueError:
                return result
            result.append(offset)

    def delete_loops(self, visited_nodes_path):
        ''' Checks if there is a loop in the resulting path and deletes it '''
        path_withloop = list(visited_nodes_path)
        for element in path_withloop:
            # coincidence 是列表; 获取element重复的位置
            coincidences = self.get_coincidence_indices(path_withloop, element)

            # reverse de list to delete elements from back to front of the list
            coincidences.reverse()  # 用于反向列表中元素

            for i,coincidence in enumerate(coincidences):
                if not i == len(coincidences)-1:  # 判断到最后一个元素都已经操作过了
                    path_withloop[coincidences[i+1]:coincidence] = []  # 将coincidence[i+1]到coincidence[i]之间的路径删除

        return path_withloop

    def calculate_euclidean_distance(self, path):
        ''' Calculate the total Euclidean distance of a path '''
        total_distance = 0
        for i in range(len(path)-1):
            current_node = path[i]
            next_node = path[i+1]
            distance = np.sqrt((next_node[0] - current_node[0])**2 + 
                             (next_node[1] - current_node[1])**2)
            total_distance += distance
        return total_distance

    def calculate_path(self):
        ''' Carries out the process to get the best path '''
        # Repeat the cicle for the specified no of times
        for i in range(self.iterations):            # 迭代iters次数

            for ant in self.ants:                   # 迭代蚂蚁的个数
                ant.setup_ant()                     # 初始化/清除 上一个iter蚁群的visited表，将蚂蚁的初始位置 set -> start_pos
                while not ant.final_node_reached:   # 判断是否到达终点； 条件为 not false -> true
                    # Randomly selection of the node to visit
                    node_to_visit = self.select_next_node(self.map.nodes_array[int(ant.actual_node[0])][int(ant.actual_node[1])])
                    # 获取in_map 中的nodes_array 中对应actual_node位置的 点 和 边 的信息
                    # node_to_visit 存储下一个要走的点

                    # Move ant to the next node randomly selected
                    ant.move_ant(node_to_visit)
                    # 1.更换actual_node（下一步node）； 2.remember_visited_node

                    # 判断是否到达了final_node
                    # Check if solution has been reached
                    ant.is_final_node_reached()

                # Add the resulting path to the path list
                visited_nodes_path = ant.get_visited_nodes()           # 第i iteration中一只蚂蚁走的路径，经历过的nodes
                path_withloop = self.delete_loops(visited_nodes_path)  # 清除循环路径
                self.add_to_path_results(path_withloop)
             #   self.add_to_path_results(self.delete_loops(ant.get_visited_nodes()))

                # Enable the ant for a new search
                ant.enable_start_new_path()             # flag final_node_reached = False 复位，准备下一只蚂蚁

            # Update the global pheromone level
            self.pheromone_update()
            self.best_result = self.paths[0]
            # Empty the list of paths
            self.empty_paths()
            
            path_length = self.calculate_euclidean_distance(self.best_result)
            print('Iteration: ', i, ' path length: ', round(path_length, 2), ' nodes: ', len(self.best_result))

            self.res.append(self.best_result) # 记录每一次的best_result
            self.shortest_route = min(self.res,key=len)
            # self.shortest_route = min(self.res,key=self.calculate_euclidean_distance) # 记录下最短的一条路径
        return self.shortest_route


