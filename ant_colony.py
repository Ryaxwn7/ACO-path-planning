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

    def __init__(self, in_map, n_ants, iterations, evaporation_factor, pheromone_adding_constant, alpha, beta, constraints=None):
        self.map = in_map
        self.n_ants = n_ants
        self.iterations = iterations
        self.evaporation_factor = evaporation_factor
        self.pheromone_adding_constant = pheromone_adding_constant
        self.alpha = alpha
        self.beta = beta
        self.paths = []
        self.ants = self.create_ants()
        self.best_result = []
        self.res = []
        self.shortest_route = []
        self.constraints = constraints if constraints is not None else []

    # 初始化n_ants只蚂蚁，每只蚂蚁都记录了始、末位置，当前位置，访问过的位置 和 是否到达目的地的flag
    def create_ants(self):
        ''' Creates a list contain in the total number of ants specified in the initial node '''
        ants = []
        for i in range(self.n_ants):
            ants.append(self.Ant(self.map.initial_node, self.map.final_node))
        return ants
    #在map-nodes_array中选择候选地点


    def select_next_node(self, actual_node, current_time=0, visited_nodes=[]):
        ''' Randomly selects the next node to visit based on pheromone levels and heuristic information, with constraints '''
        # Compute the total sum of the pheromone of each edge
        total_pheromone = 0
        edges_list = []
        valid_edges = []
        for edge in actual_node.edges:
            # 检查约束：当前时间步不能到达该点
            forbidden = False
            if (edge['FinalNode'] in visited_nodes):
                forbidden = True
            for c in self.constraints:
                # 处理CBSConstraint对象
                if hasattr(c, 'loc'):  # CBSConstraint对象
                    if isinstance(c.loc, tuple) and len(c.loc) == 2:
                        # 边约束
                        if (actual_node.node_pos == c.loc[0] and 
                            edge['FinalNode'] == c.loc[1] and 
                            current_time + 1 == c.timestep):
                            forbidden = True
                            break
                    else:
                        # 顶点约束
                        if (edge['FinalNode'] == c.loc and 
                            current_time + 1 == c.timestep):
                            forbidden = True
                            break
                else:
                    # 字典格式的约束
                    if (edge['FinalNode'] == c['pos'] and current_time+1 == c['time']):
                        forbidden = True
                        break
            if not forbidden:
                valid_edges.append(edge)
                # total_pheromone += edge['Pheromone']
        if not valid_edges:
            return None  # 没有可选节点
        # Calculate probability of each edge with heuristic factor
        p = []
        att =[0]*len(valid_edges)
        total_attractiveness = 0
        for i,edge in enumerate(valid_edges):
            # 启发式因子
            dist_to_goal = np.sqrt((actual_node.node_pos[0] - self.map.final_node[0])**2 + (actual_node.node_pos[1] - self.map.final_node[1])**2)
            heuristic_factor = 1.0 / (edge['Distance']+ dist_to_goal) # 
            # heuristic_factor = 1.0 / (edge['Distance'])

            attractiveness = (edge['Pheromone'] ** self.alpha) * (heuristic_factor ** self.beta)
            # attractiveness = edge['Pheromone'] ** self.alpha
            total_attractiveness += attractiveness
            att[i] = attractiveness
        for i,edge in enumerate(valid_edges):
            if total_attractiveness > 0:
                probability = att[i] / total_attractiveness
            else:
                probability = 1.0 / len(valid_edges)
            edge['Probability'] = probability
            # print(f"edge: {edge['FinalNode']}, probability: {probability}")
            edges_list.append(edge)
            p.append(probability)
            
        for edge in valid_edges:
            edge['Probability'] = 0.0
        # 随机选择下一个节点
        return np.random.choice(edges_list,1, p)[0]['FinalNode']


    def sort_paths(self):
        ''' Sorts the paths based on their Euclidean distance '''
        # 按照欧几里得距离排序
        self.paths.sort(key=self.calculate_euclidean_distance)
        # 按照路径时间步长度排序
        #self.paths.sort(key=len)

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
  
        self.sort_paths() # 按照路径长度排序
        max_pheromone = 0
        
        # 对所有边进行信息素蒸发
        for i in range(len(self.map.nodes_array)):
            for j in range(len(self.map.nodes_array[i])):
                node = self.map.nodes_array[i][j]
                for edge in node.edges:
                    edge['Pheromone'] = (1.0 - self.evaporation_factor) * edge['Pheromone']
        
        # 然后对路径上的边进行信息素累加
        for path in self.paths:
            path_length = self.calculate_euclidean_distance(path)
            for j in range(len(path)-1):
                current_node = path[j]
                next_node = path[j+1]
                for edge in self.map.nodes_array[current_node[0]][current_node[1]].edges:
                    if edge['FinalNode'] == next_node:
                        edge['Pheromone'] += self.pheromone_adding_constant/float(path_length)
                        #edge['Pheromone'] += self.pheromone_adding_constant/float(len(path))
                        if edge['Pheromone'] > max_pheromone:
                            max_pheromone = edge['Pheromone']
        # for i, path in enumerate(self.paths):
        #     path_length = self.calculate_euclidean_distance(path)
        #     for j, element in enumerate(path):
        #         for edge in self.map.nodes_array[element[0]][element[1]].edges:
        #             if (j+1) < len(path): # 判断是否到达了终点
        #                 if edge['FinalNode'] == path[j+1]: # 判断是否到达了下一个节点
        #                     # 更新pheromone
        #                     edge['Pheromone'] = (1.0 - self.evaporation_factor) * edge['Pheromone'] + self.pheromone_adding_constant/float(path_length)
        #                     if edge['Pheromone'] > max_pheromone:
        #                         max_pheromone = edge['Pheromone']
        #                 else:
        #                     edge['Pheromone'] = (1.0 - self.evaporation_factor) * edge['Pheromone']

        print("max_pheromone: ", max_pheromone)

    def empty_paths(self):
        ''' Empty the list of paths '''
        self.paths = []

    def add_to_path_results(self, path_withloop):   
        ''' Appends the path to the results path list'''
        self.paths.append(path_withloop)

    def get_coincidence_indices(self,path_withloop, element): # 获取重复元素的索引
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
                current_time = 0
                while not ant.final_node_reached:   # 判断是否到达终点； 条件为 not false -> true
                    node_to_visit = self.select_next_node(self.map.nodes_array[int(ant.actual_node[0])][int(ant.actual_node[1])], current_time, ant.visited_nodes)
                    if node_to_visit is None:
                        # print(f"No valid nodes to visit for ant {i+1} at time step {current_time}")
                        break # 进入死胡同，判定为死亡。
                    ant.move_ant(node_to_visit)
                    ant.is_final_node_reached()
                    current_time += 1
                if(ant.final_node_reached):  # 如果蚂蚁到达终点
                    visited_nodes_path = ant.get_visited_nodes()           # 第i iteration中一只蚂蚁走的路径，经历过的nodes
                    # path_withoutloop = self.delete_loops(visited_nodes_path)  # 清除循环路径
                    self.add_to_path_results(visited_nodes_path)
                ant.enable_start_new_path()             # flag final_node_reached = False 复位，准备下一次迭代

            self.pheromone_update()
            self.best_result = self.paths[0]
            self.empty_paths()
            path_length = self.calculate_euclidean_distance(self.best_result)
            print('Iteration: ', i, ' path length: ', round(path_length, 2), ' nodes: ', len(self.best_result))
            self.res.append(self.best_result) # 记录每一次的best_result
            #self.shortest_route = min(self.res,key=len) # 记录下最短的一条路径
            self.shortest_route = min(self.res,key=self.calculate_euclidean_distance) # 记录下最短的一条路径
        return self.shortest_route


