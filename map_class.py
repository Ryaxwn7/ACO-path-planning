#!/usr/bin/env python
# mxm map, square matrix shape.
# S indicates the starting point, just one starting point is allowed.
# F indicates the final point, just one final point is allowed.
# E indicates if a point in the map is empty.
# O indicates if a point in the map is occupied.

import numpy as np
import matplotlib.pyplot as plt
import copy

class Map:
    ''' Class used for handling the information provided by the input map '''

    # 每一个node有3个属性：1.位置（x，y）；2.edges; 3.spec={S,F,E,O};
                                   # edges: 1.可以取的点；2.phermone；3.probability 4.distance
    class Nodes:
        ''' Class for representing the nodes used by the ACO algorithm '''

        def __init__(self, row, col, in_map, spec):
            self.node_pos = (row, col)
            self.edges = self.compute_edges(in_map)
            self.spec = spec

        def compute_edges(self, map_arr):
            ''' class that returns the edges connected to each node '''
            imax = map_arr.shape[0]  # map_arr的行数
            jmax = map_arr.shape[1]  # map_arr的列数
            edges = []
            if map_arr[self.node_pos[0]][self.node_pos[1]] == 1:
                for dj in [-1, 0, 1]:
                    for di in [-1, 0, 1]:
                        newi = self.node_pos[0] + di
                        newj = self.node_pos[1] + dj
                        # 允许原地不动
                        # if (dj == 0 and di == 0):
                        #     continue
                        if (newj >= 0 and newj < jmax and newi >= 0 and newi < imax):
                            if map_arr[newi][newj] == 1: # 判断是否为可行位置
                                # 通过判断计算距离，避免使用sqrt
                                if di == 0 and dj != 0:
                                    distance = 1.0  # 上下左右方向
                                elif di == 0 and dj == 0:
                                    distance = 0.0
                                else:
                                    distance = 1.414  # 对角线方向
                                edges.append({'FinalNode': (newi, newj),
                                              'Pheromone': 1.0, 'Probability':
                                                  0.0, 'Distance': distance})
            return edges

    def __init__(self, map_name):
        self.in_map = self._read_map(map_name)  # in_map类型为str
        self.occupancy_map = self._map_2_occupancy_map()  # 输入in_map 将地图转化成int matrix
        self.initial_node = self.add_initial_node()
        self.final_node = self.add_final_node()
        # 读取始末坐标位置
        # 多个体目的地
        # self.nodes_array = self._create_nodes()  # 地图中各个点可以走一步到达的位置集合，并记录概率和信息素
        self.nodes_array = []
        # (self, row, col, in_map, spec)

    def _create_nodes(self): # 创建节点
        ''' Create nodes out of the initial map '''
        return [[self.Nodes(i, j, self.occupancy_map, self.in_map[i][j]) for j in
                 range(self.in_map.shape[1])] for i in range(self.in_map.shape[0])]

    # 读取map文件
    def _read_map(self, map_name):
        ''' Reads data from an input map txt file'''
        in_map = np.loadtxt('./maps/' + map_name, dtype=str )
        return in_map

    def add_initial_node(self):
        ''' Get all starting positions marked with 'S' '''
        points = np.where(self.in_map == 'S')
        initial_nodes = []
        for i in range(len(points[0])):  # points[0] contains row indices, points[1] contains column indices
            initial_nodes.append([int(points[0][i]), int(points[1][i])])
        return initial_nodes

    def add_final_node(self):
        ''' Get all goal positions marked with 'F' '''
        points = np.where(self.in_map == 'F')
        final_nodes = []
        for i in range(len(points[0])):  # points[0] contains row indices, points[1] contains column indices
            final_nodes.append([int(points[0][i]), int(points[1][i])])
        return final_nodes

    # str地图转化为int matrice
    def _map_2_occupancy_map(self):
        ''' Takes the matrix and converts it into a float array '''
        map_arr = np.copy(self.in_map)  # 复制一个in_map,命名为 map_arr
        map_arr[map_arr == 'O'] = 0  # 障碍
        map_arr[map_arr == 'E'] = 1  # 空的
        map_arr[map_arr == 'S'] = 1  # 开始
        map_arr[map_arr == 'F'] = 1  # 目的地
        return map_arr.astype(np.int_)  # astype 函数用于array中数值类型转换



