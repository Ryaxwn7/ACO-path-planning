import numpy as np
import heapq
from map_class import Map
from plot_picture import plot_picture, motion_move

class Node:
    def __init__(self, position, g_cost=0, h_cost=0, parent=None):
        self.position = position  # (x, y)
        self.g_cost = g_cost     # 从起点到当前节点的实际代价
        self.h_cost = h_cost     # 从当前节点到终点的估计代价
        self.f_cost = g_cost + h_cost  # 总代价
        self.parent = parent     # 父节点，用于重建路径

    def __lt__(self, other):
        return self.f_cost < other.f_cost

class AStarPlanner:
    def __init__(self, map_obj):
        self.map = map_obj
        self.occupancy_map = map_obj.occupancy_map
        self.rows, self.cols = self.occupancy_map.shape

    def get_neighbors(self, node):
        """获取节点的相邻节点"""
        x, y = node.position
        neighbors = []
        # 8个方向：上、下、左、右、左上、右上、左下、右下
        directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
        
        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            # 检查是否在地图范围内且不是障碍物（0表示障碍物，1表示可通行）
            if (0 <= new_x < self.rows and 0 <= new_y < self.cols and 
                self.occupancy_map[new_x][new_y] == 1):
                neighbors.append((new_x, new_y))
        return neighbors

    def calculate_h_cost(self, current, goal):
        """计算启发式代价（使用欧几里得距离）"""
        return np.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)

    def calculate_g_cost(self, current, neighbor):
        """计算从当前节点到相邻节点的实际代价"""
        # 如果是斜向移动，代价为根号2，否则为1
        if abs(current[0] - neighbor[0]) + abs(current[1] - neighbor[1]) == 2:
            return np.sqrt(2)
        return 1

    def reconstruct_path(self, node):
        """重建路径"""
        path = []
        current = node
        while current is not None:
            path.append(current.position)
            current = current.parent
        return path[::-1]  # 反转路径，从起点到终点

    def find_path(self, start, goal):
        """使用A*算法寻找从起点到终点的路径"""
        # 初始化开放列表和关闭列表
        open_list = []
        closed_set = set()
        
        # 创建起点节点
        start_node = Node(start, 0, self.calculate_h_cost(start, goal))
        heapq.heappush(open_list, start_node)
        
        # 用于快速查找节点
        node_dict = {start: start_node}
        
        while open_list:
            current = heapq.heappop(open_list)
            
            # 如果到达目标
            if current.position == goal:
                return self.reconstruct_path(current)
            
            # 将当前节点加入关闭列表
            closed_set.add(current.position)
            
            # 检查所有相邻节点
            for neighbor_pos in self.get_neighbors(current):
                if neighbor_pos in closed_set:
                    continue
                
                # 计算新的g_cost
                new_g_cost = current.g_cost + self.calculate_g_cost(current.position, neighbor_pos)
                
                # 如果节点已经在开放列表中
                if neighbor_pos in node_dict:
                    neighbor = node_dict[neighbor_pos]
                    if new_g_cost >= neighbor.g_cost:
                        continue
                else:
                    # 创建新节点
                    h_cost = self.calculate_h_cost(neighbor_pos, goal)
                    neighbor = Node(neighbor_pos, new_g_cost, h_cost, current)
                    node_dict[neighbor_pos] = neighbor
                    heapq.heappush(open_list, neighbor)
                
                # 更新节点信息
                neighbor.g_cost = new_g_cost
                neighbor.f_cost = new_g_cost + neighbor.h_cost
                neighbor.parent = current
        
        return None  # 如果没有找到路径

def astar_resolve_path(map_obj, display=1):
    """使用A*算法解决多机器人路径规划问题"""
    planner = AStarPlanner(map_obj)
    routes = []
    
    # 打印地图信息
    print(f"Map size: {map_obj.occupancy_map.shape}")
    print(f"Number of robots: {len(map_obj.initial_node)}")
    
    # 为每个机器人规划路径
    for i in range(len(map_obj.initial_node)):
        start = tuple(map_obj.initial_node[i])
        goal = tuple(map_obj.final_node[i])
        
        print(f"\nPlanning path for robot {i+1}:")
        print(f"Start position: {start}")
        print(f"Goal position: {goal}")
        
        # 检查起点和终点是否有效
        if (not (0 <= start[0] < map_obj.occupancy_map.shape[0] and 
                 0 <= start[1] < map_obj.occupancy_map.shape[1]) or
            not (0 <= goal[0] < map_obj.occupancy_map.shape[0] and 
                 0 <= goal[1] < map_obj.occupancy_map.shape[1])):
            print(f"Error: Start or goal position out of map bounds for robot {i+1}")
            continue
            
        # 检查起点和终点是否在障碍物上（0表示障碍物，1表示可通行）
        if (map_obj.occupancy_map[start[0]][start[1]] == 0 or 
            map_obj.occupancy_map[goal[0]][goal[1]] == 0):
            print(f"Error: Start or goal position is on an obstacle for robot {i+1}")
            continue
        
        # 寻找路径
        path = planner.find_path(start, goal)
        if path is None:
            print(f"Warning: No path found for robot {i+1}")
            continue
        
        print(f"Path found for robot {i+1} with length {len(path)-1}")
        routes.append(path)
    
    # 检查是否找到任何路径
    if not routes:
        print("\nError: No valid paths found for any robot!")
        return []
    
    # 显示结果
    if display > 0:
        plot_picture(display, routes, len(map_obj.initial_node), map_obj)
        motion_move(routes, map_obj)
    
    return routes

if __name__ == "__main__":
    # 测试代码
    map_file = "small.txt"
    try:
        map_obj = Map(map_file)
        routes = astar_resolve_path(map_obj)
        if not routes:
            print("No valid paths were found. Please check the map and robot positions.")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        print("Please check if the map file exists and is properly formatted.") 