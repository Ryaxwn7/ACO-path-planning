import numpy as np
from map_class import Map
from plot_picture import plot_picture, motion_move
import heapq
import copy

class Constraint:
    def __init__(self, robot_id, position, time_step):
        self.robot_id = robot_id
        self.position = position  # (x, y)
        self.time_step = time_step

class CBSNode:
    def __init__(self, constraints=None, solution=None, cost=0):
        self.constraints = constraints if constraints is not None else []
        self.solution = solution if solution is not None else []
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost

class CBSACO:
    def __init__(self, map_obj, num_ants=10, alpha=1.0, beta=2.0, rho=0.1, Q=1.0):
        self.map = map_obj
        self.occupancy_map = map_obj.occupancy_map
        self.rows, self.cols = self.occupancy_map.shape
        self.num_ants = num_ants
        self.alpha = alpha  # 信息素重要程度
        self.beta = beta    # 启发式因子重要程度
        self.rho = rho      # 信息素挥发率
        self.Q = Q          # 信息素增加强度
        self.pheromone = np.ones((self.rows, self.cols)) * 0.1
        self.max_time_steps = 100  # 最大时间步数

    def get_neighbors(self, position, time_step, constraints):
        """获取考虑时空约束的相邻节点"""
        x, y = position
        neighbors = []
        directions = [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]
        
        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            if (0 <= new_x < self.rows and 0 <= new_y < self.cols and 
                self.occupancy_map[new_x][new_y] == 1):
                # 检查是否违反约束
                if not self._check_constraint((new_x, new_y), time_step + 1, constraints):
                    neighbors.append((new_x, new_y))
        return neighbors

    def _check_constraint(self, position, time_step, constraints):
        """检查位置和时间是否违反约束"""
        for constraint in constraints:
            if (constraint.position == position and 
                constraint.time_step == time_step):
                return True
        return False

    def calculate_heuristic(self, current, goal):
        """计算启发式值（使用欧几里得距离）"""
        return np.sqrt((current[0] - goal[0])**2 + (current[1] - goal[1])**2)

    def ant_colony_search(self, start, goal, constraints):
        """使用蚁群算法在给定约束下寻找路径"""
        best_path = None
        best_length = float('inf')
        
        for _ in range(self.num_ants):
            current_pos = start
            path = [current_pos]
            time_step = 0
            
            while current_pos != goal and time_step < self.max_time_steps:
                neighbors = self.get_neighbors(current_pos, time_step, constraints)
                if not neighbors:
                    break
                
                # 计算转移概率
                probabilities = []
                for next_pos in neighbors:
                    tau = self.pheromone[next_pos[0]][next_pos[1]]
                    eta = 1.0 / (self.calculate_heuristic(next_pos, goal) + 1e-6)
                    prob = (tau ** self.alpha) * (eta ** self.beta)
                    probabilities.append(prob)
                
                # 选择下一个位置
                probabilities = np.array(probabilities)
                probabilities = probabilities / np.sum(probabilities)
                next_pos = neighbors[np.random.choice(len(neighbors), p=probabilities)]
                
                path.append(next_pos)
                current_pos = next_pos
                time_step += 1
            
            if current_pos == goal and len(path) < best_length:
                best_path = path
                best_length = len(path)
        
        return best_path

    def find_conflicts(self, solution):
        """查找路径之间的冲突"""
        conflicts = []
        max_length = max(len(path) for path in solution)
        
        for t in range(max_length):
            positions = {}
            for i, path in enumerate(solution):
                if t < len(path):
                    pos = path[t]
                    if pos in positions:
                        conflicts.append((i, positions[pos], pos, t))
                    positions[pos] = i
                    
                    # 检查交换冲突
                    if t > 0 and i in positions:
                        prev_pos = path[t-1]
                        other_robot = positions[pos]
                        if other_robot < len(solution) and t < len(solution[other_robot]):
                            if solution[other_robot][t-1] == pos and solution[other_robot][t] == prev_pos:
                                conflicts.append((i, other_robot, pos, t))
        
        return conflicts

    def cbs_search(self):
        """使用CBS算法进行路径规划"""
        # 初始化根节点
        root = CBSNode()
        open_list = []
        heapq.heappush(open_list, root)
        
        while open_list:
            current = heapq.heappop(open_list)
            
            # 为每个机器人规划路径
            solution = []
            for i in range(len(self.map.initial_node)):
                start = tuple(self.map.initial_node[i])
                goal = tuple(self.map.final_node[i])
                path = self.ant_colony_search(start, goal, current.constraints)
                if path is None:
                    break
                solution.append(path)
            
            if len(solution) == len(self.map.initial_node): # 所有机器人都有路径
                current.solution = solution # 保存路径
                current.cost = sum(len(path) for path in solution) # 计算总路径长度
                
                # 检查冲突
                conflicts = self.find_conflicts(solution)
                if not conflicts:
                    return solution # 没有冲突，返回路径
                
                # 处理第一个冲突
                robot1, robot2, position, time_step = conflicts[0]
                
                # 创建两个子节点
                constraints1 = current.constraints + [Constraint(robot1, position, time_step)]
                constraints2 = current.constraints + [Constraint(robot2, position, time_step)]
                
                node1 = CBSNode(constraints1)
                node2 = CBSNode(constraints2)
                
                heapq.heappush(open_list, node1) # 将两个子节点加入open_list
                heapq.heappush(open_list, node2)
        
        return None

def cbs_aco_resolve_path(map_obj, display=1):
    """使用CBS-ACO算法解决多机器人路径规划问题"""
    planner = CBSACO(map_obj)
    routes = planner.cbs_search()
    
    if routes is None:
        print("No solution found!")
        return []
    
    # 显示结果
    if display > 0:
        plot_picture(display, routes, len(map_obj.initial_node), map_obj)
        motion_move(routes, map_obj)
    
    return routes

if __name__ == "__main__":
    # 测试代码
    map_file = "middle.txt"
    try:
        map_obj = Map(map_file)
        routes = cbs_aco_resolve_path(map_obj)
        if not routes:
            print("No valid paths were found. Please check the map and robot positions.")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        print("Please check if the map file exists and is properly formatted.") 