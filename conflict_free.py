# 解决点冲突问题

# 检查是否有冲突出现 并 解决冲突
import copy
from ant_colony import AntColony
from heapq import heappush, heappop

class CBSConstraint:
    """Constraint class for CBS"""
    def __init__(self, agent, loc, timestep):
        self.agent = agent       # Agent that is constrained
        self.loc = loc          # Location that is constrained
        self.timestep = timestep # Timestep at which the constraint is active
    
    def __eq__(self, other):
        """Check if two constraints are equal (same agent, location, and timestep)"""
        if not isinstance(other, CBSConstraint):
            return False
        return (self.agent == other.agent and 
                self.loc == other.loc and 
                self.timestep == other.timestep)
    
    def __hash__(self):
        """Hash function for constraint comparison"""
        return hash((self.agent, self.loc, self.timestep))
    
    def __repr__(self):
        """String representation for debugging"""
        return f"CBSConstraint(agent={self.agent}, loc={self.loc}, timestep={self.timestep})"

class CBSNode:
    """Node class for CBS constraint tree"""
    def __init__(self, solution, cost, constraints=None):
        self.solution = solution    # List of paths for each agent
        self.cost = cost           # Sum of individual path costs
        self.constraints = constraints if constraints else []  # List of constraints

    def __lt__(self, other):
        return self.cost < other.cost

def get_path_cost(path):
    """Calculate the cost of a path"""
    return len(path) if path else 0

def detect_conflicts(solution):
    """Detect both vertex and edge conflicts between all pairs of agents"""
    conflicts = []
    for i in range(len(solution)):
        for j in range(i + 1, len(solution)):
            flag = False
            path1, path2 = solution[i], solution[j]
            
            # Pad shorter path with its last position
            max_len = max(len(path1), len(path2))
            path1_padded = path1 + [path1[-1]] * (max_len - len(path1))
            path2_padded = path2 + [path2[-1]] * (max_len - len(path2))
            
            # Check for vertex conflicts
            for t in range(max_len):
                if path1_padded[t] == path2_padded[t]:
                    conflicts.append({
                        'type': 'vertex',
                        'time': t,
                        'agents': (i, j),
                        'loc': path1_padded[t]
                    })
                    break   
            
            # Check for edge conflicts
            for t in range(max_len - 1):
                if path1_padded[t] == path2_padded[t+1] and path1_padded[t+1] == path2_padded[t]:
                    conflicts.append({
                        'type': 'edge',
                        'time': t,
                        'agents': (i, j),
                        'loc1': path1_padded[t],
                        'loc2': path1_padded[t+1]
                    })
                    break

    return conflicts

def find_new_path(agent_path, constraints, start, goal, w, ants, iterations, p, Q, alpha, beta):
    """Find a new path that satisfies the constraints using A* search"""
    # For simplicity, we'll just modify the existing path to avoid constraints
    # In a full implementation, this should be replaced with A* search
    new_path1 = copy.deepcopy(agent_path)
    
    # 等待策略
    for constraint in constraints:
        if constraint.timestep < len(new_path1):
            # 简单处理，采用等待策略
            print("agent: ", constraint.agent, "timestep: ", constraint.timestep, "loc: ", constraint.loc,"wait")
            new_path1.insert(constraint.timestep, new_path1[constraint.timestep - 1])
    return new_path1
    # # 使用蚁群算法
    # Colony = AntColony(w, ants, iterations, p, Q, alpha, beta, constraints)
    # new_path2 = Colony.calculate_path()

    # # 选择最优路径
    # if new_path1 and new_path2:
    #     cost1 = get_path_cost(new_path1)
    #     cost2 = get_path_cost(new_path2)
    #     if cost1 < cost2:
    #         print("new_path1 cost: ", cost1, "new_path2 cost: ", cost2)
    #         return new_path1
    #     else:
    #         print("new_path2 cost: ", cost2, "new_path1 cost: ", cost1)
    #         return new_path2
    # elif new_path1: # 只有new_path1
    #     return new_path1
    # return new_path2

def do_conflict_free(routes, M, ants, iterations, p, Q, alpha, beta):
    """
    Implement Conflict-Based Search (CBS) for multi-agent path finding
    
    Args:
        routes: List of paths for each agent, where each path is a list of coordinates
    
    Returns:
        Conflict-free routes for all agents
    """
    # Initialize CBS
    root = CBSNode(routes, sum(get_path_cost(path) for path in routes))
    open_list = []
    heappush(open_list, root) # 将根节点加入到open_list中
    
    while open_list:
        node = heappop(open_list)
        
        # Detect conflicts in current solution
        conflicts = detect_conflicts(node.solution)
        
        if not conflicts:  # Solution is conflict-free
            print("Solution is conflict-free")
            return node.solution
            
        # Take first conflict and create two child nodes
        conflict = conflicts[0]
        for agent_idx in conflict['agents']: # 遍历冲突的agent(左右子节点)
            # Create new constraints
            new_constraints = copy.deepcopy(node.constraints)
            if conflict['type'] == 'vertex': # 顶点冲突
                print("Conflict type: vertex, loc: ", conflict['loc'])
                new_constraint = CBSConstraint(
                    agent_idx,
                    conflict['loc'],
                    conflict['time']
                )
            else:  # 边冲突 
                print("Conflict type: edge, loc1: ", conflict['loc1'], "loc2: ", conflict['loc2'])
                new_constraint = CBSConstraint(
                    agent_idx,
                    (conflict['loc1'], conflict['loc2']),
                    conflict['time']
                )
            
            # 检查约束是否已存在，避免重复添加
            if new_constraint not in new_constraints:
                new_constraints.append(new_constraint)
                print(f"添加新约束: {new_constraint}")
            else:
                print(f"约束已存在，跳过: {new_constraint}")
                continue  # 如果约束已存在，跳过这个分支
            
            # Find new path for constrained agent
            new_solution = copy.deepcopy(node.solution)
            w = copy.deepcopy(M[agent_idx])
            w.nodes_array = w._create_nodes()
            new_path = find_new_path(
                new_solution[agent_idx],
                new_constraints,
                new_solution[agent_idx][0],  # start
                new_solution[agent_idx][-1],  # goal
                w, ants, iterations, p, Q, alpha, beta
            )
            
            if new_path:  # If a new path is found
                new_solution[agent_idx] = new_path
                new_cost = sum(get_path_cost(path) for path in new_solution)
                new_node = CBSNode(new_solution, new_cost, new_constraints)
                heappush(open_list, new_node)
    
    # # If no solution is found, return original routes
    return routes







