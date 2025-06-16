# 解决点冲突问题

# 检查是否有冲突出现 并 解决冲突
import copy
from heapq import heappush, heappop

class CBSConstraint:
    """Constraint class for CBS"""
    def __init__(self, agent, loc, timestep):
        self.agent = agent       # Agent that is constrained
        self.loc = loc          # Location that is constrained
        self.timestep = timestep # Timestep at which the constraint is active

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
    
    return conflicts

def find_new_path(agent_path, constraints, start, goal):
    """Find a new path that satisfies the constraints using A* search"""
    # For simplicity, we'll just modify the existing path to avoid constraints
    # In a full implementation, this should be replaced with A* search
    new_path = copy.deepcopy(agent_path)
    
    for constraint in constraints:
        if constraint.timestep < len(new_path):
            # Simple conflict resolution: wait one timestep
            new_path.insert(constraint.timestep, new_path[constraint.timestep - 1])
    
    return new_path

def do_conflict_free(routes):
    """
    Implement Conflict-Based Search (CBS) for multi-agent path finding
    
    Args:
        routes: List of paths for each agent, where each path is a list of coordinates
    
    Returns:
        Conflict-free routes for all agents
    """
    # # Initialize CBS
    # root = CBSNode(routes, sum(get_path_cost(path) for path in routes))
    # open_list = []
    # heappush(open_list, root)
    
    # while open_list:
    #     node = heappop(open_list)
        
    #     # Detect conflicts in current solution
    #     conflicts = detect_conflicts(node.solution)
        
    #     if not conflicts:  # Solution is conflict-free
    #         return node.solution
            
    #     # Take first conflict and create two child nodes
    #     conflict = conflicts[0]
    #     for agent_idx in conflict['agents']:
    #         # Create new constraints
    #         new_constraints = copy.deepcopy(node.constraints)
    #         if conflict['type'] == 'vertex':
    #             new_constraint = CBSConstraint(
    #                 agent_idx,
    #                 conflict['loc'],
    #                 conflict['time']
    #             )
    #         else:  # edge conflict
    #             new_constraint = CBSConstraint(
    #                 agent_idx,
    #                 (conflict['loc1'], conflict['loc2']),
    #                 conflict['time']
    #             )
    #         new_constraints.append(new_constraint)
            
    #         # Find new path for constrained agent
    #         new_solution = copy.deepcopy(node.solution)
    #         new_path = find_new_path(
    #             new_solution[agent_idx],
    #             new_constraints,
    #             new_solution[agent_idx][0],  # start
    #             new_solution[agent_idx][-1]  # goal
    #         )
            
    #         if new_path:  # If a new path is found
    #             new_solution[agent_idx] = new_path
    #             new_cost = sum(get_path_cost(path) for path in new_solution)
    #             new_node = CBSNode(new_solution, new_cost, new_constraints)
    #             heappush(open_list, new_node)
    
    # # If no solution is found, return original routes
    return routes







