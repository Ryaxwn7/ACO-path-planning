#!/usr/bin/env python

from map_class import Map
from ant_colony import AntColony
from plot_picture import plot_picture, motion_move
import time
from conflict_free import do_conflict_free
import copy
import os

def tuple_to_list(route):
    """Convert tuple coordinates to list format"""
    result = []
    for path in route:
        path_list = []
        for coord in path:
            if isinstance(coord, tuple):
                path_list.append(list(coord))
            else:
                path_list.append(coord)
        result.append(path_list)
    return result

if __name__ == '__main__':
    # Initialize parameters
    t0 = time.process_time()
    ants = 20          # Number of ants
    iterations = 20    # Number of iterations
    p = 0.1           # Pheromone evaporation rate
    Q = 1.0           # Pheromone adding constant
    display = True    # Whether to display the result
    
    # Available maps: 'map1.txt', 'map2.txt', 'map3.txt', 'small.txt', 'middle.txt', 'big.txt'
    map_path = 'middle.txt'  # Map file path
    
    print(f"Starting path planning with map: {map_path}")
    print(f"Parameters: ants={ants}, iterations={iterations}, p={p}, Q={Q}")
    
    try:
        # Get the map
        map = Map(map_path)
        
        if not map.initial_node or not map.final_node:
            raise ValueError("No start (S) or goal (F) positions found in the map!")
            
        if len(map.initial_node) != len(map.final_node):
            raise ValueError(f"Number of start positions ({len(map.initial_node)}) does not match number of goal positions ({len(map.final_node)})!")
        
        print(f"\nFound {len(map.initial_node)} robots to plan for")
        print(f"Start positions: {map.initial_node}")
        print(f"Goal positions: {map.final_node}")

        if len(map.initial_node) > 1:  # Multiple robots case
            M = []                     # Split maps for each robot
            n = len(map.initial_node)
            for i in range(n):
                robot = copy.deepcopy(map)
                robot.initial_node = robot.initial_node[i]
                robot.final_node = robot.final_node[i]
                M.append(robot)
            
            route = []  # Store all paths

            # Generate pheromone matrix for each robot
            for i in range(len(M)): # 遍历每个机器人
                print(f"\nPlanning path for robot {i+1}...")
                w = copy.deepcopy(M[i])
                w.nodes_array = w._create_nodes()
                
                # Debug information
                start_pos = w.initial_node
                print(f"Robot {i+1} start position: {start_pos}")
                start_node = w.nodes_array[start_pos[0]][start_pos[1]]
                print(f"Robot {i+1} start node edges: {start_node.edges}")
                print(f"Robot {i+1} occupancy map at start: {w.occupancy_map[start_pos[0]][start_pos[1]]}")
                
                Colony = AntColony(w, ants, iterations, p, Q)
                path = Colony.calculate_path()
                route.append(path)
                print(f"Initial path for robot {i+1}: {path}")
                print(f"Robot {i+1} path length: {Colony.calculate_euclidean_distance(path)}, time step: {len(path)}")
            
            print("\nAll routes before conflict resolution:", route)
            
            # Check and resolve conflicts
            print("\nResolving conflicts...")
            route_sort = do_conflict_free(tuple_to_list(route))
            print("\nConflict-free routes:", route_sort)

            # Calculate time
            t1 = time.process_time()
            time_loss = t1-t0
            print('\nTime taken: {:.2f}s'.format(time_loss))

            # Plot results
            if display:
                plot_picture(display=display, route=route_sort, n=len(route_sort), map=map)
                motion_move(route_sort, map)
        else:
            print("Single robot case - no conflict resolution needed")
            w = copy.deepcopy(map)
            w.nodes_array = w._create_nodes()
            Colony = AntColony(w, ants, iterations, p, Q)
            path = Colony.calculate_path()
            print(f"Path found: {path}")
            
            if display:
                plot_picture(display=display, route=[path], n=1, map=map)
    
    except Exception as e:
        print(f"Error occurred: {str(e)}")



















