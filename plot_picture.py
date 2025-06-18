# 画图
import matplotlib.pyplot as plt
import numpy as np
import random
import os
from matplotlib.animation import FuncAnimation, PillowWriter

# 随机颜色
def randomcolor():
    colorArr = ['1','2','3','4','5','6','7','8','9','A','B','C','D','E','F']#16进制颜色
    color = ""
    for i in range(6):
        color += colorArr[random.randint(0,14)]
    return "#"+color

def plot_picture(display,route,n,map): #display 是否画图； route 所有路径； n个体数;map最开始读取到的地图
    if display > 0:
        ''' Represents the path in the map '''
        # size = np.shape(map.occupancy_map)
        #
        # fig = plt.figure(figsize=(size))

        for i, path in enumerate(route):  #
            x = []
            y = []
            for p in path:
                x.append(p[1])
                y.append(p[0])
            plt.plot(x, y, marker='o', color=randomcolor(), markersize=4, label=f'Path {i+1}')

        for i in range(n):  # 分别画两组的起始点
            # a = map.initial_node[i]
            plt.plot(map.initial_node[i][1], map.initial_node[i][0], 'bo', markersize=8, label='Start' if i == 0 else "")
            # red 实心圆；markersize设置标记大小；marker设置标记形状
            # b = map.final_node[i]
            plt.plot(map.final_node[i][1], map.final_node[i][0], 'r*', markersize=8, label='Goal' if i == 0 else "")

    plt.imshow(map.occupancy_map, cmap='gray', interpolation='nearest')
    plt.legend(loc='best')
    plt.show()
    plt.close()


def motion_move(route, map, save_gif=False, output_folder='output', filename='motion_animation.gif'):
    # 按路径长度排序
    route_sort = sorted(route, key=lambda i: len(i), reverse=True)
    max_length = len(route_sort[0])  # 使用最长路径的长度
    
    # 创建图形窗口
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 为每条路径生成随机颜色
    colors = [randomcolor() for _ in range(len(route_sort))]
    
    # 初始化函数
    def init():
        ax.clear()
        ax.imshow(map.occupancy_map, cmap='gray', interpolation='nearest')
        return []
    
    # 动画更新函数
    def animate(t):
        ax.clear()
        ax.imshow(map.occupancy_map, cmap='gray', interpolation='nearest')
        
        # 存储当前时间步的所有位置
        current_positions = []
        next_positions = []
        
        # 绘制每条路径到当前时间步
        for i, path in enumerate(route_sort):
            x = []
            y = []
            # 获取到当前时间步的路径点
            for p in path[:t+1]:
                x.append(p[1])
                y.append(p[0])
            ax.plot(x, y, marker='o', color=colors[i], markersize=4, label=f'Path {i+1}')
            
            # 记录当前位置和下一位置
            if t+1 < len(path):
                current_positions.append((path[t+1][0], path[t+1][1], i))
            if t + 2  < len(path):
                next_positions.append((path[t+2][0], path[t+2][1], i))
        for i, path in enumerate(route_sort):
            # 突出显示当前时间步的位置点（使用三角形标记）
            if t < len(path):
                current_point = path[t]
                ax.plot(current_point[1], current_point[0], marker='^', color=colors[i], 
                       markersize=10,  
                       label=f'Current {i+1}' if t == 0 else "")
        
        # 检查当前时间步的位置冲突
        for i in range(len(current_positions)): # 遍历所有当前位置
            for j in range(i+1, len(current_positions)): # 遍历当前位置之后的所有位置
                pos1, pos2 = current_positions[i], current_positions[j]
                if pos1[0] == pos2[0] and pos1[1] == pos2[1]:
                    ax.plot(pos1[1], pos1[0], 'ro', markersize=8, alpha=0.5)
        
        # 检查当前时间步和下一时间步之间的交换
        for i in range(len(current_positions)):
            for j in range(len(next_positions)):
                if current_positions[i][2] != next_positions[j][2]:  # 确保不是同一条路径
                    curr_pos = current_positions[i]
                    next_pos = next_positions[j]
                    # 检查是否存在交换
                    if (curr_pos[0] == next_pos[0] and curr_pos[1] == next_pos[1] and
                        next_positions[i][0] == current_positions[j][0] and 
                        next_positions[i][1] == current_positions[j][1]):
                        ax.plot(curr_pos[1], curr_pos[0], 'ro', markersize=8, alpha=0.5)
                        ax.plot(next_pos[1], next_pos[0], 'ro', markersize=8, alpha=0.5)
        
        # 绘制起点和终点
        for i in range(len(map.initial_node)):
            ax.plot(map.initial_node[i][1], map.initial_node[i][0], 'bo', markersize=8, label='Start' if i == 0 else "")
            ax.plot(map.final_node[i][1], map.final_node[i][0], 'r*', markersize=8, label='Goal' if i == 0 else "")
        
        ax.set_title(f'Time Step: {t}')
        ax.legend(loc='best')
        return []
    
    # 创建动画
    anim = FuncAnimation(fig, animate, init_func=init, frames=max_length, 
                        interval=1000, blit=False, repeat=True)
    
    # 如果需要保存GIF
    if save_gif:
        # 确保输出文件夹存在
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)
        
        # 构建完整的文件路径
        filepath = os.path.join(output_folder, filename)
        
        # 保存GIF
        writer = PillowWriter(fps=1)  # 1帧每秒
        anim.save(filepath, writer=writer)
        print(f"GIF动画已保存到: {filepath}")
    
    plt.show()
    plt.close()
