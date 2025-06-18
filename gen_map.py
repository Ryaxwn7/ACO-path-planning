import random
import os

def generate_map(rows, cols, num_robots, obstacle_density, filename):
    """
    生成地图并保存为txt文件
    :param rows: 行数
    :param cols: 列数
    :param num_robots: 机器人数量（2~6）
    :param obstacle_density: 障碍物密度（0~1之间的小数，建议0.1~0.4）
    :param filename: 保存的文件名
    """
    assert 2 <= num_robots <= 6, "机器人数量应在2~6之间"
    total_cells = rows * cols
    num_obstacles = int(total_cells * obstacle_density)

    # 初始化全空地
    grid = [['E' for _ in range(cols)] for _ in range(rows)]

    # 随机放障碍物
    obstacle_positions = set()
    while len(obstacle_positions) < num_obstacles:
        r = random.randint(0, rows-1)
        c = random.randint(0, cols-1)
        obstacle_positions.add((r, c))
    for (r, c) in obstacle_positions:
        grid[r][c] = 'O'

    # 随机放置起点和终点，不能和障碍物重叠
    free_positions = [(r, c) for r in range(rows) for c in range(cols) if grid[r][c] == 'E']
    random.shuffle(free_positions)
    assert len(free_positions) >= 2 * num_robots, "空地太少，无法放置所有机器人起点和终点"

    for i in range(num_robots):
        sr, sc = free_positions.pop()
        grid[sr][sc] = 'S'
    for i in range(num_robots):
        fr, fc = free_positions.pop()
        grid[fr][fc] = 'F'

    # 保存到文件
    with open(filename, 'w') as f:
        for row in grid:
            f.write('\t'.join(row) + '\n')
    print(f"地图已保存到 {filename}")

# 示例用法
if __name__ == '__main__':
    os.makedirs('maps', exist_ok=True)
    # 生成小地图
    generate_map(rows=8, cols=8, num_robots=3, obstacle_density=0.18, filename='maps/my_small.txt')
    # 生成中地图
    generate_map(rows=12, cols=12, num_robots=4, obstacle_density=0.22, filename='maps/my_middle.txt')
    # 生成大地图
    generate_map(rows=24, cols=24, num_robots=6, obstacle_density=0.4, filename='maps/my_big.txt')