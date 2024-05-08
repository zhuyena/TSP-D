import numpy as np


def read_data(file_path):
    file = open(file_path)
    lines = file.readlines()
    linenumbers = len(lines)
    matrix = []
    numbers = []
    location = []

    # 仓库节点
    depot = lines[7].split()
    location.append(depot)

    # 第9行----length-1行
    for line_number in range(9, linenumbers):
        line = lines[line_number]
        data = line.split(' ')
        location.append(data)

    # 计算距离矩阵
    nodes_num = len(location)
    disMatrix = np.zeros((nodes_num, nodes_num))
    for i in range(0, nodes_num):
        loci = np.array([location[i][0], location[i][1]])
        loci = [float(x) for x in loci]
        loci = np.array(loci)
        for j in range(0, nodes_num):
            locj = np.array([location[j][0], location[j][1]])
            locj = [float(x) for x in locj]
            locj = np.array(locj)
            disMatrix[i][j] = distance_euclidean(loci, locj)
    file.close()

    return [location, disMatrix]


def distance_euclidean(vector1, vector2):
    return np.sqrt(np.sum(np.square(vector1 - vector2)))

def nearest_neighbor(disMatrix):
    num_cities = disMatrix.shape[0]
    visited = [False] * num_cities  # 记录每个城市是否已访问
    tour = []  # 存储最终的遍历路线
    current_city = 0  # 从第一个城市开始

    # 遍历所有城市
    for _ in range(num_cities):
        tour.append(current_city)
        visited[current_city] = True

        # 找到当前城市的最近邻城市
        nearest_nb = None
        min_dist = np.inf
        for neighbor in range(num_cities):
            if not visited[neighbor] and disMatrix[current_city][neighbor] < min_dist:
                nearest_nb = neighbor
                min_dist = disMatrix[current_city][neighbor]

        current_city = nearest_nb

    # 将最后一个城市添加到路线中，形成闭环
    tour.append(0)

    return tour

