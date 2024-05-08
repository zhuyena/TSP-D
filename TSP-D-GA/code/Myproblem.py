import copy
import math
import random
import bisect
from cmath import inf

import numpy as np

from Individual import Individual
from tools import read_data, nearest_neighbor
from LKH import lkh

class MyProblem:
    def __init__(self, file_path):
        # file = read_data(file_path)
        [location, disMatrix] = read_data(file_path)
        self.location = location  # 节点坐标
        self.dismatrix = disMatrix  # 距离矩阵
        self.node_num = len(self.location)  # 节点数量
        self.Iter = 250  # 迭代次数
        self.alpha = 2  # 无人机速度/卡车速度
        self.kappa = 500  # 无人机耐力范围
        self.drone_num = 1  # 无人机数量
        self.idv = Individual()
        self.LTL = math.ceil((self.node_num-1-self.drone_num)/(self.drone_num+1))
        # 用lkh求解初始解
        self.initial_tsp = lkh(self.dismatrix)

    # def initial_tour(self):
    #     # truck_tour = nearest_neighbor(self.disMatrix)
    #     truck_tour = lkh(self.dismatrix)
    #     return truck_tour

    def neighborhoods(self, pop, value, operator):

        if operator == '2opt':
            [solution_current, value_current, count] = self.n_2opt(pop, value)

        elif operator == 'swap':
            [solution_current, value_current, count] = self.n_swap(pop, value)

        elif operator == 'relocate':
            [solution_current, value_current, count] = self.n_relocate(pop, value)

        elif operator == '3opt':
            [solution_current, value_current, count] = self.n_3opt(pop, value)

        elif operator == 'swap2':
            [solution_current, value_current, count] = self.n_swap2(pop, value)

        # elif operator == 'id_greedy':
        #     solution_current = self.n_idgreedy(pop, value)
        #
        # elif operator == 'id_random':
        #     solution_current = self.n_idrandom(pop, value)
        #
        # elif operator == 'drone_launch_swap':
        #     solution_current = self.n_drone_launch_swap(pop, value)
        #
        # elif operator == 'drone_rdv_swap':
        #     solution_current = self.n_drone_rdv_swap(pop, value)
        #
        # elif operator == 'launch_rdv_swap':
        #     solution_current = self.n_launch_rdv_swap(pop, value)
        #
        # elif operator == 'convert_to_drone':
        #     solution_current = self.n_convert_to_drone(pop, value)

        else:
            print('wrong operations')

        return [solution_current, value_current, count]

    def neighborhoods1(self, pop, value, idvi, operator):

        if operator == '2opt':
            [solution_current, value_current, count] = self.n_2opt1(pop, value, idvi)

        elif operator == 'swap':
            [solution_current, value_current, count] = self.n_swap1(pop, value, idvi)

        elif operator == 'relocate':
            [solution_current, value_current, count] = self.n_relocate1(pop, value, idvi)

        elif operator == '3opt':
            [solution_current, value_current, count] = self.n_3opt1(pop, value, idvi)

        elif operator == 'swap2':
            [solution_current, value_current, count] = self.n_swap2_1(pop, value, idvi)

        else:
            print('wrong operations')

        return [solution_current, value_current, count]

    def n_2opt(self, pop, value):
        count = 0  # 用来记录有多少个解方案在局部搜索后得到了优化
        dismatrix = self.dismatrix
        # p在pop的索引i
        idx = 0
        for p in pop:
            route = copy.deepcopy(p)  # 第一层染色体
            n = len(route)

            max_num = 5  # 2-opt次数
            temp_distance = []
            temp_route = []

            for num in range(max_num):
                indices = random.sample(range(1, n - 1), 2)  # 生成两个不是头尾元素的随机索引
                i = min(indices)
                j = max(indices)
                a = dismatrix[route[i - 1]][route[j]] + dismatrix[route[i]][route[j + 1]]  # 2-opt后变化的路径
                b = dismatrix[route[i - 1]][route[i]] + dismatrix[route[j]][route[j + 1]]
                r = copy.deepcopy(route)
                r[i:j + 1] = reversed(r[i:j + 1])
                temp_distance.append(a-b)  # 减少的距离，如果距离少了，则数越小（会变负数）
                temp_route.append(r)

            min_index = temp_distance.index(min(temp_distance))  # 2-opt后距离减少最多的
            fin_route = temp_route[min_index]
            # 根据2_opt结果来重新分配
            [[assigned_route], assigned_value, complete_solution] = self.idv.assign_drone(fin_route, self.dismatrix, self.alpha, self.kappa)

            # 对比结果是否变好，如果结果更优，则保留
            # 原来的结果更好，则需要对比是不是比最后一个结果好
            if value[idx] < assigned_value:
                # 与最后一个对比
                if value[-1] < assigned_value:
                    pass
                else:
                    if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                        pass
                    else:  # 添加
                        value = value[:-1]  # 使用切片删除数组的最后一个元素
                        pop = pop[:-1]
                        bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                        index = value.index(assigned_value)
                        pop.insert(index, assigned_route)

            if value[idx] == assigned_value:
                pass

            # 局部搜索后的结果更好
            if value[idx] > assigned_value:
                # count计数+1
                count += 1
                # 判断种群中是否有一样的
                if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                    pass
                else:  # 添加
                    value = value[:-1]  # 使用切片删除数组的最后一个元素
                    pop = pop[:-1]
                    bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                    index = value.index(assigned_value)
                    pop.insert(index, assigned_route)

            idx += 1
            if idx >= 50:
                break

        return [pop, value, count]

    def n_2opt1(self, pop, value, idvi):
        count = 0  # 用来记录有多少个解方案在局部搜索后得到了优化
        dismatrix = self.dismatrix

        p = pop[idvi]
        route = copy.deepcopy(p)  # 第一层染色体
        n = len(route)

        max_num = 5  # 2-opt次数
        temp_distance = []
        temp_route = []

        for num in range(max_num):
            indices = random.sample(range(1, n - 1), 2)  # 生成两个不是头尾元素的随机索引
            i = min(indices)
            j = max(indices)
            a = dismatrix[route[i - 1]][route[j]] + dismatrix[route[i]][route[j + 1]]  # 2-opt后变化的路径
            b = dismatrix[route[i - 1]][route[i]] + dismatrix[route[j]][route[j + 1]]
            r = copy.deepcopy(route)
            r[i:j + 1] = reversed(r[i:j + 1])
            temp_distance.append(a - b)  # 减少的距离，如果距离少了，则数越小（会变负数）
            temp_route.append(r)

        min_index = temp_distance.index(min(temp_distance))  # 2-opt后距离减少最多的
        fin_route = temp_route[min_index]
        # 根据2_opt结果来重新分配
        [[assigned_route], assigned_value, complete_solution] = self.idv.assign_drone(fin_route, self.dismatrix,
                                                                                      self.alpha, self.kappa)

        # 对比结果是否变好，如果结果更优，则保留
        # 原来的结果更好，则需要对比是不是比最后一个结果好
        if value[idvi] < assigned_value:
            # 与最后一个对比
            if value[-1] < assigned_value:
                pass
            else:
                if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                    pass
                else:  # 添加
                    value = value[:-1]  # 使用切片删除数组的最后一个元素
                    pop = pop[:-1]
                    bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                    index = value.index(assigned_value)
                    pop.insert(index, assigned_route)

        if value[idvi] == assigned_value:
            pass

        # 局部搜索后的结果更好
        if value[idvi] > assigned_value:
            # count计数+1
            count += 1
            # 判断种群中是否有一样的
            if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                pass
            else:  # 添加
                value = value[:-1]  # 使用切片删除数组的最后一个元素
                pop = pop[:-1]
                bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                index = value.index(assigned_value)
                pop.insert(index, assigned_route)


        return [pop, value, count]

    def n_3opt(self, pop, value):
        count = 0  # 用来记录有多少个解方案在局部搜索后得到了优化
        dismatrix = self.dismatrix
        # p在pop的索引i
        idx = 0
        for p in pop:
            route = copy.deepcopy(p)  # 第一层染色体
            n = len(route)

            # 改为随机变换十次，在这十次里如果有变好，则保留变好的这一次，如果没有变好，就跳过

            max_num = 5  # 2-opt次数
            temp_distance = []
            temp_route = []

            for num in range(max_num):
                # 生成三个随索引
                i = random.randint(1, n-6)
                j = random.randint(i+2, n-4)
                k = random.randint(j+2, n-2)

                new_route = route[:i+1] + route[j+1:k+1] + route[i+1:j+1] + route[k+1:]
                new_distance = self.calculate_dis(new_route)
                temp_distance.append(new_distance)
                temp_route.append(new_route)

            min_index = temp_distance.index(min(temp_distance))  # 2-opt后距离减少最多的
            fin_route = temp_route[min_index]
            # 根据2_opt结果来重新分配
            [[assigned_route], assigned_value, complete_solution] = self.idv.assign_drone(fin_route, self.dismatrix, self.alpha, self.kappa)

            # 对比结果是否变好，如果结果更优，则保留
            # 原来的结果更好，则需要对比是不是比最后一个结果好
            if value[idx] < assigned_value:
                # 与最后一个对比
                if value[-1] < assigned_value:
                    pass
                else:
                    if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                        pass
                    else:  # 添加
                        value = value[:-1]  # 使用切片删除数组的最后一个元素
                        pop = pop[:-1]
                        bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                        index = value.index(assigned_value)
                        pop.insert(index, assigned_route)

            if value[idx] == assigned_value:
                pass

            # 局部搜索后的结果更好
            if value[idx] > assigned_value:
                # count计数+1
                count += 1
                # 判断种群中是否有一样的
                if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                    pass
                else:  # 添加
                    value = value[:-1]  # 使用切片删除数组的最后一个元素
                    pop = pop[:-1]
                    bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                    index = value.index(assigned_value)
                    pop.insert(index, assigned_route)

            idx += 1
            if idx >= 50:
                break
        return [pop, value, count]

    def n_3opt1(self, pop, value, idvi):
        count = 0  # 用来记录有多少个解方案在局部搜索后得到了优化
        dismatrix = self.dismatrix

        p = pop[idvi]
        route = copy.deepcopy(p)  # 第一层染色体
        n = len(route)

        # 改为随机变换十次，在这十次里如果有变好，则保留变好的这一次，如果没有变好，就跳过

        max_num = 5  # 2-opt次数
        temp_distance = []
        temp_route = []

        for num in range(max_num):
            # 生成三个随索引
            i = random.randint(1, n - 6)
            j = random.randint(i + 2, n - 4)
            k = random.randint(j + 2, n - 2)

            new_route = route[:i + 1] + route[j + 1:k + 1] + route[i + 1:j + 1] + route[k + 1:]
            new_distance = self.calculate_dis(new_route)
            temp_distance.append(new_distance)
            temp_route.append(new_route)

        min_index = temp_distance.index(min(temp_distance))  # 2-opt后距离减少最多的
        fin_route = temp_route[min_index]
        # 根据2_opt结果来重新分配
        [[assigned_route], assigned_value, complete_solution] = self.idv.assign_drone(fin_route, self.dismatrix,
                                                                                      self.alpha, self.kappa)

        # 对比结果是否变好，如果结果更优，则保留
        # 原来的结果更好，则需要对比是不是比最后一个结果好
        if value[idvi] < assigned_value:
            # 与最后一个对比
            if value[-1] < assigned_value:
                pass
            else:
                if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                    pass
                else:  # 添加
                    value = value[:-1]  # 使用切片删除数组的最后一个元素
                    pop = pop[:-1]
                    bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                    index = value.index(assigned_value)
                    pop.insert(index, assigned_route)

        if value[idvi] == assigned_value:
            pass

        # 局部搜索后的结果更好
        if value[idvi] > assigned_value:
            # count计数+1
            count += 1
            # 判断种群中是否有一样的
            if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                pass
            else:  # 添加
                value = value[:-1]  # 使用切片删除数组的最后一个元素
                pop = pop[:-1]
                bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                index = value.index(assigned_value)
                pop.insert(index, assigned_route)
        return [pop, value, count]

    def n_swap2(self, pop, value):
        count = 0  # 用来记录有多少个解方案在局部搜索后得到了优化
        dismatrix = self.dismatrix
        # p在pop的索引i
        idx = 0
        for p in pop:
            route = copy.deepcopy(p)  # 第一层染色体
            n = len(route)

            # 改为随机变换十次，在这十次里如果有变好，则保留变好的这一次，如果没有变好，就跳过

            max_num = 5  # 2-opt次数
            temp_distance = []
            temp_route = []

            for num in range(max_num):
                # 生成2个随索引
                i = random.randint(1, n-5)
                j = random.randint(i+2, n-3)

                swap1 = route[i:i+2]
                swap2 = route[j:j+2]
                route[i:i+2] = swap2
                route[j:j+2] = swap1

                # new_route = route[:i+1] + route[j+1:k+1] + route[i+1:j+1] + route[k+1:]
                new_distance = self.calculate_dis(route)
                temp_distance.append(new_distance)
                temp_route.append(route)

            min_index = temp_distance.index(min(temp_distance))  # 2-opt后距离减少最多的
            fin_route = temp_route[min_index]
            # 根据2_opt结果来重新分配
            [[assigned_route], assigned_value, complete_solution] = self.idv.assign_drone(fin_route, self.dismatrix, self.alpha, self.kappa)

            # 对比结果是否变好，如果结果更优，则保留
            # 原来的结果更好，则需要对比是不是比最后一个结果好
            if value[idx] < assigned_value:
                # 与最后一个对比
                if value[-1] < assigned_value:
                    pass
                else:
                    if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                        pass
                    else:  # 添加
                        value = value[:-1]  # 使用切片删除数组的最后一个元素
                        pop = pop[:-1]
                        bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                        index = value.index(assigned_value)
                        pop.insert(index, assigned_route)

            if value[idx] == assigned_value:
                pass

            # 局部搜索后的结果更好
            if value[idx] > assigned_value:
                # count计数+1
                count += 1
                # 判断种群中是否有一样的
                if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                    pass
                else:  # 添加
                    value = value[:-1]  # 使用切片删除数组的最后一个元素
                    pop = pop[:-1]
                    bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                    index = value.index(assigned_value)
                    pop.insert(index, assigned_route)

            idx += 1
            if idx >= 50:
                break
        return [pop, value, count]

    def n_swap2_1(self, pop, value, idvi):
        count = 0  # 用来记录有多少个解方案在局部搜索后得到了优化
        dismatrix = self.dismatrix

        p = pop[idvi]
        route = copy.deepcopy(p)  # 第一层染色体
        n = len(route)

        # 改为随机变换十次，在这十次里如果有变好，则保留变好的这一次，如果没有变好，就跳过

        max_num = 5  # 2-opt次数
        temp_distance = []
        temp_route = []

        for num in range(max_num):
            # 生成2个随索引
            i = random.randint(1, n - 5)
            j = random.randint(i + 2, n - 3)

            swap1 = route[i:i + 2]
            swap2 = route[j:j + 2]
            route[i:i + 2] = swap2
            route[j:j + 2] = swap1

            # new_route = route[:i+1] + route[j+1:k+1] + route[i+1:j+1] + route[k+1:]
            new_distance = self.calculate_dis(route)
            temp_distance.append(new_distance)
            temp_route.append(route)

        min_index = temp_distance.index(min(temp_distance))  # 2-opt后距离减少最多的
        fin_route = temp_route[min_index]
        # 根据2_opt结果来重新分配
        [[assigned_route], assigned_value, complete_solution] = self.idv.assign_drone(fin_route, self.dismatrix,
                                                                                      self.alpha, self.kappa)

        # 对比结果是否变好，如果结果更优，则保留
        # 原来的结果更好，则需要对比是不是比最后一个结果好
        if value[idvi] < assigned_value:
            # 与最后一个对比
            if value[-1] < assigned_value:
                pass
            else:
                if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                    pass
                else:  # 添加
                    value = value[:-1]  # 使用切片删除数组的最后一个元素
                    pop = pop[:-1]
                    bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                    index = value.index(assigned_value)
                    pop.insert(index, assigned_route)

        if value[idvi] == assigned_value:
            pass

        # 局部搜索后的结果更好
        if value[idvi] > assigned_value:
            # count计数+1
            count += 1
            # 判断种群中是否有一样的
            if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                pass
            else:  # 添加
                value = value[:-1]  # 使用切片删除数组的最后一个元素
                pop = pop[:-1]
                bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                index = value.index(assigned_value)
                pop.insert(index, assigned_route)

        return [pop, value, count]

    def n_relocate(self, pop, value):
        count = 0  # 用来记录有多少个解方案在局部搜索后得到了优化
        dismatrix = self.dismatrix
        # p在pop的索引i
        idx = 0
        for p in pop:
            route = copy.deepcopy(p)  # 第一层染色体
            n = len(route)

            max_num = 5  # relocate次数
            temp_distance = []
            temp_route = []

            for num in range(max_num):
                ind = random.randint(1, n-2)  # 生成一个不是首元素的随机索引
                r = copy.deepcopy(route)
                remove_node = r[ind]
                r = [x for x in r if x != remove_node]  # 删去这个节点
                insert = random.randint(1, n-2)
                while insert == ind:  # 如果插入的位置和原来一样了，就重新生成插入位置
                    insert = random.randint(1, n - 2)
                r.insert(insert, remove_node)  # 重新插入新位置
                dis = self.calculate_dis(r)
                temp_distance.append(dis)  # 减少的距离，如果距离少了，则数越小（会变负数）
                temp_route.append(r)

            min_index = temp_distance.index(min(temp_distance))  # 2-opt后距离减少最多的
            fin_route = temp_route[min_index]
            # 根据2_opt结果来重新分配
            [[assigned_route], assigned_value, complete_solution] = self.idv.assign_drone(fin_route, self.dismatrix, self.alpha, self.kappa)

            # 对比结果是否变好，如果结果更优，则保留
            # 原来的结果更好，则需要对比是不是比最后一个结果好
            if value[idx] < assigned_value:
                # 与最后一个对比
                if value[-1] < assigned_value:
                    pass
                else:
                    if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                        pass
                    else:  # 添加
                        value = value[:-1]  # 使用切片删除数组的最后一个元素
                        pop = pop[:-1]
                        bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                        index = value.index(assigned_value)
                        pop.insert(index, assigned_route)

            if value[idx] == assigned_value:
                pass

            # 局部搜索后的结果更好
            if value[idx] > assigned_value:
                # count计数+1
                count += 1
                # 判断种群中是否有一样的
                if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                    pass
                else:  # 添加
                    value = value[:-1]  # 使用切片删除数组的最后一个元素
                    pop = pop[:-1]
                    bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                    index = value.index(assigned_value)
                    pop.insert(index, assigned_route)

            idx += 1
            if idx >= 50:
                break
        return [pop, value, count]

    def n_relocate1(self, pop, value, idvi):
        count = 0  # 用来记录有多少个解方案在局部搜索后得到了优化
        dismatrix = self.dismatrix

        p = pop[idvi]
        route = copy.deepcopy(p)  # 第一层染色体
        n = len(route)

        max_num = 5  # relocate次数
        temp_distance = []
        temp_route = []

        for num in range(max_num):
            ind = random.randint(1, n - 2)  # 生成一个不是首元素的随机索引
            r = copy.deepcopy(route)
            remove_node = r[ind]
            r = [x for x in r if x != remove_node]  # 删去这个节点
            insert = random.randint(1, n - 2)
            while insert == ind:  # 如果插入的位置和原来一样了，就重新生成插入位置
                insert = random.randint(1, n - 2)
            r.insert(insert, remove_node)  # 重新插入新位置
            dis = self.calculate_dis(r)
            temp_distance.append(dis)  # 减少的距离，如果距离少了，则数越小（会变负数）
            temp_route.append(r)

        min_index = temp_distance.index(min(temp_distance))  # 2-opt后距离减少最多的
        fin_route = temp_route[min_index]
        # 根据2_opt结果来重新分配
        [[assigned_route], assigned_value, complete_solution] = self.idv.assign_drone(fin_route, self.dismatrix,
                                                                                      self.alpha, self.kappa)

        # 对比结果是否变好，如果结果更优，则保留
        # 原来的结果更好，则需要对比是不是比最后一个结果好
        if value[idvi] < assigned_value:
            # 与最后一个对比
            if value[-1] < assigned_value:
                pass
            else:
                if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                    pass
                else:  # 添加
                    value = value[:-1]  # 使用切片删除数组的最后一个元素
                    pop = pop[:-1]
                    bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                    index = value.index(assigned_value)
                    pop.insert(index, assigned_route)

        if value[idvi] == assigned_value:
            pass

        # 局部搜索后的结果更好
        if value[idvi] > assigned_value:
            # count计数+1
            count += 1
            # 判断种群中是否有一样的
            if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                pass
            else:  # 添加
                value = value[:-1]  # 使用切片删除数组的最后一个元素
                pop = pop[:-1]
                bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                index = value.index(assigned_value)
                pop.insert(index, assigned_route)
        return [pop, value, count]

    def n_idgreedy(self, pop, value):
        dismatrix = self.dismatrix
        # truck_route = pop[0].copy()
        solution_current = []
        idx = 0
        for p in pop:
            truck_route = copy.deepcopy(p[1][0])
            drone_sortie = copy.deepcopy(p[1][1])
            max_cost = 0
            for sortie in drone_sortie:
                s_cost = (dismatrix[sortie[0]][sortie[1]] + dismatrix[sortie[1]][sortie[2]])/self.alpha
                if s_cost > max_cost:
                    max_cost = s_cost
                    max_sortie = sortie
                    remove_node = max_sortie[1]
            # 将花销最大的删除
            drone_sortie = [x for x in drone_sortie if x != max_sortie]
            # 将删除的节点重新分配给无人机
            # 候选起降点
            cand_set = []
            for i in range(0, len(truck_route) - 1):
                cand = [truck_route[i], truck_route[i + 1]]
                cand_set.append(cand)
            for q in drone_sortie:
                idx1 = truck_route.index(q[0])
                if q[2] == 0:
                    idx2 = len(truck_route) - 1
                else:
                    idx2 = truck_route.index(q[2])
                for index in range(idx1,idx2):
                    cand_set[index] = [0,0]
            cand_set = [x for x in cand_set if x != [0,0]]
            # 在剩下的候选起降点中选择花费最少的
            mincost = inf
            for node in cand_set:
                truck_cost = self.dismatrix[node[0]][node[1]]
                dnode_cost = (self.dismatrix[node[0]][remove_node] + self.dismatrix[remove_node][node[1]]) / self.alpha
                cost = max(truck_cost, dnode_cost)
                if cost < mincost:
                    mincost = cost
                    minsortie = [node[0], remove_node, node[1]]
            drone_sortie.append(minsortie)
            # 给无人机架次排序
            l = len(drone_sortie)
            drone_sorties = []
            launch_list = truck_route
            launch_list = launch_list[:len(launch_list) - 1]
            for t in launch_list:
                for idx in range(0, l):
                    if drone_sortie[idx][0] == t:
                        drone_sorties.append(drone_sortie[idx])
            final_sol = [truck_route, drone_sorties]


            # 合并成big_tour
            final_solution = []
            # 将解方案合成big tour
            big_tour = copy.deepcopy(truck_route)
            for s in drone_sorties:
                if s[2] != 0:
                    idx = big_tour.index(s[2])
                    big_tour.insert(idx, s[1])
                else:
                    big_tour.insert(len(big_tour) - 1, s[1])
            final_solution.append(big_tour)
            final_solution.append(final_sol)

            solution_current.append(final_solution)

        return solution_current

    def n_idrandom(self, pop, value):
        dismatrix = self.dismatrix
        # truck_route = pop[0].copy()
        solution_current = []
        idx = 0
        for p in pop:
            truck_route = copy.deepcopy(p[1][0])
            drone_sortie = copy.deepcopy(p[1][1])
            # 随机生成一个数
            random_number = random.randint(0, len(drone_sortie)-1)
            remove_sortie = drone_sortie[random_number]
            remove_node = remove_sortie[1]
            drone_sortie = [x for x in drone_sortie if x != remove_sortie]
            # 将删除的节点重新分配给无人机
            # 候选起降点
            cand_set = []
            for i in range(0, len(truck_route) - 1):
                cand = [truck_route[i], truck_route[i + 1]]
                cand_set.append(cand)
            for q in drone_sortie:
                idx1 = truck_route.index(q[0])
                if q[2] == 0:
                    idx2 = len(truck_route) - 1
                else:
                    idx2 = truck_route.index(q[2])
                for index in range(idx1, idx2):
                    cand_set[index] = [0, 0]
            cand_set = [x for x in cand_set if x != [0, 0]]
            # 在剩下的候选起降点中选择花费最少的
            mincost = inf
            for node in cand_set:
                truck_cost = self.dismatrix[node[0]][node[1]]
                dnode_cost = (self.dismatrix[node[0]][remove_node] + self.dismatrix[remove_node][node[1]]) / self.alpha
                cost = max(truck_cost, dnode_cost)
                if cost < mincost:
                    mincost = cost
                    minsortie = [node[0], remove_node, node[1]]
            drone_sortie.append(minsortie)
            # 给无人机架次排序
            l = len(drone_sortie)
            drone_sorties = []
            launch_list = truck_route
            launch_list = launch_list[:len(launch_list) - 1]
            for t in launch_list:
                for idx in range(0, l):
                    if drone_sortie[idx][0] == t:
                        drone_sorties.append(drone_sortie[idx])
            final_sol = [truck_route, drone_sorties]

            # 合并成big_tour
            final_solution = []
            # 将解方案合成big tour
            big_tour = copy.deepcopy(truck_route)
            for s in drone_sorties:
                if s[2] != 0:
                    idx = big_tour.index(s[2])
                    big_tour.insert(idx, s[1])
                else:
                    big_tour.insert(len(big_tour) - 1, s[1])
            final_solution.append(big_tour)
            final_solution.append(final_sol)

            solution_current.append(final_solution)

        return solution_current

    def n_swap(self, pop, value):
        dismatrix = self.dismatrix
        solution_current = []
        count = 0

        idx = 0
        for p in pop:
            # truck_route = copy.deepcopy(p[1][0])
            route = copy.deepcopy(p)  # 第一层染色体
            # n = len(truck_route)
            n = len(route)

            max_num = 5  # 2-opt次数
            temp_distance = []
            temp_route = []
            for num in range(max_num):
                indices = random.sample(range(1, n - 1), 2)  # 生成两个不是头尾元素的随机索引
                i = min(indices)
                j = max(indices)
                swap1 = route[i]
                swap2 = route[j]
                r = copy.deepcopy(route)
                r[i] = swap2  # 交换两个索引位的元素
                r[j] = swap1
                if j == i+1:  # 如果相邻
                    a = dismatrix[route[i - 1]][route[j]] + dismatrix[route[i]][route[j + 1]]  # swap后变化的路径
                    b = dismatrix[route[i - 1]][route[i]] + dismatrix[route[j]][route[j + 1]]
                    temp_distance.append(a - b)  # 减少的距离，如果距离少了，则数越小（会变负数）
                    temp_route.append(r)
                else:
                    a = dismatrix[route[i - 1]][route[j]] + dismatrix[route[i + 1]][route[j]] + dismatrix[route[i]][route[j + 1]] + dismatrix[route[i]][route[j - 1]]  # swap后变化的路径
                    b = dismatrix[route[i - 1]][route[i]] + dismatrix[route[i + 1]][route[i]] + dismatrix[route[j]][route[j + 1]] + dismatrix[route[j]][route[j - 1]]
                    temp_distance.append(a - b)  # 减少的距离，如果距离少了，则数越小（会变负数）
                    temp_route.append(r)

            min_index = temp_distance.index(min(temp_distance))  # swap后距离减少最多的
            fin_route = temp_route[min_index]

            # 根据swap结果来重新分配
            [[assigned_route], assigned_value, complete_solution] = self.idv.assign_drone(fin_route, self.dismatrix, self.alpha, self.kappa)

            final_solution = []
            # 对比结果是否变好，如果结果更优，则保留
            if value[idx] < assigned_value:  # 原来的结果更好，则需要对比是不是比最后一个结果好
                # solution_current.append(p)
                # value_current.append(value[idx])

                # 与最后一个对比
                if value[-1] < assigned_value:
                    pass
                else:
                    if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                        pass
                    else:  # 添加
                        value = value[:-1]  # 使用切片删除数组的最后一个元素
                        pop = pop[:-1]
                        bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                        index = value.index(assigned_value)
                        pop.insert(index, assigned_route)

            if value[idx] == assigned_value:
                pass

            # 局部搜索后的结果更好
            if value[idx] > assigned_value:
                count += 1
                # 判断种群中是否有一样的
                if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                    pass
                else:  # 添加
                    value = value[:-1]  # 使用切片删除数组的最后一个元素
                    pop = pop[:-1]
                    bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                    index = value.index(assigned_value)
                    pop.insert(index, assigned_route)
            idx += 1
            if idx >= 50:
                break
        return [pop, value, count]

    def n_swap1(self, pop, value, idvi):
        dismatrix = self.dismatrix
        solution_current = []
        count = 0
        p = pop[idvi]
        route = copy.deepcopy(p)  # 第一层染色体
        # n = len(truck_route)
        n = len(route)

        max_num = 5  # 2-opt次数
        temp_distance = []
        temp_route = []
        for num in range(max_num):
            indices = random.sample(range(1, n - 1), 2)  # 生成两个不是头尾元素的随机索引
            i = min(indices)
            j = max(indices)
            swap1 = route[i]
            swap2 = route[j]
            r = copy.deepcopy(route)
            r[i] = swap2  # 交换两个索引位的元素
            r[j] = swap1
            if j == i + 1:  # 如果相邻
                a = dismatrix[route[i - 1]][route[j]] + dismatrix[route[i]][route[j + 1]]  # swap后变化的路径
                b = dismatrix[route[i - 1]][route[i]] + dismatrix[route[j]][route[j + 1]]
                temp_distance.append(a - b)  # 减少的距离，如果距离少了，则数越小（会变负数）
                temp_route.append(r)
            else:
                a = dismatrix[route[i - 1]][route[j]] + dismatrix[route[i + 1]][route[j]] + dismatrix[route[i]][
                    route[j + 1]] + dismatrix[route[i]][route[j - 1]]  # swap后变化的路径
                b = dismatrix[route[i - 1]][route[i]] + dismatrix[route[i + 1]][route[i]] + dismatrix[route[j]][
                    route[j + 1]] + dismatrix[route[j]][route[j - 1]]
                temp_distance.append(a - b)  # 减少的距离，如果距离少了，则数越小（会变负数）
                temp_route.append(r)

        min_index = temp_distance.index(min(temp_distance))  # swap后距离减少最多的
        fin_route = temp_route[min_index]

        # 根据swap结果来重新分配
        [[assigned_route], assigned_value, complete_solution] = self.idv.assign_drone(fin_route, self.dismatrix,
                                                                                      self.alpha, self.kappa)

        final_solution = []
        # 对比结果是否变好，如果结果更优，则保留
        if value[idvi] < assigned_value:  # 原来的结果更好，则需要对比是不是比最后一个结果好
            # solution_current.append(p)
            # value_current.append(value[idx])

            # 与最后一个对比
            if value[-1] < assigned_value:
                pass
            else:
                if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                    pass
                else:  # 添加
                    value = value[:-1]  # 使用切片删除数组的最后一个元素
                    pop = pop[:-1]
                    bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                    index = value.index(assigned_value)
                    pop.insert(index, assigned_route)

        if value[idvi] == assigned_value:
            pass

        # 局部搜索后的结果更好
        if value[idvi] > assigned_value:
            count += 1
            # 判断种群中是否有一样的
            if self.is_value_in_list(assigned_value, value):  # 存在相同的适应度值
                pass
            else:  # 添加
                value = value[:-1]  # 使用切片删除数组的最后一个元素
                pop = pop[:-1]
                bisect.insort(value, assigned_value)  # 将元素插入到从小到大排列的数组中
                index = value.index(assigned_value)
                pop.insert(index, assigned_route)
        return [pop, value, count]

    def n_drone_launch_swap(self,pop, value):
        solution_current = []
        for p in pop:
            sol = []
            final_solution = []
            truck_route = copy.deepcopy(p[1][0])
            drone_sortie = copy.deepcopy(p[1][1])
            random_sortie = random.choice(drone_sortie)
            # 判断是否与仓库节点交换，若从仓库节点起飞，则不进行交换
            if random_sortie[0] == 0:
                final_solution = p
            else:
                launch = random_sortie[0]
                service = random_sortie[1]
                index = drone_sortie.index(random_sortie)
                drone_sortie[index][0] = service
                drone_sortie[index][1] = launch
                # 若修改的节点与前一个架次有关联，则修改前一个架次的信息
                if index != 0 and drone_sortie[index-1][2] == launch:
                    drone_sortie[index - 1][2] = service
                # 修改卡车路径相应节点
                truck_index = truck_route.index(launch)
                truck_route[truck_index] = service
                sol.append(truck_route)
                sol.append(drone_sortie)

                # 合并成big_tour
                # final_solution = []
                # 将解方案合成big tour
                big_tour = copy.deepcopy(truck_route)
                for s in drone_sortie:
                    if s[2] != 0:
                        idx = big_tour.index(s[2])
                        big_tour.insert(idx, s[1])
                    else:
                        big_tour.insert(len(big_tour) - 1, s[1])
                final_solution.append(big_tour)
                final_solution.append(sol)

            solution_current.append(final_solution)
        return solution_current

    def n_drone_rdv_swap(self, pop, value):
        solution_current = []
        for p in pop:
            sol = []
            final_solution = []
            truck_route = copy.deepcopy(p[1][0])
            drone_sortie = copy.deepcopy(p[1][1])
            random_sortie = random.choice(drone_sortie)
            # 判断是否与仓库节点交换，若从仓库节点起飞，则不进行交换
            if random_sortie[2] == 0:
                final_solution = p
            else:
                rdv = random_sortie[2]
                service = random_sortie[1]
                index = drone_sortie.index(random_sortie)
                drone_sortie[index][2] = service
                drone_sortie[index][1] = rdv
                # 若修改的节点与后一个架次有关联，则修改后一个架次的信息
                if index != len(drone_sortie)-1 and drone_sortie[index + 1][0] == rdv:
                    drone_sortie[index + 1][0] = service
                # 修改卡车路径相应节点
                truck_index = truck_route.index(rdv)
                truck_route[truck_index] = service
                sol.append(truck_route)
                sol.append(drone_sortie)

                # 合并成big_tour
                # final_solution = []
                # 将解方案合成big tour
                big_tour = copy.deepcopy(truck_route)
                for s in drone_sortie:
                    if s[2] != 0:
                        idx = big_tour.index(s[2])
                        big_tour.insert(idx, s[1])
                    else:
                        big_tour.insert(len(big_tour) - 1, s[1])
                final_solution.append(big_tour)
                final_solution.append(sol)

            solution_current.append(final_solution)
        return solution_current

    def n_launch_rdv_swap(self, pop, value):
        solution_current = []
        for p in pop:
            sol = []
            final_solution = []
            truck_route = copy.deepcopy(p[1][0])
            drone_sortie = copy.deepcopy(p[1][1])
            random_sortie = random.choice(drone_sortie)
            # 判断是否与仓库节点交换，若从仓库节点起飞或降落，则不进行交换
            if random_sortie[0] == 0 or random_sortie[2] == 0:
                final_solution = p
            else:
                launch = random_sortie[0]
                rdv = random_sortie[2]
                index = drone_sortie.index(random_sortie)
                drone_sortie[index][0] = rdv
                drone_sortie[index][2] = launch
                # 若修改的节点与前一个架次有关联，则修改前一个架次的信息
                if index != 0 and drone_sortie[index - 1][2] == launch:
                    drone_sortie[index - 1][2] = rdv
                if index != len(drone_sortie)-1 and drone_sortie[index + 1][0] == rdv:
                    drone_sortie[index + 1][0] = launch
                truck_index1 = truck_route.index(launch)
                truck_index2 = truck_route.index(rdv)
                truck_route[truck_index1] = rdv
                truck_route[truck_index2] = launch
                sol.append(truck_route)
                sol.append(drone_sortie)

                # 合并成big_tour
                # final_solution = []
                # 将解方案合成big tour
                big_tour = copy.deepcopy(truck_route)
                for s in drone_sortie:
                    if s[2] != 0:
                        idx = big_tour.index(s[2])
                        big_tour.insert(idx, s[1])
                    else:
                        big_tour.insert(len(big_tour) - 1, s[1])
                final_solution.append(big_tour)
                final_solution.append(sol)

            solution_current.append(final_solution)

        return solution_current

    def n_convert_to_drone(self, pop, value):

        solution_current = []
        cand = []  # 候选的连续三个卡车节点的组合
        for p in pop:
            sol = []
            final_solution = []
            truck_route = copy.deepcopy(p[1][0])
            truck_only_route = copy.deepcopy(p[1][0])
            drone_sortie = copy.deepcopy(p[1][1])
            for s in drone_sortie:
                # 移除无人机和卡车公共节点
                remove_node1 = s[0]
                remove_node2 = s[2]
                truck_only_route = [x for x in truck_only_route if x != remove_node1]
                truck_only_route = [x for x in truck_only_route if x != remove_node2]
            # 移除仓库节点
            truck_only_route = [x for x in truck_only_route if x != 0]
            # 选择连续的三个卡车节点
            for i in range(len(truck_only_route)-2):
                idx1 = truck_route.index(truck_only_route[i])
                idx2 = truck_route.index(truck_only_route[i+1])
                idx3 = truck_route.index(truck_only_route[i+2])
                if idx2 == idx1+1 and idx3 == idx2+1:
                    convert = [truck_only_route[i], truck_only_route[i+1], truck_only_route[i+2]]
                    cand.append(convert)

                else:
                    i += 1

            # 如果存在多个连续的卡车节点组合，随机选择其中一个
            if cand:
                convert_cand = random.choice(cand)
                # 将连续节点变成无人机架次
                convert_sortie = [convert_cand[0], convert_cand[1], convert_cand[2]]
                # 将新的架次加入原有的架次集合，并删除卡车节点
                drone_sortie.append(convert_sortie)
                truck_only_route = [x for x in truck_only_route if x != convert_cand[1]]
                truck_route = [x for x in truck_route if x != convert_cand[1]]


        return solution_current

    def is_value_in_list(self,value, valuelist):
        return value in valuelist

    def calculate_dis(self, r):
        distance = 0
        for i in range(len(r)-1):
            distance += self.dismatrix[r[i]][r[i+1]]
        return distance

