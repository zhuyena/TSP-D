import bisect
import copy
import random
from math import inf

from Individual import Individual


class Population:
    def __init__(self, popsize, problem):
        self.popsize = popsize
        self.problem = problem
        self.dismatrix = problem.dismatrix
        self.alpha = problem.alpha
        self.kappa = problem.kappa
        self.idv = Individual()
        self.crossover_rate = 0
        self.mutation_rate = 1

    def creat_pop(self, popsize):
        P = []
        # for i in range(popsize):
        solution = self.problem.initial_tsp
        P.append(solution)
        l = len(solution) - 2

        # 进行随机扰动
        while len(P) < self.popsize:
            idx1 = random.randint(1, l)
            idx2 = random.randint(1, l)

            if idx1 > idx2:
                temp = idx1
                idx1 = idx2
                idx2 = temp

            change_sol = copy.deepcopy(solution)  # 复制求解器求解的初始解
            r = random.random()  # 生成一个随机数
            if r < 0.5:
                change_sol[idx1:idx2 + 1] = change_sol[idx1:idx2 + 1][::-1]
            else:
                elements_to_shuffle = change_sol[idx1:idx2 + 1]
                random.shuffle(elements_to_shuffle)
                change_sol[idx1:idx2 + 1] = elements_to_shuffle

            if self.check_sol(change_sol, P):  # check == true 原来种群中没有这个解，则加入
                P.append(change_sol)

        return P

    def check_sol(self, sol, P):
        for idv in P:
            if sol == idv:
                return False
        return True

    def next_pop(self, Pop, rate=[0.8, 1]):
        self.crossover_rate = rate[0]  # 交叉概率
        self.mutation_rate = rate[1]  # 变异概率


        idv_num = 0  # 生成的个体数
        while idv_num < self.popsize or len(Pop[0]) < self.popsize:  # 子代需要达到种群大小
            [parent_a, a_value] = self.binary_tournament(Pop)  # 二进制锦标赛
            if random.random() < self.crossover_rate:  # 若选择的染色体达到交叉概率，则利用二进制锦标赛选择另一个染色体与之进行交叉
                [parent_b, b_value] = self.binary_tournament(Pop)
                # 卡车与无人机的顺序交叉，交叉产生两个子代
                Pop = self.crossover(parent_a, parent_b, Pop[0], Pop[1])
                idv_num += 2

            else:  # 若该染色体没到交叉概率，则保留
                if random.random() < self.mutation_rate:  # 看是否到达变异概率

                    # 从四种变异算子中随机选择，并且对变异后的算子进行重新分配
                    p = self.mutation(parent_a)  # 整个big_tour进行变异
                    children = []
                    children.append(p)  # 把第一层的染色体进行分配
                    # 第二层染色体的卡车无人机通过分配来决定

                    Pop = self.assigned(children, Pop[0], Pop[1])

                    idv_num += 1
        return Pop

    def binary_tournament(self, population):
        # 二进制锦标赛
        # parent_a = random.choice(population[0])  # 随机选择染色体
        random_number1 = random.randint(0, len(population[0])-1)  # 生成一个随机数
        parent_a = population[0][random_number1]  # 随机选择的染色体
        a_value = population[1][random_number1]  # 该染色体的value值
        # 定位到该染色体的下标
        random_number2 = random.randint(0, len(population[0])-1)  # 生成一个随机数
        parent_b = population[0][random_number2]  # 随机选择的染色体
        b_value = population[1][random_number2]  # 该染色体的value值

        # parent_b = random.choice(population[0])、
        if a_value < b_value:
            return [parent_a, a_value]
        else:
            return [parent_b,b_value]
        # return self.distance_comparision(parent_a, parent_b)  # 比较总距离，选择距离少的

    # def distance_comparision(self, parent_a, parent_b):
    #     total_distance_a = self.evaluate(parent_a)
    #     total_distance_b = self.evaluate(parent_b)
    #
    #     if total_distance_a < total_distance_b:
    #         return [parent_a, total_distance_a]
    #     else:
    #         return [parent_b,total_distance_b]

    # def caculate_tour(self,tour):
    #     total_distance = 0
    #     i = 0
    #     while i < len(tour)-1:
    #         total_distance += self.dismatrix[tour[i]][tour[i+1]]
    #         i += 1
    #     return total_distance
    def evaluate(self, parent):
        truck = parent[1][0]
        drone = parent[1][1]
        total_distance = 0
        i = 0

        while i < len(truck) - 1:  # 遍历卡车路径的节点
            trucki = truck[i]
            # 首先判断无人机架次中是否有以这个为起点的架次
            while drone:
                remove = inf
                for n in range(0,len(drone)):
                    if drone[n][0] == trucki:
                        remove = 0
                        sortie = (self.dismatrix[drone[n][0]][drone[n][1]] + self.dismatrix[drone[n][1]][drone[n][2]]) / self.alpha
                        if drone[n][2] == 0:
                            index = len(truck) - 1
                            truck_cost = 0
                            for idx in range(i, index):
                                truck_cost += self.dismatrix[truck[idx]][truck[idx + 1]]
                            total_distance += max(sortie, truck_cost)
                            i = index
                            # # 将计算过的架次从无人机数组中移除
                            # drone = drone[:n]+drone[n+1:]
                            remove = n
                            break
                        else:
                            index = truck.index(drone[n][2])
                            truck_cost = 0
                            for idx in range(i, index):
                                truck_cost += self.dismatrix[truck[idx]][truck[idx + 1]]
                            total_distance += max(sortie, truck_cost)
                            i = index
                            # # 将计算过的架次从无人机数组中移除
                            # drone = drone[:n]+drone[n+1:]
                            remove = n

                if remove == inf:
                    total_distance += self.dismatrix[truck[i]][truck[i + 1]]
                    # i += 1
                break

            if i == len(truck)-1:
                break
            elif i != len(truck)-1 and drone == []:
                total_distance += self.dismatrix[truck[i]][truck[i + 1]]
                i = i + 1
            elif i != len(truck)-1 and drone != [] and remove != inf:
                drone = drone[:remove] + drone[remove + 1:]
            elif i != len(truck) - 1 and drone != [] and remove == inf:
                i += 1

        return total_distance

    def crossover(self, parent_a, parent_b, final_pop, final_pop_value):
        a = copy.deepcopy(parent_a)  # parent_a的big tour
        a = a[1:-1]
        b = copy.deepcopy(parent_b)  # parent_b的big tour
        b = b[1:-1]

        # 交叉位置
        y = random.randint(0, len(a))
        # 记录交叉项
        fragment1 = a[y:]
        fragment2 = b[y:]
        aa = []
        bb = []
        for i in a[:y]:
            while i in fragment2:
                i = fragment1[fragment2.index(i)]
            aa.append(i)
        for i in b[:y]:
            while i in fragment1:
                i = fragment2[fragment1.index(i)]
            bb.append(i)

        children1 = aa + fragment2
        children1.insert(0,0)
        children1.append(0)
        children2 = bb + fragment1
        children2.insert(0, 0)
        children2.append(0)

        # 第二层染色体的卡车无人机通过分配来决定
        [final_pop, final_pop_value]= self.assigned([children1], final_pop, final_pop_value)  # 交叉后的第一层染色体进行分配

        [final_pop, final_pop_value] = self.assigned([children2] , final_pop, final_pop_value)

        return [final_pop, final_pop_value]

    # 一点交叉
    def crossover1(self, parent_a, parent_b):
        # child1 = parent_a
        # child2 = parent_b
        # 提取所有节点
        a = parent_a[0].copy()
        a = a[1:-1]
        la = len(a)
        for i in parent_a[1]:
            a.append(i[1])
        b = parent_b[0].copy()
        b = b[1:-1]
        lb = len(b)
        for j in parent_b[1]:
            b.append(j[1])

        # 随机选择交叉点位
        cross_point = random.randint(0, len(a)-1)
        # 交换a与b中交叉点位的元素
        point1 = a[cross_point]
        point2 = b[cross_point]
        a[cross_point] = point2
        b[cross_point] = point1

        point12 = inf
        point22 = inf

        # 处理冲突
        for i in range(0,len(a)):
            if i != cross_point:
                if a[i] == point2:
                    point12 = i  # 记录冲突位置
                    a[i] = point1
                    break
        for j in range(0,len(b)):
            if j != cross_point:
                if b[j] == point1:
                    point22 = j
                    b[j] = point2  # 记录冲突位置
                    break

        parenta1 = copy.deepcopy(parent_a[1])
        parentb1 = copy.deepcopy(parent_b[1])
        if point12 == inf and point22 == inf:
            child1 = parent_a
            child2 = parent_b
        else:
            if point12 >= la and cross_point >= la:  # 只交换了无人机节点
                # 遍历无人机节点，交换节点位置
                for sortie in parenta1:
                    node = sortie[1]
                    if node == a[cross_point]:
                        sortie[1] = a[point12]
                    if node == a[point12]:
                        sortie[1] = a[cross_point]
                drone_sortie = parenta1
                truck_route = a[:la]
                truck_route.insert(0, 0)
                truck_route.append(0)
                child1 = [truck_route, drone_sortie]

            if point22 >= lb and cross_point >= lb:  # 只交换了无人机节点
                for sortie in parentb1:
                    node = sortie[1]
                    if node == b[cross_point]:
                        sortie[1] = b[point22]
                    if node == b[point22]:
                        sortie[1] = b[cross_point]
                drone_sortie = parentb1
                truck_route = b[:lb]
                truck_route.insert(0, 0)
                truck_route.append(0)
                child2 = [truck_route, drone_sortie]

            if cross_point < la and point12 < la:  # 只交换了卡车节点
                for sortie in parenta1:
                    node_idx = 0
                    for node in sortie:
                        # node_idx = sortie.index(node)
                        if node == a[cross_point]:
                            sortie[node_idx] = a[point12]
                        if node == a[point12]:
                            sortie[node_idx] = a[cross_point]
                        node_idx += 1
                drone_sortie = parenta1
                truck_route = a[:la]
                truck_route.insert(0, 0)
                truck_route.append(0)
                child1 = [truck_route, drone_sortie]

            if cross_point < lb and point22 < lb:  # 只交换了卡车节点
                for sortie in parentb1:
                    node_idx = 0
                    for node in sortie:
                        # node_idx = sortie.index(node)
                        if node == b[cross_point]:
                            sortie[node_idx] = b[point22]
                        if node == b[point22]:
                            sortie[node_idx] = b[cross_point]
                        node_idx += 1
                drone_sortie = parentb1
                truck_route = b[:lb]
                truck_route.insert(0, 0)
                truck_route.append(0)
                child2 = [truck_route, drone_sortie]

            if cross_point < la and point12 >= la:  # 卡车与无人机节点互换
                for sortie in parenta1:
                    node_idx = 0
                    for node in sortie:
                        # node_idx = sortie.index(node)
                        if node == a[cross_point]:
                            sortie[node_idx] = a[point12]
                        if node == a[point12]:
                            sortie[node_idx] = a[cross_point]
                        node_idx += 1
                drone_sortie = parenta1
                truck_route = a[:la]
                truck_route.insert(0, 0)
                truck_route.append(0)
                child1 = [truck_route, drone_sortie]

            if cross_point < lb and point22 >= lb:  # 卡车与无人机节点互换
                for sortie in parentb1:
                    node_idx = 0
                    for node in sortie:
                        # node_idx = sortie.index(node)
                        if node == b[cross_point]:
                            sortie[node_idx] = b[point22]
                        if node == b[point22]:
                            sortie[node_idx] = b[cross_point]
                        node_idx += 1
                drone_sortie = parentb1
                truck_route = b[:lb]
                truck_route.insert(0, 0)
                truck_route.append(0)
                child2 = [truck_route, drone_sortie]

            if cross_point >= la and point12 < la:  # 卡车与无人机节点互换
                for sortie in parenta1:
                    node_idx = 0
                    for node in sortie:
                        # node_idx = sortie.index(node)
                        if node == a[cross_point]:
                            sortie[node_idx] = a[point12]
                        if node == a[point12]:
                            sortie[node_idx] = a[cross_point]
                        node_idx += 1
                drone_sortie = parenta1
                truck_route = a[:la]
                truck_route.insert(0, 0)
                truck_route.append(0)
                child1 = [truck_route, drone_sortie]

            if cross_point >= lb and point22 < lb:  # 卡车与无人机节点互换
                for sortie in parentb1:
                    node_idx = 0
                    for node in sortie:
                        # node_idx = sortie.index(node)
                        if node == b[cross_point]:
                            sortie[node_idx] = b[point22]
                        if node == b[point22]:
                            sortie[node_idx] = b[cross_point]
                        node_idx += 1
                drone_sortie = parentb1
                truck_route = b[:lb]
                truck_route.insert(0, 0)
                truck_route.append(0)
                child2 = [truck_route, drone_sortie]

        return [child1, child2]

    def mutation1(self,child):
        p = self.mutation(child[1][0])  # 将卡车部分用来变异
        children_ = []
        children_.append(p)
        cand_set = []
        # mincost = inf
        for i in range(0, len(p) - 1):
            cand = [p[i], p[i + 1]]
            cand_set.append(cand)
        # 用新的卡车路径更新无人机分配方案
        for j in child[1][1]:
            dnode = j[1]  # 提取架次中的无人机节点
            mincost = inf
            for node in cand_set:
                truck_cost = self.dismatrix[node[0]][node[1]]
                dnode_cost = (self.dismatrix[node[0]][dnode] + self.dismatrix[dnode][node[1]])/self.alpha
                cost = max(truck_cost, dnode_cost)
                if cost < mincost:
                    mincost = cost
                    minsortie = [node[0], dnode, node[1]]
                    removecand = node
            children_.append(minsortie)
            cand_set = [x for x in cand_set if x != removecand]
        # 给无人机架次进行排序
        l = len(children_)
        drone_sortie = []
        launch_list = children_[0]
        launch_list = launch_list[:len(launch_list) - 1]
        for t in launch_list:
            for idx in range(1, l):
                if children_[idx][0] == t:
                    drone_sortie.append(children_[idx])
        mutation_children = [children_[0], drone_sortie]


        final_solution = []
        # 将解方案合成big tour
        big_tour = copy.deepcopy(mutation_children[0])
        for s in mutation_children[1]:
            if s[2] != 0:
                idx = big_tour.index(s[2])
                big_tour.insert(idx, s[1])
            else:
                big_tour.insert(len(big_tour)-1, s[1])


        final_solution.append(big_tour)
        final_solution.append(mutation_children)
        value = self.evaluate(final_solution)

        return [final_solution, value]

    def mutation(self, child):
        # 从四种方式中随机选择一种方式进行变异
        # 1.基于位置的变异：随机地产生两个变异位，然后将第二个变异位上的基因移动到第一个变异位之前。
        # 2.基于位置的变异：随机地产生两个变异位，然后将第一个变异位上的基因移动到第二个变异位之后。
        # 3.基于次序的变异：该方法随机地产生两个变异位，然后交换这两个变异位上的基因。
        # 4.翻转切片变异：该方法随机产生两个变异位，作为起始位置和结束位置，将两位置之间的基因翻转。
        mutation_child = []
        i = random.randint(1, 4)
        if i == 1:
            mutation_child = self.position_based1(child)
        elif i == 2:
            mutation_child = self.position_based2(child)
        elif i == 3:
            mutation_child = self.order_based(child)
        elif i == 4:
            mutation_child = self.slice_mutation(child)
        return mutation_child

    def position_based1(self, child):
        child_cut = child[1:-1]
        size = len(child_cut)

        # 生成两个不重复的随机变异位
        mutation_points = random.sample(range(1, size), 2)

        # 获取两个变异位的索引
        mutation_point1 = min(mutation_points)
        mutation_point2 = max(mutation_points)

        # 移动第二个变异位上的基因到第一个变异位之前
        gene_to_move = child_cut.pop(mutation_point2)
        child_cut.insert(mutation_point1, gene_to_move)

        child_cut.insert(0, 0)
        child_cut.append(0)

        return child_cut

    def position_based2(self, child):
        child_cut = child[1:-1]
        size = len(child_cut)

        # 生成两个不重复的随机变异位
        mutation_points = random.sample(range(1, size), 2)

        # 获取两个变异位的索引
        mutation_point1 = min(mutation_points)
        mutation_point2 = max(mutation_points)

        # 移动第一个变异位上的基因到第二个变异位之后
        gene_to_move = child_cut.pop(mutation_point1)
        child_cut.insert(mutation_point2 + 1, gene_to_move)

        child_cut.insert(0, 0)
        child_cut.append(0)

        return child_cut

    def order_based(self, child):
        child_cut = child[1:-1]
        size = len(child_cut)

        # 生成两个不重复的随机变异位
        mutation_points = random.sample(range(size), 2)

        # 获取两个变异位的索引
        mutation_point1 = mutation_points[0]
        mutation_point2 = mutation_points[1]

        # 交换两个变异位上的基因
        child_cut[mutation_point1], child_cut[mutation_point2] = child_cut[mutation_point2], child_cut[mutation_point1]
        child_cut.insert(0, 0)
        child_cut.append(0)

        return child_cut

    def slice_mutation(self, child):
        child_cut = child[1:-1]
        size = len(child_cut)

        # 生成两个不重复的随机变异位
        mutation_points = random.sample(range(size), 2)

        # 获取两个变异位的索引
        mutation_point1 = min(mutation_points)
        mutation_point2 = max(mutation_points)

        # 翻转两个变异位之间的基因
        child_cut[mutation_point1:mutation_point2 + 1] = child_cut[mutation_point1:mutation_point2 + 1][::-1]
        child_cut.insert(0, 0)
        child_cut.append(0)

        return child_cut

    def assigned(self, pop, final_pop, final_pop_value):
        f_pop = final_pop
        f_pop_value = final_pop_value
        complete_solution = []
        F_value = []
        for i in range(len(pop)):
            l = len(pop[i])
            cut_num = (l-1) // 10
            truck_tour = []
            drone_sorties = []
            f_value = 0
            start_sortie = []
            start_truck = []
            start_value = 0
            end_sortie = []
            end_truck = []
            end_value = 0
            combine = []
            combine_value_origin = 0
            for cut in range(cut_num):
                # 将染色体表示的tsp路径分为10个节点一段
                sub = pop[i][(cut*10):((cut+1)*10+1)]
                # 每个子路径求解
                [[sub_tour], sub_value, sub_solution] = self.idv.assign_drone(sub, self.dismatrix, self.alpha, self.kappa)
                # 如果是第一个路径
                if cut == 0:
                    t1 = 0
                    t2 = 0
                    end_sortie = sub_solution[0][1][len(sub_solution[0][1])-1]
                    t1 = (self.dismatrix[end_sortie[0]][end_sortie[1]] + self.dismatrix[end_sortie[1]][end_sortie[2]]) / self.alpha
                    index = sub_solution[0][0].index(end_sortie[0])
                    end_truck = sub_solution[0][0][index:]
                    for node in range(len(end_truck)-1):
                        t2 = t2 + self.dismatrix[end_truck[node]][end_truck[node+1]]
                    end_value = max(t1,t2)

                    f_value = f_value + sub_value
                    truck_tour = sub_solution[0][0]
                    drone_sorties = drone_sorties + sub_solution[0][1]

                # 如果是最后一个路径
                if cut == cut_num-1 and cut_num != 1:
                    t1 = 0
                    t2 = 0
                    start_sortie = sub_solution[0][1][0]
                    t1 = (self.dismatrix[start_sortie[0]][start_sortie[1]] + self.dismatrix[start_sortie[1]][
                        start_sortie[2]]) / self.alpha
                    index = sub_solution[0][0].index(start_sortie[2])
                    start_truck = sub_solution[0][0][0:index + 1]
                    for node in range(len(start_truck) - 1):
                        t2 = t2 + self.dismatrix[start_truck[node]][start_truck[node + 1]]
                    start_value = max(t1, t2)

                    combine = pop[i][(pop[i].index(end_sortie[0])):(pop[i].index(start_sortie[2]) + 1)]
                    combine_value_origin = start_value + end_value
                    [[combine_tour], combine_value, combine_solution] = self.idv.assign_drone(combine, self.dismatrix,
                                                                                              self.alpha, self.kappa)
                    # 比较修改过后的value值,如果比原来的小，则用修改后的方案代替原来的
                    if combine_value < combine_value_origin:
                        f_value = f_value + (sub_value - (combine_value_origin - combine_value))
                        # 卡车路径
                        truck_tour = truck_tour[:truck_tour.index(end_sortie[0])] + combine_solution[0][0] + sub_solution[0][0][(sub_solution[0][0].index(start_sortie[2]) + 1):]
                        # truck_tour = truck_tour[:-1] + sub_solution[0][0]

                        drone_sorties = drone_sorties[:-1] + combine_solution[0][1] + sub_solution[0][1][1:]

                    else:
                        f_value = f_value + sub_value
                        truck_tour = sub_solution[0][0]
                        drone_sorties = drone_sorties + sub_solution[0][1]

                # 其他情况
                if cut > 0 and cut < cut_num-1:
                    t1 = 0
                    t2 = 0
                    start_sortie = sub_solution[0][1][0]
                    t1 = (self.dismatrix[start_sortie[0]][start_sortie[1]] + self.dismatrix[start_sortie[1]][start_sortie[2]]) / self.alpha
                    index = sub_solution[0][0].index(start_sortie[2])
                    start_truck = sub_solution[0][0][0:index+1]
                    for node in range(len(start_truck)-1):
                        t2 = t2 + self.dismatrix[start_truck[node]][start_truck[node+1]]
                    start_value = max(t1,t2)

                    combine = pop[i][(pop[i].index(end_sortie[0])):(pop[i].index(start_sortie[2])+1)]
                    combine_value_origin = start_value + end_value
                    [[combine_tour], combine_value, combine_solution] = self.idv.assign_drone(combine, self.dismatrix, self.alpha, self.kappa)
                    # 比较修改过后的value值,如果比原来的小，则用修改后的方案代替原来的
                    if combine_value < combine_value_origin:
                        f_value = f_value + (sub_value - (combine_value_origin - combine_value))
                        # 卡车路径
                        truck_tour = truck_tour[:truck_tour.index(end_sortie[0])] + combine_solution[0][0] + sub_solution[0][0][(sub_solution[0][0].index(start_sortie[2]) + 1):]
                        # truck_tour = truck_tour[:-1] + sub_solution[0][0]

                        drone_sorties = drone_sorties - drone_sorties[-1] + combine_solution[0][1] + sub_solution[0][1][1:]
                        # drone_sorties = drone_sorties + sub_solution[0][1]
                    else:
                        f_value = f_value + sub_value
                        truck_tour = sub_solution[0][0]
                        drone_sorties = drone_sorties + sub_solution[0][1]

                    end_sortie = sub_solution[0][1][len(sub_solution[0][1]) - 1]
                    t1 = (self.dismatrix[end_sortie[0]][end_sortie[1]] + self.dismatrix[end_sortie[1]][end_sortie[2]]) / self.alpha
                    index = sub_solution[0][0].index(end_sortie[0])
                    end_truck = sub_solution[0][0][index:]
                    for node in range(len(end_truck) - 1):
                        t2 = t2 + self.dismatrix[end_truck[node]][end_truck[node + 1]]
                    end_value = max(t1, t2)

            if len(final_pop) < self.popsize:
                if self.is_value_in_list(f_value, f_pop_value):  # 存在相同的适应度值
                    pass
                else:
                    bisect.insort(f_pop_value, f_value)  # 将元素插入到从小到大排列的数组中
                    index = f_pop_value.index(f_value)
                    f_pop.insert(index, pop[i])

            if len(final_pop) == self.popsize:
                if self.is_value_in_list(f_value, f_pop_value):  # 存在相同的适应度值
                    pass
                else:
                    if f_value >= f_pop_value[-1]:
                        pass
                    else:
                        f_pop_value = f_pop_value[:-1]  # 使用切片删除数组的最后一个元素
                        f_pop = f_pop[:-1]
                        bisect.insort(f_pop_value, f_value)  # 将元素插入到从小到大排列的数组中
                        index = f_pop_value.index(f_value)
                        f_pop.insert(index, pop[i])

        return [f_pop, f_pop_value]

    def is_value_in_list(self,value, valuelist):
        return value in valuelist

    def environmental_selection(self, pop, popsize):
        sorted_indices = sorted(range(len(pop[1])), key=lambda x: pop[1][x])
        sorted_value = [pop[1][i] for i in sorted_indices]
        sorted_solution = [pop[0][i] for i in sorted_indices]
        sorted_value = sorted_value[:popsize]
        sorted_solution = sorted_solution[:popsize]
        pop = [sorted_solution, sorted_value]
        return pop
