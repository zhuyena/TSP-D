import copy
import time
from cmath import inf
import random
import numpy as np
from matplotlib import pyplot as plt

from Individual import Individual
from population import Population


class GA:
    def __init__(self, problem, popsize, max_iter):
        self.assigned_Qt = None
        self.best_solution = None
        self.low_cost = None
        self.problem = problem
        self.popsize = popsize
        self.Pop = []  # 只有卡车路径的种群
        self.assign_pop = []  # 分配完的种群，包含卡车路径与无人机路径
        self.idv = Individual()  # 个体
        self.populations = Population(popsize,problem)
        self.node_num = problem.node_num
        self.max_iter = max_iter
        self.Qt = []  # 存放后代
        # 存放最优解
        self.final_pop = []
        self.final_pop_value = []
        # self.kappa = MyProblem.kappa  # 飞行耐力
        # self.operations = ['2opt', 'relocate', 'drone_launch_swap', 'drone_rdv_swap', 'launch_rdv_swap', 'id_random', 'id_greedy']  # 局部搜索算子
        self.operations = ['swap2', '3opt', 'relocate', 'swap', '2opt']
        self.reward = [25]*len(self.operations)  # 考究初始值为多少合适？
        self.prop = self.reward / np.sum(self.reward)  # 各个算子初始化的选择概率
        self.use_times = [0]*len(self.operations)
        self.fitnessvalue = []

    def run(self):
        # evaluate_count = 0  # 计算评价次数
        self.Pop = self.populations.creat_pop(self.popsize)  # 生成一个都由卡车服务的种群
        self.assign_pop= self.populations.assigned(self.Pop, self.final_pop, self.final_pop_value)
        current_pop = self.assign_pop

        gen = 0
        while gen < self.max_iter:
            gen += 1

            self.assign_pop = self.populations.next_pop(current_pop, rate=[0.8, 1])

            # self.assign_pop = self.localsearch(self.assign_pop, gen)
            # self.assign_pop = self.localsearch1(self.assign_pop, gen)  # 粗
            self.assign_pop = self.localsearch2(self.assign_pop, gen)  # 细

            current_pop = self.assign_pop

            self.fitnessvalue.append(self.assign_pop[1][0])

        self.low_cost = self.assign_pop[1][0]
        self.best_solution = self.assign_pop[0][0]

    def localsearch(self, pop, gen):

        operations = self.operations
        pop_to_ls = copy.deepcopy(pop)
        # 设置一个初始的reward列表
        reward = self.reward
        # 各个算子初始化的选择概率
        prop = self.prop
        use_times = self.use_times

        new_pop = []
        new_value = []

        # 前1/4代，此时的解方案还未进行优化，多种局部搜索算子同时进行
        if gen <= self.max_iter/4:
            for operator in operations:
                op_index = operations.index(operator)
                # 进行局部搜索的种群副本
                idv_copy = copy.deepcopy(pop_to_ls[0])
                idv_value_copy = copy.deepcopy(pop_to_ls[1])
                # 局部搜索
                [new_pop, new_value, count] = self.problem.neighborhoods(idv_copy, idv_value_copy, operator)  # 返回新的种群和优化的染色体数
                reward[op_index] += count

        else:
            p = np.random.random()  # 随机生成一个概率
            sum_prop = 0
            for j in range(len(operations)):
                sum_prop += prop[j]
                if p <= sum_prop:
                    operator = operations[j]

                    idv_copy = copy.deepcopy(pop_to_ls[0])
                    idv_value_copy = copy.deepcopy(pop_to_ls[1])
                    # 局部搜索
                    [new_pop, new_value, count] = self.problem.neighborhoods(idv_copy, idv_value_copy, operator)  # 返回新的种群和优化的染色体数
                    reward[j] += count
                    use_times[j] += 1
                    break

                else:
                    j += 1

        self.reward = reward
        self.prop = self.reward / np.sum(self.reward)
        self.use_times = use_times

        return [new_pop, new_value]

    def localsearch1(self, pop, gen):

        operations = self.operations
        pop_to_ls = copy.deepcopy(pop)
        # 设置一个初始的reward列表
        reward = self.reward
        # 各个算子初始化的选择概率
        prop = self.prop
        use_times = self.use_times

        new_pop = []
        new_value = []

        p = np.random.random()  # 随机生成一个概率
        # p = 0.9
        sum_prop = 0
        for j in range(len(operations)):
            sum_prop += prop[j]
            if p <= sum_prop:
                operator = operations[j]

                idv_copy = copy.deepcopy(pop_to_ls[0])
                idv_value_copy = copy.deepcopy(pop_to_ls[1])
                # 局部搜索
                [new_pop, new_value, count] = self.problem.neighborhoods(idv_copy, idv_value_copy,
                                                                         operator)  # 返回新的种群和优化的染色体数
                reward[j] += count
                use_times[j] += 1
                break

            else:
                j += 1

        self.reward = reward
        self.prop = self.reward / np.sum(self.reward)
        self.use_times = use_times

        return [new_pop, new_value]

    def localsearch2(self, pop, gen):

        operations = self.operations
        pop_to_ls = copy.deepcopy(pop)
        # 设置一个初始的reward列表
        reward = self.reward
        # 各个算子初始化的选择概率
        prop = self.prop
        use_times = self.use_times

        new_pop = []
        new_value = []

        for idvi in range(50):
            p = np.random.random()  # 随机生成一个概率
            # p = 0.9
            sum_prop = 0
            for j in range(len(operations)):
                sum_prop += prop[j]
                if p <= sum_prop:
                    operator = operations[j]

                    idv_copy = copy.deepcopy(pop_to_ls[0])
                    idv_value_copy = copy.deepcopy(pop_to_ls[1])
                    # 局部搜索
                    [new_pop, new_value, count] = self.problem.neighborhoods1(idv_copy, idv_value_copy, idvi, operator)  # 返回新的种群和优化的染色体数
                    reward[j] += count
                    use_times[j] += 1
                    break

                else:
                    j += 1

            self.reward = reward
            self.prop = self.reward / np.sum(self.reward)
            self.use_times = use_times

        return [new_pop, new_value]

    def plot_map(self):
        x_values = [self.problem.location[i][0] for i in range(len(self.problem.location))]
        x_values = [float(x) for x in x_values]
        y_values = [self.problem.location[i][1] for i in range(len(self.problem.location))]
        y_values = [float(x) for x in y_values]
        city_coordinates = zip(x_values, y_values)
        index = np.arange(self.problem.node_num)
        index.tolist()
        city = dict(zip(index,city_coordinates))

        #  绘制卡车路线
        X = []
        Y = []
        x = []
        y = []
        text_list = []
        text_list_total = []
        for a in range(len(city)):
            X.append(city[a][0])
            Y.append(city[a][1])
            text_list_total.append(str(a))


        [[idvi], f_value, complete_solution] = self.idv.assign_drone(self.best_solution, self.problem.dismatrix, self.problem.alpha,
                                                                     self.problem.kappa)  # 分配卡车与无人机路径
        for v in complete_solution[0][0]:
            x.append(city[v][0])
            y.append(city[v][1])
            text_list.append(str(v))

        # for i in range(len(text_list)):
        #     plt.text(x[i], y[i], text_list[i], ha='center', va='center_baseline')
        for i in range(len(text_list_total)):
            plt.text(X[i], Y[i], text_list_total[i], ha='center', va='center_baseline')

        plt.plot(x, y, 'c-', linewidth=2, markersize=12)
        #  无人机架次
        for s in complete_solution[0][1]:
            x1 = []
            y1 = []
            for s1 in s:
                x1.append(city[s1][0])
                y1.append(city[s1][1])
            plt.plot(x1, y1, 'r--', linewidth=2, markersize=12)

        # 显示图形
        plt.show()

    def plot_convergence(self):
        # 模拟一些示例数据作为收敛曲线上的点
        num_generations = self.max_iter
        fitness_values = self.fitnessvalue

        # 绘制收敛曲线
        plt.figure()
        plt.plot(range(num_generations), fitness_values, marker='o', linestyle='-')

        plt.title('Genetic Algorithm Convergence Curve for TSP-D(no-ls,d-68-20)')
        plt.xlabel('Generation')
        plt.ylabel('Fitness Value')
        plt.grid(True)
        plt.show()