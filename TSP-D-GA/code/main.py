import time
from math import inf

import numpy as np
from Myproblem import MyProblem

# 实例文件路径
from GA import GA
# test
file_path = '../TSP-D-Instances-master/uniform/uniform-65-n20.txt'
problemName = 'uniform-61-n20.txt'

# 存储结果路径填
out_path = '../output_results'
repeat = 0  # 遗传算法运行次数

problem = MyProblem(file_path)  # define problem
# popsize = int(np.square(len(problem.location)))  # 种群大小
popsize = 100
max_iter = problem.Iter  # 最大迭代次数
# initial_solution = problem.initial_tour()

total_cost = 0
best_cost = inf
best_solution = []
while repeat < 10:
    print('{}th run, benchmark:{}'.format(repeat, problemName))
    myAlgorithm = GA(problem, popsize, max_iter)
    # 记录运行时间
    start_time = time.time()
    myAlgorithm.run()
    end_time = time.time()
    execution_time = end_time - start_time
    current_cost = myAlgorithm.low_cost
    use_times = myAlgorithm.use_times
    # best_solution.append(myAlgorithm.best_solution)
    # best solution：进行10次实验得到最好的结果
    if current_cost <= best_cost:
        best_cost = current_cost
        best_solution = myAlgorithm.best_solution
    total_cost += current_cost
    # myAlgorithm.draw(problemName, out_path, repeat)  # 作图
    myAlgorithm.plot_map()
    myAlgorithm.plot_convergence()
    print('current cost is', current_cost)
    print('current solution is', best_solution)
    print('operators use times are', use_times)
    print('execution_time is ', execution_time)
    print('\n')
    repeat += 1

# mean solution：进行10次实验的平均结果
mean_cost = total_cost / 10
print(problemName, 'finished')
print('best cost is ', best_cost)
print('mean cost is ', mean_cost)

