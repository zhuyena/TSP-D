import numpy as np

file_path = 'TSP-D-Instances-master/uniform/uniform-73-n50.txt'
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

X = []
Y = []

for i in range(len(location)):
    loci = np.array([location[i][0], location[i][1]])
    list = loci.tolist()
    float_list = [float(x) for x in list]
    X.append(float_list[0])
    Y.append(float_list[1])

print(X)
print("\n")
print(Y)