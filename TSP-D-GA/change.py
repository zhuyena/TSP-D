import numpy as np

file_path = 'TSP-D-Instances-master/uniform/uniform-85-n75.txt'
file = open(file_path)
lines = file.readlines()
linenumbers = len(lines)

location = []
# 仓库节点


# 第9行----length-1行
for line_number in range(9, linenumbers):
    line = lines[line_number]
    data = line.split(' ')
    location.append(data)

depot = lines[7].split()
location.append(depot)

output_string = ''
for i in range(len(location)):
    loci = np.array([location[i][0], location[i][1]])

    # for num in loci:
    #     print(num, end=' ')
    # print('1')

    for num in loci:
        output_string += str(num) + ' '
    output_string += '1 '
print(output_string)



