import os
import shutil
import subprocess


def par():
    lkh_dir = '/LKH/LKH-2.0.10/'
    tsplib_dir = '/TSPLIB/'
    lkh_cmd = 'LKH-2'
    pwd = os.getcwd()
    return [pwd, lkh_dir, lkh_cmd, tsplib_dir]

def lkh(disMatrix):
    fname_tsp = "city031"
    user_comment = "a comment by the user"

    [fileID1, fileID2] = writeTSPLIBfile_FE(fname_tsp, disMatrix, user_comment)
    run_LKHsolver_cmd(fname_tsp)
    copy_toTSPLIBdir_cmd(fname_tsp)
    rm_solution_file_cmd(fname_tsp)
    [tour, cost] = get_tour(fname_tsp,disMatrix)
    return tour

def writeTSPLIBfile_FE(fname_tsp, CostMatrix, user_comment):

    lkh_dir = '/LKH/LKH-2.0.10/'
    tsplib_dir = '/TSPLIB/'
    lkh_cmd = 'LKH-2'
    pwd = os.getcwd()

    dims_tsp = len(CostMatrix)
    name_line = 'NAME : ' + fname_tsp + '\n'
    type_line = 'TYPE: TSP' + '\n'
    comment_line = 'COMMENT : ' + user_comment + '\n'
    tsp_line = 'TYPE : ' + 'TSP' + '\n'
    dimension_line = 'DIMENSION : ' + str(dims_tsp) + '\n'
    edge_weight_type_line = 'EDGE_WEIGHT_TYPE : ' + 'EXPLICIT' + '\n'  # explicit only
    edge_weight_format_line = 'EDGE_WEIGHT_FORMAT: ' + 'FULL_MATRIX' + '\n'
    display_data_type_line = 'DISPLAY_DATA_TYPE: ' + 'NO_DISPLAY' + '\n'  # 'NO_DISPLAY'
    edge_weight_section_line = 'EDGE_WEIGHT_SECTION' + '\n'
    eof_line = 'EOF\n'
    Cost_Matrix_STRline = []
    for i in range(0, dims_tsp):
        cost_matrix_strline = ''
        for j in range(0, dims_tsp - 1):
            cost_matrix_strline = cost_matrix_strline + str(int(CostMatrix[i][j]*1000)) + ' '

        j = dims_tsp - 1
        cost_matrix_strline = cost_matrix_strline + str(int(CostMatrix[i][j]*1000))
        cost_matrix_strline = cost_matrix_strline + '\n'
        Cost_Matrix_STRline.append(cost_matrix_strline)

    fileID = open((pwd + tsplib_dir + fname_tsp + '.tsp'), "w")
    print(name_line)
    fileID.write(name_line)
    fileID.write(comment_line)
    fileID.write(tsp_line)
    fileID.write(dimension_line)
    fileID.write(edge_weight_type_line)
    fileID.write(edge_weight_format_line)
    fileID.write(edge_weight_section_line)
    for i in range(0, len(Cost_Matrix_STRline)):
        fileID.write(Cost_Matrix_STRline[i])

    fileID.write(eof_line)
    fileID.close()

    fileID2 = open((pwd + tsplib_dir + fname_tsp + '.par'), "w")

    problem_file_line = 'PROBLEM_FILE = ' + pwd + tsplib_dir + fname_tsp + '.tsp' + '\n'  # remove pwd + tsplib_dir
    optimum_line = 'OPTIMUM 378032' + '\n'
    move_type_line = 'MOVE_TYPE = 5' + '\n'
    patching_c_line = 'PATCHING_C = 3' + '\n'
    patching_a_line = 'PATCHING_A = 2' + '\n'
    runs_line = 'RUNS = 10' + '\n'
    tour_file_line = 'TOUR_FILE = ' + fname_tsp + '.txt' + '\n'

    fileID2.write(problem_file_line)
    fileID2.write(optimum_line)
    fileID2.write(move_type_line)
    fileID2.write(patching_c_line)
    fileID2.write(patching_a_line)
    fileID2.write(runs_line)
    fileID2.write(tour_file_line)
    fileID2.close()
    return fileID, fileID2

def run_LKHsolver_cmd(fname_basis):
    [pwd, lkh_dir, lkh_cmd, tsplib_dir] = par()
    run_lkh_cmd = pwd + lkh_dir + lkh_cmd + ' ' + pwd + tsplib_dir + fname_basis + '.par'
    # os.system(run_lkh_cmd)
    process = subprocess.Popen(run_lkh_cmd, shell=True)
    return process

def copy_toTSPLIBdir_cmd(fname_basis):
    [pwd, lkh_dir, lkh_cmd, tsplib_dir] = par()
    srcfile = pwd + '/' + fname_basis + '.txt'
    dstpath = pwd + tsplib_dir
    shutil.copy(srcfile, dstpath)

def rm_solution_file_cmd(fname_basis):
    [pwd, lkh_dir, lkh_cmd, tsplib_dir] = par()
    del_file = pwd + '/' + fname_basis + '.txt'
    os.remove(del_file)

def get_tour(fname_basis, disMatrix):
    city_num = len(disMatrix)
    x = []
    txt = open('./TSPLIB/city031.txt', 'r').readlines()
    for i in range(len(txt)):  # 读取路径数据
        if i > 5 and i <= (city_num + 5):
            x += [eval(txt[i][:-1])]
    # 输出结果
    print("最优路径：")

    x = [i - 1 for i in x]  # 将列表中每个元素减1
    tour = print_path(x)
    print(tour)
    grad = get_total_distance(x, disMatrix)
    print('总距离:', grad)
    return [tour, grad]

def print_path(best_line):
    result_cur_best=[]
    for i in best_line:
        result_cur_best+=[i]
    result_path = result_cur_best
    result_path.append(result_path[0])
    return result_path

def get_total_distance(x,disMatrix):
    dista = 0
    for i in range(len(x)):
        if i == len(x) - 1:
            dista += disMatrix[x[i]][x[0]]
        else:
            dista += disMatrix[x[i]][x[i + 1]]
    return dista
